/* SPDX-License-Identifier: Apache-2.0 */
#include "cb_transport.hpp"

#include <libusb.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <cfgmgr32.h>
#include <setupapi.h>
#include <winusb.h>
#endif

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace
{

constexpr uint16_t kCameraVid = 0x2207;
constexpr uint16_t kCameraPid = 0x0019;

bool is_camera(const libusb_device_descriptor &descriptor)
{
    return descriptor.idVendor == kCameraVid && descriptor.idProduct == kCameraPid;
}

struct Candidate
{
    libusb_device *device = nullptr;
    int interface_number = -1;
    uint8_t endpoint_in = 0;
    uint8_t endpoint_out = 0;
};

struct DeviceCandidates
{
    libusb_device *device = nullptr;
    std::vector<Candidate> interfaces;
};

std::string usb_error(int code, const char *where)
{
    std::ostringstream out;
    out << where << ": " << libusb_strerror(static_cast<libusb_error>(code)) << " (" << code << ")";
    return out.str();
}

bool is_bulk(const libusb_endpoint_descriptor &endpoint)
{
    return (endpoint.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK;
}

bool find_endpoints(const libusb_interface_descriptor &alt, uint8_t *endpoint_in,
                    uint8_t *endpoint_out)
{
    uint8_t in = 0;
    uint8_t out = 0;
    for (uint8_t i = 0; i < alt.bNumEndpoints; ++i)
    {
        const auto &ep = alt.endpoint[i];
        if (!is_bulk(ep))
            continue;
        if ((ep.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN)
        {
            if (!in)
                in = ep.bEndpointAddress;
        }
        else if (!out)
        {
            out = ep.bEndpointAddress;
        }
    }
    if (!in || !out)
        return false;
    *endpoint_in = in;
    *endpoint_out = out;
    return true;
}

bool prefers_index(const std::string &requested, bool control, size_t candidate_index)
{
    if (requested.empty())
        return candidate_index == (control ? 1u : 0u);
    std::string text = requested;
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (text.find("control") != std::string::npos || text.find("gser.1") != std::string::npos ||
        text.find("serial.1") != std::string::npos || text.find("bulk1") != std::string::npos)
        return candidate_index == 1u;
    if (text.find("debug") != std::string::npos || text.find("gser.0") != std::string::npos ||
        text.find("serial.0") != std::string::npos || text.find("bulk0") != std::string::npos)
        return candidate_index == 0u;
    return candidate_index == (control ? 1u : 0u);
}

// Product strings belong to the selected USB device, not the logical bulk
// channel. Read once when opening the port and keep status polling I/O-free.
template <typename ReadDescriptor> std::string read_product_name(uint8_t index, ReadDescriptor read)
{
    if (!index)
        return {};
    unsigned char bytes[256]{};
    uint16_t language = 0x0409;
    int count = read(0, 0, bytes, sizeof(bytes));
    if (count >= 4 && bytes[0] >= 4 && bytes[1] == LIBUSB_DT_STRING)
        language = uint16_t(bytes[2]) | (uint16_t(bytes[3]) << 8);
    count = read(index, language, bytes, sizeof(bytes));
    if (count < 2 || bytes[1] != LIBUSB_DT_STRING || bytes[0] < 2 || bytes[0] > count ||
        bytes[0] % 2)
        return {};
    std::string name;
    for (int i = 2; i < bytes[0]; i += 2)
    {
        uint32_t code = uint32_t(bytes[i]) | (uint32_t(bytes[i + 1]) << 8);
        if (code >= 0xd800 && code <= 0xdbff)
        {
            if (i + 3 >= bytes[0])
                return {};
            const uint32_t low = uint32_t(bytes[i + 2]) | (uint32_t(bytes[i + 3]) << 8);
            if (low < 0xdc00 || low > 0xdfff)
                return {};
            code = 0x10000 + ((code - 0xd800) << 10) + low - 0xdc00;
            i += 2;
        }
        else if (code >= 0xdc00 && code <= 0xdfff)
            return {};
        if (code <= 0x20 || code == 0x7f)
        {
            if (!name.empty() && name.back() != ' ')
                name += ' ';
        }
        else if (code < 0x80)
            name += char(code);
        else if (code < 0x800)
        {
            name += char(0xc0 | (code >> 6));
            name += char(0x80 | (code & 0x3f));
        }
        else if (code < 0x10000)
        {
            name += char(0xe0 | (code >> 12));
            name += char(0x80 | ((code >> 6) & 0x3f));
            name += char(0x80 | (code & 0x3f));
        }
        else
        {
            name += char(0xf0 | (code >> 18));
            name += char(0x80 | ((code >> 12) & 0x3f));
            name += char(0x80 | ((code >> 6) & 0x3f));
            name += char(0x80 | (code & 0x3f));
        }
    }
    if (!name.empty() && name.back() == ' ')
        name.pop_back();
    return name;
}

#ifdef _WIN32

const char *winusb_guid_for_interface(int interface_number)
{
    switch (interface_number)
    {
    case 0:
        return "7F3A2B10-6D4E-4C91-9A52-0E7D2F8B6101";
    case 1:
        return "7F3A2B11-6D4E-4C91-9A52-0E7D2F8B6102";
    default:
        return nullptr;
    }
}

bool find_winusb_path(DEVINST device_instance, uint16_t vid, uint16_t pid, int interface_number,
                      std::string *path, std::string *error)
{
    const char *guid = winusb_guid_for_interface(interface_number);
    if (!guid)
    {
        if (error)
            *error = "unsupported WinUSB interface number";
        return false;
    }

    char needle[64];
    std::snprintf(needle, sizeof(needle), "USB\\VID_%04X&PID_%04X&MI_%02X\\", vid, pid,
                  interface_number);

    HDEVINFO devices =
        SetupDiGetClassDevsA(nullptr, "USB", nullptr, DIGCF_ALLCLASSES | DIGCF_PRESENT);
    if (devices == INVALID_HANDLE_VALUE)
    {
        if (error)
            *error = "SetupDiGetClassDevs failed";
        return false;
    }

    SP_DEVINFO_DATA data{};
    data.cbSize = sizeof(data);
    bool found = false;
    for (DWORD index = 0; SetupDiEnumDeviceInfo(devices, index, &data); ++index)
    {
        char instance_id[512]{};
        if (!SetupDiGetDeviceInstanceIdA(devices, &data, instance_id, sizeof(instance_id), nullptr))
            continue;
        std::string id(instance_id);
        std::string upper = id;
        std::transform(upper.begin(), upper.end(), upper.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        if (upper.compare(0, std::strlen(needle), needle) != 0)
            continue;

        // The bundled libusb WinUSB backend exposes DEVINST as session data.
        // Keep the interface attached to the exact composite device selected
        // above, even if another camera has the same VID/PID and interface ID.
        DEVINST parent = 0;
        if (CM_Get_Parent(&parent, data.DevInst, 0) != CR_SUCCESS || parent != device_instance)
            continue;

        std::replace(id.begin(), id.end(), '\\', '#');
        *path = "\\\\?\\" + id + "#{" + guid + "}";
        found = true;
        break;
    }
    SetupDiDestroyDeviceInfoList(devices);
    if (!found && error)
    {
        *error = "WinUSB interface for the selected CamBold device was not found";
    }
    return found;
}

bool open_winusb_interface(libusb_device *device, int interface_number, void **file_out,
                           void **handle_out, std::string *device_name, std::string *error)
{
    libusb_device_descriptor descriptor{};
    if (libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS ||
        !is_camera(descriptor))
    {
        if (error)
            *error = "USB device is not a supported CamBold camera";
        return false;
    }

    std::string path;
    const auto device_instance = static_cast<DEVINST>(libusb_get_session_data(device));
    if (!find_winusb_path(device_instance, descriptor.idVendor, descriptor.idProduct,
                          interface_number, &path, error))
        return false;

    HANDLE file =
        CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                    nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
    if (file == INVALID_HANDLE_VALUE)
    {
        if (error)
        {
            std::ostringstream out;
            out << "CreateFile(WinUSB) failed: " << GetLastError();
            *error = out.str();
        }
        return false;
    }

    WINUSB_INTERFACE_HANDLE handle = nullptr;
    if (!WinUsb_Initialize(file, &handle))
    {
        if (error)
        {
            std::ostringstream out;
            out << "WinUsb_Initialize failed: " << GetLastError();
            *error = out.str();
        }
        CloseHandle(file);
        return false;
    }
    *file_out = file;
    *handle_out = handle;
    *device_name =
        read_product_name(descriptor.iProduct,
                          [handle](uint8_t index, uint16_t language, unsigned char *bytes, int size)
                          {
                              ULONG count = 0;
                              return WinUsb_GetDescriptor(handle, LIBUSB_DT_STRING, index, language,
                                                          bytes, ULONG(size), &count)
                                         ? int(count)
                                         : -1;
                          });
    return true;
}

bool winusb_transfer(void *file_ptr, void *handle_ptr, uint8_t endpoint, uint8_t *data,
                     ULONG length, ULONG timeout_ms, bool read, ULONG *transferred, bool *timed_out,
                     std::string *error)
{
    if (timed_out)
        *timed_out = false;
    HANDLE file = static_cast<HANDLE>(file_ptr);
    WINUSB_INTERFACE_HANDLE handle = static_cast<WINUSB_INTERFACE_HANDLE>(handle_ptr);
    OVERLAPPED overlapped{};
    overlapped.hEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
    if (!overlapped.hEvent)
    {
        if (error)
            *error = "CreateEvent failed";
        return false;
    }

    ULONG count = 0;
    BOOL ok = read ? WinUsb_ReadPipe(handle, endpoint, data, length, &count, &overlapped)
                   : WinUsb_WritePipe(handle, endpoint, data, length, &count, &overlapped);
    if (!ok && GetLastError() != ERROR_IO_PENDING)
    {
        if (error)
        {
            std::ostringstream out;
            out << (read ? "WinUsb_ReadPipe failed: " : "WinUsb_WritePipe failed: ")
                << GetLastError();
            *error = out.str();
        }
        CloseHandle(overlapped.hEvent);
        return false;
    }

    DWORD wait = WaitForSingleObject(overlapped.hEvent, timeout_ms);
    if (wait == WAIT_TIMEOUT)
    {
        (void)WinUsb_AbortPipe(handle, endpoint);
        if (timed_out)
            *timed_out = true;
        CloseHandle(overlapped.hEvent);
        return true;
    }
    if (wait != WAIT_OBJECT_0 || !GetOverlappedResult(file, &overlapped, &count, FALSE))
    {
        if (error)
        {
            std::ostringstream out;
            out << "WinUSB transfer failed: " << GetLastError();
            *error = out.str();
        }
        CloseHandle(overlapped.hEvent);
        return false;
    }
    CloseHandle(overlapped.hEvent);
    if (transferred)
        *transferred = count;
    return true;
}

#endif // _WIN32

} // namespace

std::unique_ptr<CbTransport> CbTransport::open(const std::string &requested, bool control,
                                               std::string *error)
{
    auto fail = [&](const std::string &message) -> std::unique_ptr<CbTransport>
    {
        if (error)
            *error = message;
        return nullptr;
    };

    auto port = std::unique_ptr<CbTransport>(new CbTransport());
#ifdef __APPLE__
    // Capture before claiming either bulk interface. macOS capture terminates
    // existing USB clients, so capturing when preview starts would break them.
    std::string capture_error;
    port->macos_capture_ = CbMacUsbCapture::acquire(false, &capture_error);
    if (!capture_error.empty())
        return fail(capture_error);
#endif
    int rc = libusb_init(&port->context_);
    if (rc != LIBUSB_SUCCESS)
        return fail(usb_error(rc, "libusb_init"));

    libusb_device **list = nullptr;
    const ssize_t count = libusb_get_device_list(port->context_, &list);
    if (count < 0)
        return fail(usb_error(static_cast<int>(count), "libusb_get_device_list"));

    std::vector<DeviceCandidates> devices;
    size_t camera_count = 0;
    for (ssize_t n = 0; n < count; ++n)
    {
        libusb_device *device = list[n];
        libusb_device_descriptor descriptor{};
        if (libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS ||
            !is_camera(descriptor))
            continue;
        ++camera_count;

        libusb_config_descriptor *config = nullptr;
        if (libusb_get_config_descriptor(device, 0, &config) != LIBUSB_SUCCESS)
            continue;

        DeviceCandidates group;
        group.device = device;
        for (uint8_t i = 0; i < config->bNumInterfaces; ++i)
        {
            const libusb_interface &intf = config->interface[i];
            for (int a = 0; a < intf.num_altsetting; ++a)
            {
                const libusb_interface_descriptor &alt = intf.altsetting[a];
                /* f_serial is intentionally vendor-specific (0xff), so it
                 * cannot be confused with UVC, MTP, or the remaining ACM. */
                if (alt.bInterfaceClass != LIBUSB_CLASS_VENDOR_SPEC)
                    continue;
                uint8_t endpoint_in = 0;
                uint8_t endpoint_out = 0;
                if (!find_endpoints(alt, &endpoint_in, &endpoint_out))
                    continue;
                group.interfaces.push_back(
                    {device, alt.bInterfaceNumber, endpoint_in, endpoint_out});
                break;
            }
        }
        libusb_free_config_descriptor(config);
        if (group.interfaces.size() >= 2)
        {
            std::sort(group.interfaces.begin(), group.interfaces.end(),
                      [](const Candidate &a, const Candidate &b)
                      { return a.interface_number < b.interface_number; });
            devices.push_back(std::move(group));
        }
    }

    // The public API currently has no physical-camera selector. Do not let
    // independently opened control/debug channels choose different cameras.
    if (camera_count > 1)
    {
        libusb_free_device_list(list, 1);
        return fail("Multiple CamBold USB cameras found; connect only one camera");
    }

    if (devices.empty())
    {
        libusb_free_device_list(list, 1);
        /* A disconnected camera is a normal polling state, not an error to
         * deliver on every reconnect attempt. */
        return fail(std::string{});
    }

    /* A product gadget has gser.0 followed by gser.1. Use the two interfaces
     * of the selected CamBold device and the role-specific index; this also
     * works if the host assigned a different USB interface number after a
     * reconnect. */
    const DeviceCandidates *chosen_device = &devices.front();
    size_t chosen_index = control ? 1u : 0u;
    for (size_t i = 0; i < chosen_device->interfaces.size(); ++i)
    {
        if (prefers_index(requested, control, i))
        {
            chosen_index = i;
            break;
        }
    }
    if (chosen_index >= chosen_device->interfaces.size())
    {
        libusb_free_device_list(list, 1);
        return fail("camera bulk interface index is unavailable");
    }
    const Candidate candidate = chosen_device->interfaces[chosen_index];

#ifdef _WIN32
    {
        std::string native_error;
        if (open_winusb_interface(candidate.device, candidate.interface_number, &port->winusb_file_,
                                  &port->winusb_handle_, &port->device_name_, &native_error))
        {
            libusb_free_device_list(list, 1);
            libusb_exit(port->context_);
            port->context_ = nullptr;
            port->interface_number_ = candidate.interface_number;
            port->endpoint_in_ = candidate.endpoint_in;
            port->endpoint_out_ = candidate.endpoint_out;
            port->name_ = control ? "bulk-control" : "bulk-debug";
            return port;
        }
    }
#endif

    rc = libusb_open(candidate.device, &port->handle_);
    libusb_free_device_list(list, 1);
    if (rc != LIBUSB_SUCCESS)
        return fail(usb_error(rc, "libusb_open"));

    /* On Linux, auto-detach handles a stale generic USB-serial binding.
     * On macOS the shared lease owns device capture; individual vendor
     * interfaces must not detach or reattach the composite device. */
#if defined(__linux__)
    (void)libusb_set_auto_detach_kernel_driver(port->handle_, 1);
#endif
    rc = libusb_claim_interface(port->handle_, candidate.interface_number);
    if (rc != LIBUSB_SUCCESS)
    {
        port->close();
        return fail(usb_error(rc, "libusb_claim_interface"));
    }

    libusb_device_descriptor descriptor{};
    if (libusb_get_device_descriptor(libusb_get_device(port->handle_), &descriptor) ==
        LIBUSB_SUCCESS)
        port->device_name_ = read_product_name(
            descriptor.iProduct, [handle = port->handle_](uint8_t index, uint16_t language,
                                                          unsigned char *bytes, int size)
            { return libusb_get_string_descriptor(handle, index, language, bytes, size); });
    port->interface_number_ = candidate.interface_number;
    port->endpoint_in_ = candidate.endpoint_in;
    port->endpoint_out_ = candidate.endpoint_out;
    port->name_ = control ? "bulk-control" : "bulk-debug";
    return port;
}

CbTransport::~CbTransport()
{
    close();
}

size_t CbTransport::write(const std::vector<uint8_t> &data)
{
#ifdef _WIN32
    if (winusb_handle_)
    {
        size_t offset = 0;
        while (offset < data.size())
        {
            const ULONG requested =
                static_cast<ULONG>(std::min<size_t>(data.size() - offset, 64u * 1024u));
            ULONG transferred = 0;
            bool timed_out = false;
            std::string error;
            if (!winusb_transfer(winusb_file_, winusb_handle_, endpoint_out_,
                                 const_cast<uint8_t *>(data.data() + offset), requested, 1000,
                                 false, &transferred, &timed_out, &error))
                throw std::runtime_error(error.empty() ? "WinUSB bulk OUT failed" : error);
            if (timed_out || transferred == 0)
                throw std::runtime_error("WinUSB bulk OUT timed out");
            offset += transferred;
        }
        return data.size();
    }
#endif
    if (!handle_)
        throw std::runtime_error("bulk port is closed");
    size_t offset = 0;
    while (offset < data.size())
    {
        const int requested = static_cast<int>(std::min<size_t>(data.size() - offset, 64u * 1024u));
        int transferred = 0;
        const int rc = libusb_bulk_transfer(handle_, endpoint_out_,
                                            const_cast<unsigned char *>(data.data() + offset),
                                            requested, &transferred, 1000);
        if (rc != LIBUSB_SUCCESS)
            throw std::runtime_error(usb_error(rc, "bulk OUT"));
        if (transferred <= 0)
            throw std::runtime_error("bulk OUT returned zero bytes");
        offset += static_cast<size_t>(transferred);
    }
    return offset;
}

size_t CbTransport::read(uint8_t *data, size_t capacity, int timeout_ms)
{
    /* WinUSB and libusb both interpret a zero timeout differently from a
     * useful short poll.  One millisecond still gives the caller a genuinely
     * non-blocking-feeling read while avoiding an immediate AbortPipe for
     * every empty poll. */
    const unsigned int transfer_timeout = static_cast<unsigned int>(std::max(timeout_ms, 1));
#ifdef _WIN32
    if (winusb_handle_)
    {
        if (!data || capacity == 0)
            return 0;
        const ULONG length = static_cast<ULONG>(std::min<size_t>(capacity, ULONG_MAX));
        ULONG transferred = 0;
        bool timed_out = false;
        std::string error;
        if (!winusb_transfer(winusb_file_, winusb_handle_, endpoint_in_, data, length,
                             transfer_timeout, true, &transferred, &timed_out, &error))
            throw std::runtime_error(error.empty() ? "WinUSB bulk IN failed" : error);
        return timed_out ? 0 : static_cast<size_t>(transferred);
    }
#endif
    if (!handle_)
        throw std::runtime_error("bulk port is closed");
    if (!data || capacity == 0)
        return 0;
    const int length = static_cast<int>(std::min<size_t>(capacity, INT_MAX));
    int transferred = 0;
    const int rc = libusb_bulk_transfer(handle_, endpoint_in_, data, length, &transferred,
                                        static_cast<unsigned int>(transfer_timeout));
    if (rc == LIBUSB_ERROR_TIMEOUT)
        return 0;
    if (rc != LIBUSB_SUCCESS)
        throw std::runtime_error(usb_error(rc, "bulk IN"));
    return static_cast<size_t>(std::max(transferred, 0));
}

void CbTransport::close()
{
#ifdef _WIN32
    if (winusb_handle_)
    {
        WinUsb_Free(static_cast<WINUSB_INTERFACE_HANDLE>(winusb_handle_));
        winusb_handle_ = nullptr;
    }
    if (winusb_file_)
    {
        CloseHandle(static_cast<HANDLE>(winusb_file_));
        winusb_file_ = nullptr;
    }
#endif
    if (handle_)
    {
        if (interface_number_ >= 0)
            (void)libusb_release_interface(handle_, interface_number_);
        libusb_close(handle_);
        handle_ = nullptr;
    }
    if (context_)
    {
        libusb_exit(context_);
        context_ = nullptr;
    }
    interface_number_ = -1;
    endpoint_in_ = endpoint_out_ = 0;
#ifdef __APPLE__
    macos_capture_.reset();
#endif
}

int CbTransport::resize_terminal(uint16_t, uint16_t)
{
    return -ENOTSUP;
}
