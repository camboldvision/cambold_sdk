/* SPDX-License-Identifier: Apache-2.0 */
#include "cb_macos_usb_capture.hpp"

#include <libusb.h>
#include <unistd.h>

#include <algorithm>
#include <mutex>
#include <vector>

namespace
{
std::string usb_error(const char *operation, int code)
{
    return std::string(operation) + ": " + libusb_error_name(code) + " (" +
           std::to_string(code) + ")";
}
} // namespace

struct CbMacUsbCapture::Device
{
    libusb_context *context = nullptr;
    libusb_device_handle *handle = nullptr;
    int control_interface = -1;
    size_t users = 0;
    bool captured = false;
    bool disconnected = false;

    ~Device()
    {
        if (handle)
        {
            // Restoring the macOS driver can reset the USB device. Only do it
            // after every SDK interface is closed, and never after unplugging.
            if (captured && connected())
                (void)libusb_attach_kernel_driver(handle, control_interface);
            libusb_close(handle);
        }
        if (context)
            libusb_exit(context);
    }

    bool connected()
    {
        if (!handle || disconnected)
            return false;
        unsigned char status[2]{};
        const int rc = libusb_control_transfer(handle, LIBUSB_ENDPOINT_IN |
                                                   LIBUSB_REQUEST_TYPE_STANDARD |
                                                   LIBUSB_RECIPIENT_DEVICE,
                                               LIBUSB_REQUEST_GET_STATUS, 0, 0,
                                               status, sizeof(status), 500);
        if (rc == LIBUSB_ERROR_NO_DEVICE)
            disconnected = true;
        return !disconnected;
    }

    bool open(bool required, std::string *error)
    {
        auto fail = [error](const std::string &message)
        {
            if (error)
                *error = message;
            return false;
        };
        int rc = libusb_init(&context);
        if (rc != LIBUSB_SUCCESS)
            return fail(usb_error("macOS USB capture initialization", rc));
        libusb_device **list = nullptr;
        const ssize_t count = libusb_get_device_list(context, &list);
        if (count < 0)
            return fail(usb_error("macOS USB enumeration", static_cast<int>(count)));

        libusb_device *selected = nullptr;
        size_t matches = 0;
        for (ssize_t i = 0; i < count; ++i)
        {
            libusb_device_descriptor descriptor{};
            if (libusb_get_device_descriptor(list[i], &descriptor) == LIBUSB_SUCCESS &&
                descriptor.idVendor == 0x2207 && descriptor.idProduct == 0x0019)
            {
                selected = list[i];
                ++matches;
            }
        }
        if (matches != 1)
        {
            libusb_free_device_list(list, 1);
            if (matches > 1)
                return fail("Multiple CamBold USB cameras found; connect only one camera");
            return required ? fail("CamBold USB camera (2207:0019) not found") : false;
        }

        libusb_config_descriptor *config = nullptr;
        rc = libusb_get_active_config_descriptor(selected, &config);
        if (rc != LIBUSB_SUCCESS)
            rc = libusb_get_config_descriptor(selected, 0, &config);
        if (rc == LIBUSB_SUCCESS)
        {
            for (uint8_t i = 0; i < config->bNumInterfaces; ++i)
                for (int a = 0; a < config->interface[i].num_altsetting; ++a)
                {
                    const auto &alt = config->interface[i].altsetting[a];
                    if (alt.bInterfaceClass == LIBUSB_CLASS_VIDEO && alt.bInterfaceSubClass == 1)
                        control_interface = alt.bInterfaceNumber;
                }
            libusb_free_config_descriptor(config);
        }
        if (rc != LIBUSB_SUCCESS || control_interface < 0)
        {
            libusb_free_device_list(list, 1);
            return fail("CamBold UVC control interface not found in USB descriptors");
        }
        rc = libusb_open(selected, &handle);
        libusb_free_device_list(list, 1);
        if (rc != LIBUSB_SUCCESS)
            return fail(usb_error("macOS USB capture open", rc));

        rc = libusb_detach_kernel_driver(handle, control_interface);
        if (rc != LIBUSB_SUCCESS)
            return fail(usb_error("macOS raw UVC driver capture", rc));
        captured = true;
        return true;
    }
};

struct CbMacUsbCapture::Registry
{
    std::mutex mutex;
    std::vector<std::unique_ptr<Device>> devices;
};

CbMacUsbCapture::Registry &CbMacUsbCapture::registry()
{
    static Registry instance;
    return instance;
}

std::shared_ptr<CbMacUsbCapture> CbMacUsbCapture::acquire(bool required, std::string *error)
{
    if (error)
        error->clear();
    if (geteuid() != 0)
    {
        if (required && error)
            *error = "macOS raw UVC access requires administrator privileges; "
                     "start the application with sudo";
        return {};
    }

    auto lease = std::shared_ptr<CbMacUsbCapture>(new CbMacUsbCapture());
    auto &state = registry();
    std::lock_guard<std::mutex> lock(state.mutex);
    for (const auto &device : state.devices)
    {
        if (!device->connected())
            continue;
        ++device->users;
        lease->device_ = device.get();
        return lease;
    }
    auto device = std::make_unique<Device>();
    if (!device->open(required, error))
        return {};
    state.devices.push_back(std::move(device));
    lease->device_ = state.devices.back().get();
    lease->device_->users = 1;
    return lease;
}

CbMacUsbCapture::~CbMacUsbCapture()
{
    if (!device_)
        return;
    auto &state = registry();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (--device_->users == 0)
        state.devices.erase(std::find_if(state.devices.begin(), state.devices.end(),
                                        [this](const auto &device) { return device.get() == device_; }));
}
