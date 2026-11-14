/* SPDX-License-Identifier: Apache-2.0 */

#include "cb_native_video.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cwctype>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#else
extern "C"
{
#include <libuvc/libuvc.h>
}
#endif

namespace
{

constexpr int kVid = 0x2207;
constexpr int kPid = 0x0019;
constexpr unsigned int kWidth = 2160;
constexpr unsigned int kHeight = 1280;

uint32_t le32(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

uint64_t le64(const uint8_t *p)
{
    uint64_t value = 0;
    for (unsigned int i = 0; i < 8; ++i)
        value |= static_cast<uint64_t>(p[i]) << (i * 8U);
    return value;
}

#ifndef _WIN32
const char *uvc_error_text(uvc_error_t error)
{
    return uvc_strerror(error);
}
#else
std::string hresult_text(HRESULT hr)
{
    char buffer[32]{};
    std::snprintf(buffer, sizeof(buffer), "HRESULT 0x%08lx", static_cast<unsigned long>(hr));
    return buffer;
}

bool is_target_device(IMFAttributes *device)
{
    WCHAR *symbolic = nullptr;
    UINT32 symbolic_length = 0;
    if (FAILED(device->GetAllocatedString(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &symbolic,
            &symbolic_length)))
        return false;

    std::wstring value(symbolic, symbolic_length);
    CoTaskMemFree(symbolic);
    for (auto &character : value)
        character = static_cast<wchar_t>(towlower(character));

    // A USB symbolic link contains USB#VID_xxxx&PID_xxxx[&MI_xx]#instance.
    // Match only its hardware-ID component. Friendly names such as "UVC Camera"
    // are shared by unrelated products and must never be used as a fallback.
    const auto begin = value.find(L'#');
    if (begin == std::wstring::npos)
        return false;
    const auto end = value.find(L'#', begin + 1);
    if (end == std::wstring::npos)
        return false;
    const auto hardware_id = value.substr(begin + 1, end - begin - 1);
    const std::wstring identity = L"vid_2207&pid_0019";
    return hardware_id == identity ||
           hardware_id.compare(0, identity.size() + 1, identity + L"&") == 0;
}

struct NativeVideoMode
{
    IMFMediaType *type;
    UINT32 numerator;
    UINT32 denominator;
};

std::string video_mode_text(IMFMediaType *type)
{
    GUID subtype{};
    UINT32 width = 0, height = 0, numerator = 0, denominator = 0;
    type->GetGUID(MF_MT_SUBTYPE, &subtype);
    MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &width, &height);
    MFGetAttributeRatio(type, MF_MT_FRAME_RATE, &numerator, &denominator);
    char format[5]{};
    for (unsigned int i = 0; i < 4; ++i)
    {
        const auto character = (subtype.Data1 >> (8 * i)) & 0xff;
        format[i] = character >= 32 && character <= 126 ? static_cast<char>(character) : '?';
    }
    return std::string(format) + " " + std::to_string(width) + "x" + std::to_string(height) +
           " @" + std::to_string(numerator) + "/" + std::to_string(denominator);
}

bool is_native_mjpeg(IMFMediaType *type)
{
    GUID subtype{};
    UINT32 width = 0, height = 0;
    return SUCCEEDED(type->GetGUID(MF_MT_SUBTYPE, &subtype)) && subtype == MFVideoFormat_MJPG &&
           SUCCEEDED(MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &width, &height)) &&
           width == kWidth && height == kHeight;
}

HRESULT check_current_mjpeg(IMFSourceReader *reader)
{
    IMFMediaType *current = nullptr;
    HRESULT result = reader->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &current);
    if (SUCCEEDED(result) && !is_native_mjpeg(current))
        result = MF_E_INVALIDMEDIATYPE;
    if (current)
        current->Release();
    return result;
}

HRESULT configure_mjpeg(IMFSourceReader *reader, std::string *error)
{
    std::vector<NativeVideoMode> modes;
    std::string advertised;
    HRESULT enumeration_result = S_OK;
    for (DWORD index = 0;; ++index)
    {
        IMFMediaType *type = nullptr;
        enumeration_result =
            reader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, index, &type);
        if (FAILED(enumeration_result))
            break;
        // Keep the diagnostic bounded for the public C API's error buffer.
        if (advertised.size() < 240)
        {
            if (!advertised.empty())
                advertised += ", ";
            advertised += video_mode_text(type);
        }
        if (is_native_mjpeg(type))
        {
            UINT32 numerator = 0, denominator = 0;
            MFGetAttributeRatio(type, MF_MT_FRAME_RATE, &numerator, &denominator);
            modes.push_back({type, numerator, denominator});
        }
        else
            type->Release();
    }
    if (modes.empty())
    {
        if (error)
        {
            *error = "UVC did not advertise native MJPEG " + std::to_string(kWidth) + "x" +
                     std::to_string(kHeight) + "; available: " +
                     (advertised.empty() ? "none" : advertised);
            if (enumeration_result != MF_E_NO_MORE_TYPES)
                *error += "; enumeration failed: " + hresult_text(enumeration_result);
        }
        return enumeration_result == MF_E_NO_MORE_TYPES ? MF_E_INVALIDMEDIATYPE
                                                        : enumeration_result;
    }

    // Prefer the fastest interval, but try every native type, including duplicate
    // rates (e.g. VideoInfo/VideoInfo2) and types without a fixed frame-rate field.
    std::stable_sort(modes.begin(), modes.end(),
                     [](const NativeVideoMode &a, const NativeVideoMode &b)
                     {
                         const double rate_a =
                             a.denominator ? double(a.numerator) / a.denominator : 0;
                         const double rate_b =
                             b.denominator ? double(b.numerator) / b.denominator : 0;
                         return rate_a > rate_b;
                     });
    IMFSourceReaderEx *native_reader = nullptr;
    reader->QueryInterface(IID_PPV_ARGS(&native_reader));
    HRESULT result = MF_E_INVALIDMEDIATYPE;
    std::string attempts;
    for (const auto &mode : modes)
    {
        // Select the camera's compressed output before choosing the reader's
        // output. SetCurrentMediaType alone can enter decoder negotiation when
        // Windows has selected an uncompressed default camera format.
        HRESULT native_result = E_NOINTERFACE;
        if (native_reader)
        {
            DWORD flags = 0;
            native_result = native_reader->SetNativeMediaType(
                MF_SOURCE_READER_FIRST_VIDEO_STREAM, mode.type, &flags);
        }
        result = SUCCEEDED(native_result) ? check_current_mjpeg(reader) : native_result;
        if (FAILED(result))
        {
            result =
                reader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, mode.type);
            if (SUCCEEDED(result))
                result = check_current_mjpeg(reader);
        }
        if (SUCCEEDED(result))
            break;
        if (attempts.size() < 240)
        {
            if (!attempts.empty())
                attempts += ", ";
            attempts += std::to_string(mode.numerator) + "/" + std::to_string(mode.denominator) +
                        "=" + hresult_text(result);
            if (native_reader && FAILED(native_result))
                attempts += " (native " + hresult_text(native_result) + ")";
        }
    }
    for (const auto &mode : modes)
        mode.type->Release();
    if (native_reader)
        native_reader->Release();
    if (FAILED(result) && error)
    {
        *error = "UVC MJPEG mode negotiation failed: " + hresult_text(result) +
                 "; modes tried (fps): " + attempts;
        if (enumeration_result != MF_E_NO_MORE_TYPES)
            *error += "; enumeration failed: " + hresult_text(enumeration_result);
    }
    return result;
}
#endif

} // namespace

struct CbVideoSession::Impl
{
    explicit Impl(FrameCallback callback_in) : callback(std::move(callback_in)) {}

    FrameCallback callback;
#ifndef _WIN32
    uvc_context_t *context = nullptr;
    uvc_device_t *device = nullptr;
    uvc_device_handle_t *handle = nullptr;
    uvc_stream_ctrl_t stream_ctrl{};
#else
    IMFSourceReader *reader = nullptr;
    std::thread capture_thread;
    std::atomic<bool> stop_requested{false};
    bool com_initialized = false;
    bool mf_initialized = false;
#endif
    bool streaming = false;
    mutable std::mutex mutex;

#ifndef _WIN32
    static void frame_callback(uvc_frame_t *frame, void *user)
    {
        auto *self = static_cast<Impl *>(user);
        if (!self || !frame || !frame->data || frame->data_bytes == 0 ||
            frame->frame_format != UVC_FRAME_FORMAT_MJPEG)
            return;

        CbVideoSession::Frame output;
        const auto *data = static_cast<const uint8_t *>(frame->data);
        output.jpeg.assign(data, data + frame->data_bytes);
        output.sequence = frame->sequence;
        output.width = frame->width;
        output.height = frame->height;
        cb_parse_jpeg_metadata(output.jpeg.data(), output.jpeg.size(), &output);
        if (self->callback)
            self->callback(std::move(output));
    }
#else
    void capture_loop()
    {
        CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        while (!stop_requested.load(std::memory_order_acquire))
        {
            DWORD stream_index = 0;
            DWORD flags = 0;
            LONGLONG timestamp = 0;
            IMFSample *sample = nullptr;
            const HRESULT result = reader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0,
                                                      &stream_index, &flags, &timestamp, &sample);
            if (sample)
            {
                IMFMediaBuffer *buffer = nullptr;
                if (SUCCEEDED(sample->ConvertToContiguousBuffer(&buffer)))
                {
                    BYTE *data = nullptr;
                    DWORD max_length = 0;
                    DWORD current_length = 0;
                    if (SUCCEEDED(buffer->Lock(&data, &max_length, &current_length)) && data &&
                        current_length > 0)
                    {
                        CbVideoSession::Frame output;
                        output.jpeg.assign(data, data + current_length);
                        output.width = kWidth;
                        output.height = kHeight;
                        output.sequence = 0;
                        cb_parse_jpeg_metadata(output.jpeg.data(), output.jpeg.size(), &output);
                        if (callback)
                            callback(std::move(output));
                        buffer->Unlock();
                    }
                    buffer->Release();
                }
                sample->Release();
            }
            if (FAILED(result) || (flags & MF_SOURCE_READERF_ENDOFSTREAM))
                break;
        }
        CoUninitialize();
    }

    bool start_media_foundation(std::string *error)
    {
        HRESULT result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        if (FAILED(result) && result != RPC_E_CHANGED_MODE)
        {
            if (error)
                *error = "Media Foundation COM initialization failed: " + hresult_text(result);
            return false;
        }
        com_initialized = result == S_OK || result == S_FALSE;
        result = MFStartup(MF_VERSION, MFSTARTUP_FULL);
        if (FAILED(result))
        {
            if (error)
                *error = "Media Foundation startup failed: " + hresult_text(result);
            if (com_initialized)
                CoUninitialize();
            com_initialized = false;
            return false;
        }
        mf_initialized = true;

        IMFAttributes *attributes = nullptr;
        IMFActivate **devices = nullptr;
        UINT32 count = 0;
        result = MFCreateAttributes(&attributes, 2);
        if (SUCCEEDED(result))
            result = attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                                         MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (SUCCEEDED(result))
            result = MFEnumDeviceSources(attributes, &devices, &count);
        if (attributes)
            attributes->Release();
        if (FAILED(result))
        {
            if (error)
                *error = "UVC enumeration failed: " + hresult_text(result);
            MFShutdown();
            mf_initialized = false;
            if (com_initialized)
                CoUninitialize();
            com_initialized = false;
            return false;
        }

        IMFActivate *selected = nullptr;
        for (UINT32 i = 0; i < count; ++i)
        {
            if (is_target_device(devices[i]))
            {
                selected = devices[i];
                selected->AddRef();
                break;
            }
        }
        for (UINT32 i = 0; i < count; ++i)
            devices[i]->Release();
        CoTaskMemFree(devices);
        if (!selected)
        {
            if (error)
                *error = "CamBold UVC camera (USB 2207:0019) not found";
            MFShutdown();
            mf_initialized = false;
            if (com_initialized)
                CoUninitialize();
            com_initialized = false;
            return false;
        }

        IMFMediaSource *source = nullptr;
        result = selected->ActivateObject(IID_PPV_ARGS(&source));
        selected->Release();
        IMFAttributes *reader_attributes = nullptr;
        if (SUCCEEDED(result))
            result = MFCreateAttributes(&reader_attributes, 1);
        if (SUCCEEDED(result))
            result = reader_attributes->SetUINT32(MF_READWRITE_DISABLE_CONVERTERS, TRUE);
        if (SUCCEEDED(result))
            result = MFCreateSourceReaderFromMediaSource(source, reader_attributes, &reader);
        if (reader_attributes)
            reader_attributes->Release();
        if (source)
            source->Release();
        if (FAILED(result))
        {
            if (error)
                *error = "Media Foundation could not open UVC camera: " + hresult_text(result);
            if (reader)
            {
                reader->Release();
                reader = nullptr;
            }
            MFShutdown();
            mf_initialized = false;
            if (com_initialized)
                CoUninitialize();
            com_initialized = false;
            return false;
        }

        result = configure_mjpeg(reader, error);
        if (SUCCEEDED(result))
        {
            result = reader->SetStreamSelection(MF_SOURCE_READER_FIRST_VIDEO_STREAM, TRUE);
            if (FAILED(result) && error)
                *error = "UVC video stream selection failed: " + hresult_text(result);
        }
        if (FAILED(result))
        {
            reader->Release();
            reader = nullptr;
            MFShutdown();
            mf_initialized = false;
            if (com_initialized)
                CoUninitialize();
            com_initialized = false;
            return false;
        }
        stop_requested.store(false, std::memory_order_release);
        capture_thread = std::thread(&Impl::capture_loop, this);
        streaming = true;
        return true;
    }
#endif

    bool start(std::string *error)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (streaming)
            return true;

#ifndef _WIN32
        uvc_error_t result = uvc_init(&context, nullptr);
        if (result < 0)
        {
            if (error)
                *error = std::string("uvc_init: ") + uvc_error_text(result);
            context = nullptr;
            return false;
        }

        result = uvc_find_device(context, &device, kVid, kPid, nullptr);
        if (result < 0)
        {
            if (error)
                *error =
                    std::string("UVC camera ") + "2207:0019 not found: " + uvc_error_text(result);
            uvc_exit(context);
            context = nullptr;
            return false;
        }

        result = uvc_open(device, &handle);
        if (result < 0)
        {
            if (error)
            {
                *error = std::string("uvc_open: ") + uvc_error_text(result) +
                         " (" + std::to_string(static_cast<int>(result)) + ")";
                if (result == UVC_ERROR_ACCESS || result == UVC_ERROR_BUSY)
                {
#if defined(__APPLE__)
                    *error += "; macOS could not claim the UVC interface; close other camera "
                              "applications and reconnect the camera";
#else
                    *error += "; check USB device permissions and close other camera applications";
#endif
                }
            }
            uvc_unref_device(device);
            device = nullptr;
            uvc_exit(context);
            context = nullptr;
            return false;
        }

        result = uvc_get_stream_ctrl_format_size(handle, &stream_ctrl, UVC_FRAME_FORMAT_MJPEG,
                                                 kWidth, kHeight, 0);
        if (result < 0)
        {
            if (error)
                *error = std::string("UVC MJPEG mode unavailable: ") + uvc_error_text(result);
            uvc_close(handle);
            handle = nullptr;
            uvc_unref_device(device);
            device = nullptr;
            uvc_exit(context);
            context = nullptr;
            return false;
        }

        result = uvc_start_streaming(handle, &stream_ctrl, frame_callback, this, 0);
        if (result < 0)
        {
            if (error)
                *error = std::string("uvc_start_streaming: ") + uvc_error_text(result);
            uvc_close(handle);
            handle = nullptr;
            uvc_unref_device(device);
            device = nullptr;
            uvc_exit(context);
            context = nullptr;
            return false;
        }
        streaming = true;
        return true;
#else
        return start_media_foundation(error);
#endif
    }

    void stop()
    {
#ifdef _WIN32
        std::thread thread;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!streaming && !reader)
                return;
            stop_requested.store(true, std::memory_order_release);
            if (reader)
                reader->Flush(MF_SOURCE_READER_FIRST_VIDEO_STREAM);
            thread = std::move(capture_thread);
            streaming = false;
        }
        if (thread.joinable())
            thread.join();
        std::lock_guard<std::mutex> lock(mutex);
        if (reader)
            reader->Release();
        reader = nullptr;
        if (mf_initialized)
            MFShutdown();
        mf_initialized = false;
        if (com_initialized)
            CoUninitialize();
        com_initialized = false;
#else
        std::lock_guard<std::mutex> lock(mutex);
        if (streaming && handle)
            uvc_stop_streaming(handle);
        streaming = false;
        if (handle)
            uvc_close(handle);
        handle = nullptr;
        if (device)
            uvc_unref_device(device);
        device = nullptr;
        if (context)
            uvc_exit(context);
        context = nullptr;
#endif
    }
};

CbVideoSession::CbVideoSession(FrameCallback callback) : impl_(new Impl(std::move(callback))) {}

CbVideoSession::~CbVideoSession()
{
    if (impl_)
    {
        impl_->stop();
        delete impl_;
    }
}

bool CbVideoSession::start(std::string *error)
{
    return impl_->start(error);
}
void CbVideoSession::stop()
{
    impl_->stop();
}
bool CbVideoSession::running() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->streaming;
}
