/* SPDX-License-Identifier: Apache-2.0 */

#include "cb_native_video.hpp"

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
    static bool is_target_device(IMFActivate *activate)
    {
        WCHAR *symbolic = nullptr;
        UINT32 symbolic_length = 0;
        bool target = false;
        if (SUCCEEDED(activate->GetAllocatedString(
                MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &symbolic,
                &symbolic_length)))
        {
            std::wstring value(symbolic, symbolic_length);
            for (auto &character : value)
                character = static_cast<wchar_t>(towlower(character));
            target = value.find(L"vid_2207&pid_0019") != std::wstring::npos;
            CoTaskMemFree(symbolic);
        }
        if (!target)
        {
            WCHAR *friendly = nullptr;
            UINT32 friendly_length = 0;
            if (SUCCEEDED(activate->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
                                                       &friendly, &friendly_length)))
            {
                std::wstring value(friendly, friendly_length);
                for (auto &character : value)
                    character = static_cast<wchar_t>(towlower(character));
                target = value.find(L"uvc camera") != std::wstring::npos ||
                         value.find(L"rk3576") != std::wstring::npos;
                CoTaskMemFree(friendly);
            }
        }
        return target;
    }

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
                *error = "Media Foundation could not find UVC Camera (2207:0019)";
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
        if (SUCCEEDED(result))
            result = MFCreateSourceReaderFromMediaSource(source, nullptr, &reader);
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

        // Request the fastest native MJPEG mode advertised by this firmware.
        // The independent trigger control can then run at any slower rate.
        IMFMediaType *type = nullptr;
        double best_rate = 0;
        for (DWORD index = 0;; ++index)
        {
            IMFMediaType *candidate = nullptr;
            const HRESULT status =
                reader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, index, &candidate);
            if (status == MF_E_NO_MORE_TYPES)
                break;
            if (FAILED(status))
                break;
            GUID subtype{};
            UINT32 width = 0, height = 0, numerator = 0, denominator = 0;
            if (SUCCEEDED(candidate->GetGUID(MF_MT_SUBTYPE, &subtype)) &&
                subtype == MFVideoFormat_MJPG &&
                SUCCEEDED(MFGetAttributeSize(candidate, MF_MT_FRAME_SIZE, &width, &height)) &&
                width == kWidth && height == kHeight &&
                SUCCEEDED(
                    MFGetAttributeRatio(candidate, MF_MT_FRAME_RATE, &numerator, &denominator)) &&
                denominator != 0 && double(numerator) / denominator > best_rate)
            {
                if (type)
                    type->Release();
                type = candidate;
                candidate = nullptr;
                best_rate = double(numerator) / denominator;
            }
            if (candidate)
                candidate->Release();
        }
        result =
            type ? reader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, type)
                 : MF_E_INVALIDMEDIATYPE;
        if (type)
            type->Release();
        if (FAILED(result))
        {
            if (error)
                *error =
                    "UVC MJPEG mode unavailable through Media Foundation: " + hresult_text(result);
            reader->Release();
            reader = nullptr;
            MFShutdown();
            mf_initialized = false;
            if (com_initialized)
                CoUninitialize();
            com_initialized = false;
            return false;
        }
        reader->SetStreamSelection(MF_SOURCE_READER_FIRST_VIDEO_STREAM, TRUE);
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
                *error = std::string("uvc_open: ") + uvc_error_text(result) +
                         " (Windows may require WinUSB on the UVC interface)";
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
