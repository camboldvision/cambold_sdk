/* SPDX-License-Identifier: Apache-2.0 */

#include "cb_native_video.hpp"
#include "cb_sdk.h"

#include <cerrno>
#include <cstring>
#include <memory>
#include <new>
#include <string>

struct cb_video
{
    cb_video_frame_callback callback = nullptr;
    void *user = nullptr;
    std::unique_ptr<CbVideoSession> session;
};

static void copy_error(char *out, size_t capacity, const std::string &error)
{
    if (!out || capacity == 0)
        return;
    const size_t count = error.size() < capacity - 1 ? error.size() : capacity - 1;
    std::memcpy(out, error.data(), count);
    out[count] = '\0';
}

int cb_video_open(const cb_video_config *config, cb_video_t **out_uvc)
{
    if (!out_uvc)
        return -EINVAL;
    *out_uvc = nullptr;
    try
    {
        auto owner = std::make_unique<cb_video>();
        auto *uvc = owner.get();
        if (config)
        {
            uvc->callback = config->callback;
            uvc->user = config->user;
        }
        uvc->session = std::make_unique<CbVideoSession>(
            [uvc](CbVideoSession::Frame &&source)
            {
                if (!uvc->callback)
                    return;
                cb_video_frame frame{};
                frame.jpeg = source.jpeg.data();
                frame.jpeg_size = source.jpeg.size();
                frame.sequence = source.sequence;
                frame.timestamp_monotonic_ns = source.timestamp_ns;
                frame.width = source.width;
                frame.height = source.height;
                frame.flags = source.flags;
                frame.exposure_us[0] = source.exposure_us[0];
                frame.exposure_us[1] = source.exposure_us[1];
                uvc->callback(&frame, uvc->user);
            });
        *out_uvc = owner.release();
        return 0;
    }
    catch (const std::bad_alloc &)
    {
        return -ENOMEM;
    }
    catch (...)
    {
        return -EIO;
    }
}

int cb_video_start(cb_video_t *uvc, char *error, size_t error_capacity)
{
    if (!uvc || !uvc->session)
        return -EINVAL;
    try
    {
        std::string message;
        if (uvc->session->start(&message))
        {
            if (error && error_capacity)
                error[0] = '\0';
            return 0;
        }
        copy_error(error, error_capacity, message);
        return -EIO;
    }
    catch (const std::bad_alloc &)
    {
        return -ENOMEM;
    }
    catch (const std::exception &e)
    {
        copy_error(error, error_capacity, e.what());
        return -EIO;
    }
    catch (...)
    {
        return -EIO;
    }
}

void cb_video_stop(cb_video_t *uvc)
{
    if (uvc && uvc->session)
        uvc->session->stop();
}

void cb_video_close(cb_video_t *uvc)
{
    if (!uvc)
        return;
    if (uvc->session)
        uvc->session->stop();
    delete uvc;
}

int cb_video_running(const cb_video_t *uvc)
{
    return uvc && uvc->session && uvc->session->running() ? 1 : 0;
}
