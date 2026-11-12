/* SPDX-License-Identifier: Apache-2.0 */
#include "camera_broker_protocol.h"
#include "camera_client.h"
#include "cb_native_video.hpp"

#include <atomic>
#include <cerrno>
#include <mutex>
#include <thread>

struct CbVideoSession::Impl
{
    FrameCallback callback;
    std::mutex mutex;
    camera_client_t *client = nullptr;
    std::thread worker;
    std::atomic<bool> stopping{false}, streaming{false};

    explicit Impl(FrameCallback value) : callback(std::move(value)) {}
    void stop()
    {
        stopping = true;
        if (worker.joinable())
            worker.join();
        if (client)
            camera_client_close(client);
        client = nullptr;
        streaming = false;
    }
};

CbVideoSession::CbVideoSession(FrameCallback callback) : impl_(new Impl(std::move(callback))) {}
CbVideoSession::~CbVideoSession()
{
    impl_->stop();
    delete impl_;
}
bool CbVideoSession::start(std::string *error)
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->streaming)
        return true;
    impl_->stop();
    impl_->client = camera_client_open(CAMERA_BROKER_SOCKET, CAMERA_BROKER_ROLE_SDK_CLIENT,
                                       CAMERA_BROKER_STREAM_STEREO_JPEG);
    if (!impl_->client)
    {
        if (error)
            *error = "Cannot subscribe to the local camera JPEG stream";
        return false;
    }
    impl_->stopping = false;
    impl_->streaming = true;
    impl_->worker = std::thread(
        [this]
        {
            while (!impl_->stopping)
            {
                if (camera_client_wait(impl_->client, 100) < 0)
                    break;
                camera_client_frame input{};
                const int result = camera_client_read_latest_jpeg(impl_->client, &input);
                if (result == -EAGAIN)
                    continue;
                if (result)
                    break;
                Frame frame;
                frame.jpeg.assign(input.data, input.data + input.bytes);
                frame.sequence = input.sequence;
                frame.timestamp_ns = input.timestamp_monotonic_ns;
                frame.width = input.width;
                frame.height = input.height;
                frame.flags = frame.timestamp_ns ? 1 : 0;
                cb_parse_jpeg_metadata(frame.jpeg.data(), frame.jpeg.size(), &frame);
                if (impl_->callback)
                    impl_->callback(std::move(frame));
            }
            impl_->streaming = false;
        });
    return true;
}
void CbVideoSession::stop()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stop();
}
bool CbVideoSession::running() const
{
    return impl_->streaming;
}
