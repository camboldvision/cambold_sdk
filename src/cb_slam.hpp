/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "cb_protocol.h"
#include "client/client.hpp"
#include "slam_envelope.hpp"
#include <cerrno>
#include <condition_variable>
#include <cstring>
#include <functional>

// Logical multiplexing over the SDK's existing USB handle. Map assembly and
// protocol processing run in Client's worker, never in the USB/IMU receiver.
class CbSlam
{
  public:
    using Send = std::function<int(std::uint16_t, const std::uint8_t *, std::size_t)>;
    CbSlam(Send send, std::function<bool()> ready)
        : send_(std::move(send)), ready_(std::move(ready)),
          client_([this](const svio::Frame &frame, const svio::Json &json)
                  { update_imu_policy(frame, json); })
    {
    }
    ~CbSlam()
    {
        stop();
    }
    void start()
    {
        client_.connect_transport([this] { return open(); }, "SDK bulk · CamBold SLAM");
    }
    void stop()
    {
        client_.stop();
    }
    void disconnected()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++generation_;
        inbox_.clear();
        bytes_ = 0;
        dense_running_ = start_pending_ = pending_processing_ = false;
        start_request_ = 0;
        start_operation_.clear();
        cv_.notify_all();
    }
    void receive(const std::uint8_t *data, std::size_t n)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_)
            return;
        if (n < 16 || n > cambold_ipc::slam_record_limit || bytes_ + n > 256 * 1024)
        {
            ++generation_;
            inbox_.clear();
            bytes_ = 0;
            active_ = false;
            cv_.notify_all();
            return;
        }
        inbox_.emplace_back(data, data + n);
        bytes_ += n;
        cv_.notify_one();
    }
    cambold::detail::Client &client()
    {
        return client_;
    }
    bool command_ready()
    {
        return ready_() &&
               client_.scene().snapshot().at("state").value("connection", std::string()) == "READY";
    }
    bool imu_blocked() const
    {
#ifdef CB_SDK_LOCAL
        return false;
#else
        std::lock_guard<std::mutex> lock(mutex_);
        return dense_running_ || start_pending_;
#endif
    }
    void link_status(const std::uint8_t *p, std::size_t n)
    {
        if (n >= 3)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!p[0] && active_)
            {
                active_ = false;
                ++generation_;
                inbox_.clear();
                bytes_ = 0;
                cv_.notify_all();
            }
        }
    }

  private:
    // The USB SDK owns its bandwidth policy. LOCAL clients never change their
    // IMU subscriptions when another application starts dense reconstruction.
    void before_request(const svio::Frame &frame)
    {
#ifndef CB_SDK_LOCAL
        if (frame.type != svio::Command || frame.flags != svio::json_flag)
            return;
        const auto json = svio::parse_json(frame);
        const auto command = json.value("command", std::string());
        const auto args = json.value("args", svio::Json::object());
        if ((command != "START" && command != "SET_PROCESSING") ||
            !args.value("backend_enabled", true) || !args.value("dense_reconstruction", true))
            return;
        // Setting idle defaults does not reserve USB bandwidth.
        if (command == "SET_PROCESSING")
        {
            const auto state = client_.scene().snapshot().at("state").value("state", std::string());
            if (state != "RUNNING" && state != "INITIALIZING" && state != "STARTING" &&
                state != "CONFIGURING_CAMERA")
                return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (start_pending_ && start_request_ == frame.request_id)
                return; // Retry of the same idempotent command.
            start_pending_ = true;
            pending_processing_ = command == "SET_PROCESSING";
            start_request_ = frame.request_id;
            start_operation_.clear();
        }
        disable_imu();
#endif
    }
    void disable_imu()
    {
#ifndef CB_SDK_LOCAL
        const std::uint8_t off = 0;
        if (send_(CB_IMU_STREAM_SET_REQ, &off, 1))
            throw std::runtime_error("Cannot disable USB IMU upload for dense reconstruction");
#endif
    }
    void update_imu_policy(const svio::Frame &frame, const svio::Json &json)
    {
#ifndef CB_SDK_LOCAL
        bool disable = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const bool was_blocked = dense_running_ || start_pending_;
            if (frame.type == svio::HelloReply || frame.type == svio::StatusReply ||
                frame.type == svio::StatusEvent || frame.type == svio::EpochEvent)
            {
                const auto state = json.value("state", std::string());
                const bool active = state == "CONFIGURING_CAMERA" || state == "STARTING" ||
                                    state == "INITIALIZING" || state == "RUNNING" || state == "STOPPING";
                dense_running_ = active && json.value("dense_reconstruction", false);
                if ((!pending_processing_ && active) || state == "ERROR" || state == "WAITING_CALIBRATION")
                    start_pending_ = false;
            }
            if (start_pending_ && frame.request_id == start_request_)
            {
                if (frame.type == svio::CommandReply)
                    start_operation_ = json.value("operation_id", std::string());
                else if (frame.type == svio::Error)
                    start_pending_ = false;
            }
            if (start_pending_ && (frame.type == svio::OperationEvent || frame.type == svio::OperationReply) &&
                (frame.request_id == start_request_ ||
                 (!start_operation_.empty() && json.value("operation_id", std::string()) == start_operation_)))
            {
                const auto status = json.value("status", std::string());
                if (status == "completed" || status == "failed")
                {
                    start_pending_ = false;
                    if (status == "completed")
                        dense_running_ = json.value("dense_reconstruction", !pending_processing_);
                }
            }
            disable = !was_blocked && (dense_running_ || start_pending_);
        }
        if (disable)
            disable_imu();
#endif
    }
    class Transport final : public svio::Transport
    {
        CbSlam &owner_;
        std::uint64_t generation_;
        svio::Bytes input_;
        std::size_t offset_ = 0;
        svio::Decoder decoder_;

      public:
        Transport(CbSlam &owner, std::uint64_t generation) : owner_(owner), generation_(generation)
        {
        }
        ~Transport() override
        {
            close();
        }
        std::size_t read(std::uint8_t *p, std::size_t n, int timeout) override
        {
            std::unique_lock<std::mutex> lock(owner_.mutex_);
            owner_.cv_.wait_for(lock, std::chrono::milliseconds(timeout),
                                [&]
                                {
                                    return generation_ != owner_.generation_ || !input_.empty() ||
                                           !owner_.inbox_.empty();
                                });
            if (generation_ != owner_.generation_)
                throw std::runtime_error("SDK SLAM link reset");
            if (input_.empty() && !owner_.inbox_.empty())
            {
                auto record = std::move(owner_.inbox_.front());
                owner_.inbox_.pop_front();
                owner_.bytes_ -= record.size();
                lock.unlock();
                auto r = cambold_ipc::unpack_slam(record.data(), record.size());
                svio::Frame f;
                f.type = r.type;
                f.flags = r.flags;
                f.request_id = r.request;
                f.timestamp_ns = r.timestamp;
                f.payload = std::move(r.payload);
                input_ = svio::encode_frame(f);
                offset_ = 0;
            }
            const auto count = std::min(n, input_.size() - offset_);
            if (count)
                std::memcpy(p, input_.data() + offset_, count);
            offset_ += count;
            if (offset_ == input_.size())
            {
                input_.clear();
                offset_ = 0;
            }
            return count;
        }
        std::size_t write(const std::uint8_t *p, std::size_t n, int) override
        {
            {
                std::lock_guard<std::mutex> lock(owner_.mutex_);
                if (generation_ != owner_.generation_)
                    throw std::runtime_error("SDK SLAM link reset");
            }
            for (auto &f : decoder_.feed(p, n))
            {
                owner_.before_request(f);
                auto record = cambold_ipc::pack_slam(
                    {f.type, f.flags, f.request_id, f.timestamp_ns, std::move(f.payload)});
                if (owner_.send_(cambold_ipc::slam_packet, record.data(), record.size()))
                    throw std::runtime_error("SDK SLAM request queue unavailable");
            }
            return n;
        }
        void close() override
        {
            if (!generation_)
                return;
            bool disable = false;
            {
                std::lock_guard<std::mutex> lock(owner_.mutex_);
                if (generation_ == owner_.generation_)
                {
                    owner_.active_ = false;
                    disable = true;
                    owner_.inbox_.clear();
                    owner_.bytes_ = 0;
                    owner_.dense_running_ = owner_.start_pending_ = owner_.pending_processing_ = false;
                    owner_.start_request_ = 0;
                    owner_.start_operation_.clear();
                }
            }
            generation_ = 0;
            if (disable)
            {
                const std::uint8_t off = 0;
                (void)owner_.send_(cambold_ipc::slam_link, &off, 1);
            }
        }
    };
    std::unique_ptr<svio::Transport> open()
    {
        if (!ready_())
            throw std::runtime_error("Waiting for SDK bulk connection / SLAM capability");
        std::uint64_t generation;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            generation = ++generation_;
            inbox_.clear();
            bytes_ = 0;
            active_ = true;
        }
        const std::uint8_t on = 1;
        if (send_(cambold_ipc::slam_link, &on, 1))
            throw std::runtime_error("Cannot enable SDK SLAM link");
        return std::make_unique<Transport>(*this, generation);
    }
    Send send_;
    std::function<bool()> ready_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<svio::Bytes> inbox_;
    std::size_t bytes_ = 0;
    std::uint64_t generation_ = 0;
    bool active_ = false, dense_running_ = false, start_pending_ = false;
    bool pending_processing_ = false;
    std::uint32_t start_request_ = 0;
    std::string start_operation_;
    cambold::detail::Client client_;
};
