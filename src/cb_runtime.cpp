/* SPDX-License-Identifier: Apache-2.0 */
#include "cb_internal.h"
#include "cb_protocol.h"

#include "cb_disparity_assembler.hpp"
#include "cb_slam.hpp"
#include "cb_transport.hpp"
#include <cstdlib>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

struct cb
{
    void *impl;
};

namespace
{

using Clock = std::chrono::steady_clock;

constexpr size_t kControlQueueLimit = 128;
constexpr size_t kDebugTxLimit = 64 * 1024;
constexpr size_t kDebugRxLimit = 256 * 1024;

template <class F> int api_result(F &&operation)
{
    try
    {
        return operation();
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

uint16_t get16(const uint8_t *p)
{
    return (uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8);
}

uint32_t get32(const uint8_t *p)
{
    uint32_t value = 0;
    for (unsigned int i = 0; i < 4; ++i)
        value |= (uint32_t)p[i] << (8u * i);
    return value;
}

void put16(std::vector<uint8_t> &out, uint16_t value)
{
    out.push_back((uint8_t)(value & 0xffu));
    out.push_back((uint8_t)((value >> 8) & 0xffu));
}

void put32(std::vector<uint8_t> &out, uint32_t value)
{
    for (unsigned int i = 0; i < 4; ++i)
        out.push_back((uint8_t)((value >> (8u * i)) & 0xffu));
}

void put64(std::vector<uint8_t> &out, uint64_t value)
{
    for (unsigned int i = 0; i < 8; ++i)
        out.push_back((uint8_t)((value >> (8u * i)) & 0xffu));
}

uint64_t monotonic_ns()
{
    const auto now = Clock::now().time_since_epoch();
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

void copy_string(const std::string &value, char *out, size_t capacity)
{
    if (!out || capacity == 0)
        return;
    const size_t count = std::min(value.size(), capacity - 1);
    std::memcpy(out, value.data(), count);
    out[count] = '\0';
}

class SdkImpl
{
  public:
    explicit SdkImpl(const cb_config &config)
        : callback_(config.callback), user_(config.user),
          control_baud_(config.control_baudrate ? config.control_baudrate : 115200)
    {
        if (config.control_port)
            control_requested_port_ = config.control_port;
    }

    ~SdkImpl()
    {
        close();
    }

    int open()
    {
        stopping_.store(false);
        slam_ = std::make_unique<CbSlam>([this](uint16_t type, const uint8_t *p, size_t n)
                                         { return send(type, p, n, next_request_id()); }, [this]
                                         { return connected_.load() && slam_supported_.load(); });
        disparity_thread_ = std::thread(&SdkImpl::disparity_main, this);
        control_thread_ = std::thread(&SdkImpl::control_thread_main, this);
        return 0;
    }

    void close()
    {
        if (slam_)
            slam_->stop();
        if (stopping_.exchange(true))
            return;
        control_cv_.notify_all();
        disparity_cv_.notify_all();
        debug_cv_.notify_all();
        debug_rx_cv_.notify_all();
        if (control_thread_.joinable())
            control_thread_.join();
        if (disparity_thread_.joinable())
            disparity_thread_.join();
        close_debug();
        {
            std::lock_guard<std::mutex> lock(control_tx_mutex_);
            control_tx_.clear();
        }
    }

    int get_disparity(uint64_t after, cb_disparity *info, uint16_t *d, size_t dc, uint16_t *rgb,
                      size_t cc)
    {
        if (!info)
            return -EINVAL;
        std::shared_ptr<std::vector<uint8_t>> frame;
        {
            std::lock_guard<std::mutex> lock(disparity_mutex_);
            frame = latest_disparity_;
        }
        if (!frame)
            return -EAGAIN;
        const uint8_t *disp, *color;
        int result = cb_decode_disparity(frame->data(), frame->size(), info, &disp, &color);
        if (result)
            return result;
        if (info->frame_id <= after)
            return -EAGAIN;
        const size_t pixels = size_t(info->width) * info->height;
        if (!d || !rgb || dc < pixels || cc < pixels)
            return -ENOSPC;
        for (size_t i = 0; i < pixels; ++i)
        {
            d[i] = get16(disp + i * 2);
            rgb[i] = get16(color + i * 2);
        }
        return 0;
    }
    int connected() const
    {
        return connected_.load() ? 1 : 0;
    }
    CbSlam *slam()
    {
        if (slam_)
            std::call_once(slam_start_, [this] { slam_->start(); });
        return slam_.get();
    }

    int refresh_connection()
    {
        const int hello = enqueue_frame(CB_HELLO_REQ, nullptr, 0, next_request_id());
        const int caps = enqueue_frame(CB_GET_CAPS_REQ, nullptr, 0, next_request_id());
        return hello != 0 ? hello : caps;
    }

    uint64_t next_request_id()
    {
        return request_id_.fetch_add(1);
    }

    int send(uint16_t type, const uint8_t *payload, size_t payload_size, uint64_t request_id)
    {
        if (type == CB_CALIBRATION_COMMAND && (!connected_ || !calibration_supported_))
            return !connected_ ? -ENOTCONN : -ENOTSUP;
#ifdef CB_SDK_LOCAL
        if (type == CB_TERMINAL_SIZE_SET_REQ)
        {
            if (payload_size != 4)
                return -EINVAL;
            terminal_size_ = uint32_t(get16(payload)) | (uint32_t(get16(payload + 2)) << 16);
        }
#endif
        if (!connected() && type != CB_HELLO_REQ && type != CB_GET_CAPS_REQ)
            return -ENOTCONN;
        return enqueue_frame(type, payload, payload_size, request_id);
    }

    int debug_open(const char *port, uint32_t baud)
    {
        {
            std::lock_guard<std::mutex> lock(debug_config_mutex_);
            debug_requested_port_ = (port && *port) ? port : std::string();
            debug_baud_ = baud ? baud : 115200;
            debug_requested_ = true;
            debug_stopping_ = false;
        }
        if (!debug_thread_.joinable())
            debug_thread_ = std::thread(&SdkImpl::debug_thread_main, this);
        debug_cv_.notify_all();
        return 0;
    }

    void close_debug()
    {
        {
            std::lock_guard<std::mutex> lock(debug_config_mutex_);
            debug_stopping_ = true;
            debug_requested_ = false;
        }
        debug_cv_.notify_all();
        if (debug_thread_.joinable())
            debug_thread_.join();
        debug_rx_cv_.notify_all();
        {
            std::lock_guard<std::mutex> lock(debug_rx_mutex_);
            debug_rx_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(debug_tx_mutex_);
            debug_tx_.clear();
            debug_tx_bytes_ = 0;
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        debug_port_.clear();
    }

    int debug_write(const uint8_t *data, size_t size)
    {
        if (!data && size != 0)
            return -EINVAL;
        if (size == 0)
            return 0;
        if (!debug_requested_.load())
            return -ENODEV;
        std::lock_guard<std::mutex> lock(debug_tx_mutex_);
        if (size > kDebugTxLimit - std::min(kDebugTxLimit, debug_tx_bytes_))
            return -EAGAIN;
        debug_tx_.insert(debug_tx_.end(), data, data + size);
        debug_tx_bytes_ += size;
        debug_cv_.notify_all();
        return 0;
    }

    int debug_read(uint8_t *data, size_t size, int timeout_ms)
    {
        if (!data && size != 0)
            return -EINVAL;
        if (size == 0)
            return 0;
        std::unique_lock<std::mutex> lock(debug_rx_mutex_);
        const auto ready = [this] { return !debug_rx_.empty() || debug_stopping_ || stopping_; };
        if (timeout_ms < 0)
            debug_rx_cv_.wait(lock, ready);
        else if (timeout_ms > 0)
            debug_rx_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), ready);
        const size_t count = std::min(size, debug_rx_.size());
        for (size_t i = 0; i < count; ++i)
        {
            data[i] = debug_rx_.front();
            debug_rx_.pop_front();
        }
        return (int)count;
    }

    int copy_metrics(char *out, size_t capacity) const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (metrics_.empty())
            return -ENOENT;
        if (!out || capacity == 0)
            return -EINVAL;
        if (metrics_.size() + 1 > capacity)
            return -ENOSPC;
        std::memcpy(out, metrics_.c_str(), metrics_.size() + 1);
        return (int)metrics_.size();
    }

    int copy_error(char *out, size_t capacity) const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!out || capacity == 0)
            return -EINVAL;
        copy_string(error_, out, capacity);
        return (int)std::strlen(out);
    }

    int copy_control_port(char *out, size_t capacity) const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!out || capacity == 0)
            return -EINVAL;
        if (control_port_.empty())
            return -ENOENT;
        if (control_port_.size() + 1 > capacity)
            return -ENOSPC;
        std::memcpy(out, control_port_.c_str(), control_port_.size() + 1);
        return (int)control_port_.size();
    }

    int copy_debug_port(char *out, size_t capacity) const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!out || capacity == 0)
            return -EINVAL;
        if (debug_port_.empty())
            return -ENOENT;
        if (debug_port_.size() + 1 > capacity)
            return -ENOSPC;
        std::memcpy(out, debug_port_.c_str(), debug_port_.size() + 1);
        return (int)debug_port_.size();
    }

    int copy_device_name(char *out, size_t capacity) const
    {
        if (!out || capacity == 0)
            return -EINVAL;
        out[0] = '\0';
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (device_name_.empty())
            return -ENOENT;
        if (device_name_.size() + 1 > capacity)
            return -ENOSPC;
        std::memcpy(out, device_name_.c_str(), device_name_.size() + 1);
        return (int)device_name_.size();
    }

    int calibration_status(char *out, size_t capacity)
    {
        if (!out || !capacity)
            return -EINVAL;
        std::lock_guard<std::mutex> lock(calibration_mutex_);
        if (capacity <= calibration_status_.size())
            return -ENOSPC;
        std::memcpy(out, calibration_status_.c_str(), calibration_status_.size() + 1);
        return 0;
    }
    int calibration_preview(uint64_t after, cb_calibration_preview *info, uint8_t *jpeg,
                            size_t *capacity)
    {
        if (!info || !capacity)
            return -EINVAL;
        std::shared_ptr<std::vector<uint8_t>> frame;
        {
            std::lock_guard<std::mutex> lock(disparity_mutex_);
            frame = latest_calibration_preview_;
        }
        if (!frame || get64(frame->data() + 16) <= after)
            return -EAGAIN;
        info->width = get32(frame->data() + 4);
        info->height = get32(frame->data() + 8);
        info->camera_count = get32(frame->data() + 12);
        info->sequence = get64(frame->data() + 16);
        info->timestamp_monotonic_ns = get64(frame->data() + 24);
        size_t size = frame->size() - 32, available = *capacity;
        *capacity = size;
        if (!jpeg || available < size)
            return -ENOSPC;
        std::memcpy(jpeg, frame->data() + 32, size);
        return 0;
    }

  private:
    std::mutex calibration_mutex_;
    std::string calibration_status_ = "{}";
    std::shared_ptr<std::vector<uint8_t>> latest_calibration_preview_;
    std::vector<uint8_t> calibration_bytes_;
    uint64_t calibration_frame_id_ = 0;
    uint32_t calibration_total_ = 0;
    Clock::time_point calibration_start_{};
    std::vector<uint8_t> make_frame(uint16_t type, const uint8_t *payload, size_t payload_size,
                                    uint64_t request_id)
    {
        std::vector<uint8_t> frame;
        frame.reserve(CB_PROTOCOL_HEADER_SIZE + payload_size);
        put16(frame, CB_PROTOCOL_MAGIC);
        frame.push_back(CB_PROTOCOL_MAJOR);
        frame.push_back(CB_PROTOCOL_MINOR);
        put16(frame, CB_PROTOCOL_HEADER_SIZE);
        put16(frame, type);
        put32(frame, 0);
        put32(frame, (uint32_t)payload_size);
        put64(frame, request_id);
        put64(frame, sequence_.fetch_add(1));
        put64(frame, monotonic_ns());
        if (payload_size != 0 && payload != nullptr)
            frame.insert(frame.end(), payload, payload + payload_size);
        return frame;
    }

    int enqueue_frame(uint16_t type, const uint8_t *payload, size_t payload_size,
                      uint64_t request_id)
    {
        if (payload_size > CB_PROTOCOL_MAX_PAYLOAD)
            return -E2BIG;
        auto frame = make_frame(type, payload, payload_size, request_id);
        {
            std::lock_guard<std::mutex> lock(control_tx_mutex_);
#ifndef CB_SDK_LOCAL
            if (type == CB_IMU_STREAM_SET_REQ && payload_size && payload && payload[0] &&
                slam_ && slam_->imu_blocked())
                return -EBUSY;
#endif
            if (control_tx_.size() >= kControlQueueLimit)
                return -EAGAIN;
            control_tx_.push_back(std::move(frame));
        }
        control_cv_.notify_all();
        return 0;
    }

    void set_error(const std::string &error)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            error_ = error;
        }
        emit(CB_EVENT_ERROR, 0, CB_RESULT_IO, nullptr, 0, error);
    }

    void emit(enum cb_event_type type, uint64_t request_id, int32_t result, const uint8_t *payload,
              size_t payload_size, const std::string &text = {})
    {
        if (!callback_)
            return;
        cb_event event{};
        event.type = type;
        event.request_id = request_id;
        event.result = result;
        event.payload = payload;
        event.payload_size = payload_size;
        event.text = text.empty() ? nullptr : text.c_str();
        callback_(&event, user_);
    }

    static uint64_t get64(const uint8_t *p)
    {
        uint64_t value = 0;
        for (unsigned int i = 0; i < 8; ++i)
            value |= (uint64_t)p[i] << (8u * i);
        return value;
    }

    void parse_control()
    {
        for (;;)
        {
            if (rx_.size() < CB_PROTOCOL_HEADER_SIZE)
                return;
            if (get16(rx_.data()) != CB_PROTOCOL_MAGIC || rx_[2] != CB_PROTOCOL_MAJOR ||
                get16(rx_.data() + 4) < CB_PROTOCOL_HEADER_SIZE || get16(rx_.data() + 4) > 256 ||
                get32(rx_.data() + 12) > CB_PROTOCOL_MAX_PAYLOAD)
            {
                rx_.erase(rx_.begin());
                continue;
            }
            const size_t header_size = get16(rx_.data() + 4);
            const size_t payload_size = get32(rx_.data() + 12);
            const size_t frame_size = header_size + payload_size;
            if (rx_.size() < frame_size)
                return;
            const uint16_t type = get16(rx_.data() + 6);
            const uint64_t request_id = get64(rx_.data() + 16);
            const uint8_t *payload = rx_.data() + header_size;
            handle_control(type, request_id, payload, payload_size);
            rx_.erase(rx_.begin(), rx_.begin() + frame_size);
        }
    }

    void disparity_main()
    {
        uint64_t last = 0, calibration_last = 0;
        while (!stopping_)
        {
            std::shared_ptr<std::vector<uint8_t>> frame, calibration_frame;
            {
                std::unique_lock<std::mutex> lock(disparity_mutex_);
                disparity_cv_.wait(lock,
                                   [&]
                                   {
                                       return stopping_ ||
                                              (latest_disparity_ &&
                                               get64(latest_disparity_->data() + 8) != last) ||
                                              (latest_calibration_preview_ &&
                                               get64(latest_calibration_preview_->data() + 16) !=
                                                   calibration_last);
                                   });
                if (stopping_)
                    break;
                if (latest_disparity_ && get64(latest_disparity_->data() + 8) != last)
                {
                    frame = latest_disparity_;
                    last = get64(frame->data() + 8);
                }
                if (latest_calibration_preview_ &&
                    get64(latest_calibration_preview_->data() + 16) != calibration_last)
                {
                    calibration_frame = latest_calibration_preview_;
                    calibration_last = get64(calibration_frame->data() + 16);
                }
            }
            if (frame)
                emit(CB_EVENT_DISPARITY, 0, 0, frame->data(), frame->size());
            if (calibration_frame)
                emit(CB_EVENT_CALIBRATION_PREVIEW, 0, 0, calibration_frame->data(),
                     calibration_frame->size());
        }
    }
    void calibration_chunk(const uint8_t *p, size_t n)
    {
        if (n <= 24 || n > 4120)
        {
            calibration_bytes_.clear();
            return;
        }
        uint64_t id = get64(p);
        uint32_t total = get32(p + 12), offset = get32(p + 16);
        if (!id || total < 36 || offset > total || n - 24 > total - offset ||
            CbDisparityAssembler::crc32(p + 24, n - 24) != get32(p + 20))
        {
            calibration_bytes_.clear();
            return;
        }
        if (offset == 0)
        {
            calibration_bytes_.clear();
            calibration_frame_id_ = id;
            calibration_total_ = total;
            calibration_start_ = Clock::now();
        }
        if (id != calibration_frame_id_ || total != calibration_total_ ||
            offset != calibration_bytes_.size() ||
            Clock::now() - calibration_start_ > std::chrono::milliseconds(500))
        {
            calibration_bytes_.clear();
            return;
        }
        calibration_bytes_.insert(calibration_bytes_.end(), p + 24, p + n);
        if (calibration_bytes_.size() == total)
        {
            auto &v = calibration_bytes_;
            if (get32(v.data()) == 1 && get64(v.data() + 16) == id && get32(v.data() + 4) > 0 &&
                get32(v.data() + 4) <= 1280 && get32(v.data() + 8) > 0 &&
                get32(v.data() + 8) <= 1280 && get32(v.data() + 12) == 2 && v[32] == 0xff &&
                v[33] == 0xd8)
            {
                std::lock_guard<std::mutex> lock(disparity_mutex_);
                latest_calibration_preview_ = std::make_shared<std::vector<uint8_t>>(std::move(v));
                disparity_cv_.notify_one();
            }
            calibration_bytes_.clear();
        }
    }
    void handle_control(uint16_t type, uint64_t request_id, const uint8_t *payload, size_t size)
    {
        if (type == CB_CALIBRATION_STATUS)
        {
            std::string text(reinterpret_cast<const char *>(payload), size);
            if (text.find("\"heartbeat_ns\":") != std::string::npos)
            {
                std::lock_guard<std::mutex> lock(calibration_mutex_);
                calibration_status_ = text;
            }
            emit(CB_EVENT_CALIBRATION_STATUS, request_id, 0, payload, size, text);
            return;
        }
        if (type == CB_CALIBRATION_PREVIEW_CHUNK)
        {
            calibration_chunk(payload, size);
            return;
        }
        if (type == CB_SLAM_PACKET)
        {
            if (slam_)
                slam_->receive(payload, size);
            return;
        }
        if (type == CB_SLAM_LINK_STATUS)
        {
            if (slam_)
                slam_->link_status(payload, size);
            return;
        }
        if (type == CB_DISPARITY_CHUNK)
        {
            auto frame = disparity_assembler_.accept(payload, size);
            if (frame)
            {
                cb_disparity info{};
                if (cb_decode_disparity(frame->data(), frame->size(), &info, nullptr, nullptr) == 0)
                {
                    {
                        std::lock_guard<std::mutex> lock(disparity_mutex_);
                        latest_disparity_ = std::move(frame);
                    }
                    disparity_cv_.notify_one();
                }
            }
            return;
        }
        if (type == CB_STEREO_STATUS_EVT)
        {
            cb_stereo_status status{};
            const int r = cb_decode_stereo_status(payload, size, &status);
            if (!r && !(status.flags & CB_STEREO_ENABLED))
            {
                disparity_assembler_.reset();
                std::lock_guard<std::mutex> lock(disparity_mutex_);
                latest_disparity_.reset();
            }
            emit(CB_EVENT_STEREO_STATUS, request_id, r, payload, size,
                 r ? "Invalid stereo status" : status.message);
            return;
        }
        if (type == CB_HELLO_RSP)
        {
            slam_supported_ = size >= 8 && (get32(payload + 4) & CB_CAP_SLAM);
            calibration_supported_ = size >= 8 && (get32(payload + 4) & CB_CAP_CALIBRATION);
            if (!connected_.exchange(true))
            {
                emit(CB_EVENT_CONNECTED, request_id, 0, payload, size);
                (void)enqueue_frame(CB_STEREO_STATUS_REQ, nullptr, 0, next_request_id());
                (void)enqueue_frame(CB_DEVICE_CONFIG_REQ, nullptr, 0, next_request_id());
            }
            return;
        }
        if (type == CB_DEVICE_CONFIG_EVT)
        {
            cb_device_config config{};
            const int result = cb_decode_device_config(payload, size, &config);
            emit(CB_EVENT_DEVICE_CONFIG, request_id, result, payload, size);
            return;
        }
        if (type == CB_CONTROL_RESULT_EVT)
        {
            int32_t result = CB_RESULT_INVALID;
            if (size >= 12)
                result = (int32_t)get32(payload + 8);
            std::string text;
            if (size > 12)
                text.assign((const char *)payload + 12, size - 12);
            emit(CB_EVENT_RESULT, request_id, result, size > 12 ? payload + 12 : nullptr,
                 size > 12 ? size - 12 : 0, text);
            return;
        }
        if (type == CB_SYSTEM_METRICS_EVT || type == CB_STREAM_STATUS_EVT)
        {
            const std::string text((const char *)payload, size);
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (type == CB_SYSTEM_METRICS_EVT)
                    metrics_ = text;
                else
                    stream_status_ = text;
            }
            emit(type == CB_SYSTEM_METRICS_EVT ? CB_EVENT_METRICS : CB_EVENT_STREAM_STATUS,
                 request_id, 0, payload, size, text);
            return;
        }
        if (type == CB_SENSOR_STATUS)
        {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                stream_status_.assign((const char *)payload, size);
            }
            emit(CB_EVENT_SENSOR_STATUS, request_id, 0, payload, size,
                 std::string((const char *)payload, size));
            return;
        }
        if (type == CB_IMU_STREAM_STATUS_EVT)
        {
            cb_event event{};
            event.type = CB_EVENT_IMU_STATUS;
            event.request_id = request_id;
            event.enabled = size > 0 && payload[0] != 0;
            event.payload = payload;
            event.payload_size = size;
            if (callback_)
                callback_(&event, user_);
            return;
        }
        if (type == CB_IMU_BATCH || type == CB_FRAME_META)
        {
            emit(type == CB_IMU_BATCH ? CB_EVENT_IMU_BATCH : CB_EVENT_FRAME_META, request_id, 0,
                 payload, size);
        }
    }

    void mark_control_disconnected()
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            device_name_.clear();
        }
        slam_supported_ = false;
        calibration_supported_ = false;
        calibration_bytes_.clear();
        calibration_frame_id_ = 0;
        calibration_total_ = 0;
        {
            std::lock_guard<std::mutex> lock(calibration_mutex_);
            calibration_status_ = "{}";
        }
        if (slam_)
            slam_->disconnected();
        disparity_assembler_.reset();
        {
            std::lock_guard<std::mutex> lock(disparity_mutex_);
            latest_disparity_.reset();
            latest_calibration_preview_.reset();
        }
        if (connected_.exchange(false))
            emit(CB_EVENT_DISCONNECTED, 0, CB_RESULT_IO, nullptr, 0);
    }

    bool open_control_port()
    {
        std::string selected;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            selected = control_requested_port_.empty() ? std::string() : control_requested_port_;
        }
        std::string error;
        auto candidate = CbTransport::open(selected, true, &error);
        if (!candidate)
        {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                control_port_.clear();
                device_name_.clear();
            }
            if (!error.empty())
                set_error(error);
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            control_port_ = candidate->name();
            device_name_ = candidate->device_name();
        }
        try
        {
            control_ = std::move(candidate);
            (void)enqueue_frame(CB_HELLO_REQ, nullptr, 0, next_request_id());
            (void)enqueue_frame(CB_GET_CAPS_REQ, nullptr, 0, next_request_id());
            return true;
        }
        catch (const std::exception &e)
        {
            set_error(e.what());
            return false;
        }
    }

    void control_thread_main()
    {
        std::vector<uint8_t> chunk(64 * 1024);
        unsigned int reconnect_ms = 250;
        auto next_handshake = Clock::now();
        auto next_heartbeat = Clock::now();
        while (!stopping_.load())
        {
            if (!control_)
            {
                if (!open_control_port())
                {
                    std::unique_lock<std::mutex> lock(control_wait_mutex_);
                    control_cv_.wait_for(lock, std::chrono::milliseconds(reconnect_ms),
                                         [this] { return stopping_.load(); });
                    reconnect_ms = std::min(5000u, reconnect_ms * 2u);
                    continue;
                }
                reconnect_ms = 250;
                next_handshake = Clock::now();
            }

            try
            {
                bool did_work = false;
                if (connected_ && Clock::now() >= next_heartbeat)
                {
                    (void)enqueue_frame(CB_HEARTBEAT, nullptr, 0, next_request_id());
                    next_heartbeat = Clock::now() + std::chrono::seconds(2);
                }
                /* A USB gadget can enumerate before the board-side protocol
                 * worker opens ttyGS1.  Keep retrying the handshake on the
                 * already-open bulk interface instead of leaving the UI in waiting
                 * forever after the first HELLO was lost. */
                if (!connected_.load() && Clock::now() >= next_handshake)
                {
                    (void)enqueue_frame(CB_HELLO_REQ, nullptr, 0, next_request_id());
                    (void)enqueue_frame(CB_GET_CAPS_REQ, nullptr, 0, next_request_id());
                    next_handshake = Clock::now() + std::chrono::milliseconds(500);
                    did_work = true;
                }
                for (;;)
                {
                    std::vector<uint8_t> frame;
                    {
                        std::lock_guard<std::mutex> lock(control_tx_mutex_);
                        if (control_tx_.empty())
                            break;
                        frame = std::move(control_tx_.front());
                        control_tx_.pop_front();
                    }
                    const size_t written = control_->write(frame);
                    if (written != frame.size())
                        throw std::runtime_error("short control write");
                    did_work = true;
                }
                /* The libusb bulk read has a short timeout, so this remains a
                   bounded, non-blocking poll while still submitting a real
                   bulk-IN transfer to the gadget. */
                const size_t count = control_->read(chunk.data(), chunk.size());
                if (count != 0)
                {
                    rx_.insert(rx_.end(), chunk.begin(), chunk.begin() + count);
                    parse_control();
                    did_work = true;
                }
                if (!did_work)
                {
                    std::unique_lock<std::mutex> lock(control_wait_mutex_);
                    control_cv_.wait_for(lock, std::chrono::milliseconds(2),
                                         [this] { return stopping_.load(); });
                }
            }
            catch (const std::exception &e)
            {
                set_error(e.what());
                try
                {
                    control_->close();
                }
                catch (...)
                {
                }
                control_.reset();
                rx_.clear();
                mark_control_disconnected();
                std::unique_lock<std::mutex> lock(control_wait_mutex_);
                control_cv_.wait_for(lock, std::chrono::milliseconds(reconnect_ms),
                                     [this] { return stopping_.load(); });
                reconnect_ms = std::min(5000u, reconnect_ms * 2u);
            }
        }
        if (control_)
        {
            if (connected_)
            {
                try
                {
                    const uint8_t off = 0;
                    for (const auto type :
                         {CB_IMU_STREAM_SET_REQ, CB_STEREO_ENABLE_REQ, CB_SLAM_LINK})
                        control_->write(make_frame(type, &off, 1, next_request_id()));
                }
                catch (...)
                {
                }
            }
            try
            {
                control_->close();
            }
            catch (...)
            {
            }
            control_.reset();
        }
        mark_control_disconnected();
    }

    void append_debug_rx(const uint8_t *data, size_t size)
    {
        std::lock_guard<std::mutex> lock(debug_rx_mutex_);
        if (size > kDebugRxLimit)
        {
            data += size - kDebugRxLimit;
            size = kDebugRxLimit;
        }
        while (debug_rx_.size() + size > kDebugRxLimit)
            debug_rx_.pop_front();
        debug_rx_.insert(debug_rx_.end(), data, data + size);
        debug_rx_cv_.notify_all();
    }

    void debug_thread_main()
    {
        std::vector<uint8_t> chunk(4096);
        unsigned int reconnect_ms = 250;
        while (true)
        {
            bool stopping;
            bool requested;
            std::string requested_port;
            uint32_t baud;
            {
                std::lock_guard<std::mutex> lock(debug_config_mutex_);
                stopping = debug_stopping_;
                requested = debug_requested_;
                requested_port = debug_requested_port_;
                baud = debug_baud_;
            }
            (void)baud; /* bulk endpoints have no baud-rate setting */
            if (stopping || !requested)
                break;
            if (!debug_)
            {
                std::string error;
                auto candidate = CbTransport::open(requested_port, false, &error);
                if (!candidate)
                {
                    if (!error.empty())
                        set_error(error);
                    std::unique_lock<std::mutex> lock(debug_wait_mutex_);
                    debug_cv_.wait_for(lock, std::chrono::milliseconds(reconnect_ms),
                                       [this] { return debug_stopping_.load(); });
                    reconnect_ms = std::min(5000u, reconnect_ms * 2u);
                    continue;
                }
                debug_ = std::move(candidate);
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    debug_port_ = debug_->name();
                }
                reconnect_ms = 250;
            }
            const uint32_t dimensions = terminal_size_.load();
            if (dimensions)
                (void)debug_->resize_terminal(uint16_t(dimensions), uint16_t(dimensions >> 16));
            try
            {
                bool did_work = false;
                for (;;)
                {
                    std::vector<uint8_t> bytes;
                    {
                        std::lock_guard<std::mutex> lock(debug_tx_mutex_);
                        if (debug_tx_.empty())
                            break;
                        const size_t count = std::min<size_t>(4096, debug_tx_.size());
                        bytes.reserve(count);
                        for (size_t i = 0; i < count; ++i)
                        {
                            bytes.push_back(debug_tx_.front());
                            debug_tx_.pop_front();
                        }
                        debug_tx_bytes_ -= bytes.size();
                    }
                    const size_t written = debug_->write(bytes);
                    if (written != bytes.size())
                        throw std::runtime_error("short debug write");
                    did_work = true;
                }
                /* Debug input is interactive terminal traffic.  Do not let
                 * the USB backend hold it for the control channel's 100 ms
                 * timeout: a short poll keeps held keys responsive while the
                 * worker remains independent of the WebView refresh rate. */
                const size_t count = debug_->read(chunk.data(), chunk.size(), 1);
                if (count != 0)
                    append_debug_rx(chunk.data(), count);
                did_work = count != 0;
                if (!did_work)
                {
                    std::unique_lock<std::mutex> lock(debug_wait_mutex_);
                    debug_cv_.wait_for(lock, std::chrono::milliseconds(5),
                                       [this] { return debug_stopping_.load(); });
                }
            }
            catch (const std::exception &e)
            {
                set_error(e.what());
                try
                {
                    debug_->close();
                }
                catch (...)
                {
                }
                debug_.reset();
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    debug_port_.clear();
                }
                std::unique_lock<std::mutex> lock(debug_wait_mutex_);
                debug_cv_.wait_for(lock, std::chrono::milliseconds(reconnect_ms),
                                   [this] { return debug_stopping_.load(); });
                reconnect_ms = std::min(5000u, reconnect_ms * 2u);
            }
        }
        if (debug_)
        {
            try
            {
                debug_->close();
            }
            catch (...)
            {
            }
            debug_.reset();
        }
    }

    std::string control_port() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return control_port_;
    }

    cb_event_callback callback_ = nullptr;
    void *user_ = nullptr;
    uint32_t control_baud_ = 115200;
    std::string control_requested_port_;
    std::string control_port_;
    std::string debug_port_;
    std::string device_name_;
    std::atomic<uint32_t> terminal_size_{0};
    std::unique_ptr<CbTransport> control_;
    std::unique_ptr<CbTransport> debug_;

    mutable std::mutex state_mutex_;
    std::string metrics_;
    std::string stream_status_;
    std::string error_;

    std::atomic<bool> stopping_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> slam_supported_{false};
    std::atomic<bool> calibration_supported_{false};
    std::unique_ptr<CbSlam> slam_;
    std::once_flag slam_start_;
    std::atomic<uint64_t> request_id_{1};
    std::atomic<uint64_t> sequence_{1};

    CbDisparityAssembler disparity_assembler_;
    std::mutex disparity_mutex_;
    std::condition_variable disparity_cv_;
    std::shared_ptr<std::vector<uint8_t>> latest_disparity_;
    std::thread disparity_thread_;
    std::thread control_thread_;
    std::mutex control_tx_mutex_;
    std::deque<std::vector<uint8_t>> control_tx_;
    std::mutex control_wait_mutex_;
    std::condition_variable control_cv_;
    std::vector<uint8_t> rx_;

    std::thread debug_thread_;
    std::mutex debug_config_mutex_;
    std::string debug_requested_port_;
    uint32_t debug_baud_ = 115200;
    std::atomic<bool> debug_requested_{false};
    std::atomic<bool> debug_stopping_{true};
    std::mutex debug_tx_mutex_;
    std::deque<uint8_t> debug_tx_;
    size_t debug_tx_bytes_ = 0;
    std::mutex debug_rx_mutex_;
    std::condition_variable debug_rx_cv_;
    std::deque<uint8_t> debug_rx_;
    std::mutex debug_wait_mutex_;
    std::condition_variable debug_cv_;
};

SdkImpl *impl(cb_t *host)
{
    return host ? static_cast<SdkImpl *>(host->impl) : nullptr;
}

const SdkImpl *impl(const cb_t *host)
{
    return host ? static_cast<const SdkImpl *>(host->impl) : nullptr;
}

} // namespace

extern "C" int cb_internal_open(const cb_config *config, cb_t **out_host)
{
    if (!out_host)
        return -EINVAL;
    *out_host = nullptr;
    if (!config)
        return -EINVAL;
    try
    {
        auto host = std::make_unique<cb>();
        auto object = std::make_unique<SdkImpl>(*config);
        const int result = object->open();
        if (result)
            return result;
        host->impl = object.release();
        *out_host = host.release();
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

extern "C" void cb_internal_close(cb_t *host)
{
    if (!host)
        return;
    delete impl(host);
    delete host;
}

extern "C" int cb_internal_connected(const cb_t *host)
{
    const auto *object = impl(host);
    return object ? object->connected() : 0;
}

extern "C" int cb_internal_refresh_connection(cb_t *host)
{
    auto *object = impl(host);
    return api_result([&] { return object ? object->refresh_connection() : -EINVAL; });
}

extern "C" uint64_t cb_internal_next_request_id(cb_t *host)
{
    auto *object = impl(host);
    return object ? object->next_request_id() : 0;
}

extern "C" int cb_internal_request(cb_t *host, uint16_t type, const uint8_t *payload,
                                   size_t payload_size, uint64_t request_id)
{
    if (payload_size && !payload)
        return -EINVAL;
    auto *object = impl(host);
    try
    {
        return object ? object->send(type, payload, payload_size, request_id) : -EINVAL;
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

extern "C" int cb_internal_debug_open(cb_t *host, const char *port, uint32_t baudrate)
{
    auto *object = impl(host);
    return api_result([&] { return object ? object->debug_open(port, baudrate) : -EINVAL; });
}

extern "C" void cb_internal_debug_close(cb_t *host)
{
    if (auto *object = impl(host))
        object->close_debug();
}

extern "C" int cb_internal_debug_write(cb_t *host, const uint8_t *data, size_t size)
{
    auto *object = impl(host);
    return api_result([&] { return object ? object->debug_write(data, size) : -EINVAL; });
}

extern "C" int cb_internal_debug_read(cb_t *host, uint8_t *data, size_t size, int timeout_ms)
{
    auto *object = impl(host);
    return object ? object->debug_read(data, size, timeout_ms) : -EINVAL;
}

extern "C" int cb_internal_copy_control_port(const cb_t *host, char *out, size_t capacity)
{
    const auto *object = impl(host);
    return object ? object->copy_control_port(out, capacity) : -EINVAL;
}

extern "C" int cb_internal_copy_debug_port(const cb_t *host, char *out, size_t capacity)
{
    const auto *object = impl(host);
    return object ? object->copy_debug_port(out, capacity) : -EINVAL;
}

extern "C" int cb_internal_copy_device_name(const cb_t *host, char *out, size_t capacity)
{
    const auto *object = impl(host);
    return object ? object->copy_device_name(out, capacity) : -EINVAL;
}

extern "C" int cb_internal_copy_metrics(const cb_t *host, char *out, size_t capacity)
{
    const auto *object = impl(host);
    return object ? object->copy_metrics(out, capacity) : -EINVAL;
}

extern "C" int cb_internal_copy_error(const cb_t *host, char *out, size_t capacity)
{
    const auto *object = impl(host);
    return object ? object->copy_error(out, capacity) : -EINVAL;
}

extern "C" int cb_get_disparity(cb_t *host, uint64_t after, cb_disparity *info, uint16_t *d,
                                size_t dc, uint16_t *rgb, size_t cc)
{
    auto *p = impl(host);
    return p ? p->get_disparity(after, info, d, dc, rgb, cc) : -EINVAL;
}

namespace
{
template <class F> int slam_call(cb_t *host, F &&fn)
{
    auto *p = impl(host);
    if (!p)
        return -EINVAL;
    try
    {
        fn(*p->slam());
        return 0;
    }
    catch (const std::bad_alloc &)
    {
        return -ENOMEM;
    }
    catch (...)
    {
        return -EINVAL;
    }
}
template <class F> int slam_command(cb_t *host, F &&fn)
{
    auto *p = impl(host);
    if (!p)
        return -EINVAL;
    try
    {
        if (!p->slam()->command_ready())
            return -ENOTCONN;
    }
    catch (const std::bad_alloc &)
    {
        return -ENOMEM;
    }
    catch (...)
    {
        return -EINVAL;
    }
    return slam_call(host, std::forward<F>(fn));
}
int slam_json(cb_t *host, bool consume, char **out)
{
    if (!out)
        return -EINVAL;
    *out = nullptr;
    return slam_call(host,
                     [&](CbSlam &s)
                     {
                         const auto value =
                             (consume ? s.client().poll() : s.client().scene().snapshot()).dump();
                         auto *p = static_cast<char *>(std::malloc(value.size() + 1));
                         if (!p)
                             throw std::bad_alloc();
                         std::memcpy(p, value.c_str(), value.size() + 1);
                         *out = p;
                     });
}
} // namespace
extern "C" int cb_slam_start(cb_t *h, const cb_slam_options *o)
{
    if (!o || !o->map_name || !*o->map_name || o->map_mode > 2)
        return -EINVAL;
    return slam_command(h,
                        [&](CbSlam &s)
                        {
                            s.client().command(
                                "START", {{"map_name", o->map_name},
                                          {"map_mode", o->map_mode == 0   ? "new"
                                                       : o->map_mode == 1 ? "resume"
                                                                          : "localization"},
                                          {"dense_reconstruction", o->dense_reconstruction != 0}});
                        });
}
extern "C" int cb_slam_stop(cb_t *h)
{
    return slam_command(h, [](CbSlam &s) { s.client().command("STOP"); });
}
extern "C" int cb_slam_set_processing(cb_t *h, int backend_enabled, int dense_reconstruction)
{
    return slam_command(h, [&](CbSlam &s)
                        { s.client().command("SET_PROCESSING",
                                             {{"backend_enabled", backend_enabled != 0},
                                              {"dense_reconstruction", backend_enabled != 0 && dense_reconstruction != 0}}); });
}
extern "C" int cb_slam_refresh_maps(cb_t *h)
{
    return slam_command(h, [](CbSlam &s) { s.client().refresh_maps(); });
}
extern "C" int cb_slam_set_elevator(cb_t *h, int enabled)
{
    return slam_command(h, [&](CbSlam &s)
                        { s.client().command("SET_ELEVATOR", {{"enabled", enabled != 0}}); });
}
extern "C" int cb_slam_subscribe(cb_t *h, int preview, int map)
{
    return slam_command(h, [&](CbSlam &s) { s.client().subscribe(preview != 0, map != 0); });
}
extern "C" int cb_slam_imu_upload_blocked(cb_t *h)
{
    int blocked = 0;
    const int result = slam_call(h, [&](CbSlam &s) { blocked = int(s.imu_blocked()); });
    return result ? result : blocked;
}
extern "C" int cb_slam_get_pose(cb_t *h, cb_slam_pose *out)
{
    if (!out)
        return -EINVAL;
    int result = -EAGAIN;
    const int rc = slam_call(h,
                             [&](CbSlam &s)
                             {
                                 const auto data = s.client().scene().snapshot();
                                 const auto &p = data.at("pose");
                                 if (p.is_null() ||
                                     data.at("state").value("connection", std::string()) != "READY")
                                     return;
                                 cb_slam_pose value{};
                                 value.run_id = svio::json_u64(p, "run_id");
                                 value.timestamp_monotonic_ns = svio::json_u64(p, "timestamp_ns");
                                 value.frame_id = svio::json_u64(p, "frame_id");
                                 value.epoch = p.at("epoch").get<uint32_t>();
                                 for (unsigned i = 0; i < 3; ++i)
                                 {
                                     value.position[i] = p.at("p").at(i).get<float>();
                                     value.velocity[i] = p.at("v").at(i).get<float>();
                                 }
                                 for (unsigned i = 0; i < 4; ++i)
                                     value.quaternion_xyzw[i] = p.at("q").at(i).get<float>();
                                 *out = value;
                                 result = 0;
                             });
    return rc ? rc : result;
}
extern "C" int cb_slam_get_json(cb_t *h, char **out)
{
    return slam_json(h, false, out);
}
extern "C" int cb_slam_poll_json(cb_t *h, char **out)
{
    return slam_json(h, true, out);
}
extern "C" void cb_slam_free_json(char *p)
{
    std::free(p);
}
extern "C" int cb_slam_clear_view(cb_t *h)
{
    return slam_call(h, [](CbSlam &s) { s.client().scene().clear(); });
}

extern "C" int cb_get_calibration_status(cb_t *host, char *out, size_t capacity)
{
    auto *object = impl(host);
    return api_result([&] { return object ? object->calibration_status(out, capacity) : -EINVAL; });
}
extern "C" int cb_get_calibration_preview(cb_t *host, uint64_t after, cb_calibration_preview *info,
                                          uint8_t *jpeg, size_t *size)
{
    auto *object = impl(host);
    return api_result(
        [&] { return object ? object->calibration_preview(after, info, jpeg, size) : -EINVAL; });
}

extern "C" int cb_request_calibration_report(cb_t *host, enum cb_calibration_report_kind kind,
                                             uint32_t revision, uint32_t offset, uint64_t id)
{
    return api_result([&] {
        static const char *names[] = {"current", "joint", "imu"};
        if (kind < CB_CALIBRATION_REPORT_CURRENT || kind > CB_CALIBRATION_REPORT_IMU ||
            offset > 2 * 1024 * 1024)
            return -EINVAL;
        std::ostringstream json;
        json.imbue(std::locale::classic());
        json << "{\"action\":\"report\",\"kind\":\"" << names[kind]
             << "\",\"revision\":" << revision << ",\"offset\":" << offset << '}';
        const auto body = json.str();
        return cb_internal_request(host, CB_CALIBRATION_COMMAND,
                                   reinterpret_cast<const uint8_t *>(body.data()), body.size(), id);
    });
}

extern "C" int cb_calibration_command(cb_t *host, enum cb_calibration_action action,
                                      const cb_calibration_options *o, uint64_t id)
{
    return api_result(
        [&]
        {
            static const char *names[] = {"status", "enter",  "start_imu", "start_camera",
                                          "solve",  "cancel", "leave"};
            if (action < CB_CALIBRATION_STATUS_QUERY || action > CB_CALIBRATION_LEAVE)
                return -EINVAL;
            std::ostringstream json;
            json.imbue(std::locale::classic());
            json << std::setprecision(12);
            json << "{\"action\":\"" << names[action] << "\"";
            if (action == CB_CALIBRATION_START_CAMERA)
            {
                if (!o || !o->tag_rows || !o->tag_cols || o->tag_rows > 587 || o->tag_cols > 587 ||
                    o->tag_rows * o->tag_cols < 2 || o->tag_rows * o->tag_cols > 587 ||
                    !std::isfinite(o->tag_size_metres) || o->tag_size_metres <= 0 ||
                    o->tag_size_metres > 2 || !std::isfinite(o->tag_spacing) ||
                    o->tag_spacing < 0 || o->tag_spacing > 2)
                    return -EINVAL;
                json << ",\"rows\":" << o->tag_rows << ",\"cols\":" << o->tag_cols
                     << ",\"tag_size_m\":" << o->tag_size_metres
                     << ",\"spacing\":" << o->tag_spacing;
            }
            json << '}';
            const auto body = json.str();
            return cb_internal_request(host, CB_CALIBRATION_COMMAND,
                                       reinterpret_cast<const uint8_t *>(body.data()), body.size(),
                                       id);
        });
}
