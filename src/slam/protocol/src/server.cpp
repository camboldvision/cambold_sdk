/* SPDX-License-Identifier: Apache-2.0 */
#include "svio/server.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace svio
{
namespace
{
Frame binary_frame(std::uint16_t type, Bytes payload, std::uint64_t timestamp)
{
    Frame f;
    f.type = type;
    f.payload = std::move(payload);
    f.timestamp_ns = timestamp;
    return f;
}
double bounded_hz(const Json &j, double maximum)
{
    auto v = j.value("max_hz", maximum);
    if (!std::isfinite(v) || v <= 0 || v > 1000)
        throw std::runtime_error("Invalid max_hz");
    return std::min(v, maximum);
}
std::string boot_identity()
{
#ifdef __linux__
    std::ifstream file("/proc/sys/kernel/random/boot_id");
    std::string id;
    if (file >> id && id.size() == 36 &&
        id.find_first_not_of("0123456789abcdef-") == std::string::npos)
        return id;
#endif
    std::ostringstream out;
    out << std::hex << std::setfill('0') << std::setw(16) << random_id() << std::setw(16)
        << random_id();
    return out.str();
}
Frame dequeue_control(std::deque<Frame> &queue)
{
    auto frame = std::move(queue.front());
    queue.pop_front();
    // t3 is the dequeue/transmit timestamp, after control queue waiting.
    if (frame.type == TimeSyncReply && frame.payload.size() == 32)
    {
        auto now = monotonic_ns();
        for (unsigned i = 0; i < 8; ++i)
            frame.payload[24 + i] = std::uint8_t(now >> (8 * i));
    }
    return frame;
}
} // namespace
Server::Server(std::function<void(const CommandTask &)> callback)
    : command_(std::move(callback)), boot_(boot_identity())
{
    descriptors_["device"] = {
        {"product", "CamBold SLAM"}, {"serial", "cambold-slam"}, {"software", "CamBold SLAM 0.1"}};
    descriptors_["clock"] = {{"domain", "device_monotonic"},
                             {"unit", "ns"},
                             {"quaternion", "Hamilton_xyzw_imu_to_world"},
                             {"world", "local_gravity_aligned_z_up"}};
    descriptors_["frames"] = {{"world", "local"}, {"body", "imu"}};
    descriptors_["cameras"] = Json::array();
    descriptors_["config"] = Json::object();
    Json streams = Json::array();
    for (auto i : active_streams)
        streams.push_back({{"id", i},
                           {"name", stream_name(i)},
                           {"schema", 1},
                           {"max_hz", stream_max_hz(i)},
                           {"max_points", i == 9   ? dense_point_limit
                                          : i == 3 ? keyframe_limit
                                          : i == 5 ? 512
                                                   : 0}});
    descriptors_["streams"] = std::move(streams);
}
void Server::clear_session()
{
    session_ = 0;
    nonce_.clear();
    control_.clear();
    data_.clear();
    imu_.clear();
    subscriptions_.clear();
    last_status_ = 0;
    reset_map_sync();
    map_pose_batches_ = false;
    map_snapshots_ = false;
    map_session_visible_ = false;
    budget_ = 65536;
    tokens_ = 16384;
    last_tokens_ = monotonic_ns();
}
void Server::disconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    clear_session();
}
void Server::state(Json values)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto state = values.find("state");
    if (state != values.end() && state->is_string() &&
        (*state == "IDLE" || *state == "ERROR" || *state == "WAITING_CALIBRATION"))
        run_active_ = false;
    state_.update(values);
}
void Server::descriptor(const std::string &section, Json values)
{
    std::lock_guard<std::mutex> lock(mutex_);
    descriptors_[section] = std::move(values);
}
Json Server::status_locked() const
{
    auto j = state_;
    j["run_id"] = id_string(run_);
    j["epoch"] = epoch_;
    j["queue_dropped"] = id_string(dropped_);
    j["transmitted_frames"] = id_string(transmitted_);
    j["received_frames"] = id_string(received_);
    j["tx_budget_bytes_per_sec"] = budget_;
    return j;
}
Json Server::status() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return status_locked();
}
void Server::queue_json(std::uint16_t type, Json values, std::uint32_t request)
{
    if (session_)
        values["session_id"] = id_string(session_);
    if (control_.size() >= 64)
    {
        ++dropped_;
        return;
    }
    control_.push_back(json_frame(type, values, request));
}
void Server::queue_data(std::uint16_t stream, std::deque<Frame> frames)
{
    const auto now = frames.front().timestamp_ns;
    auto &queue = data_[stream];
    if (stream == 1)
    {
        // Preserve IMU-rate poses arriving in a short scheduling burst. Bound
        // stale history on a slow/disconnected consumer to 100 ms at 200 Hz.
        constexpr std::size_t pose_queue_limit = pose_max_hz / 10;
        while (queue.size() + frames.size() > pose_queue_limit && !queue.empty())
        {
            queue.pop_front();
            ++dropped_;
        }
        for (auto &frame : frames)
            queue.push_back(std::move(frame));
    }
    else
    {
        dropped_ += queue.size();
        queue = std::move(frames);
    }
    auto &sub = subscriptions_[stream];
    sub.last = now;
    const auto period = static_cast<std::uint64_t>(1e9 / sub.hz);
    // Schedule on capture time, independent of JPEG worker latency. Keep phase:
    // resetting to now+period would turn 20 Hz input into a 10 Hz preview.
    sub.next_due =
        (!sub.next_due || now > sub.next_due + period) ? now + period : sub.next_due + period;
}
Context Server::context(std::uint16_t stream)
{
    Context c;
    c.session = session_;
    c.run = run_;
    c.epoch = epoch_;
    c.stream = stream;
    c.sample = ++subscriptions_[stream].sample;
    return c;
}
bool Server::allowed(std::uint16_t stream, std::uint64_t now) const
{
    auto it = subscriptions_.find(stream);
    if (!session_ || it == subscriptions_.end() || !it->second.enabled)
        return false;
    const auto next = it->second.next_due;
    // Small source timestamp jitter must not skip every second 15 Hz frame.
    return !next || now >= next || next - now <= static_cast<std::uint64_t>(1e8 / it->second.hz);
}
bool Server::due(std::uint16_t stream, std::uint64_t source_time) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return allowed(stream, source_time ? source_time : monotonic_ns());
}
void Server::run(std::uint64_t id, std::uint32_t epoch, const std::string &reason)
{
    std::lock_guard<std::mutex> lock(mutex_);
    run_ = id;
    epoch_ = epoch;
    run_active_ = true;
    map_session_visible_ = session_ != 0;
    data_.clear();
    imu_.clear();
    map_.reset();
    reset_map_sync();
    for (auto &subscription : subscriptions_)
    {
        subscription.second.last = 0;
        subscription.second.next_due = 0;
        subscription.second.source_next = 0;
    }
    if (session_)
    {
        auto event = status_locked();
        event["reason"] = reason;
        queue_json(EpochEvent, std::move(event));
    }
}
void Server::complete(std::uint64_t id, bool success, const std::string &message, Json values)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &op : operations_)
        if (op.task.operation == id)
        {
            op.done = true;
            op.time = monotonic_ns();
            op.result = {{"operation_id", id_string(id)},
                         {"status", success ? "completed" : "failed"},
                         {"message", message}};
            op.result.update(values);
            if (session_)
                queue_json(OperationEvent, op.result, op.session == session_ ? op.request : 0);
            return;
        }
}
void Server::receive(const Frame &f)
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_;
    const auto now = monotonic_ns();
    if (now - request_window_ > 1000000000ull)
    {
        request_window_ = now;
        requests_in_window_ = 0;
    }
    if (f.type != MapAck && ++requests_in_window_ > 64)
        return;
    try
    {
        if (f.type == Hello)
        {
            auto j = parse_json(f);
            auto p = j.at("protocol");
            if (p.at("major") != 1 || p.at("min_minor").get<int>() > 0 ||
                p.at("max_minor").get<int>() < 0)
                throw std::runtime_error("VERSION_MISMATCH");
            const auto nonce = j.at("client_nonce").get<std::string>();
            if (nonce.size() < 16 || nonce.size() > 64)
                throw std::runtime_error("Invalid client_nonce");
            if (nonce != nonce_ || !session_)
            {
                clear_session();
                session_ = random_id();
                // A stopped run's final display belongs to its existing viewer.
                // Only an active run may restore its map to a new connection.
                map_session_visible_ = run_active_;
                nonce_ = nonce;
                payload_limit_ =
                    std::clamp<std::size_t>(j.value("max_payload_bytes", 4096), 1024, 4096);
                const auto capabilities = j.value("capabilities", Json::array());
                if (!capabilities.is_array())
                    throw std::runtime_error("Invalid client capabilities");
                map_pose_batches_ = std::find(capabilities.begin(), capabilities.end(),
                                               "map_pose_batch_v1") != capabilities.end();
                map_snapshots_ = std::find(capabilities.begin(), capabilities.end(),
                                          "map_snapshot_v2") != capabilities.end();
                hello_reply_ = {{"client_nonce", nonce_},
                                {"session_id", id_string(session_)},
                                {"device_boot_id", boot_},
                                {"protocol", {{"major", 1}, {"minor", 0}}},
                                {"max_payload_bytes", payload_limit_},
                                {"device", descriptors_["device"]},
                                {"capabilities",
                                 {"pose", "imu_calibrated", "map_keyframes", "keyframe_poses",
                                  "features_left", "image_rectified_left", "dense_map",
                                  "incremental_map_v1", "map_pose_batch_v1", "map_snapshot_v2", "time_sync", "map_management", "runtime_processing", "runtime_elevator"}}};
            }
            auto result = hello_reply_;
            result.update(status_locked());
            queue_json(HelloReply, result, f.request_id);
            last_seen_ = now;
            return;
        }
        if (!session_)
            throw std::runtime_error("NOT_READY");
        if (f.type == TimeSync)
        {
            if (f.flags != 0)
                throw std::runtime_error("Invalid time sync flags");
            Reader r(f.payload);
            if (r.u64() != session_)
                return;
            auto t1 = r.u64();
            r.done();
            Writer w;
            w.u64(session_);
            w.u64(t1);
            w.u64(now);
            w.u64(monotonic_ns());
            auto reply = binary_frame(TimeSyncReply, std::move(w.bytes), now);
            reply.request_id = f.request_id;
            if (control_.size() < 64)
                control_.push_back(std::move(reply));
            last_seen_ = now;
            return;
        }
        auto j = parse_json(f);
        if (json_u64(j, "session_id") != session_)
            return;
        last_seen_ = now;
        if (f.type == MapAck)
        {
            map_feedback(j);
            return;
        }
        if (!f.request_id)
            throw std::runtime_error("request_id must be nonzero");
        switch (f.type)
        {
        case Ping:
            queue_json(Pong, {{"device_time_ns", id_string(now)}}, f.request_id);
            break;
        case Status:
            queue_json(StatusReply, status_locked(), f.request_id);
            break;
        case Descriptor:
        {
            auto section = j.value("section", std::string("device"));
            if (!descriptors_.contains(section))
                throw std::runtime_error("Unknown descriptor section");
            queue_json(DescriptorReply,
                       {{"section", section}, {"descriptor", descriptors_[section]}}, f.request_id);
            break;
        }
        case Subscribe:
        {
            const auto budget = j.value("tx_budget_bytes_per_sec", 65536);
            if (budget < 4096 || budget > 4 * 1024 * 1024)
                throw std::runtime_error("Invalid tx budget");
            if (!j.at("streams").is_array() || j["streams"].size() > 8)
                throw std::runtime_error("Invalid streams");
            auto replacement = subscriptions_;
            Json accepted = Json::array();
            std::set<unsigned> seen;
            for (const auto &item : j["streams"])
            {
                auto name = item.at("name").get<std::string>();
                auto id = stream_id(name);
                if (!id)
                {
                    queue_json(Error,
                               {{"code", "UNSUPPORTED"},
                                {"message", "Stream not available in this VIO example: " + name}},
                               f.request_id);
                    return;
                }
                if (!seen.insert(id).second)
                    throw std::runtime_error("Duplicate stream");
                auto &sub = replacement[id];
                sub.enabled = item.at("enabled").get<bool>();
                sub.hz = bounded_hz(item, stream_max_hz(id));
                auto points = item.value("max_points", 128);
                if (points < 1 || points > (id == 9   ? int(dense_point_limit)
                                            : id == 3 ? int(keyframe_limit)
                                                      : 512))
                    throw std::runtime_error("Invalid point limit");
                sub.points = points;
                const auto batch = item.value("batch_max_ms", 50);
                if (batch < 1 || batch > 50)
                    throw std::runtime_error("batch_max_ms must be 1..50");
                sub.batch_ms = unsigned(batch);
                sub.source_next = 0;
                sub.next_due = 0;
                accepted.push_back({{"name", name},
                                    {"id", id},
                                    {"enabled", sub.enabled},
                                    {"max_hz", sub.hz},
                                    {"max_points", sub.points},
                                    {"status", "accepted"}});
                if (id == 2)
                    accepted.back()["batch_max_ms"] = sub.batch_ms;
            }
            const bool map_subscription_changed =
                replacement[3].enabled != subscriptions_[3].enabled ||
                replacement[9].enabled != subscriptions_[9].enabled;
            subscriptions_ = std::move(replacement);
            budget_ = budget;
            data_.clear();
            imu_.clear();
            if (map_subscription_changed)
                reset_map_sync();
            queue_json(SubscribeReply,
                       {{"streams", accepted}, {"tx_budget_bytes_per_sec", budget_}}, f.request_id);
            break;
        }
        case Command:
        {
            const auto name = j.at("command").get<std::string>();
            if (name != "START" && name != "STOP" && name != "RESET" && name != "MAP_LIST" &&
                name != "SET_PROCESSING" && name != "SET_ELEVATOR")
            {
                queue_json(Error, {{"code", "UNSUPPORTED"}, {"message", "Unknown example command"}},
                           f.request_id);
                break;
            }
            const auto token = j.at("command_token").get<std::string>();
            if (token.size() < 16 || token.size() > 64)
                throw std::runtime_error("Invalid command token");
            Json signature = {{"command", name}, {"args", j.value("args", Json::object())}};
            if (!signature["args"].is_object())
                throw std::runtime_error("Command args must be an object");
            auto previous = std::find_if(operations_.begin(), operations_.end(),
                                         [&](const Op &op) { return op.task.token == token; });
            if (previous != operations_.end())
            {
                if (previous->signature != signature)
                {
                    queue_json(
                        Error,
                        {{"code", "CONFLICT"}, {"message", "Token reused with different command"}},
                        f.request_id);
                    break;
                }
                queue_json(CommandReply, previous->reply, f.request_id);
                if (previous->done)
                    queue_json(OperationEvent, previous->result, f.request_id);
                break;
            }
            while (!operations_.empty() && operations_.front().done &&
                   now - operations_.front().time > 300000000000ull)
                operations_.pop_front();
            if (operations_.size() >= 256)
            {
                queue_json(Error, {{"code", "BUSY"}, {"message", "Operation cache full"}},
                           f.request_id);
                break;
            }
            Op op;
            op.task = {name, token, random_id(), signature["args"]};
            op.signature = signature;
            op.time = now;
            op.session = session_;
            op.request = f.request_id;
            op.reply = {{"status", "accepted"}, {"operation_id", id_string(op.task.operation)}};
            command_(op.task);
            queue_json(CommandReply, op.reply, f.request_id);
            operations_.push_back(std::move(op));
            break;
        }
        case Operation:
        {
            auto it = operations_.end();
            if (j.contains("operation_id"))
            {
                auto id = json_u64(j, "operation_id");
                it = std::find_if(operations_.begin(), operations_.end(),
                                  [&](const Op &op) { return op.task.operation == id; });
            }
            else if (j.contains("command_token"))
            {
                const auto token = j["command_token"].get<std::string>();
                it = std::find_if(operations_.begin(), operations_.end(),
                                  [&](const Op &op) { return op.task.token == token; });
            }
            if (it == operations_.end())
                queue_json(Error,
                           {{"code", "NOT_READY"}, {"message", "Operation result unavailable"}},
                           f.request_id);
            else
                queue_json(OperationReply, it->done ? it->result : it->reply, f.request_id);
            break;
        }
        default:
            queue_json(Error, {{"code", "UNSUPPORTED"}, {"message", "Unknown example message"}},
                       f.request_id);
            break;
        }
    }
    catch (const std::exception &e)
    {
        queue_json(Error,
                   {{"code", std::string(e.what()) == "VERSION_MISMATCH" ? "VERSION_MISMATCH"
                                                                         : "INVALID_ARGUMENT"},
                    {"message", e.what()}},
                   f.request_id);
    }
}
void Server::flush_imu()
{
    if (imu_.empty() || !session_)
        return;
    ImuBatch b;
    b.context = context(2);
    b.context.sample = imu_.front().sequence;
    const auto start = imu_.front().time;
    for (const auto &s : imu_)
    {
        auto sample = s.value;
        sample.dt = static_cast<std::uint32_t>(s.time - start);
        b.samples.push_back(sample);
    }
    imu_.clear();
    auto frame = binary_frame(ImuData, encode_imu(b), start);
    auto &q = data_[2];
    if (q.size() >= 8)
    {
        q.pop_front();
        ++dropped_;
    }
    q.push_back(std::move(frame));
}
void Server::tick()
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = monotonic_ns();
    if (!last_tokens_)
        last_tokens_ = now;
    tokens_ = std::min<double>(std::max<std::size_t>(payload_limit_ + header_size, budget_ / 4),
                               tokens_ + (now - last_tokens_) * 1e-9 * budget_);
    last_tokens_ = now;
    if (session_ && now - last_seen_ > 4000000000ull)
    {
        clear_session();
        return;
    }
    if (!imu_.empty() &&
        now - imu_started_ >= std::uint64_t(subscriptions_[2].batch_ms) * 1000000ull)
        flush_imu();
    if (session_ && now - last_status_ >= 1000000000ull)
    {
        queue_json(StatusEvent, status_locked());
        last_status_ = now;
    }
}
std::optional<Frame> Server::next_frame()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!control_.empty() && (control_burst_ < 4 || data_.empty()))
    {
        auto f = dequeue_control(control_);
        ++control_burst_;
        ++transmitted_;
        return f;
    }
    auto take = [&](unsigned id) -> std::optional<Frame>
    {
        auto it = data_.find(id);
        if (it == data_.end() || it->second.empty())
            return {};
        auto &f = it->second.front();
        auto size = header_size + f.payload.size();
        if (tokens_ < size)
            return {};
        tokens_ -= size;
        auto result = std::move(f);
        it->second.pop_front();
        if (it->second.empty())
            data_.erase(it);
        control_burst_ = 0;
        ++transmitted_;
        return result;
    };
    if (auto pose = take(1))
        return pose;
    if (auto imu = take(2))
        return imu;
    for (unsigned i = 0; i < 7; ++i)
    {
        const auto id = round_robin_;
        round_robin_ = round_robin_ >= 9 ? 3 : round_robin_ + 1;
        if (id == 9)
        {
            if (auto value = next_map_frame())
                return value;
        }
        if (auto value = take(id))
            return value;
    }
    if (!control_.empty())
    {
        auto f = dequeue_control(control_);
        ++transmitted_;
        return f;
    }
    return {};
}
void Server::pose(Pose p, std::uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!allowed(1, timestamp))
        return;
    p.context = context(1);
    queue_data(1, {binary_frame(PoseData, encode_pose(p), timestamp)});
}
void Server::imu(std::uint64_t timestamp, std::uint64_t sequence, const std::array<float, 3> &gyro,
                 const std::array<float, 3> &accel)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto &sub = subscriptions_[2];
    if (!session_ || !sub.enabled)
        return;
    // Throttle on capture time, not on how quickly the broker queue is drained.
    // A phase accumulator tolerates normal timestamp jitter at the nominal rate.
    const auto period = std::uint64_t(1e9 / sub.hz);
    if (sub.source_next && timestamp < sub.source_next && sub.source_next - timestamp > period / 10)
        return;
    if (!sub.source_next || timestamp > sub.source_next + period)
        sub.source_next = timestamp;
    sub.source_next += period;
    if (!imu_.empty() && (sequence != imu_.back().sequence + 1 || timestamp <= imu_.back().time ||
                          timestamp - imu_.front().time > 1000000000ull))
        flush_imu();
    if (imu_.empty())
        imu_started_ = monotonic_ns();
    TimedImu sample{timestamp, sequence, {}};
    sample.value.gyro = gyro;
    sample.value.accel = accel;
    imu_.push_back(sample);
    subscriptions_[2].last = monotonic_ns();
    if (imu_.size() >= std::min<std::size_t>(10, (payload_limit_ - 36) / 32))
        flush_imu();
}
void Server::cloud(std::uint16_t id, std::vector<Point> points, std::uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (id != 3 || !allowed(id, timestamp))
        return;
    points.resize(std::min(points.size(), subscriptions_[id].points));
    const auto per = (payload_limit_ - 48) / 24;
    const auto n = std::max<std::size_t>(1, (points.size() + per - 1) / per);
    auto c = context(id);
    std::deque<Frame> frames;
    for (std::size_t i = 0; i < n; ++i)
    {
        Cloud chunk;
        chunk.context = c;
        chunk.source = 0;
        chunk.index = static_cast<std::uint16_t>(i);
        chunk.chunks = static_cast<std::uint16_t>(n);
        chunk.total = static_cast<std::uint32_t>(points.size());
        auto begin = std::min(i * per, points.size()), end = std::min(begin + per, points.size());
        chunk.points.assign(points.begin() + begin, points.begin() + end);
        frames.push_back(binary_frame(PointsData, encode_cloud(chunk), timestamp));
    }
    queue_data(id, std::move(frames));
}
bool Server::keyframes(std::vector<KeyframePose> poses, std::uint64_t revision,
                       std::uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint16_t id = 3;
    if (!allowed(id, timestamp) || (data_.count(id) && !data_[id].empty()))
        return false;
    const auto limit = std::min<std::size_t>(keyframe_limit, subscriptions_[id].points);
    if (poses.size() > limit)
    {
        std::vector<KeyframePose> selected;
        selected.reserve(limit);
        for (std::size_t i = 0; i < limit; ++i)
            selected.push_back(poses[limit > 1 ? i * (poses.size() - 1) / (limit - 1) : 0]);
        poses.swap(selected);
    }
    const auto per = std::min<std::size_t>(112, (payload_limit_ - 48) / 36),
               n = std::max<std::size_t>(1, (poses.size() + per - 1) / per);
    auto c = context(id);
    std::deque<Frame> frames;
    for (std::size_t i = 0; i < n; ++i)
    {
        Keyframes part;
        part.context = c;
        part.revision = revision;
        part.index = std::uint16_t(i);
        part.chunks = std::uint16_t(n);
        part.total = std::uint32_t(poses.size());
        const auto begin = std::min(i * per, poses.size()),
                   end = std::min(begin + per, poses.size());
        part.poses.assign(poses.begin() + begin, poses.begin() + end);
        frames.push_back(binary_frame(KeyframesData, encode_keyframes(part), timestamp));
    }
    queue_data(id, std::move(frames));
    return true;
}
bool Server::dense_cloud(std::vector<DensePoint> points, std::uint64_t revision,
                         std::uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint16_t id = 9;
    // Finish an in-flight snapshot before accepting a newer revision.
    if (!allowed(id, timestamp) || (data_.count(id) && !data_[id].empty()))
        return false;
    const auto limit = std::min<std::size_t>(dense_point_limit, subscriptions_[id].points);
    if (points.size() > limit)
    {
        std::vector<DensePoint> selected;
        selected.reserve(limit);
        for (std::size_t i = 0; i < limit; ++i)
            selected.push_back(points[i * points.size() / limit]);
        points.swap(selected);
    }
    const auto per = (payload_limit_ - 48) / 16,
               n = std::max<std::size_t>(1, (points.size() + per - 1) / per);
    auto c = context(id);
    std::deque<Frame> frames;
    for (std::size_t i = 0; i < n; ++i)
    {
        DenseCloud part;
        part.context = c;
        part.revision = revision;
        part.index = std::uint16_t(i);
        part.chunks = std::uint16_t(n);
        part.total = std::uint32_t(points.size());
        const auto begin = std::min(i * per, points.size()),
                   end = std::min(begin + per, points.size());
        part.points.assign(points.begin() + begin, points.begin() + end);
        frames.push_back(binary_frame(DenseCloudData, encode_dense_cloud(part), timestamp));
    }
    queue_data(id, std::move(frames));
    return true;
}
void Server::features(Features features, std::uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint16_t id = 5;
    if (features.camera != 0 || !allowed(id, timestamp))
        return;
    features.points.resize(std::min(features.points.size(), subscriptions_[id].points));
    const auto per = (payload_limit_ - 56) / 24;
    const auto n = std::max<std::size_t>(1, (features.points.size() + per - 1) / per);
    auto c = context(id);
    std::deque<Frame> frames;
    for (std::size_t i = 0; i < n; ++i)
    {
        auto chunk = features;
        chunk.context = c;
        chunk.index = static_cast<std::uint16_t>(i);
        chunk.chunks = static_cast<std::uint16_t>(n);
        auto start = std::min(i * per, features.points.size());
        auto end = std::min(start + per, features.points.size());
        chunk.points.assign(features.points.begin() + start, features.points.begin() + end);
        frames.push_back(binary_frame(FeaturesData, encode_features(chunk), timestamp));
    }
    queue_data(id, std::move(frames));
}
void Server::image(ImagePart image, std::uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint16_t id = 7;
    if (image.camera != 0 || !allowed(id, timestamp) || image.data.empty() ||
        image.data.size() > 262144)
        return;
    const auto per = std::min<std::size_t>(2048, payload_limit_ - 60);
    auto c = context(id);
    std::deque<Frame> frames;
    for (std::size_t i = 0; i < image.data.size(); i += per)
    {
        ImagePart part;
        part.context = c;
        part.camera_frame = image.camera_frame;
        part.source = image.source;
        part.camera = image.camera;
        part.width = image.width;
        part.height = image.height;
        part.encoding = image.encoding;
        part.flags = image.flags;
        part.total = static_cast<std::uint32_t>(image.data.size());
        part.offset = static_cast<std::uint32_t>(i);
        part.data.assign(image.data.begin() + i,
                         image.data.begin() + std::min(i + per, image.data.size()));
        frames.push_back(binary_frame(ImageData, encode_image(part), timestamp));
    }
    queue_data(id, std::move(frames));
}
void ServerLink::step(Transport &port)
{
    std::uint8_t bytes[4096];
    auto count = port.read(bytes, sizeof(bytes), 1);
    rx_bytes += count;
    for (const auto &frame : decoder_.feed(bytes, count))
        server_.receive(frame);
    server_.tick();
    for (int i = 0; i < 8; ++i)
    {
        if (tx_.empty())
        {
            auto frame = server_.next_frame();
            if (!frame)
                break;
            tx_.begin(std::move(*frame));
        }
        auto n = tx_.flush(port, 5);
        tx_bytes += n;
        if (!n || !tx_.empty())
            break;
    }
}
} // namespace svio
