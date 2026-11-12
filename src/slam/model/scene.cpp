/* SPDX-License-Identifier: Apache-2.0 */
#include "scene.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace cambold::detail
{
namespace
{
svio::Json pose_json(const svio::Pose &p, std::uint64_t time)
{
    return {{"run_id", svio::id_string(p.context.run)},
            {"epoch", p.context.epoch},
            {"timestamp_ns", svio::id_string(time)},
            {"sample_id", svio::id_string(p.context.sample)},
            {"frame_id", svio::id_string(p.camera_frame)},
            {"p", p.position},
            {"q", p.quaternion},
            {"v", p.velocity}};
}
bool jpeg_dimensions(const svio::Bytes &bytes, unsigned width, unsigned height)
{
    if (bytes.size() < 4 || bytes[0] != 0xff || bytes[1] != 0xd8 ||
        bytes[bytes.size() - 2] != 0xff || bytes.back() != 0xd9)
        return false;
    std::size_t i = 2;
    while (i + 4 < bytes.size())
    {
        if (bytes[i++] != 0xff)
            continue;
        while (i < bytes.size() && bytes[i] == 0xff)
            ++i;
        if (i >= bytes.size())
            break;
        auto marker = bytes[i++];
        if (marker == 0xd9 || marker == 0xda)
            break;
        if (marker == 0x01 || (marker >= 0xd0 && marker <= 0xd7))
            continue;
        if (i + 2 > bytes.size())
            break;
        auto size = (unsigned(bytes[i]) << 8) | bytes[i + 1];
        if (size < 2 || i + size > bytes.size())
            return false;
        if ((marker >= 0xc0 && marker <= 0xc3) || (marker >= 0xc5 && marker <= 0xc7) ||
            (marker >= 0xc9 && marker <= 0xcb) || (marker >= 0xcd && marker <= 0xcf))
        {
            return size >= 8 && ((unsigned(bytes[i + 3]) << 8) | bytes[i + 4]) == height &&
                   ((unsigned(bytes[i + 5]) << 8) | bytes[i + 6]) == width;
        }
        i += size;
    }
    return false;
}
} // namespace
void Scene::log_locked(const std::string &text)
{
    logs_.push_back({{"time_ns", svio::id_string(svio::monotonic_ns())}, {"text", text}});
    if (logs_.size() > 200)
        logs_.erase(logs_.begin());
}
void Scene::log(const std::string &text)
{
    std::lock_guard<std::mutex> lock(mutex_);
    log_locked(text);
}
void Scene::connection(const std::string &value, const std::string &detail)
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_["connection"] = value;
    state_["connection_detail"] = detail;
}
void Scene::counters(std::uint64_t rx, std::uint64_t tx, std::uint64_t crc)
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_["rx_bytes"] = svio::id_string(rx);
    state_["tx_bytes"] = svio::id_string(tx);
    state_["crc_errors"] = svio::id_string(crc);
}
void Scene::round_trip(double rtt)
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_["rtt_ms"] = rtt * 1e-6;
}
void Scene::boundary(std::uint64_t run, std::uint32_t epoch)
{
    if (run == run_ && epoch == epoch_)
        return;
    run_ = run;
    epoch_ = epoch;
    last_pose_time_ = 0;
    unknown_epoch_ = false;
    pose_ = nullptr;
    reset_map();
    dense_parts_ = {};
    dense_ = nullptr;
    keyframe_parts_ = {};
    keyframes_ = nullptr;
    images_.clear();
    feature_parts_.clear();
    image_parts_.clear();
    for (auto &f : features_)
        f.clear();
    for (auto &m : image_meta_)
        m = nullptr;
    imu_ = svio::Json::array();
    pose_receipts_.clear();
    image_receipts_.clear();
    state_.erase("pose_rate_hz");
    state_.erase("image_rate_hz");
    state_["run_id"] = svio::id_string(run);
    state_["epoch"] = epoch;
}
bool Scene::context(const svio::Context &c)
{
    if (c.session != session_)
        return false;
    if (c.run != run_ || c.epoch != epoch_)
    {
        unknown_epoch_ = true;
        return false;
    }
    return true;
}
void Scene::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
    if (map_sync_)
        map_feedback_ = {{"sync_id", svio::id_string(map_sync_)},
                         {"sample_id", svio::id_string(map_sample_)},
                         {"action", "resync"}};
}
bool Scene::needs_status() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return unknown_epoch_;
}
void Scene::catalog_requested(const std::string &token)
{
    std::lock_guard<std::mutex> lock(mutex_);
    catalog_token_ = token;
    catalog_pending_ = svio::Json::array();
    catalog_next_requested_ = 0;
    map_catalog_["loading"] = true;
}
bool Scene::catalog_next(const std::string &token, std::size_t offset)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (token != catalog_token_ || !map_catalog_.value("loading", false) ||
        offset != catalog_pending_.size() || offset <= catalog_next_requested_)
        return false;
    catalog_next_requested_ = offset;
    return true;
}
void Scene::catalog_page(const svio::Json &page)
{
    if (page.value("catalog_token", std::string()) != catalog_token_)
        return;
    const auto offset = page.at("offset").get<std::size_t>();
    if (offset != catalog_pending_.size())
        return;
    if (!page.at("maps").is_array() || page["maps"].size() > 12 ||
        page.at("total").get<std::size_t>() > 100000)
        throw std::runtime_error("Invalid map catalog page");
    for (const auto &entry : page["maps"])
    {
        if (!entry.at("name").is_string() || entry["name"].get<std::string>().size() > 80 ||
            !entry.at("bytes").is_number_unsigned())
            throw std::runtime_error("Invalid map entry");
        catalog_pending_.push_back(entry);
    }
    if (page.at("next_offset").is_null())
    {
        if (catalog_pending_.size() != page["total"].get<std::size_t>())
            throw std::runtime_error("Incomplete map catalog");
        map_catalog_ = {
            {"maps", catalog_pending_}, {"loading", false}, {"catalog_token", catalog_token_}};
    }
}
void Scene::refresh_overlay(unsigned camera)
{
    if (camera >= 2 || image_meta_[camera].is_null())
        return;
    auto &image = image_meta_[camera];
    image["features"] = svio::Json::array();
    for (auto i = features_[camera].rbegin(); i != features_[camera].rend(); ++i)
    {
        const auto &f = i->value;
        if (svio::id_string(f.camera_frame) == image["frame_id"] && f.source == image["source"] &&
            f.width == image["width"] && f.height == image["height"])
        {
            for (const auto &p : f.points)
                image["features"].push_back({svio::id_string(p.id), p.u, p.v, p.flags});
            break;
        }
    }
    images_[std::to_string(camera)] = image;
}
void Scene::accept(const svio::Frame &frame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        if (frame.flags == svio::json_flag)
        {
            auto j = svio::parse_json(frame);
            if (frame.type == svio::HelloReply)
            {
                auto session = svio::json_u64(j, "session_id");
                if (session != session_)
                {
                    session_ = session;
                    ++generation_;
                    feature_parts_.clear();
                    image_parts_.clear();
                    reset_map();
                    dense_parts_ = {};
                    dense_ = nullptr;
                    keyframe_parts_ = {};
                    keyframes_ = nullptr;
                    images_ = svio::Json::object();
                    imu_ = svio::Json::array();
                    descriptors_ = svio::Json::object();
                    pose_ = nullptr;
                    last_pose_time_ = 0;
                    unknown_epoch_ = false;
                    for (auto &features : features_)
                        features.clear();
                    for (auto &meta : image_meta_)
                        meta = nullptr;
                    pose_receipts_.clear();
                    image_receipts_.clear();
                    state_ = svio::Json::object();
                }
                boundary(svio::json_u64(j, "run_id"), j.at("epoch").get<std::uint32_t>());
                state_.update(j);
                state_["connection"] = "READY";
                return;
            }
            if (!j.contains("session_id") || svio::json_u64(j, "session_id") != session_)
                return;
            if (frame.type == svio::DescriptorReply)
            {
                descriptors_[j.at("section").get<std::string>()] = j.at("descriptor");
                return;
            }
            if (frame.type == svio::StatusReply || frame.type == svio::StatusEvent ||
                frame.type == svio::EpochEvent)
            {
                boundary(svio::json_u64(j, "run_id"), j.at("epoch").get<std::uint32_t>());
                unknown_epoch_ = false;
                state_.update(j);
                return;
            }
            if ((frame.type == svio::OperationEvent || frame.type == svio::OperationReply) &&
                j.contains("map_catalog"))
            {
                catalog_page(j["map_catalog"]);
                return;
            }
            if (frame.type == svio::Error || frame.type == svio::OperationEvent ||
                frame.type == svio::OperationReply || frame.type == svio::CommandReply ||
                frame.type == svio::LogEvent)
            {
                state_["last_result"] = j;
                log_locked(j.dump());
            }
            return;
        }
        if (frame.type == svio::PoseData)
        {
            auto p = svio::decode_pose(frame.payload);
            if (p.context.stream != 1 || !context(p.context) || !(p.flags & 1) ||
                frame.timestamp_ns <= last_pose_time_)
                return;
            last_pose_time_ = frame.timestamp_ns;
            pose_ = pose_json(p, frame.timestamp_ns);
            const auto received = svio::monotonic_ns();
            pose_receipts_.push_back(received);
            while (pose_receipts_.size() > 1 && received - pose_receipts_.front() > 2000000000ull)
                pose_receipts_.pop_front();
            return;
        }
        if (frame.type == svio::ImuData)
        {
            auto b = svio::decode_imu(frame.payload);
            if (b.context.stream != 2 || !context(b.context))
                return;
            for (const auto &s : b.samples)
                imu_.push_back({svio::id_string(frame.timestamp_ns + s.dt), s.gyro[0], s.gyro[1],
                                s.gyro[2], s.accel[0], s.accel[1], s.accel[2]});
            while (imu_.size() > 400)
                imu_.erase(imu_.begin());
            return;
        }
        if (frame.type == svio::MapDeltaData)
        {
            map_delta(svio::decode_map_delta(frame.payload));
            return;
        }
        if (frame.type == svio::DenseCloudData)
        {
            auto part = svio::decode_dense_cloud(frame.payload);
            if (!context(part.context))
                return;
            auto &a = dense_parts_;
            auto now = svio::monotonic_ns();
            if (a.sample != part.context.sample || now - a.started > 10000000000ull)
            {
                a = {};
                a.sample = part.context.sample;
                a.revision = part.revision;
                a.time = frame.timestamp_ns;
                a.started = now;
                a.total = part.total;
                a.chunks.resize(part.chunks);
            }
            if (a.chunks.size() != part.chunks || a.revision != part.revision ||
                a.total != part.total || a.time != frame.timestamp_ns || a.chunks[part.index])
                throw std::runtime_error("Inconsistent dense snapshot");
            a.chunks[part.index] = std::move(part);
            if (std::any_of(a.chunks.begin(), a.chunks.end(), [](const auto &v) { return !v; }))
                return;
            svio::Writer packed;
            std::size_t count = 0;
            for (const auto &chunk : a.chunks)
                for (const auto &p : chunk->points)
                {
                    for (auto x : p.xyz)
                        packed.f32(x);
                    for (auto x : p.rgb)
                        packed.u8(x);
                    packed.u8(0);
                    ++count;
                }
            if (count != a.total)
                throw std::runtime_error("Dense count mismatch");
            dense_ = {{"revision", svio::id_string(a.revision)},
                      {"timestamp_ns", svio::id_string(a.time)},
                      {"count", count},
                      {"xyzrgb16", svio::base64(packed.bytes)}};
            state_["received_map_points"] = count;
            state_["received_map_revision"] = svio::id_string(a.revision);
            a = {};
            return;
        }
        if (frame.type == svio::KeyframesData)
        {
            auto part = svio::decode_keyframes(frame.payload);
            if (!context(part.context))
                return;
            auto &a = keyframe_parts_;
            const auto now = svio::monotonic_ns();
            if (a.sample != part.context.sample || now - a.started > 5000000000ull)
            {
                a = {};
                a.sample = part.context.sample;
                a.revision = part.revision;
                a.time = frame.timestamp_ns;
                a.started = now;
                a.total = part.total;
                a.chunks.resize(part.chunks);
            }
            if (a.chunks.size() != part.chunks || a.total != part.total ||
                a.revision != part.revision || a.time != frame.timestamp_ns || a.chunks[part.index])
                throw std::runtime_error("Inconsistent keyframe snapshot");
            a.chunks[part.index] = std::move(part);
            if (std::any_of(a.chunks.begin(), a.chunks.end(), [](const auto &v) { return !v; }))
                return;
            svio::Json poses = svio::Json::array();
            std::set<std::uint64_t> ids;
            for (const auto &chunk : a.chunks)
                for (const auto &p : chunk->poses)
                {
                    if (!ids.insert(p.id).second)
                        throw std::runtime_error("Duplicate keyframe ID");
                    poses.push_back({svio::id_string(p.id), p.position[0], p.position[1],
                                     p.position[2], p.quaternion[0], p.quaternion[1],
                                     p.quaternion[2], p.quaternion[3]});
                }
            if (poses.size() != a.total)
                throw std::runtime_error("Keyframe count mismatch");
            keyframes_ = {{"timestamp_ns", svio::id_string(a.time)},
                          {"revision", svio::id_string(a.revision)},
                          {"poses", std::move(poses)}};
            state_["received_keyframes"] = a.total;
            a = {};
            return;
        }
        if (frame.type == svio::FeaturesData)
        {
            auto part = svio::decode_features(frame.payload);
            if (part.camera != 0 || part.context.stream != 5 || !context(part.context))
                return;
            auto &a = feature_parts_[part.camera];
            auto now = svio::monotonic_ns();
            if (a.sample != part.context.sample || now - a.started > 500000000ull)
            {
                a = {};
                a.sample = part.context.sample;
                a.time = frame.timestamp_ns;
                a.started = now;
                a.chunks.resize(part.chunks);
            }
            if (a.chunks.size() != part.chunks || a.time != frame.timestamp_ns ||
                a.chunks[part.index])
                throw std::runtime_error("Inconsistent feature snapshot");
            auto camera = part.camera;
            a.chunks[part.index] = std::move(part);
            if (std::any_of(a.chunks.begin(), a.chunks.end(), [](const auto &v) { return !v; }))
                return;
            svio::Features complete = *a.chunks[0];
            complete.points.clear();
            for (const auto &chunk : a.chunks)
            {
                if (chunk->source != complete.source ||
                    chunk->camera_frame != complete.camera_frame ||
                    chunk->width != complete.width || chunk->height != complete.height)
                    throw std::runtime_error("Feature metadata mismatch");
                complete.points.insert(complete.points.end(), chunk->points.begin(),
                                       chunk->points.end());
            }
            if (complete.points.size() > 8192)
                throw std::runtime_error("Feature total limit");
            features_[camera].push_back({std::move(complete), a.time});
            if (features_[camera].size() > 16)
                features_[camera].pop_front();
            refresh_overlay(camera);
            return;
        }
        if (frame.type == svio::ImageData)
        {
            auto part = svio::decode_image(frame.payload);
            if (part.camera != 0 || part.context.stream != 7 || !context(part.context))
                return;
            auto &a = image_parts_[part.camera];
            auto now = svio::monotonic_ns();
            if (a.first.context.sample != part.context.sample || now - a.started > 500000000ull)
            {
                a = {};
                a.first = part;
                a.first.data.clear();
                a.time = frame.timestamp_ns;
                a.started = now;
                a.bytes.reserve(part.total);
            }
            if (a.time != frame.timestamp_ns || a.first.total != part.total ||
                a.first.camera_frame != part.camera_frame || a.first.source != part.source ||
                a.first.width != part.width || a.first.height != part.height ||
                part.offset != a.bytes.size())
                throw std::runtime_error("Image gap/overlap or metadata mismatch");
            a.bytes.insert(a.bytes.end(), part.data.begin(), part.data.end());
            if (a.bytes.size() != part.total)
                return;
            if (!jpeg_dimensions(a.bytes, part.width, part.height))
                throw std::runtime_error("JPEG dimensions/markers mismatch");
            image_receipts_.push_back(now);
            while (image_receipts_.size() > 1 && now - image_receipts_.front() > 2000000000ull)
                image_receipts_.pop_front();
            image_meta_[part.camera] = {{"jpeg", "data:image/jpeg;base64," + svio::base64(a.bytes)},
                                        {"frame_id", svio::id_string(part.camera_frame)},
                                        {"timestamp_ns", svio::id_string(frame.timestamp_ns)},
                                        {"source", part.source},
                                        {"width", part.width},
                                        {"height", part.height}};
            refresh_overlay(part.camera);
            image_parts_.erase(part.camera);
            return;
        }
    }
    catch (const std::exception &e)
    {
        ++invalid_;
        state_["invalid_samples"] = svio::id_string(invalid_);
        log_locked(e.what());
    }
}
svio::Json Scene::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return {{"state", state_},
            {"descriptors", descriptors_},
            {"pose", pose_},
            {"map_catalog", map_catalog_}};
}
svio::Json Scene::poll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = svio::monotonic_ns();
    while (!pose_receipts_.empty() && now - pose_receipts_.front() > 2000000000ull)
        pose_receipts_.pop_front();
    state_["pose_rate_hz"] =
        pose_receipts_.size() > 1 && pose_receipts_.back() > pose_receipts_.front()
            ? double(pose_receipts_.size() - 1) * 1e9 /
                  double(pose_receipts_.back() - pose_receipts_.front())
            : 0.0;
    while (!image_receipts_.empty() && now - image_receipts_.front() > 2000000000ull)
        image_receipts_.pop_front();
    state_["image_rate_hz"] =
        image_receipts_.size() > 1 && image_receipts_.back() > image_receipts_.front()
            ? double(image_receipts_.size() - 1) * 1e9 /
                  double(image_receipts_.back() - image_receipts_.front())
            : 0.0;
    auto result = svio::Json{{"state", state_},   {"descriptors", descriptors_},
                             {"pose", pose_},     {"generation", svio::id_string(generation_)},
                             {"images", images_}, {"logs", logs_},
                             {"imu", imu_},       {"map_catalog", map_catalog_}};
    if (map_reset_pending_ || !map_changes_.empty())
    {
        svio::Json changes = svio::Json::array();
        for (auto &entry : map_changes_)
            changes.push_back(std::move(entry.second));
        result["map_delta"] = {{"reset", map_reset_pending_}, {"updates", std::move(changes)}};
        map_changes_.clear();
        map_reset_pending_ = false;
    }
    if (!dense_.is_null())
    {
        result["dense_map"] = std::move(dense_);
        dense_ = nullptr;
    }
    if (!keyframes_.is_null())
    {
        result["keyframes"] = std::move(keyframes_);
        keyframes_ = nullptr;
    }
    images_.clear();
    imu_ = svio::Json::array();
    return result;
}
void Scene::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    session_ = 0;
    run_ = 0;
    epoch_ = 0;
    map_catalog_ = {{"maps", svio::Json::array()}, {"loading", false}};
    catalog_pending_ = svio::Json::array();
    catalog_token_.clear();
    catalog_next_requested_ = 0;
    state_ = {{"connection", "DISCONNECTED"}, {"state", "IDLE"}};
    descriptors_ = svio::Json::object();
    reset_map();
    dense_parts_ = {};
    dense_ = nullptr;
    keyframe_parts_ = {};
    keyframes_ = nullptr;
    images_ = svio::Json::object();
    pose_ = nullptr;
    last_pose_time_ = 0;
    ++generation_;
    unknown_epoch_ = false;
    feature_parts_.clear();
    image_parts_.clear();
    for (auto &features : features_)
        features.clear();
    for (auto &meta : image_meta_)
        meta = nullptr;
    pose_receipts_.clear();
    image_receipts_.clear();
    imu_ = svio::Json::array();
    logs_ = svio::Json::array();
    invalid_ = 0;
}
} // namespace cambold::detail
