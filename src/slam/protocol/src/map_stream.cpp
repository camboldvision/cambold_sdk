/* SPDX-License-Identifier: Apache-2.0 */
#include "svio/server.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace svio
{
namespace
{
bool same_pose(const KeyframePose &a, const KeyframePose &b)
{
    for (unsigned i = 0; i < 3; ++i)
        if (std::abs(a.position[i] - b.position[i]) > 1e-6f)
            return false;
    // q and -q describe the same rotation.
    float plus = 0, minus = 0;
    for (unsigned i = 0; i < 4; ++i)
    {
        plus = std::max(plus, std::abs(a.quaternion[i] - b.quaternion[i]));
        minus = std::max(minus, std::abs(a.quaternion[i] + b.quaternion[i]));
    }
    return std::min(plus, minus) <= 1e-6f;
}
} // namespace
void Server::reset_map_sync()
{
    map_sync_ = random_id();
    map_reset_ = true;
    map_paused_ = false;
    map_known_.clear();
    planned_map_.reset();
    map_updates_.clear();
    map_transfer_.reset();
}
void Server::map(std::shared_ptr<const MapSnapshot> snapshot, std::uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot == map_)
        return;
    map_ = std::move(snapshot);
    map_time_ = timestamp;
}
void Server::map_feedback(const Json &j)
{
    if (json_u64(j, "sync_id") != map_sync_)
        return;
    const auto action = j.value("action", std::string("ack"));
    if (action == "resync")
    {
        reset_map_sync();
        return;
    }
    if (!map_transfer_ || json_u64(j, "sample_id") != map_transfer_->context.sample)
        return;
    auto &t = *map_transfer_;
    if (action == "retry")
    {
        t.offset = 0;
        t.sent = false;
        return;
    }
    if (action == "pause")
    {
        map_paused_ = true;
        map_transfer_.reset();
        return;
    }
    if (action != "ack" || !t.sent)
        return;
    const auto &u = t.update;
    if (u.action == MapAction::Reset)
    {
        map_known_.clear();
        map_reset_ = false;
        planned_map_.reset();
    }
    else if (u.action == MapAction::Erase)
        map_known_.erase(u.id);
    else
        map_known_[u.id] = u.node;
    map_transfer_.reset();
}
std::optional<Frame> Server::next_map_frame()
{
    const auto enabled = [&](unsigned id)
    {
        auto it = subscriptions_.find(id);
        return it != subscriptions_.end() && it->second.enabled;
    };
    const bool dense = enabled(9);
    if (!session_ || !map_ || map_paused_ || (!dense && !enabled(3)))
        return {};
    const auto now = monotonic_ns();
    if (map_transfer_ && map_transfer_->sent)
    {
        if (now - map_transfer_->sent_at < 2000000000ull)
            return {};
        // Retry only this node, never all previously acknowledged geometry.
        map_transfer_->offset = 0;
        map_transfer_->sent = false;
    }
    if (!map_transfer_)
    {
        if (map_reset_)
            map_updates_ = {{MapAction::Reset, 0, map_->revision, {}}};
        else if (map_ != planned_map_)
        {
            // Diff metadata once per immutable revision. No traversal of points.
            map_updates_.clear();
            for (const auto &old : map_known_)
                if (!map_->nodes.count(old.first))
                    map_updates_.push_back({MapAction::Erase, old.first, map_->revision, {}});
            for (const auto &entry : map_->nodes)
            {
                const auto old = map_known_.find(entry.first);
                const auto &node = entry.second;
                if (old == map_known_.end() || (dense && old->second.points != node.points))
                    map_updates_.push_back({MapAction::Upsert, entry.first, map_->revision, node});
                else if (!same_pose(old->second.pose, node.pose))
                    map_updates_.push_back({MapAction::Pose, entry.first, map_->revision, node});
            }
            planned_map_ = map_;
        }
        if (map_updates_.empty())
            return {};
        MapTransfer t;
        t.timestamp = map_time_;
        t.update = std::move(map_updates_.front());
        map_updates_.pop_front();
        t.context = {session_, run_, epoch_, std::uint16_t(dense ? 9 : 3), 1, ++map_sequence_};
        map_transfer_ = std::move(t);
    }
    // Encoding is lazy and bounded to one negotiated payload. Pose/IMU queues
    // have already been serviced; backpressure never allocates a full map queue.
    if (tokens_ < header_size + payload_limit_)
        return {};
    auto &t = *map_transfer_;
    const auto &u = t.update;
    MapDelta d;
    d.context = t.context;
    d.sync = map_sync_;
    d.revision = u.revision;
    d.id = u.id;
    d.pose = u.node.pose;
    d.pose.id = u.id;
    d.action = u.action;
    if (u.action == MapAction::Upsert && dense && u.node.points)
    {
        if (u.node.points->size() > dense_point_limit)
            throw std::runtime_error("Map node exceeds per-node wire limit");
        d.total = std::uint32_t(u.node.points->size());
        d.offset = std::uint32_t(t.offset);
        const auto n = std::min<std::size_t>((payload_limit_ - map_delta_header_size) / 16,
                                             d.total - t.offset);
        d.points.assign(u.node.points->begin() + t.offset, u.node.points->begin() + t.offset + n);
        t.offset += n;
    }
    Frame f;
    f.type = MapDeltaData;
    f.timestamp_ns = t.timestamp;
    f.payload = encode_map_delta(d);
    if (t.offset == d.total)
    {
        t.sent = true;
        t.sent_at = now;
    }
    tokens_ -= header_size + f.payload.size();
    control_burst_ = 0;
    ++transmitted_;
    return f;
}
} // namespace svio
