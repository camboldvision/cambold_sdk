/* SPDX-License-Identifier: Apache-2.0 */
#include "svio/server.hpp"
#include <algorithm>
#include <cmath>
#include <iterator>
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
    float plus = 0, minus = 0;
    for (unsigned i = 0; i < 4; ++i)
    {
        plus = std::max(plus, std::abs(a.quaternion[i] - b.quaternion[i]));
        minus = std::max(minus, std::abs(a.quaternion[i] + b.quaternion[i]));
    }
    return std::min(plus, minus) <= 1e-6f;
}
bool same_graph(const MapSnapshot &a, const MapSnapshot &b)
{
    if (a.nodes.size() != b.nodes.size())
        return false;
    auto x = a.nodes.begin(), y = b.nodes.begin();
    for (; x != a.nodes.end(); ++x, ++y)
        if (x->first != y->first || !same_pose(x->second.pose, y->second.pose))
            return false;
    return true;
}
} // namespace
void Server::reset_map_sync()
{
    map_sync_ = random_id();
    map_reset_ = true;
    map_paused_ = false;
    map_known_.clear();
    planned_map_.reset();
    planned_graph_.reset();
    map_updates_.clear();
    map_transfers_.clear();
}
void Server::map(std::shared_ptr<const MapSnapshot> snapshot, std::uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot == map_)
        return;
    if (snapshot && map_ && snapshot->geometry_generation != map_->geometry_generation)
        reset_map_sync();
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
    const auto sample = json_u64(j, "sample_id");
    auto found = std::find_if(map_transfers_.begin(), map_transfers_.end(),
                              [&](const auto &t) { return t.context.sample == sample; });
    if (found == map_transfers_.end())
        return;
    if (action == "retry")
    {
        // Samples form an ordered stream. Replay the missing transfer and the
        // bounded suffix the receiver discarded, preserving every sample ID.
        for (auto it = found; it != map_transfers_.end(); ++it)
        {
            it->offset = it->wire_bytes = 0;
            it->sent = false;
        }
        return;
    }
    if (action == "pause")
    {
        map_paused_ = true;
        map_transfers_.clear();
        map_updates_.clear();
        return;
    }
    if (action != "ack" || !found->sent)
        return;
    const auto count = std::size_t(std::distance(map_transfers_.begin(), found)) + 1;
    if ((!map_snapshots_ && count != 1) ||
        std::any_of(map_transfers_.begin(), std::next(found), [](const auto &t) { return !t.sent; }))
        return;
    // v2 ACKs are cumulative. Kernel/USB ordering and the receiver's strict
    // sample check make a window possible without a round trip per node.
    for (std::size_t i = 0; i < count; ++i)
    {
        const auto &t = map_transfers_.front();
        const auto &u = t.update;
        if (u.action == MapAction::Reset)
        {
            map_known_.clear();
            map_reset_ = false;
            planned_map_.reset();
        }
        else if (u.action == MapAction::Graph)
        {
            for (auto it = map_known_.begin(); it != map_known_.end();)
                if (!u.snapshot->nodes.count(it->first)) it = map_known_.erase(it);
                else ++it;
            for (const auto &p : u.snapshot->nodes)
                map_known_[p.first].pose = p.second.pose;
        }
        else if (u.action == MapAction::Erase)
            map_known_.erase(u.id);
        else if (u.action == MapAction::PoseBatch)
            for (const auto &pose : t.pose_updates)
                map_known_[pose.id] = pose.node;
        else
            map_known_[u.id] = u.node;
        map_transfers_.pop_front();
    }
}
std::optional<Frame> Server::next_map_frame()
{
    const auto enabled = [&](unsigned id)
    {
        auto it = subscriptions_.find(id);
        return it != subscriptions_.end() && it->second.enabled;
    };
    const bool dense = enabled(9);
    if (!session_ || !map_session_visible_ || !map_ || map_paused_ || (!dense && !enabled(3)))
        return {};
    const auto now = monotonic_ns();
    if (!map_transfers_.empty() && map_transfers_.front().sent &&
        now - map_transfers_.front().sent_at >= 2000000000ull)
        for (auto &t : map_transfers_)
        {
            t.offset = t.wire_bytes = 0;
            t.sent = false;
        }
    auto current = std::find_if(map_transfers_.begin(), map_transfers_.end(),
                                [](const auto &t) { return !t.sent; });
    if (current == map_transfers_.end())
    {
        if (map_reset_)
        {
            if (!map_transfers_.empty()) return {};
            map_updates_ = {{MapAction::Reset, 0, map_->revision, {}}};
        }
        else if (map_ != planned_map_)
        {
            // Replan only against the latest snapshot and the bounded promised
            // window. Never drain a FIFO of obsolete map revisions.
            map_updates_.clear();
            if (map_snapshots_)
            {
                if (!planned_graph_ || !same_graph(*planned_graph_, *map_))
                    map_updates_.push_back({MapAction::Graph, 0, map_->revision, {}, map_});
                if (dense)
                {
                    std::map<std::uint64_t, std::shared_ptr<const std::vector<DensePoint>>> promised;
                    for (const auto &t : map_transfers_)
                        if (t.update.action == MapAction::Upsert)
                            promised[t.update.id] = t.update.node.points;
                    for (auto it = map_->nodes.rbegin(); it != map_->nodes.rend(); ++it)
                    {
                        const auto queued = promised.find(it->first);
                        const auto old = map_known_.find(it->first);
                        const auto points = queued != promised.end() ? queued->second :
                            old == map_known_.end() ? nullptr : old->second.points;
                        if (points != it->second.points)
                            map_updates_.push_back({MapAction::Upsert, it->first, map_->revision, it->second});
                    }
                }
            }
            else
            {
                for (const auto &old : map_known_)
                    if (!map_->nodes.count(old.first))
                        map_updates_.push_back({MapAction::Erase, old.first, map_->revision, {}});
                std::deque<MapUpdate> poses;
                for (auto it = map_->nodes.rbegin(); it != map_->nodes.rend(); ++it)
                {
                    const auto old = map_known_.find(it->first);
                    if (old == map_known_.end() || (dense && old->second.points != it->second.points))
                        map_updates_.push_back({MapAction::Upsert, it->first, map_->revision, it->second});
                    else if (!same_pose(old->second.pose, it->second.pose))
                        poses.push_back({MapAction::Pose, it->first, map_->revision, it->second});
                }
                map_updates_.insert(map_updates_.end(), std::make_move_iterator(poses.begin()),
                                    std::make_move_iterator(poses.end()));
            }
            planned_map_ = map_;
        }
        if (map_updates_.empty()) return {};
        std::size_t pending_bytes = 0;
        for (const auto &t : map_transfers_) pending_bytes += t.wire_bytes;
        const bool graph = map_updates_.front().action == MapAction::Graph;
        if (!map_transfers_.empty() && (!map_snapshots_ ||
            (!graph && (map_transfers_.size() >= 64 || pending_bytes >= 256 * 1024))))
            return {};
        MapTransfer t;
        t.timestamp = map_time_;
        t.update = std::move(map_updates_.front());
        map_updates_.pop_front();
        if (t.update.action == MapAction::Graph)
        {
            if (t.update.snapshot->nodes.size() > map_display_node_limit)
                throw std::runtime_error("Map exceeds display node limit");
            t.graph_poses.reserve(t.update.snapshot->nodes.size());
            for (const auto &p : t.update.snapshot->nodes)
            {
                t.graph_poses.push_back(p.second.pose);
                t.graph_poses.back().id = p.first;
            }
            planned_graph_ = t.update.snapshot;
        }
        else if (map_pose_batches_ && t.update.action == MapAction::Pose)
        {
            const auto limit = (payload_limit_ - map_delta_header_size) / map_pose_wire_size;
            t.pose_updates.push_back(std::move(t.update));
            while (!map_updates_.empty() && map_updates_.front().action == MapAction::Pose &&
                   t.pose_updates.size() < limit)
            {
                t.pose_updates.push_back(std::move(map_updates_.front()));
                map_updates_.pop_front();
            }
            t.update = {MapAction::PoseBatch, 0, t.pose_updates.front().revision, {}};
        }
        t.context = {session_, run_, epoch_, std::uint16_t(dense ? 9 : 3), 1, ++map_sequence_};
        map_transfers_.push_back(std::move(t));
        current = std::prev(map_transfers_.end());
    }
    auto &t = *current;
    const auto &u = t.update;
    MapDelta d;
    d.context = t.context;
    d.sync = map_sync_;
    d.revision = u.revision;
    d.id = u.id;
    d.pose = u.node.pose;
    d.pose.id = u.id;
    d.action = u.action;
    auto metadata_size = header_size + map_delta_header_size +
        (u.action == MapAction::PoseBatch ? t.pose_updates.size() * map_pose_wire_size : 0);
    // Wait for a complete chunk's budget. Emitting one point/pose whenever a
    // few tokens arrive wastes bandwidth on headers under sustained map load.
    if (u.action == MapAction::Graph)
        metadata_size += std::min((payload_limit_ - map_delta_header_size) / map_pose_wire_size,
                                   t.graph_poses.size() - t.offset) * map_pose_wire_size;
    else if (u.action == MapAction::Upsert && dense && u.node.points)
        metadata_size += std::min((payload_limit_ - map_delta_header_size) / 16,
                                   u.node.points->size() - t.offset) * 16;
    if (tokens_ < metadata_size) return {};
    const auto available = std::min<double>(payload_limit_, tokens_ - header_size);
    if (u.action == MapAction::Graph)
    {
        d.total = std::uint32_t(t.graph_poses.size());
        d.offset = std::uint32_t(t.offset);
        const auto n = std::min<std::size_t>((std::size_t(available) - map_delta_header_size) / map_pose_wire_size,
                                             d.total - t.offset);
        if (d.total != t.offset && !n) return {};
        d.poses.assign(t.graph_poses.begin() + t.offset, t.graph_poses.begin() + t.offset + n);
    }
    else if (u.action == MapAction::PoseBatch)
    {
        d.total = std::uint32_t(t.pose_updates.size());
        for (const auto &pose : t.pose_updates)
        {
            d.poses.push_back(pose.node.pose);
            d.poses.back().id = pose.id;
        }
    }
    else if (u.action == MapAction::Upsert && dense && u.node.points)
    {
        if (u.node.points->size() > dense_point_limit)
            throw std::runtime_error("Map node exceeds per-node wire limit");
        d.total = std::uint32_t(u.node.points->size());
        d.offset = std::uint32_t(t.offset);
        const auto n = std::min<std::size_t>((std::size_t(available) - map_delta_header_size) / 16, d.total - t.offset);
        if (d.total != t.offset && !n) return {};
        d.points.assign(u.node.points->begin() + t.offset, u.node.points->begin() + t.offset + n);
    }
    Frame f;
    f.type = MapDeltaData;
    f.timestamp_ns = t.timestamp;
    f.payload = encode_map_delta(d);
    t.offset += u.action == MapAction::Graph ? d.poses.size() : d.points.size();
    if ((u.action != MapAction::Upsert && u.action != MapAction::Graph) || t.offset == d.total)
    {
        t.sent = true;
        t.sent_at = now;
    }
    const auto bytes = header_size + f.payload.size();
    t.wire_bytes += bytes;
    tokens_ -= bytes;
    control_burst_ = 0;
    ++transmitted_;
    return f;
}
} // namespace svio
