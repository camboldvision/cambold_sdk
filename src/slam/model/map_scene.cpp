/* SPDX-License-Identifier: Apache-2.0 */
#include "scene.hpp"
#include <algorithm>
#include <stdexcept>

namespace cambold::detail
{
void Scene::reset_map()
{
    map_assembly_.reset();
    map_feedback_.reset();
    map_sync_ = map_sample_ = 0;
    map_points_ = 0;
    map_nodes_.clear();
    map_poses_.clear();
    map_snapshot_stream_ = false;
    map_graph_revision_ = 0;
    map_changes_.clear();
    map_reset_pending_ = false;
    state_["received_map_points"] = 0;
    state_["received_keyframes"] = 0;
    state_.erase("received_map_revision");
    state_.erase("map_sync_error");
}
std::optional<svio::Json> Scene::map_feedback()
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::move(map_feedback_);
    map_feedback_.reset();
    return result;
}
void Scene::map_delta(const svio::MapDelta &d)
{
    if (!context(d.context))
        return;
    const auto feedback = [&](const char *action, std::uint64_t sample = 0)
    {
        map_feedback_ = {{"sync_id", svio::id_string(d.sync)},
                         {"sample_id", svio::id_string(sample ? sample : d.context.sample)},
                         {"action", action}};
    };
    const auto end = d.action == svio::MapAction::Graph
                     ? d.offset + d.poses.size() == d.total
                     : d.action == svio::MapAction::PoseBatch ||
                     d.offset + d.points.size() == d.total;
    if (d.context.sample <= map_sample_)
    {
        if (d.sync == map_sync_ && (map_snapshot_stream_ || (d.context.sample == map_sample_ && end)))
            feedback("ack", map_snapshot_stream_ ? map_sample_ : d.context.sample);
        return;
    }
    if (d.action == svio::MapAction::Reset)
    {
        map_sync_ = d.sync;
        map_nodes_.clear();
        map_poses_.clear();
        map_graph_revision_ = 0;
        map_snapshot_stream_ = false;
        map_changes_.clear();
        map_points_ = 0;
        map_reset_pending_ = true;
        map_assembly_.reset();
        state_.erase("map_sync_error");
    }
    else
    {
        if (d.sync != map_sync_)
        {
            feedback("resync");
            return;
        }
        if (d.action == svio::MapAction::Graph)
            map_snapshot_stream_ = true;
        if (map_snapshot_stream_ && d.context.sample != map_sample_ + 1)
        {
            feedback("retry", map_sample_ + 1);
            return;
        }
        if (d.action == svio::MapAction::Graph)
        {
            if (d.offset == 0)
            {
                map_assembly_ = MapAssembly{d, {}, {}};
                map_assembly_->first.poses.clear();
                map_assembly_->poses.reserve(d.total);
            }
            if (!map_assembly_ || map_assembly_->first.action != d.action ||
                map_assembly_->first.context.sample != d.context.sample ||
                map_assembly_->first.revision != d.revision ||
                map_assembly_->first.total != d.total || map_assembly_->poses.size() != d.offset)
            {
                map_assembly_.reset();
                feedback("retry");
                return;
            }
            auto &poses = map_assembly_->poses;
            for (const auto &p : d.poses)
            {
                if (!poses.empty() && p.id <= poses.back().id)
                {
                    map_assembly_.reset();
                    feedback("retry");
                    return;
                }
                poses.push_back(p);
            }
            if (!end) return;
            std::map<std::uint64_t, svio::KeyframePose> next;
            for (const auto &p : poses) next.emplace(p.id, p);
            // One mutex-protected commit feeds one UI delta. Incomplete graph
            // chunks never change the displayed transforms or membership.
            for (auto it = map_nodes_.begin(); it != map_nodes_.end();)
                if (!next.count(it->first))
                {
                    map_changes_[it->first] = {{"action", "erase"}, {"id", svio::id_string(it->first)}};
                    map_points_ -= it->second;
                    it = map_nodes_.erase(it);
                }
                else ++it;
            for (const auto &p : next)
            {
                if (!map_nodes_.count(p.first))
                {
                    map_nodes_[p.first] = 0;
                    map_changes_[p.first] = {{"action", "upsert"}, {"id", svio::id_string(p.first)},
                        {"p", p.second.position}, {"q", p.second.quaternion}, {"count", 0}, {"xyzrgb16", ""}};
                }
                else
                {
                    auto change = map_changes_.find(p.first);
                    if (change != map_changes_.end() && change->second["action"] == "upsert")
                    {
                        change->second["p"] = p.second.position;
                        change->second["q"] = p.second.quaternion;
                    }
                    else
                        map_changes_[p.first] = {{"action", "pose"}, {"id", svio::id_string(p.first)},
                            {"p", p.second.position}, {"q", p.second.quaternion}};
                }
            }
            map_poses_ = std::move(next);
            map_graph_revision_ = d.revision;
            map_assembly_.reset();
        }
        else if (d.action == svio::MapAction::Upsert)
        {
            if (d.offset == 0)
            {
                map_assembly_ = MapAssembly{d, {}};
                map_assembly_->first.points.clear();
                map_assembly_->points.reserve(d.total);
            }
            if (!map_assembly_)
            {
                feedback("retry");
                return;
            }
            auto &a = *map_assembly_;
            const auto &f = a.first;
            if (f.action != d.action || f.context.sample != d.context.sample || f.id != d.id || f.total != d.total ||
                f.revision != d.revision || f.pose.position != d.pose.position ||
                f.pose.quaternion != d.pose.quaternion || d.offset != a.points.size())
            {
                map_assembly_.reset();
                feedback("retry");
                return;
            }
            a.points.insert(a.points.end(), d.points.begin(), d.points.end());
            if (!end)
                return;
            const auto previous = map_nodes_.find(d.id);
            const auto old = previous == map_nodes_.end() ? 0 : previous->second;
            if (map_points_ - old + d.total > svio::map_display_point_limit ||
                (previous == map_nodes_.end() && map_nodes_.size() >= svio::map_display_node_limit))
            {
                state_["map_sync_error"] = "地图预览达到内存上限，点云同步已暂停；板端建图继续运行";
                log_locked(state_["map_sync_error"]);
                map_assembly_.reset();
                feedback("pause");
                return;
            }
            svio::Writer packed;
            packed.bytes.reserve(d.total * 16);
            for (const auto &p : a.points)
            {
                for (float x : p.xyz)
                    packed.f32(x);
                for (auto x : p.rgb)
                    packed.u8(x);
                packed.u8(0);
            }
            const auto pose = map_poses_.find(d.id);
            if (map_snapshot_stream_ && pose == map_poses_.end())
            {
                map_assembly_.reset();
                feedback("resync");
                return;
            }
            const auto &display_pose = map_snapshot_stream_ ? pose->second : d.pose;
            map_changes_[d.id] = {{"action", "upsert"},   {"id", svio::id_string(d.id)},
                                  {"p", display_pose.position}, {"q", display_pose.quaternion},
                                  {"count", d.total},     {"xyzrgb16", svio::base64(packed.bytes)}};
            map_points_ = map_points_ - old + d.total;
            map_nodes_[d.id] = d.total;
            map_assembly_.reset();
        }
        else if (d.action == svio::MapAction::Pose || d.action == svio::MapAction::PoseBatch)
        {
            const auto apply_pose = [&](const svio::KeyframePose &pose)
            {
                auto it = map_changes_.find(pose.id);
                if (it != map_changes_.end() && it->second["action"] == "upsert")
                {
                    it->second["p"] = pose.position;
                    it->second["q"] = pose.quaternion;
                }
                else
                    map_changes_[pose.id] = {{"action", "pose"},
                                              {"id", svio::id_string(pose.id)},
                                              {"p", pose.position},
                                              {"q", pose.quaternion}};
            };
            const bool missing = d.action == svio::MapAction::Pose
                ? !map_nodes_.count(d.id)
                : std::any_of(d.poses.begin(), d.poses.end(),
                              [&](const auto &pose) { return !map_nodes_.count(pose.id); });
            if (missing)
            {
                feedback("resync");
                return;
            }
            if (d.action == svio::MapAction::Pose)
                apply_pose(d.pose);
            else
                for (const auto &pose : d.poses)
                    apply_pose(pose);
        }
        else if (d.action == svio::MapAction::Erase)
        {
            const auto it = map_nodes_.find(d.id);
            if (it != map_nodes_.end())
            {
                map_points_ -= it->second;
                map_nodes_.erase(it);
            }
            map_changes_[d.id] = {{"action", "erase"}, {"id", svio::id_string(d.id)}};
        }
    }
    map_sample_ = d.context.sample;
    state_["received_map_points"] = map_points_;
    state_["received_keyframes"] = map_nodes_.size();
    state_["received_map_revision"] = svio::id_string(std::max(d.revision, map_graph_revision_));
    feedback("ack");
}
} // namespace cambold::detail
