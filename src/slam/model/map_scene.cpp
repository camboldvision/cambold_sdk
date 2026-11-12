/* SPDX-License-Identifier: Apache-2.0 */
#include "scene.hpp"
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
    map_changes_.clear();
    map_reset_pending_ = false;
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
    const auto feedback = [&](const char *action)
    {
        map_feedback_ = {{"sync_id", svio::id_string(d.sync)},
                         {"sample_id", svio::id_string(d.context.sample)},
                         {"action", action}};
    };
    const auto end = d.offset + d.points.size() == d.total;
    if (d.context.sample <= map_sample_)
    {
        if (d.sync == map_sync_ && d.context.sample == map_sample_ && end)
            feedback("ack");
        return;
    }
    if (d.action == svio::MapAction::Reset)
    {
        map_sync_ = d.sync;
        map_nodes_.clear();
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
        if (d.action == svio::MapAction::Upsert)
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
            if (f.context.sample != d.context.sample || f.id != d.id || f.total != d.total ||
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
            map_changes_[d.id] = {{"action", "upsert"},   {"id", svio::id_string(d.id)},
                                  {"p", d.pose.position}, {"q", d.pose.quaternion},
                                  {"count", d.total},     {"xyzrgb16", svio::base64(packed.bytes)}};
            map_points_ = map_points_ - old + d.total;
            map_nodes_[d.id] = d.total;
            map_assembly_.reset();
        }
        else if (d.action == svio::MapAction::Pose)
        {
            if (!map_nodes_.count(d.id))
            {
                feedback("resync");
                return;
            }
            auto it = map_changes_.find(d.id);
            if (it != map_changes_.end() && it->second["action"] == "upsert")
            {
                it->second["p"] = d.pose.position;
                it->second["q"] = d.pose.quaternion;
            }
            else
                map_changes_[d.id] = {{"action", "pose"},
                                      {"id", svio::id_string(d.id)},
                                      {"p", d.pose.position},
                                      {"q", d.pose.quaternion}};
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
    state_["received_map_revision"] = svio::id_string(d.revision);
    feedback("ack");
}
} // namespace cambold::detail
