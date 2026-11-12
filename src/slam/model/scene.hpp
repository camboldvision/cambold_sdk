/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "svio/protocol.hpp"
#include <deque>
#include <map>
#include <mutex>
#include <optional>

namespace cambold::detail
{
class Scene
{
  public:
    void accept(const svio::Frame &);
    svio::Json poll();
    svio::Json snapshot() const;
    void reset();
    void connection(const std::string &, const std::string &detail = "");
    void log(const std::string &);
    void counters(std::uint64_t rx, std::uint64_t tx, std::uint64_t crc_errors);
    void round_trip(double rtt_ns);
    void clear();
    bool needs_status() const;
    void catalog_requested(const std::string &token);
    bool catalog_next(const std::string &token, std::size_t offset);
    std::optional<svio::Json> map_feedback();

  private:
    void map_delta(const svio::MapDelta &);
    void reset_map();
    struct MapAssembly
    {
        svio::MapDelta first;
        std::vector<svio::DensePoint> points;
    };
    std::optional<MapAssembly> map_assembly_;
    std::optional<svio::Json> map_feedback_;
    std::uint64_t map_sync_ = 0, map_sample_ = 0;
    std::size_t map_points_ = 0;
    std::map<std::uint64_t, std::size_t> map_nodes_;
    std::map<std::uint64_t, svio::Json> map_changes_;
    bool map_reset_pending_ = false;
    struct KeyframeAssembly
    {
        std::uint64_t sample = 0, revision = 0, time = 0, started = 0;
        std::uint32_t total = 0;
        std::vector<std::optional<svio::Keyframes>> chunks;
    };
    struct FeatureAssembly
    {
        std::uint64_t sample = 0, time = 0, started = 0;
        std::vector<std::optional<svio::Features>> chunks;
    };
    struct ImageAssembly
    {
        svio::ImagePart first;
        std::uint64_t time = 0, started = 0;
        svio::Bytes bytes;
    };
    struct FeatureSet
    {
        svio::Features value;
        std::uint64_t time = 0;
    };
    struct DenseAssembly
    {
        std::uint64_t sample = 0, revision = 0, time = 0, started = 0;
        std::uint32_t total = 0;
        std::vector<std::optional<svio::DenseCloud>> chunks;
    };
    DenseAssembly dense_parts_;
    KeyframeAssembly keyframe_parts_;
    svio::Json dense_ = nullptr, keyframes_ = nullptr;
    void boundary(std::uint64_t, std::uint32_t);
    bool context(const svio::Context &);
    void log_locked(const std::string &);
    void refresh_overlay(unsigned camera);
    void catalog_page(const svio::Json &page);
    mutable std::mutex mutex_;
    std::uint64_t session_ = 0, run_ = 0, generation_ = 0, last_pose_time_ = 0;
    std::uint32_t epoch_ = 0;
    bool unknown_epoch_ = false;
    svio::Json state_ = {{"connection", "DISCONNECTED"}, {"state", "IDLE"}},
               descriptors_ = svio::Json::object(), pose_ = nullptr;
    svio::Json images_ = svio::Json::object(), logs_ = svio::Json::array(),
               imu_ = svio::Json::array();
    std::deque<std::uint64_t> pose_receipts_, image_receipts_;
    std::map<unsigned, FeatureAssembly> feature_parts_;
    std::map<unsigned, ImageAssembly> image_parts_;
    std::array<std::deque<FeatureSet>, 2> features_;
    std::array<svio::Json, 2> image_meta_;
    std::uint64_t invalid_ = 0;
    svio::Json map_catalog_ = {{"maps", svio::Json::array()}, {"loading", false}},
               catalog_pending_ = svio::Json::array();
    std::string catalog_token_;
    std::size_t catalog_next_requested_ = 0;
};
} // namespace cambold::detail
