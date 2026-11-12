/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "transport.hpp"
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>

namespace svio
{
struct CommandTask
{
    std::string name, token;
    std::uint64_t operation = 0;
    Json args = Json::object();
};
// Thread-safe semantic publisher; protocol work is performed on the link thread.
// The command callback must only enqueue work, never run an estimator or block.
class Server
{
  public:
    explicit Server(std::function<void(const CommandTask &)> command);
    void disconnect();
    void receive(const Frame &);
    void tick();
    std::optional<Frame> next_frame();
    void state(Json values);
    Json status() const;
    void descriptor(const std::string &section, Json values);
    void run(std::uint64_t id, std::uint32_t epoch, const std::string &reason);
    void complete(std::uint64_t operation, bool success, const std::string &message,
                  Json values = Json::object());
    bool due(std::uint16_t stream, std::uint64_t source_time = 0) const;
    void pose(Pose value, std::uint64_t timestamp);
    void imu(std::uint64_t timestamp, std::uint64_t source_sequence,
             const std::array<float, 3> &gyro, const std::array<float, 3> &accel);
    void cloud(std::uint16_t stream, std::vector<Point> points, std::uint64_t timestamp);
    bool keyframes(std::vector<KeyframePose> poses, std::uint64_t revision,
                   std::uint64_t timestamp);
    bool dense_cloud(std::vector<DensePoint> points, std::uint64_t revision,
                     std::uint64_t timestamp);
    void map(std::shared_ptr<const MapSnapshot>, std::uint64_t timestamp);
    void features(Features value, std::uint64_t timestamp);
    void image(ImagePart value, std::uint64_t timestamp);

  private:
    struct Subscription
    {
        bool enabled = false;
        double hz = 0;
        std::size_t points = 128;
        std::uint64_t last = 0, next_due = 0, sample = 0, source_next = 0;
        unsigned batch_ms = 50;
    };
    struct Op
    {
        CommandTask task;
        Json signature, reply, result;
        std::uint64_t time = 0, session = 0;
        std::uint32_t request = 0;
        bool done = false;
    };
    struct TimedImu
    {
        std::uint64_t time, sequence;
        ImuSample value;
    };
    Context context(std::uint16_t stream);
    bool allowed(std::uint16_t stream, std::uint64_t now) const;
    void queue_json(std::uint16_t type, Json values, std::uint32_t request = 0);
    void queue_data(std::uint16_t stream, std::deque<Frame> frames);
    void flush_imu();
    void clear_session();
    void reset_map_sync();
    void map_feedback(const Json &);
    std::optional<Frame> next_map_frame();
    struct MapUpdate
    {
        MapAction action = MapAction::Reset;
        std::uint64_t id = 0, revision = 0;
        MapNode node;
    };
    struct MapTransfer
    {
        MapUpdate update;
        Context context;
        std::size_t offset = 0;
        bool sent = false;
        std::uint64_t sent_at = 0, timestamp = 0;
    };
    std::shared_ptr<const MapSnapshot> map_, planned_map_;
    std::map<std::uint64_t, MapNode> map_known_;
    std::deque<MapUpdate> map_updates_;
    std::optional<MapTransfer> map_transfer_;
    std::uint64_t map_sync_ = 0, map_sequence_ = 0, map_time_ = 0;
    bool map_reset_ = true, map_paused_ = false;
    Json status_locked() const;
    mutable std::mutex mutex_;
    std::function<void(const CommandTask &)> command_;
    std::string boot_;
    std::uint64_t session_ = 0, run_ = 0, last_seen_ = 0, last_status_ = 0, last_tokens_ = 0,
                  imu_started_ = 0;
    std::uint32_t epoch_ = 0;
    std::string nonce_;
    Json hello_reply_, state_ = {{"state", "IDLE"}}, descriptors_ = Json::object();
    std::map<std::uint16_t, Subscription> subscriptions_;
    std::deque<Frame> control_;
    std::map<std::uint16_t, std::deque<Frame>> data_;
    std::deque<TimedImu> imu_;
    std::deque<Op> operations_;
    std::size_t payload_limit_ = 4096, budget_ = 65536;
    double tokens_ = 16384;
    unsigned control_burst_ = 0, round_robin_ = 2;
    std::uint64_t dropped_ = 0, transmitted_ = 0, received_ = 0, request_window_ = 0;
    unsigned requests_in_window_ = 0;
};
// Transport pumping stays outside the algorithm. On unplug, destroy/reset this
// object and call Server::disconnect(); the Server's run and operation history survive.
class ServerLink
{
  public:
    explicit ServerLink(Server &server) : server_(server) {}
    void step(Transport &);
    std::uint64_t tx_bytes = 0, rx_bytes = 0;

  private:
    Server &server_;
    Decoder decoder_;
    Transmitter tx_;
};
} // namespace svio
