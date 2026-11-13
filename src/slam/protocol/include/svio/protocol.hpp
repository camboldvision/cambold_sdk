/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace svio
{
using Json = nlohmann::json;
using Bytes = std::vector<std::uint8_t>;
constexpr std::size_t header_size = 40, max_payload = 65536, json_limit = 4096;
constexpr std::uint16_t json_flag = 1;
// Keep wire IDs stable; 4, 6 and 8 are retired, never reassigned.
inline constexpr std::array<std::uint16_t, 6> active_streams{1, 2, 3, 5, 7, 9};
inline constexpr unsigned preview_max_hz = 15;
inline constexpr unsigned pose_max_hz = 200;
constexpr unsigned stream_max_hz(std::uint16_t id)
{
    return id == 2 ? 200 : id == 1 ? pose_max_hz : id == 3 ? 10 : id == 9 ? 2 : preview_max_hz;
}
enum Type : std::uint16_t
{
    Hello = 0x0001,
    HelloReply = 0x8001,
    Descriptor = 0x0002,
    DescriptorReply = 0x8002,
    Ping = 0x0003,
    Pong = 0x8003,
    TimeSync = 0x0004,
    TimeSyncReply = 0x8004,
    Subscribe = 0x0010,
    SubscribeReply = 0x8010,
    Command = 0x0020,
    CommandReply = 0x8020,
    Status = 0x0021,
    StatusReply = 0x8021,
    Operation = 0x0022,
    OperationReply = 0x8022,
    MapAck = 0x0023,
    PoseData = 0x0100,
    ImuData = 0x0101,
    PointsData = 0x0102,
    FeaturesData = 0x0103,
    ImageData = 0x0104,
    DenseCloudData = 0x0105,
    KeyframesData = 0x0106,
    MapDeltaData = 0x0107,
    StatusEvent = 0x0200,
    EpochEvent = 0x0201,
    OperationEvent = 0x0202,
    LogEvent = 0x0203,
    Error = 0xffff
};
struct Frame
{
    std::uint16_t type = 0, flags = 0;
    std::uint32_t sequence = 0, request_id = 0;
    std::uint64_t timestamp_ns = 0;
    Bytes payload;
};
std::uint64_t monotonic_ns();
std::uint64_t random_id();
std::uint32_t crc32c(const std::uint8_t *, std::size_t);
Bytes encode_frame(const Frame &);
Json parse_json(const Frame &);
Frame json_frame(std::uint16_t type, const Json &, std::uint32_t request = 0);
std::uint64_t json_u64(const Json &, const char *key);
std::string id_string(std::uint64_t);

class Decoder
{
  public:
    std::vector<Frame> feed(const std::uint8_t *, std::size_t);
    std::vector<Frame> feed(const Bytes &b)
    {
        return feed(b.data(), b.size());
    }
    void reset();
    std::uint64_t crc_errors = 0, header_errors = 0, discarded_bytes = 0;

  private:
    void parse(std::vector<Frame> &);
    Bytes buffer_;
    std::uint64_t pending_since_ = 0;
};

class Writer
{
  public:
    Bytes bytes;
    void u8(std::uint8_t);
    void u16(std::uint16_t);
    void u32(std::uint32_t);
    void u64(std::uint64_t);
    void f32(float);
    void f64(double);
};
class Reader
{
  public:
    explicit Reader(const Bytes &b) : data_(b) {}
    std::uint8_t u8();
    std::uint16_t u16();
    std::uint32_t u32();
    std::uint64_t u64();
    float f32();
    double f64();
    std::size_t remaining() const
    {
        return data_.size() - offset_;
    }
    void done() const;
    Bytes rest();

  private:
    const Bytes &data_;
    std::size_t offset_ = 0;
};
struct Context
{
    std::uint64_t session = 0, run = 0;
    std::uint32_t epoch = 0;
    std::uint16_t stream = 0, schema = 1;
    std::uint64_t sample = 0;
};
void write_context(Writer &, const Context &);
Context read_context(Reader &);
struct Pose
{
    Context context;
    std::array<double, 3> position{};
    std::array<double, 4> quaternion{0, 0, 0, 1};
    std::array<float, 3> velocity{};
    std::uint32_t flags = 3;
    std::uint64_t camera_frame = 0;
};
Bytes encode_pose(const Pose &);
Pose decode_pose(const Bytes &);
struct ImuSample
{
    std::uint32_t dt = 0;
    std::array<float, 3> gyro{}, accel{};
    std::uint32_t flags = 1;
};
struct ImuBatch
{
    Context context;
    std::vector<ImuSample> samples;
};
Bytes encode_imu(const ImuBatch &);
ImuBatch decode_imu(const Bytes &);
struct Point
{
    std::uint64_t id = 0;
    std::array<float, 3> xyz{};
    std::uint16_t flags = 1, camera = 0xffff;
};
struct Cloud
{
    Context context;
    std::uint8_t source = 0, mode = 0;
    std::uint16_t index = 0, chunks = 1;
    std::uint32_t revision = 0, total = 0;
    std::vector<Point> points;
};
Bytes encode_cloud(const Cloud &);
Cloud decode_cloud(const Bytes &);
constexpr std::uint32_t keyframe_limit = 8192;
struct KeyframePose
{
    std::uint64_t id = 0;
    std::array<float, 3> position{};
    std::array<float, 4> quaternion{0, 0, 0, 1}; // Hamilton xyzw, IMU -> map.
};
struct Keyframes
{
    Context context;
    std::uint64_t revision = 0;
    std::uint16_t index = 0, chunks = 1;
    std::uint32_t total = 0;
    std::vector<KeyframePose> poses;
};
Bytes encode_keyframes(const Keyframes &);
Keyframes decode_keyframes(const Bytes &);
constexpr std::uint32_t dense_point_limit = 131072;
struct DensePoint
{
    std::array<float, 3> xyz{};
    std::array<std::uint8_t, 3> rgb{};
};
struct DenseCloud
{
    Context context;
    std::uint64_t revision = 0;
    std::uint16_t index = 0, chunks = 1;
    std::uint32_t total = 0;
    std::vector<DensePoint> points;
};
Bytes encode_dense_cloud(const DenseCloud &);
DenseCloud decode_dense_cloud(const Bytes &);
// Incremental map: geometry is expressed in each node's IMU frame. Metadata
// snapshots share immutable point arrays; only changed nodes cross the link.
struct MapNode
{
    KeyframePose pose;
    std::shared_ptr<const std::vector<DensePoint>> points;
};
struct MapSnapshot
{
    std::uint64_t revision = 0;
    std::map<std::uint64_t, MapNode> nodes;
};
enum class MapAction : std::uint8_t
{
    Reset = 0,
    Upsert = 1,
    Pose = 2,
    Erase = 3
};
constexpr std::size_t map_delta_header_size = 96;
constexpr std::size_t map_display_point_limit = 16 * 1024 * 1024, map_display_node_limit = 65536;
struct MapDelta
{
    Context context;
    std::uint64_t sync = 0, revision = 0, id = 0;
    MapAction action = MapAction::Reset;
    std::uint32_t offset = 0, total = 0;
    KeyframePose pose;
    std::vector<DensePoint> points;
};
Bytes encode_map_delta(const MapDelta &);
MapDelta decode_map_delta(const Bytes &);
struct Feature
{
    std::uint64_t id = 0;
    float u = 0, v = 0;
    std::uint16_t age = 0, flags = 1;
};
struct Features
{
    Context context;
    std::uint16_t source = 0, camera = 0;
    std::uint64_t camera_frame = 0;
    std::uint16_t width = 0, height = 0, index = 0, chunks = 1;
    std::vector<Feature> points;
};
Bytes encode_features(const Features &);
Features decode_features(const Bytes &);
struct ImagePart
{
    Context context;
    std::uint64_t camera_frame = 0;
    std::uint16_t source = 0, camera = 0, width = 0, height = 0;
    std::uint8_t encoding = 1, flags = 0;
    std::uint32_t total = 0, offset = 0;
    Bytes data;
};
Bytes encode_image(const ImagePart &);
ImagePart decode_image(const Bytes &);
const char *stream_name(std::uint16_t);
std::uint16_t stream_id(const std::string &);
std::string base64(const Bytes &);
} // namespace svio
