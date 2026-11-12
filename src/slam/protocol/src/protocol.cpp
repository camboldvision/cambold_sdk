/* SPDX-License-Identifier: Apache-2.0 */
#include "svio/protocol.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <random>
#include <set>
#include <stdexcept>

namespace svio
{
namespace
{
constexpr std::uint8_t magic[] = {'S', 'V', 'I', 'O'};
template <class T> void finite(T v)
{
    if (!std::isfinite(v))
        throw std::runtime_error("Non-finite sample");
}
void require(bool ok, const char *text)
{
    if (!ok)
        throw std::runtime_error(text);
}
std::uint32_t get32(const std::uint8_t *p)
{
    return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) | (std::uint32_t(p[2]) << 16) |
           (std::uint32_t(p[3]) << 24);
}
} // namespace
std::uint64_t monotonic_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
std::uint64_t random_id()
{
    static std::mutex mutex;
    static std::mt19937_64 generator(std::random_device{}());
    std::lock_guard<std::mutex> lock(mutex);
    auto v = generator();
    return v ? v : 1;
}
std::string id_string(std::uint64_t value)
{
    return std::to_string(value);
}
std::uint32_t crc32c(const std::uint8_t *p, std::size_t size)
{
    std::uint32_t crc = 0xffffffffu;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= p[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc >> 1) ^ (0x82f63b78u & (0u - (crc & 1u)));
    }
    return crc ^ 0xffffffffu;
}
void Writer::u8(std::uint8_t v)
{
    bytes.push_back(v);
}
void Writer::u16(std::uint16_t v)
{
    u8(v & 255);
    u8(v >> 8);
}
void Writer::u32(std::uint32_t v)
{
    for (int i = 0; i < 4; ++i)
        u8((v >> (8 * i)) & 255);
}
void Writer::u64(std::uint64_t v)
{
    for (int i = 0; i < 8; ++i)
        u8((v >> (8 * i)) & 255);
}
void Writer::f32(float v)
{
    std::uint32_t u;
    std::memcpy(&u, &v, 4);
    u32(u);
}
void Writer::f64(double v)
{
    std::uint64_t u;
    std::memcpy(&u, &v, 8);
    u64(u);
}
std::uint8_t Reader::u8()
{
    require(remaining() >= 1, "Truncated payload");
    return data_[offset_++];
}
std::uint16_t Reader::u16()
{
    auto a = u8();
    return std::uint16_t(a) | (std::uint16_t(u8()) << 8);
}
std::uint32_t Reader::u32()
{
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i)
        v |= std::uint32_t(u8()) << (i * 8);
    return v;
}
std::uint64_t Reader::u64()
{
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
        v |= std::uint64_t(u8()) << (i * 8);
    return v;
}
float Reader::f32()
{
    auto u = u32();
    float v;
    std::memcpy(&v, &u, 4);
    finite(v);
    return v;
}
double Reader::f64()
{
    auto u = u64();
    double v;
    std::memcpy(&v, &u, 8);
    finite(v);
    return v;
}
void Reader::done() const
{
    require(remaining() == 0, "Unexpected payload suffix");
}
Bytes Reader::rest()
{
    Bytes b(data_.begin() + offset_, data_.end());
    offset_ = data_.size();
    return b;
}
Bytes encode_frame(const Frame &f)
{
    require(f.payload.size() <= max_payload && (f.flags & ~json_flag) == 0,
            "Invalid frame size/flags");
    Writer w;
    for (auto c : magic)
        w.u8(c);
    w.u8(1);
    w.u8(0);
    w.u16(header_size);
    w.u16(f.type);
    w.u16(f.flags);
    w.u32(static_cast<std::uint32_t>(f.payload.size()));
    w.u32(f.sequence);
    w.u32(f.request_id);
    w.u64(f.timestamp_ns);
    w.u32(crc32c(f.payload.data(), f.payload.size()));
    w.u32(crc32c(w.bytes.data(), w.bytes.size()));
    w.bytes.insert(w.bytes.end(), f.payload.begin(), f.payload.end());
    return std::move(w.bytes);
}
void Decoder::reset()
{
    buffer_.clear();
    pending_since_ = 0;
}
std::vector<Frame> Decoder::feed(const std::uint8_t *p, std::size_t n)
{
    std::vector<Frame> frames;
    if (pending_since_ && monotonic_ns() - pending_since_ > 2000000000ull)
    {
        discarded_bytes += buffer_.size();
        reset();
    }
    while (n)
    {
        auto amount = std::min<std::size_t>(n, 4096);
        buffer_.insert(buffer_.end(), p, p + amount);
        p += amount;
        n -= amount;
        parse(frames);
    }
    if (buffer_.empty())
        pending_since_ = 0;
    else if (!pending_since_)
        pending_since_ = monotonic_ns();
    return frames;
}
void Decoder::parse(std::vector<Frame> &frames)
{
    while (true)
    {
        auto start =
            std::search(buffer_.begin(), buffer_.end(), std::begin(magic), std::end(magic));
        if (start == buffer_.end())
        {
            auto keep = std::min<std::size_t>(3, buffer_.size());
            discarded_bytes += buffer_.size() - keep;
            buffer_.erase(buffer_.begin(), buffer_.end() - keep);
            return;
        }
        discarded_bytes += std::distance(buffer_.begin(), start);
        buffer_.erase(buffer_.begin(), start);
        if (buffer_.size() < header_size)
            return;
        const auto *p = buffer_.data();
        auto size = get32(p + 12);
        if (p[4] != 1 || p[5] != 0 || p[6] != 40 || p[7] != 0 || p[10] > 1 || p[11] != 0 ||
            size > max_payload || get32(p + 36) != crc32c(p, 36))
        {
            ++header_errors;
            buffer_.erase(buffer_.begin());
            continue;
        }
        if (buffer_.size() < header_size + size)
            return;
        if (get32(p + 32) != crc32c(p + 40, size))
        {
            ++crc_errors;
            buffer_.erase(buffer_.begin());
            continue;
        }
        Bytes header(p, p + header_size);
        Reader r(header);
        r.u32();
        r.u8();
        r.u8();
        r.u16();
        Frame f;
        f.type = r.u16();
        f.flags = r.u16();
        r.u32();
        f.sequence = r.u32();
        f.request_id = r.u32();
        f.timestamp_ns = r.u64();
        f.payload.assign(p + 40, p + 40 + size);
        frames.push_back(std::move(f));
        buffer_.erase(buffer_.begin(), buffer_.begin() + 40 + size);
        pending_since_ = 0;
    }
}
Json parse_json(const Frame &f)
{
    require(f.flags == json_flag && f.payload.size() <= json_limit, "Invalid JSON frame");
    std::vector<std::set<std::string>> keys;
    auto callback = [&](int depth, Json::parse_event_t event, Json &value)
    {
        require(depth <= 16, "JSON nesting limit");
        if (event == Json::parse_event_t::object_start)
            keys.emplace_back();
        if (event == Json::parse_event_t::key)
        {
            const auto key = value.get<std::string>();
            require(key.size() <= 2048, "JSON key limit");
            require(!keys.empty() && keys.back().insert(key).second, "Duplicate JSON key");
        }
        if (event == Json::parse_event_t::object_end)
            keys.pop_back();
        if (event == Json::parse_event_t::value && value.is_string())
            require(value.get_ref<const std::string &>().size() <= 2048, "JSON string limit");
        return true;
    };
    auto j = Json::parse(f.payload.begin(), f.payload.end(), callback);
    require(j.is_object(), "JSON object required");
    return j;
}
Frame json_frame(std::uint16_t type, const Json &value, std::uint32_t request)
{
    auto text = value.dump();
    require(text.size() <= json_limit, "JSON payload limit");
    Frame f;
    f.type = type;
    f.flags = json_flag;
    f.request_id = request;
    f.timestamp_ns = monotonic_ns();
    f.payload.assign(text.begin(), text.end());
    return f;
}
std::uint64_t json_u64(const Json &j, const char *key)
{
    require(j.contains(key) && j[key].is_string(), "uint64 must be a decimal string");
    const auto s = j[key].get<std::string>();
    require(!s.empty() && s.size() <= 20, "Invalid uint64");
    std::uint64_t value = 0;
    for (char c : s)
    {
        require(c >= '0' && c <= '9', "Invalid uint64");
        auto digit = static_cast<unsigned>(c - '0');
        require(value <= (std::numeric_limits<std::uint64_t>::max() - digit) / 10,
                "uint64 overflow");
        value = value * 10 + digit;
    }
    return value;
}
void write_context(Writer &w, const Context &c)
{
    w.u64(c.session);
    w.u64(c.run);
    w.u32(c.epoch);
    w.u16(c.stream);
    w.u16(c.schema);
    w.u64(c.sample);
}
Context read_context(Reader &r)
{
    Context c;
    c.session = r.u64();
    c.run = r.u64();
    c.epoch = r.u32();
    c.stream = r.u16();
    c.schema = r.u16();
    c.sample = r.u64();
    require(c.schema == 1 && c.session != 0, "Invalid telemetry context");
    return c;
}
Bytes encode_pose(const Pose &p)
{
    Writer w;
    write_context(w, p.context);
    double norm = 0;
    for (auto v : p.position)
    {
        finite(v);
        w.f64(v);
    }
    for (auto v : p.quaternion)
    {
        finite(v);
        norm += v * v;
        w.f64(v);
    }
    require(std::abs(norm - 1) < 0.02, "Invalid quaternion");
    for (auto v : p.velocity)
    {
        finite(v);
        w.f32(v);
    }
    w.u32(p.flags);
    w.u64(p.camera_frame);
    return std::move(w.bytes);
}
Pose decode_pose(const Bytes &b)
{
    Reader r(b);
    Pose p;
    p.context = read_context(r);
    for (auto &v : p.position)
        v = r.f64();
    double norm = 0;
    for (auto &v : p.quaternion)
    {
        v = r.f64();
        norm += v * v;
    }
    require(std::abs(norm - 1) < 0.02, "Invalid quaternion");
    for (auto &v : p.quaternion)
        v /= std::sqrt(norm);
    for (auto &v : p.velocity)
        v = r.f32();
    p.flags = r.u32();
    p.camera_frame = r.u64();
    r.done();
    require((p.flags & ~3u) == 0, "Invalid pose flags");
    return p;
}
Bytes encode_imu(const ImuBatch &b)
{
    require(!b.samples.empty() && b.samples.size() <= 128, "IMU count limit");
    Writer w;
    write_context(w, b.context);
    w.u16(static_cast<std::uint16_t>(b.samples.size()));
    w.u16(0);
    for (const auto &s : b.samples)
    {
        w.u32(s.dt);
        for (auto v : s.gyro)
            w.f32(v);
        for (auto v : s.accel)
            w.f32(v);
        w.u32(s.flags);
    }
    return std::move(w.bytes);
}
ImuBatch decode_imu(const Bytes &b)
{
    Reader r(b);
    ImuBatch v;
    v.context = read_context(r);
    auto n = r.u16();
    require(r.u16() == 0 && n > 0 && n <= 128 && r.remaining() == std::size_t(n) * 32,
            "Invalid IMU batch");
    for (unsigned i = 0; i < n; ++i)
    {
        ImuSample s;
        s.dt = r.u32();
        for (auto &x : s.gyro)
            x = r.f32();
        for (auto &x : s.accel)
            x = r.f32();
        s.flags = r.u32();
        require(s.flags == 1 && (i == 0 ? s.dt == 0 : s.dt > v.samples.back().dt),
                "Invalid IMU sample");
        v.samples.push_back(s);
    }
    r.done();
    return v;
}
Bytes encode_cloud(const Cloud &c)
{
    require(c.points.size() <= 2048, "Cloud chunk limit");
    Writer w;
    write_context(w, c.context);
    w.u8(c.source);
    w.u8(c.mode);
    w.u16(c.index);
    w.u16(c.chunks);
    w.u16(static_cast<std::uint16_t>(c.points.size()));
    w.u32(c.revision);
    w.u32(c.total);
    for (const auto &p : c.points)
    {
        w.u64(p.id);
        for (auto x : p.xyz)
            w.f32(x);
        w.u16(p.flags);
        w.u16(p.camera);
    }
    return std::move(w.bytes);
}
Cloud decode_cloud(const Bytes &b)
{
    Reader r(b);
    Cloud c;
    c.context = read_context(r);
    c.source = r.u8();
    c.mode = r.u8();
    c.index = r.u16();
    c.chunks = r.u16();
    auto n = r.u16();
    c.revision = r.u32();
    c.total = r.u32();
    require(c.source <= 1 && c.mode == 0 && c.chunks > 0 && c.chunks <= 128 && c.index < c.chunks &&
                c.total <= 8192 && r.remaining() == std::size_t(n) * 24,
            "Invalid cloud");
    for (unsigned i = 0; i < n; ++i)
    {
        Point p;
        p.id = r.u64();
        for (auto &x : p.xyz)
            x = r.f32();
        p.flags = r.u16();
        p.camera = r.u16();
        require(p.flags == 1, "Invalid point flags");
        c.points.push_back(p);
    }
    r.done();
    return c;
}
Bytes encode_keyframes(const Keyframes &c)
{
    require(c.context.stream == 3 && c.chunks > 0 && c.chunks <= 4096 && c.index < c.chunks &&
                c.total <= keyframe_limit && c.poses.size() <= 112 && c.poses.size() <= c.total &&
                (c.total == 0 ? (c.chunks == 1 && c.poses.empty()) : !c.poses.empty()),
            "Invalid keyframes");
    Writer w;
    write_context(w, c.context);
    w.u64(c.revision);
    w.u16(c.index);
    w.u16(c.chunks);
    w.u32(c.total);
    for (const auto &p : c.poses)
    {
        require(p.id > 0, "Invalid keyframe ID");
        w.u64(p.id);
        for (float x : p.position)
        {
            finite(x);
            w.f32(x);
        }
        double norm = 0;
        for (float x : p.quaternion)
        {
            finite(x);
            norm += x * x;
        }
        require(std::abs(norm - 1) < 0.02, "Invalid keyframe quaternion");
        for (float x : p.quaternion)
            w.f32(x);
    }
    return std::move(w.bytes);
}
Keyframes decode_keyframes(const Bytes &b)
{
    Reader r(b);
    Keyframes c;
    c.context = read_context(r);
    c.revision = r.u64();
    c.index = r.u16();
    c.chunks = r.u16();
    c.total = r.u32();
    require(c.context.stream == 3 && c.chunks > 0 && c.chunks <= 4096 && c.index < c.chunks &&
                c.total <= keyframe_limit && r.remaining() % 36 == 0 && r.remaining() / 36 <= 112,
            "Invalid keyframes");
    const auto n = r.remaining() / 36;
    require(n <= c.total && (c.total == 0 ? (c.chunks == 1 && n == 0) : n > 0),
            "Invalid keyframe count");
    for (std::size_t i = 0; i < n; ++i)
    {
        KeyframePose p;
        p.id = r.u64();
        require(p.id > 0, "Invalid keyframe ID");
        for (auto &x : p.position)
            x = r.f32();
        double norm = 0;
        for (auto &x : p.quaternion)
        {
            x = r.f32();
            norm += x * x;
        }
        require(std::abs(norm - 1) < 0.02, "Invalid keyframe quaternion");
        for (auto &x : p.quaternion)
            x = float(x / std::sqrt(norm));
        c.poses.push_back(p);
    }
    r.done();
    return c;
}
Bytes encode_features(const Features &c)
{
    require(c.points.size() <= 2048, "Feature chunk limit");
    Writer w;
    write_context(w, c.context);
    w.u16(c.source);
    w.u16(c.camera);
    w.u64(c.camera_frame);
    w.u16(c.width);
    w.u16(c.height);
    w.u16(c.index);
    w.u16(c.chunks);
    w.u16(static_cast<std::uint16_t>(c.points.size()));
    w.u16(0);
    for (const auto &p : c.points)
    {
        w.u64(p.id);
        w.f32(p.u);
        w.f32(p.v);
        w.u16(p.age);
        w.u16(p.flags);
        w.u32(0);
    }
    return std::move(w.bytes);
}
Features decode_features(const Bytes &b)
{
    Reader r(b);
    Features c;
    c.context = read_context(r);
    c.source = r.u16();
    c.camera = r.u16();
    c.camera_frame = r.u64();
    c.width = r.u16();
    c.height = r.u16();
    c.index = r.u16();
    c.chunks = r.u16();
    auto n = r.u16();
    require(r.u16() == 0 && c.width && c.height && c.camera < 2 && c.chunks > 0 &&
                c.chunks <= 128 && c.index < c.chunks && r.remaining() == std::size_t(n) * 24,
            "Invalid features");
    for (unsigned i = 0; i < n; ++i)
    {
        Feature p;
        p.id = r.u64();
        p.u = r.f32();
        p.v = r.f32();
        p.age = r.u16();
        p.flags = r.u16();
        require(r.u32() == 0 && (p.flags & ~3u) == 0, "Invalid feature flags");
        c.points.push_back(p);
    }
    r.done();
    return c;
}
Bytes encode_image(const ImagePart &p)
{
    Writer w;
    write_context(w, p.context);
    w.u64(p.camera_frame);
    w.u16(p.source);
    w.u16(p.camera);
    w.u16(p.width);
    w.u16(p.height);
    w.u8(p.encoding);
    w.u8(p.flags);
    w.u16(0);
    w.u32(p.total);
    w.u32(p.offset);
    w.bytes.insert(w.bytes.end(), p.data.begin(), p.data.end());
    return std::move(w.bytes);
}
ImagePart decode_image(const Bytes &b)
{
    Reader r(b);
    ImagePart p;
    p.context = read_context(r);
    p.camera_frame = r.u64();
    p.source = r.u16();
    p.camera = r.u16();
    p.width = r.u16();
    p.height = r.u16();
    p.encoding = r.u8();
    p.flags = r.u8();
    require(r.u16() == 0, "Invalid image reserved field");
    p.total = r.u32();
    p.offset = r.u32();
    p.data = r.rest();
    require(p.encoding == 1 && p.flags == 0 && p.camera < 2 && p.width && p.height &&
                p.width <= 4096 && p.height <= 4096 && p.total > 0 && p.total <= 262144 &&
                p.offset <= p.total && !p.data.empty() && p.data.size() <= p.total - p.offset,
            "Invalid image chunk");
    return p;
}
Bytes encode_dense_cloud(const DenseCloud &c)
{
    require(c.context.stream == 9 && c.chunks > 0 && c.chunks <= 4096 && c.index < c.chunks &&
                c.total <= dense_point_limit && c.points.size() <= 253,
            "Invalid dense cloud");
    Writer w;
    write_context(w, c.context);
    w.u64(c.revision);
    w.u16(c.index);
    w.u16(c.chunks);
    w.u32(c.total);
    for (const auto &p : c.points)
    {
        for (float x : p.xyz)
        {
            finite(x);
            w.f32(x);
        }
        for (auto x : p.rgb)
            w.u8(x);
        w.u8(0);
    }
    return std::move(w.bytes);
}
DenseCloud decode_dense_cloud(const Bytes &b)
{
    Reader r(b);
    DenseCloud c;
    c.context = read_context(r);
    c.revision = r.u64();
    c.index = r.u16();
    c.chunks = r.u16();
    c.total = r.u32();
    require(c.context.stream == 9 && c.chunks > 0 && c.chunks <= 4096 && c.index < c.chunks &&
                c.total <= dense_point_limit && r.remaining() % 16 == 0 &&
                r.remaining() / 16 <= 253,
            "Invalid dense cloud");
    const auto n = r.remaining() / 16;
    require(n <= c.total && (c.total == 0 ? (c.chunks == 1 && n == 0) : n > 0),
            "Invalid dense count");
    for (std::size_t i = 0; i < n; ++i)
    {
        DensePoint p;
        for (auto &x : p.xyz)
            x = r.f32();
        for (auto &x : p.rgb)
            x = r.u8();
        require(r.u8() == 0, "Invalid dense reserved byte");
        c.points.push_back(p);
    }
    r.done();
    return c;
}
Bytes encode_map_delta(const MapDelta &d)
{
    const auto action = unsigned(d.action);
    require((d.context.stream == 9 || d.context.stream == 3) && d.sync && d.context.sample &&
                action <= 3,
            "Invalid map delta context");
    require((d.action == MapAction::Reset ? d.id == 0 : d.id > 0) && d.pose.id == d.id,
            "Invalid map node ID");
    require(d.total <= dense_point_limit && d.offset <= d.total && d.points.size() <= 250 &&
                d.points.size() <= d.total - d.offset,
            "Invalid map delta size");
    require(d.action == MapAction::Upsert ? (d.total == 0 || !d.points.empty())
                                          : (!d.total && !d.offset && d.points.empty()),
            "Invalid map action payload");
    Writer w;
    write_context(w, d.context);
    w.u64(d.sync);
    w.u64(d.revision);
    w.u64(d.id);
    w.u8(std::uint8_t(d.action));
    w.u8(0);
    w.u16(0);
    w.u32(d.offset);
    w.u32(d.total);
    for (float x : d.pose.position)
    {
        finite(x);
        w.f32(x);
    }
    double norm = 0;
    for (float x : d.pose.quaternion)
    {
        finite(x);
        norm += x * x;
        w.f32(x);
    }
    require(std::abs(norm - 1) < 0.02, "Invalid map quaternion");
    for (const auto &p : d.points)
    {
        for (float x : p.xyz)
        {
            finite(x);
            w.f32(x);
        }
        for (auto c : p.rgb)
            w.u8(c);
        w.u8(0);
    }
    return std::move(w.bytes);
}
MapDelta decode_map_delta(const Bytes &b)
{
    Reader r(b);
    MapDelta d;
    d.context = read_context(r);
    d.sync = r.u64();
    d.revision = r.u64();
    d.id = r.u64();
    const auto action = r.u8();
    require(action <= 3 && r.u8() == 0 && r.u16() == 0, "Invalid map action");
    d.action = MapAction(action);
    d.offset = r.u32();
    d.total = r.u32();
    d.pose.id = d.id;
    for (auto &x : d.pose.position)
        x = r.f32();
    for (auto &x : d.pose.quaternion)
        x = r.f32();
    require(r.remaining() % 16 == 0 && r.remaining() / 16 <= 250, "Invalid map point payload");
    while (r.remaining())
    {
        DensePoint p;
        for (auto &x : p.xyz)
            x = r.f32();
        for (auto &c : p.rgb)
            c = r.u8();
        require(r.u8() == 0, "Invalid map reserved byte");
        d.points.push_back(p);
    }
    // Use the same validation for both directions, without normalizing poses:
    // chunk metadata must remain bit-identical throughout a transaction.
    encode_map_delta(d);
    return d;
}
const char *stream_name(std::uint16_t id)
{
    switch (id)
    {
    case 1:
        return "pose";
    case 2:
        return "imu_calibrated";
    case 3:
        return "map_keyframes";
    case 5:
        return "features_left";
    case 7:
        return "image_rectified_left";
    case 9:
        return "dense_map";
    default:
        return "unknown";
    }
}
std::uint16_t stream_id(const std::string &name)
{
    for (auto i : active_streams)
        if (name == stream_name(i))
            return i;
    return 0;
}
std::string base64(const Bytes &b)
{
    static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((b.size() + 2) / 3 * 4);
    for (std::size_t i = 0; i < b.size(); i += 3)
    {
        std::uint32_t x = std::uint32_t(b[i]) << 16;
        if (i + 1 < b.size())
            x |= std::uint32_t(b[i + 1]) << 8;
        if (i + 2 < b.size())
            x |= b[i + 2];
        out += table[(x >> 18) & 63];
        out += table[(x >> 12) & 63];
        out += i + 1 < b.size() ? table[(x >> 6) & 63] : '=';
        out += i + 2 < b.size() ? table[x & 63] : '=';
    }
    return out;
}
} // namespace svio
