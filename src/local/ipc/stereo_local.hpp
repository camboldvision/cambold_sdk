// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace cambold_ipc
{
constexpr const char *stereo_socket = "/run/soc-camera/stereo.sock";
constexpr std::uint32_t stereo_magic = 0x4c534243;
constexpr std::uint32_t stereo_version = 2;
constexpr std::size_t stereo_capacity = 512 * 512 * 10;
// Native little-endian AArch64 IPC. Both RGB24 eyes and optical-axis depth
// (float32 metres, 0 invalid) refer to this same capture timestamp and grid.
struct StereoFrameInfo
{
    // reserved is the optional trailing gray record byte count. Version 2 adds
    // projection metadata; publisher and consumers must use the same version.
    std::uint32_t magic = stereo_magic, version = stereo_version, width = 0, height = 0,
                  generation = 0, reserved = 0;
    std::uint64_t sequence = 0, timestamp_ns = 0;
    float fx = 0, fy = 0, cx = 0, cy = 0, baseline = 0;
    std::uint32_t payload_bytes = 0;
    std::uint32_t projection = 0; // 0 perspective, 1 cylindrical about X
    float vertical_fov = 0;       // degrees; fy is pixels/radian in cylindrical mode
    std::array<double, 16> T_left_imu{}, T_right_imu{};
};
struct StereoGrayInfo
{
    std::uint32_t magic = 0x59475243, version = 1, width = 0, height = 0;
    float fx = 0, fy = 0, cx = 0, cy = 0, rectification_ms = 0;
    std::uint32_t reserved = 0;
};
struct StereoFrame
{
    StereoFrameInfo info;
    std::vector<std::uint8_t> pixels;
    // Left then right GRAY8, generated from the same native capture as pixels.
    StereoGrayInfo gray;
    std::vector<std::uint8_t> gray_pixels;
};
// Short control requests do not acquire a frame/inference lease. Keep the
// original two-word frame handshake and shared-memory layout compatible.
struct StereoSettings
{
    std::uint32_t mode = 0, frame_rate = 0, projection = 0;
    float focal_px = 0, vertical_fov = 180;
    float cy = std::numeric_limits<float>::quiet_NaN(); // NaN selects image centre.
};
struct StereoMode
{
    std::uint32_t id = 0, width = 0, height = 0, max_frame_rate = 0, projection = 0;
};
struct StereoConfiguration
{
    StereoSettings settings;
    std::uint32_t enabled = 0, mode_count = 0;
    std::array<StereoMode, 32> modes{};
};
StereoConfiguration stereo_configuration(const StereoSettings *desired = nullptr,
                                         const std::string &path = stereo_socket);
class StereoServer
{
  public:
    using Lease = std::function<int(bool, bool)>;
    using Latest = std::function<std::shared_ptr<const StereoFrame>()>;
    using Configure =
        std::function<int(const StereoSettings *, StereoConfiguration &, std::string &)>;
    StereoServer(Lease, Latest, const std::string &path = stereo_socket, Configure configure = {});
    ~StereoServer();
    StereoServer(const StereoServer &) = delete;
    StereoServer &operator=(const StereoServer &) = delete;
    bool demanded() const
    {
        return demand_.load();
    }

  private:
    void run();
    Lease lease_;
    Latest latest_;
    std::string path_;
    Configure configure_;
    int listener_ = -1, memory_ = -1;
    void *mapping_ = nullptr;
    std::atomic<bool> stop_{false}, demand_{false};
    std::thread thread_;
};
class StereoClient
{
  public:
    explicit StereoClient(const std::string &path = stereo_socket, bool dense = true);
    ~StereoClient();
    StereoClient(const StereoClient &) = delete;
    StereoClient &operator=(const StereoClient &) = delete;
    bool read(StereoFrame &frame);
    // Change only inference demand, keeping the capture lease and geometry.
    void setDense(bool dense);

  private:
    int socket_ = -1, memory_ = -1;
    void *mapping_ = nullptr;
    std::uint64_t cursor_ = 0, minimum_timestamp_ = 0;
};
} // namespace cambold_ipc
