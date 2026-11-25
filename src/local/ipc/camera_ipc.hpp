// SPDX-License-Identifier: Apache-2.0
// Small Linux-only IPC layer for the RK3576 camera processes.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

#include "camera_broker_protocol.h"

struct CameraIpcRecord
{
    uint64_t sequence = 0;
    uint64_t timestamp_monotonic_ns = 0;
    uint32_t payload_size = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    uint32_t format = 0;
    uint32_t flags = 0;
};

class CameraIpcRings
{
  public:
    struct Ring;

    CameraIpcRings();
    ~CameraIpcRings();

    CameraIpcRings(const CameraIpcRings &) = delete;
    CameraIpcRings &operator=(const CameraIpcRings &) = delete;

    // Called by soc-uvc-broker. The returned file descriptors are retained by
    // this object and can be passed to clients with send_offer().
    int create_owner(uint32_t rgb_slot_count, uint32_t jpeg_slot_count, uint32_t imu_slot_count,
                     FILE *log);

    // Called by stereo_imu_capture or a VIO/SDK client after receiving the
    // broker's FD_OFFER message. writer=true is only used by the capture
    // publisher.
    int receive_offer(int socket_fd, bool writer, FILE *log);
    int send_offer(int socket_fd, uint32_t role, uint32_t stream_mask,
                   int notify_fd_override = -1) const;

    // Owner-side broker state. The capture publisher reads this mask from its
    // mapped headers; changing it never rebuilds the rings or interrupts the
    // V4L2 loop.
    void set_consumer_stream_mask(uint32_t stream_mask);
    uint32_t consumer_stream_mask() const;

    // Producer only, after all image writers have stopped. Keep the ring ABI
    // and monotonically increasing sequences, but reclaim resident pixels.
    void release_image_payloads();

    bool publish_rgb(int camera_index, const uint8_t *data, size_t bytes, uint32_t width,
                     uint32_t height, uint32_t stride, uint64_t timestamp_ns,
                     uint64_t *sequence_out);
    bool publish_preview(int camera_index, const uint8_t *data, size_t bytes, uint32_t width,
                         uint32_t height, uint32_t stride, uint64_t timestamp_ns);
    bool publish_jpeg(const uint8_t *data, size_t bytes, uint32_t width, uint32_t height,
                      uint64_t timestamp_ns, uint64_t *sequence_out);
    bool publish_imu(const camera_broker_imu_sample &sample, uint64_t *sequence_out);

    bool read_latest(uint32_t ring_id, uint64_t *cursor, void *payload, size_t payload_capacity,
                     CameraIpcRecord *record) const;
    bool read_next(uint32_t ring_id, uint64_t *cursor, void *payload, size_t payload_capacity,
                   CameraIpcRecord *record) const;
    // Read only the metadata of the next record.  This avoids copying a full
    // JPEG when a control client only needs its sequence/timestamp.
    bool read_next_metadata(uint32_t ring_id, uint64_t *cursor, CameraIpcRecord *record) const;

    int notification_fd() const
    {
        return notify_fd_;
    }
    size_t payload_capacity(uint32_t ring_id) const;
    bool valid() const
    {
        return valid_;
    }

  private:
    std::array<Ring *, CAMERA_BROKER_RING_COUNT> rings_{};
    int notify_fd_ = -1;
    bool owner_ = false;
    bool writer_ = false;
    bool valid_ = false;
    FILE *log_ = nullptr;

    void close_all();
};

// Connects to the broker and obtains the shared-memory rings. The connection
// is deliberately kept open: its lifetime is the SDK/VIO subscription lease.
class CameraIpcConnection
{
  public:
    CameraIpcConnection() = default;
    ~CameraIpcConnection();

    CameraIpcConnection(const CameraIpcConnection &) = delete;
    CameraIpcConnection &operator=(const CameraIpcConnection &) = delete;

    int connect(const std::string &socket_path, uint32_t role, uint32_t stream_mask,
                CameraIpcRings *rings, FILE *log);
    void close();
    int fd() const
    {
        return fd_;
    }

  private:
    int fd_ = -1;
};
