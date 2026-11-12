/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CB_CAMERA_BROKER_PROTOCOL_H
#define CB_CAMERA_BROKER_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CAMERA_BROKER_MAGIC 0x4d414353u /* "SCAM" in little endian */
#define CAMERA_BROKER_VERSION 1u
#define CAMERA_BROKER_SOCKET "/run/soc-camera/broker.sock"
#define CAMERA_CAPTURE_CONTROL_SOCKET "/run/soc-camera/capture-control.sock"

    enum camera_broker_message_type
    {
        CAMERA_BROKER_HELLO = 1,
        CAMERA_BROKER_SUBSCRIBE = 2,
        CAMERA_BROKER_UNSUBSCRIBE = 3,
        CAMERA_BROKER_START = 4,
        CAMERA_BROKER_STOP = 5,
        CAMERA_BROKER_STATUS = 6,
        CAMERA_BROKER_PING = 7,
        CAMERA_BROKER_ERROR = 8,
        CAMERA_BROKER_FD_OFFER = 9,
    };

    enum camera_broker_endpoint_role
    {
        CAMERA_BROKER_ROLE_CAPTURE_PUBLISHER = 1,
        /* Reserved for a future external control client. The product SDK data
         * processing currently runs inside stereo_imu_capture, not here. */
        CAMERA_BROKER_ROLE_SDK_CLIENT = 2,
        CAMERA_BROKER_ROLE_VIO_CLIENT = 3,
    };

    enum camera_broker_stream_mask
    {
        CAMERA_BROKER_STREAM_LEFT = 1u << 0,
        CAMERA_BROKER_STREAM_RIGHT = 1u << 1,
        CAMERA_BROKER_STREAM_STEREO_JPEG = 1u << 2,
        CAMERA_BROKER_STREAM_IMU = 1u << 3,
        CAMERA_BROKER_STREAM_STEREO_NV12 = 1u << 4,
        CAMERA_BROKER_STREAM_LEFT_RGB = 1u << 5,
        CAMERA_BROKER_STREAM_RIGHT_RGB = 1u << 6,
    };

    enum camera_broker_frame_format
    {
        CAMERA_BROKER_FORMAT_NV12 = 1,
        CAMERA_BROKER_FORMAT_MJPEG = 2,
        CAMERA_BROKER_FORMAT_RGB24 = 3,
        CAMERA_BROKER_FORMAT_IMU = 4,
    };

    enum camera_broker_ring_id
    {
        CAMERA_BROKER_RING_LEFT_RGB = 0,
        CAMERA_BROKER_RING_RIGHT_RGB = 1,
        CAMERA_BROKER_RING_STEREO_JPEG = 2,
        CAMERA_BROKER_RING_IMU = 3,
        CAMERA_BROKER_RING_COUNT = 4,
    };

#define CAMERA_BROKER_FD_COUNT (CAMERA_BROKER_RING_COUNT + 1)

    /* All fields are little-endian, fixed-width, and intentionally contain no
     * pointers.  A message is sent as one SOCK_SEQPACKET record. */
    struct camera_broker_message
    {
        uint32_t magic;
        uint16_t version;
        uint16_t type;
        uint32_t size;
        uint32_t stream_mask;
        uint64_t request_id;
    };

    struct camera_broker_hello
    {
        struct camera_broker_message message;
        uint16_t role;
        uint16_t reserved;
        uint32_t pid;
    };

    struct camera_broker_fd_offer
    {
        struct camera_broker_message message;
        uint32_t ring_mask;
        uint32_t slot_count;
        uint32_t slot_size;
        uint32_t reserved;
    };

    /* Shared-memory ring metadata.  The producer publishes a slot by copying its
     * payload first and storing slot.sequence with release ordering. Readers use
     * acquire loads and verify the sequence again after copying. */
    struct camera_broker_ring_header
    {
        uint32_t magic;
        uint16_t version;
        uint16_t ring_id;
        uint32_t header_size;
        uint32_t slot_count;
        uint32_t slot_size;
        volatile uint64_t write_sequence;
        /* Set by the broker and observed by the capture publisher.  This is a
         * live demand mask, so UVC-only operation need not run the expensive
         * NV12 -> RGB RGA path when no VIO/SDK image client is connected. */
        volatile uint32_t consumer_stream_mask;
        uint32_t reserved;
    };

    struct camera_broker_ring_slot
    {
        volatile uint64_t sequence;
        uint64_t timestamp_monotonic_ns;
        uint32_t payload_size;
        uint32_t width;
        uint32_t height;
        uint32_t stride;
        uint32_t format;
        uint32_t flags;
        uint32_t reserved;
    };

    struct camera_broker_imu_sample
    {
        uint64_t sequence;
        uint64_t timestamp_monotonic_ns;
        float accel[3];
        float gyro[3];
        float temperature;
        uint32_t flags;
        uint32_t reserved;
    };

    struct camera_broker_frame_header
    {
        uint32_t magic;
        uint16_t version;
        uint16_t format;
        uint32_t header_size;
        uint32_t payload_size;
        uint32_t width;
        uint32_t height;
        uint32_t stride;
        uint32_t flags;
        uint64_t sequence;
        uint64_t timestamp_monotonic_ns;
    };

#ifdef __cplusplus
}
#endif

#endif
