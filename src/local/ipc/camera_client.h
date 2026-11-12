/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CB_CAMERA_CLIENT_H
#define CB_CAMERA_CLIENT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct camera_client camera_client_t;

    struct camera_client_frame
    {
        const uint8_t *data;
        size_t bytes;
        uint32_t width;
        uint32_t height;
        uint32_t stride;
        uint32_t format;
        uint64_t sequence;
        uint64_t timestamp_monotonic_ns;
        uint32_t flags;
    };

    struct camera_client_imu
    {
        uint64_t sequence;
        uint64_t timestamp_monotonic_ns;
        float accel[3];
        float gyro[3];
        float temperature;
        uint32_t flags;
    };

    camera_client_t *camera_client_open(const char *socket_path, uint32_t role,
                                        uint32_t stream_mask);
    void camera_client_close(camera_client_t *client);

    /* Waits for a shared-memory notification. Returns 1 when data may be
     * available, 0 on timeout, and a negative errno-style value on failure. */
    int camera_client_wait(camera_client_t *client, int timeout_ms);

    /* Video is latest-frame-wins; this function intentionally skips stale frames
     * so a slow VIO process does not accumulate latency. */
    int camera_client_read_latest_rgb(camera_client_t *client, int camera_index,
                                      struct camera_client_frame *frame);

    /* The UVC/rosbag JPEG carries the same timestamp in its SCAM APP1 marker;
     * the shared-memory SDK path also exposes it directly in this frame record. */
    int camera_client_read_latest_jpeg(camera_client_t *client, struct camera_client_frame *frame);

    /* Parse the SCAM APP1 marker written by stereo_imu_capture.  All output
     * pointers are optional; returns 0 for a valid marker or -ENOENT otherwise. */
    int camera_client_jpeg_get_timestamp(const uint8_t *jpeg, size_t bytes, uint64_t *sequence,
                                         uint64_t *timestamp_monotonic_ns, uint32_t *width,
                                         uint32_t *height);

    /* IMU uses read-next semantics. If the client falls behind the ring it
     * resynchronizes to the oldest sample still present. */
    int camera_client_read_next_imu(camera_client_t *client, struct camera_client_imu *sample);

    int camera_client_subscribe(camera_client_t *client, uint32_t stream_mask);
    int camera_client_unsubscribe(camera_client_t *client, uint32_t stream_mask);
    int camera_client_fd(const camera_client_t *client);

#ifdef __cplusplus
}
#endif

#endif
