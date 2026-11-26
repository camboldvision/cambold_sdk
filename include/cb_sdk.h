/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CB_SDK_H
#define CB_SDK_H

#include <stddef.h>
#include <stdint.h>

#define CB_SDK_VERSION_MAJOR 1
#define CB_SDK_VERSION_MINOR 0
#define CB_SDK_VERSION_PATCH 0
#define CB_SDK_VERSION "1.0.0"

/* One public C ABI for applications on a PC and on a CamBold camera.
   Select CB_SDK_BACKEND=USB or LOCAL at CMake configure time. No application
   source changes are required. Initialize all option structs with zero. */

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct cb cb_t;
    typedef struct cb_video cb_video_t;

    enum cb_event_type
    {
        CB_EVENT_CONNECTED = 1,
        CB_EVENT_DISCONNECTED = 2,
        CB_EVENT_RESULT = 3,
        CB_EVENT_METRICS = 4,
        CB_EVENT_IMU_STATUS = 7,
        CB_EVENT_STREAM_STATUS = 8,
        CB_EVENT_ERROR = 9,
        CB_EVENT_FRAME_META = 10,
        CB_EVENT_IMU_BATCH = 11,
        CB_EVENT_SENSOR_STATUS = 12,
        CB_EVENT_DEVICE_CONFIG = 13,
        CB_EVENT_STEREO_STATUS = 14,
        CB_EVENT_DISPARITY = 15,
        CB_EVENT_CALIBRATION_STATUS = 16,
        CB_EVENT_CALIBRATION_PREVIEW = 17
    };

    typedef struct cb_event
    {
        enum cb_event_type type;
        uint64_t request_id;
        int32_t result;
        int32_t enabled;
        const uint8_t *payload;
        size_t payload_size;
        const char *text;
    } cb_event;

    typedef void (*cb_event_callback)(const cb_event *event, void *user);

    enum cb_calibration_action
    {
        CB_CALIBRATION_STATUS_QUERY = 0,
        CB_CALIBRATION_ENTER = 1,
        CB_CALIBRATION_START_IMU = 2,
        CB_CALIBRATION_START_CAMERA = 3,
        CB_CALIBRATION_SOLVE = 4,
        CB_CALIBRATION_CANCEL = 5,
        CB_CALIBRATION_LEAVE = 6
    };
    typedef struct cb_calibration_options
    {
        uint32_t tag_rows, tag_cols;
        double tag_size_metres; /* Outer black tag edge, metres. */
        double tag_spacing;     /* Gap / tag edge. Tag36h11, black border 2. */
    } cb_calibration_options;
    typedef struct cb_calibration_preview
    {
        uint32_t width, height, camera_count;
        uint64_t sequence, timestamp_monotonic_ns;
    } cb_calibration_preview;

    /* Asynchronous: results/status arrive as CB_EVENT_CALIBRATION_STATUS JSON
       (version 1); request_id matches this call. Opening calibration pauses
       ordinary bulk samples, not UVC. A task survives PC disconnection.
       START_CAMERA uses existing valid IMU noise in the shared YAML.
       SOLVE for static IMU requires at least 7200 continuous stationary seconds.
       LEAVE requires a finished/cancelled task and restores the camera rate. */
    int cb_calibration_command(cb_t *, enum cb_calibration_action, const cb_calibration_options *,
                               uint64_t request_id);
    int cb_get_calibration_status(cb_t *, char *json, size_t capacity);
    /* Copies the newest complete JPEG (both raw eyes with coverage overlay).
       Set *jpeg_size to capacity; ENOSPC returns required size. EAGAIN means
       sequence <= after_sequence. No image pointer survives this call. */
    int cb_get_calibration_preview(cb_t *, uint64_t after_sequence, cb_calibration_preview *,
                                   uint8_t *jpeg, size_t *jpeg_size);
    enum cb_calibration_report_kind {
        CB_CALIBRATION_REPORT_CURRENT = 0,
        CB_CALIBRATION_REPORT_JOINT = 1,
        CB_CALIBRATION_REPORT_IMU = 2
    };
    /* Reads at most 8192 ASCII JSON bytes per reply, without entering calibration.
       CB_EVENT_CALIBRATION_STATUS with this request_id contains ok/error and
       report_chunk:{kind,revision,offset,total,data}. Start with revision=offset=0;
       repeat with returned revision and offset+data.size() until total bytes.
       Verify FNV-1a-32 of the concatenated ASCII data equals revision, then parse
       the JSON. Max 2 MiB. Restart from zero if the report changes or reconnects.
       Reports survive restart; old YAML-only calibrations have no detailed data.
       The API and response are identical for USB and LOCAL transports. */
    int cb_request_calibration_report(cb_t *, enum cb_calibration_report_kind,
                                      uint32_t revision, uint32_t offset, uint64_t request_id);

    /* Native JPEG frames belong to the SDK. The callback receives the
           original MJPEG payload; the data pointer is valid only until the callback
           returns.  The SDK parses the SCAM APP1 marker before invoking the callback. */
    typedef struct cb_video_frame
    {
        const uint8_t *jpeg;
        size_t jpeg_size;
        uint64_t sequence;
        /* Board monotonic ns at the left image's optical exposure midpoint.
           Same source value as cb_disparity.timestamp_monotonic_ns.
           Valid only when CB_VIDEO_TIMESTAMP_VALID is set. */
        uint64_t timestamp_monotonic_ns;
        uint32_t width;
        uint32_t height;
        uint32_t flags;
        /* Exposure duration in microseconds for left/right images. Zero means
           unavailable; see CB_VIDEO_EXPOSURE_*_VALID. Rebuild SDK clients when
           upgrading this header. */
        uint32_t exposure_us[2];
    } cb_video_frame;

    enum cb_video_frame_flags
    {
        CB_VIDEO_TIMESTAMP_VALID = 1u,
        CB_VIDEO_EXPOSURE_LEFT_VALID = 2u,
        CB_VIDEO_EXPOSURE_RIGHT_VALID = 4u
    };

#define CB_MAX_FRAME_RATES 64u
    enum cb_device_config_flags
    {
        CB_CONFIG_FRAME_RATE_VALID = 1u,
        CB_CONFIG_IMU_VALID = 2u
    };
    typedef struct cb_device_config
    {
        uint32_t valid_flags;
        uint32_t frame_rate;
        uint32_t imu_output_enabled;
        uint32_t frame_rate_count;
        uint32_t frame_rates[CB_MAX_FRAME_RATES];
    } cb_device_config;

    typedef void (*cb_video_frame_callback)(const cb_video_frame *frame, void *user);

    typedef struct cb_video_config
    {
        cb_video_frame_callback callback;
        void *user;
    } cb_video_config;

    typedef struct cb_imu_sample
    {
        uint64_t sequence;
        uint64_t timestamp_monotonic_ns;
        float accel[3];
        float gyro[3];
        float temperature;
        uint32_t flags;
    } cb_imu_sample;

    typedef struct cb_frame_meta
    {
        uint64_t sequence;
        uint64_t timestamp_monotonic_ns;
        uint32_t payload_size;
        uint32_t width;
        uint32_t height;
        uint32_t stride;
        uint32_t format;
        uint32_t flags;
    } cb_frame_meta;

    typedef struct cb_config
    {
        /* NULL: discover USB automatically, or use /run/soc-camera/sdk.sock
           with the LOCAL backend. An explicit LOCAL value is a socket path. */
        const char *control_port;
        /* Reserved transport setting; USB bulk and local sockets ignore baud. */
        uint32_t control_baudrate;
        cb_event_callback callback;
        void *user;
    } cb_config;

#define CB_MAX_STEREO_MODES 16u
#define CB_MAX_DISPARITY_WIDTH 512u
#define CB_MAX_DISPARITY_HEIGHT 512u
    enum cb_stereo_flags
    {
        CB_STEREO_AVAILABLE = 1u,
        CB_STEREO_CALIBRATION_VALID = 2u,
        CB_STEREO_MODELS_VALID = 4u,
        CB_STEREO_ENABLED = 8u
    };
    enum cb_stereo_error
    {
        CB_STEREO_CALIBRATION_ERROR = -2001,
        CB_STEREO_MODEL_ERROR = -2002
    };
    typedef struct cb_stereo_mode
    {
        uint32_t id, width, height, network_width, network_height;
        uint32_t projection; /* 0 perspective/horizontal, 1 cylindrical/vertical. */
        uint32_t max_fps;    /* Per-mode camera/disparity rate ceiling: 20, 15 or 10. */
    } cb_stereo_mode;
    typedef struct cb_stereo_status
    {
        uint32_t flags;
        int32_t error;
        uint32_t mode_id;
        float focal_px;
        uint32_t mode_count;
        cb_stereo_mode modes[CB_MAX_STEREO_MODES];
        char message[256];
        uint32_t projection;
        float vertical_fov_deg;
        float cy; /* Principal point Y in rectified output pixels; may lie outside the image. */
    } cb_stereo_status;
/* Version 3 adds projection metadata; v2/v3 use binary16 pixel disparity. */
#define CB_DISPARITY_FRAME_VERSION 3u
    enum cb_projection
    {
        CB_PERSPECTIVE = 0,
        CB_CYLINDRICAL = 1
    };
    enum cb_disparity_format
    {
        CB_DISPARITY_FP16 = 1
    };
    /* Shared metadata for a paired FP16 disparity + rectified left RGB565 frame.
       Both planes have width*height pixels, identical rectification/cropping,
       and the same frame_id, timestamp and camera parameters below. */
    typedef struct cb_disparity
    {
        uint32_t generation;
        uint64_t frame_id;
        /* Board CLOCK_MONOTONIC, nanoseconds, at the source left image's optical
           exposure midpoint. Identical to timestamp_monotonic_ns in the matching
           UVC JPEG metadata; not inference completion or host arrival time.
           Rectification/cropping and depth reconstruction preserve this timestamp.
           frame_id is a disparity-stream ID, not the UVC sensor sequence. */
        uint64_t timestamp_monotonic_ns;
        uint32_t mode_id, width, height;
        float focal_px, cx, cy, baseline_metres;
        uint32_t disparity_format; /* CB_DISPARITY_FP16 */
        uint32_t projection;
        float fy, vertical_fov_deg; /* fy: pixels/radian for cylindrical; pixels otherwise */
    } cb_disparity;
    /* Stereo defaults off. Enable caps the camera at the selected mode's max_fps
       (persisted); lower camera frame rates are preserved.
       Configure while disabled; focal_px is finite, positive, in output pixels.
       Choices are read from the board, never inferred from model filenames.
       Requests are asynchronous; completion uses EVENT_RESULT and STEREO_STATUS. */
    int cb_request_stereo_status(cb_t *, uint64_t request_id);
    int cb_set_disparity_enabled(cb_t *, int enable, uint64_t request_id);
    int cb_set_disparity_config(cb_t *, uint32_t mode_id, float focal_px, uint64_t request_id);
    /* Extended configuration: cylindrical mode 13 is 256x256. vertical_fov_deg
       is finite, 30..180 degrees. Stop SLAM and disable disparity before changing.
       Legacy set_disparity_config selects perspective and preserves its API. */
    int cb_set_disparity_config_ex(cb_t *, uint32_t mode_id, float focal_px, uint32_t projection,
                                   float vertical_fov_deg, uint64_t request_id);
    /* Explicit output principal point. Legacy configuration APIs reset cy to
       (output_height - 1) / 2. Configure while stereo/SLAM is disabled. */
    int cb_set_disparity_config_with_cy(cb_t *, uint32_t mode_id, float focal_px,
                                       uint32_t projection, float vertical_fov_deg,
                                       float cy, uint64_t request_id);
    /* Reconstruct optical-camera XYZ metres from a disparity value in pixels.
       Cylindrical: rho=fx*B/d, X=(u-cx)*rho/fx, Y=rho*sin(theta),
       Z=rho*cos(theta), theta=(v-cy)/fy. Returns -EINVAL for invalid input. */
    int cb_disparity_to_point(const cb_disparity *, float u, float v, float disparity_px,
                              float xyz[3]);
    int cb_decode_stereo_status(const uint8_t *, size_t, cb_stereo_status *);
    /* EVENT_DISPARITY is delivered on a separate latest-frame worker. It can
       overlap other SDK callbacks. Payload pointers expire on callback return.
       Decoder returns two LE byte planes: FP16 disparity (0 invalid),
       then rectified, same-frame RGB565 color. For cylindrical projection,
       focal*baseline/d is the cylinder radius, not optical-axis depth.
       Use cb_disparity_to_point() for either projection.
       The decoded timestamp identifies that source image on the board clock.
       Coordinates: +X right, +Y down, +Z forward in rectified left camera. */
    int cb_decode_disparity(const uint8_t *, size_t, cb_disparity *, const uint8_t **disparity_le,
                            const uint8_t **rgb565_le);
    /* Convert an IEEE-754 binary16 bit pattern to float, including subnormals,
       signed zero, infinities and NaNs. No CPU FP16 support is required.
       Decode disparity with this function before computing depth; skip values
       that are not finite and positive. */
    float cb_fp16_to_float(uint16_t bits);
    /* Single-call, thread-safe acquisition of both planes from one complete frame.
       The SDK holds that immutable frame throughout both copies; a new arrival
       cannot pair disparity from one frame with RGB from another. RGB is produced
       on the board from the rectified left image, independently of raw UVC preview.
       Output planes use native-endian uint16 elements. Disparity
       elements hold FP16 bit patterns, not integer pixel values; use
       cb_fp16_to_float(). RGB565 elements retain their usual bit layout.
       after_id=0 accepts any latest frame. -EAGAIN if no newer frame; -ENOSPC fills metadata
       so callers can allocate width*height elements per plane. Both buffers are
       required; neither is copied if either is missing or too small. Capacities
       are element counts, not byte counts. No IMU-thread work. */
    int cb_get_disparity(cb_t *, uint64_t after_id, cb_disparity *, uint16_t *disparity,
                         size_t disparity_capacity, uint16_t *rgb565, size_t rgb565_capacity);

    /* SLAM uses the same SDK bulk connection; the user's ACM remains free.
       Commands are asynchronous: inspect state.last_result in get_json/poll_json
       for board acceptance/completion and errors. Map selection is fixed for
       each run; processing can change through cb_slam_set_processing().
       Dense reconstruction blocks USB SDK IMU upload; LOCAL and internal VIO
       IMU remain active. Sparse mode does not request the depth network.
       START may automatically apply the map's camera configuration. Command and
       subscription APIs return -ENOTCONN until the SLAM handshake is ready;
       a zero return means queued, not completed. Commands are not replayed after
       reconnect. */
    enum cb_slam_map_mode
    {
        CB_SLAM_NEW = 0,
        CB_SLAM_RESUME = 1,
        CB_SLAM_LOCALIZATION = 2
    };
    typedef struct cb_slam_options
    {
        const char *map_name;
        uint32_t map_mode;
        int dense_reconstruction;
    } cb_slam_options;
    int cb_slam_start(cb_t *, const cb_slam_options *);
    int cb_slam_stop(cb_t *);
    /* Set defaults while idle, or switch processing during a run without
       resetting VIO or discarding the map. Disabling the backend also disables
       dense reconstruction. Re-enabling resumes the retained backend/map.
       Completion and applied backend_enabled/dense_reconstruction flags are
       reported asynchronously in SLAM JSON. USB IMU policy also applies when
       enabling dense reconstruction during a run; LOCAL IMU is unaffected. */
    int cb_slam_set_processing(cb_t *, int backend_enabled, int dense_reconstruction);
    /* Elevator motion estimation is disabled by default. Set the idle default
       or switch it during a run without restarting SLAM. Disabling composes
       accumulated cabin motion into ordinary VIO and resumes mapping.
       Completion, elevator_enabled and elevator_options_pending are reported
       asynchronously in SLAM JSON; zero means queued, not completed. */
    int cb_slam_set_elevator(cb_t *, int enabled);
    int cb_slam_refresh_maps(cb_t *);
    int cb_slam_subscribe(cb_t *, int preview, int map);
    /* USB: nonzero during dense SLAM, and enabling IMU returns -EBUSY.
       LOCAL: always zero; dense reconstruction does not limit IMU delivery. */
    int cb_slam_imu_upload_blocked(cb_t *);
    typedef struct cb_slam_pose
    {
        uint64_t run_id, timestamp_monotonic_ns, frame_id;
        uint32_t epoch;
        float position[3], quaternion_xyzw[4], velocity[3];
    } cb_slam_pose;
    /* Latest valid pose, map frame / Hamilton XYZW. Updated at 200 Hz after
       initialization (and map alignment in resume/localization). Non-consuming;
       compare timestamp_monotonic_ns when polling. -EAGAIN before a valid pose
       or while disconnected. */
    int cb_slam_get_pose(cb_t *, cb_slam_pose *);
    /* JSON contains state, pose, descriptors and map_catalog. get_json does not
       consume updates. poll_json additionally returns images and incremental
       map_delta: reset + upsert/pose/erase by keyframe id. Apply each poll once;
       use exactly one consumer per host. Point payload xyzrgb16 is base64 LE
       float32 XYZ + RGB888 + pad in each keyframe's IMU frame; pose transforms
       it into map coordinates. Sparse points use the same layout and channel.
       Both APIs allocate *out_json; free it with cb_slam_free_json(). */
    int cb_slam_get_json(cb_t *, char **out_json);
    int cb_slam_poll_json(cb_t *, char **out_json);
    void cb_slam_free_json(char *);
    int cb_slam_clear_view(cb_t *);

    /* The returned cb_t owns asynchronous control/debug workers. The control
       worker delivers callbacks, so callbacks must return quickly. Payload
       pointers are valid only until the callback returns; copy data if it must be
       retained. Opening starts discovery/reconnection and may succeed before the
       service/device is ready. cb_close joins all SDK workers. Call it from an
       application thread, never from an SDK callback. */
    int cb_open(const cb_config *config, cb_t **out_host);
    void cb_close(cb_t *host);
    int cb_connected(const cb_t *host);
    /* Re-send the non-blocking HELLO/CAPS probe.  This is useful when the USB
       gadget or the board-side service became ready after the desktop app. */
    int cb_refresh_connection(cb_t *host);
    uint64_t cb_next_request_id(cb_t *host);

    int cb_video_open(const cb_video_config *config, cb_video_t **out_uvc);
    int cb_video_start(cb_video_t *uvc, char *error, size_t error_capacity);
    void cb_video_stop(cb_video_t *uvc);
    void cb_video_close(cb_video_t *uvc);
    int cb_video_running(const cb_video_t *uvc);

    int cb_request_metrics(cb_t *host, uint64_t request_id);
    /* Read actual board state and board-provided frame-rate options. Also sent
       automatically after HELLO; the callback receives EVENT_DEVICE_CONFIG. */
    int cb_request_device_config(cb_t *host, uint64_t request_id);
    int cb_decode_device_config(const uint8_t *payload, size_t payload_size,
                                cb_device_config *out_config);
    int cb_set_imu_output(cb_t *host, int enable, uint64_t request_id);
    int cb_set_frame_rate(cb_t *host, uint32_t fps, uint64_t request_id);
    /* Update the debug terminal size. USB sends the request to ttyGS0 on the
       board; LOCAL updates the calling SDK object's private shell PTY. */
    int cb_set_terminal_size(cb_t *host, uint16_t cols, uint16_t rows, uint64_t request_id);

    /* The debug interface is intentionally independent from the binary
       control bulk interface. `port` is optional and may be "gser.0"/"bulk0";
       baudrate is ignored. LOCAL opens a private shell PTY with the calling
       user's privileges, without accessing either USB debug or user ACM. */
    int cb_debug_open(cb_t *host, const char *port, uint32_t baudrate);
    void cb_debug_close(cb_t *host);
    int cb_debug_write(cb_t *host, const uint8_t *data, size_t size);
    /* Returns bytes read, 0 on timeout/closed debug channel, or a negative error.
       timeout_ms: 0 polls, positive waits up to that duration, negative waits
       until bytes arrive or the channel/SDK is closed. */
    int cb_debug_read(cb_t *host, uint8_t *data, size_t size, int timeout_ms);

    /* Returns the selected logical transport: bulk-control/bulk-debug for USB;
       local-control/local-debug for LOCAL. These are not COM device paths. */
    int cb_get_control_port(const cb_t *host, char *out, size_t capacity);
    int cb_get_debug_port(const cb_t *host, char *out, size_t capacity);

    /* Copies the selected camera's product name as NUL-terminated UTF-8.
       USB reads its product descriptor; LOCAL reads the gadget product setting.
       Returns its byte length excluding NUL, -ENOENT if no name is available,
       -ENOSPC if capacity cannot hold the full name and NUL, or -EINVAL for
       invalid arguments. No USB I/O is performed by this getter. The cached name
       is refreshed on connection and cleared on disconnect. */
    int cb_get_device_name(const cb_t *host, char *out, size_t capacity);

    /* Copies the latest JSON metrics/status into caller-owned storage. */
    int cb_get_metrics_json(const cb_t *host, char *out, size_t capacity);
    int cb_get_last_error(const cb_t *host, char *out, size_t capacity);

    /* Decode one board IMU_BATCH payload (the event callback supplies the payload
       bytes). Returns 0 on success and -EINVAL/-ENOSPC on malformed input. */
    int cb_decode_imu_batch(const uint8_t *payload, size_t payload_size, cb_imu_sample *out_samples,
                            size_t *inout_sample_count);

    /* Decode one board FRAME_META payload. */
    int cb_decode_frame_meta(const uint8_t *payload, size_t payload_size, cb_frame_meta *out_frames,
                             size_t *inout_frame_count);

#ifdef __cplusplus
}
#endif

#endif
