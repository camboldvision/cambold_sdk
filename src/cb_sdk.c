/* SPDX-License-Identifier: Apache-2.0 */
#include "cb_sdk.h"
#include "cb_internal.h"
#include "cb_protocol.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static int send_empty(cb_t *host, uint16_t type, uint64_t request_id)
{
    return cb_internal_request(host, type, NULL, 0, request_id);
}

int cb_open(const cb_config *config, cb_t **out_host)
{
    return cb_internal_open(config, out_host);
}

void cb_close(cb_t *host)
{
    cb_internal_close(host);
}

int cb_connected(const cb_t *host)
{
    return cb_internal_connected(host);
}

int cb_refresh_connection(cb_t *host)
{
    return cb_internal_refresh_connection(host);
}

uint64_t cb_next_request_id(cb_t *host)
{
    return cb_internal_next_request_id(host);
}

int cb_request_metrics(cb_t *host, uint64_t request_id)
{
    return send_empty(host, CB_SYSTEM_METRICS_REQ, request_id);
}

int cb_request_device_config(cb_t *host, uint64_t request_id)
{
    return send_empty(host, CB_DEVICE_CONFIG_REQ, request_id);
}

int cb_get_antenna(cb_t *host, uint64_t request_id)
{
    return send_empty(host, CB_ANTENNA_GET_REQ, request_id);
}

int cb_set_antenna(cb_t *host, int wired, uint64_t request_id)
{
    const uint8_t value = wired ? 1u : 0u;
    return cb_internal_request(host, CB_ANTENNA_SET_REQ, &value, sizeof(value), request_id);
}

int cb_set_imu_output(cb_t *host, int enable, uint64_t request_id)
{
    const uint8_t value = enable ? 1u : 0u;
    return cb_internal_request(host, CB_IMU_STREAM_SET_REQ, &value, sizeof(value), request_id);
}

int cb_set_frame_rate(cb_t *host, uint32_t fps, uint64_t request_id)
{
    uint8_t payload[4];
    payload[0] = (uint8_t)(fps & 0xffu);
    payload[1] = (uint8_t)((fps >> 8) & 0xffu);
    payload[2] = (uint8_t)((fps >> 16) & 0xffu);
    payload[3] = (uint8_t)((fps >> 24) & 0xffu);
    return cb_internal_request(host, CB_FRAME_RATE_SET_REQ, payload, sizeof(payload), request_id);
}

int cb_set_terminal_size(cb_t *host, uint16_t cols, uint16_t rows, uint64_t request_id)
{
    uint8_t payload[4];
    payload[0] = (uint8_t)(cols & 0xffu);
    payload[1] = (uint8_t)((cols >> 8) & 0xffu);
    payload[2] = (uint8_t)(rows & 0xffu);
    payload[3] = (uint8_t)((rows >> 8) & 0xffu);
    return cb_internal_request(host, CB_TERMINAL_SIZE_SET_REQ, payload, sizeof(payload),
                               request_id);
}

static int basename_copy(const char *input, char *out, size_t capacity)
{
    const char *begin;
    const char *p;
    size_t length;
    if (!input || !out || capacity < 2)
        return -EINVAL;
    begin = input;
    p = input;
    while (*p)
    {
        if (*p == '/' || *p == '\\')
            begin = p + 1;
        ++p;
    }
    length = strlen(begin);
    if (length == 0 || strcmp(begin, ".") == 0 || strcmp(begin, "..") == 0)
        return -EINVAL;
    if (length >= capacity)
        return -ENAMETOOLONG;
    memcpy(out, begin, length + 1);
    return 0;
}

int cb_rosbag_start(cb_t *host, const char *name, int record_image, int record_imu,
                    uint64_t request_id)
{
    char base[256];
    uint8_t payload[1 + sizeof(base)];
    size_t length;
    int ret;
    /* The current board protocol has only the image flag. It always records
       IMU when the rosbag worker is enabled, so never pretend IMU=0 worked. */
    if (!record_imu)
        return -ENOTSUP;
    ret = basename_copy(name, base, sizeof(base));
    if (ret != 0)
        return ret;
    length = strlen(base);
    payload[0] = record_image ? 1u : 0u;
    memcpy(payload + 1, base, length);
    return cb_internal_request(host, CB_ROSBAG_START_REQ, payload, length + 1, request_id);
}

int cb_rosbag_stop(cb_t *host, uint64_t request_id)
{
    return send_empty(host, CB_ROSBAG_STOP_REQ, request_id);
}

int cb_debug_open(cb_t *host, const char *port, uint32_t baudrate)
{
    return cb_internal_debug_open(host, port, baudrate);
}

void cb_debug_close(cb_t *host)
{
    cb_internal_debug_close(host);
}

int cb_debug_write(cb_t *host, const uint8_t *data, size_t size)
{
    return cb_internal_debug_write(host, data, size);
}

int cb_debug_read(cb_t *host, uint8_t *data, size_t size, int timeout_ms)
{
    return cb_internal_debug_read(host, data, size, timeout_ms);
}

int cb_get_control_port(const cb_t *host, char *out, size_t capacity)
{
    return cb_internal_copy_control_port(host, out, capacity);
}

int cb_get_debug_port(const cb_t *host, char *out, size_t capacity)
{
    return cb_internal_copy_debug_port(host, out, capacity);
}

int cb_get_device_name(const cb_t *host, char *out, size_t capacity)
{
    return cb_internal_copy_device_name(host, out, capacity);
}

int cb_get_metrics_json(const cb_t *host, char *out, size_t capacity)
{
    return cb_internal_copy_metrics(host, out, capacity);
}

int cb_get_last_error(const cb_t *host, char *out, size_t capacity)
{
    return cb_internal_copy_error(host, out, capacity);
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint64_t read_u64_le(const uint8_t *p)
{
    uint64_t value = 0;
    unsigned int i;
    for (i = 0; i < 8; ++i)
        value |= ((uint64_t)p[i]) << (8u * i);
    return value;
}

int cb_decode_device_config(const uint8_t *payload, size_t payload_size,
                            cb_device_config *out_config)
{
    uint32_t count, i;
    if (!payload || !out_config || payload_size < 24 || read_u32_le(payload) != 1)
        return -EINVAL;
    count = read_u32_le(payload + 20);
    if (count > CB_MAX_FRAME_RATES || payload_size != 24u + count * 4u)
        return -EINVAL;
    memset(out_config, 0, sizeof(*out_config));
    out_config->valid_flags = read_u32_le(payload + 4);
    out_config->frame_rate = read_u32_le(payload + 8);
    out_config->imu_output_enabled = read_u32_le(payload + 12);
    out_config->antenna_wired = read_u32_le(payload + 16);
    out_config->frame_rate_count = count;
    if (out_config->imu_output_enabled > 1 || out_config->antenna_wired > 1 ||
        ((out_config->valid_flags & CB_CONFIG_FRAME_RATE_VALID) && out_config->frame_rate == 0))
        return -EINVAL;
    for (i = 0; i < count; ++i)
    {
        out_config->frame_rates[i] = read_u32_le(payload + 24u + i * 4u);
        if (out_config->frame_rates[i] == 0)
            return -EINVAL;
    }
    return 0;
}

int cb_decode_imu_batch(const uint8_t *payload, size_t payload_size, cb_imu_sample *out_samples,
                        size_t *inout_sample_count)
{
    uint32_t count, sample_size;
    size_t i;
    if (!payload || !inout_sample_count || payload_size < 8)
        return -EINVAL;
    count = read_u32_le(payload);
    sample_size = read_u32_le(payload + 4);
    if (sample_size < 56 || payload_size < 8u + (size_t)count * sample_size)
        return -EINVAL;
    if (!out_samples || *inout_sample_count < count)
    {
        *inout_sample_count = count;
        return -ENOSPC;
    }
    for (i = 0; i < count; ++i)
    {
        const uint8_t *p = payload + 8u + i * sample_size;
        uint32_t bits;
        unsigned int axis;
        out_samples[i].sequence = read_u64_le(p);
        out_samples[i].timestamp_monotonic_ns = read_u64_le(p + 8);
        for (axis = 0; axis < 3; ++axis)
        {
            memcpy(&bits, p + 16u + axis * 4u, sizeof(bits));
            memcpy(&out_samples[i].accel[axis], &bits, sizeof(bits));
            memcpy(&bits, p + 28u + axis * 4u, sizeof(bits));
            memcpy(&out_samples[i].gyro[axis], &bits, sizeof(bits));
        }
        memcpy(&bits, p + 40, sizeof(bits));
        memcpy(&out_samples[i].temperature, &bits, sizeof(bits));
        out_samples[i].flags = read_u32_le(p + 44);
    }
    *inout_sample_count = count;
    return 0;
}

int cb_decode_frame_meta(const uint8_t *payload, size_t payload_size, cb_frame_meta *out_frames,
                         size_t *inout_frame_count)
{
    uint32_t count, item_size;
    size_t i;
    if (!payload || !inout_frame_count || payload_size < 8)
        return -EINVAL;
    count = read_u32_le(payload);
    item_size = read_u32_le(payload + 4);
    if (item_size < 40 || payload_size < 8u + (size_t)count * item_size)
        return -EINVAL;
    if (!out_frames || *inout_frame_count < count)
    {
        *inout_frame_count = count;
        return -ENOSPC;
    }
    for (i = 0; i < count; ++i)
    {
        const uint8_t *p = payload + 8u + i * item_size;
        out_frames[i].sequence = read_u64_le(p);
        out_frames[i].timestamp_monotonic_ns = read_u64_le(p + 8);
        out_frames[i].payload_size = read_u32_le(p + 16);
        out_frames[i].width = read_u32_le(p + 20);
        out_frames[i].height = read_u32_le(p + 24);
        out_frames[i].stride = read_u32_le(p + 28);
        out_frames[i].format = read_u32_le(p + 32);
        out_frames[i].flags = read_u32_le(p + 36);
    }
    *inout_frame_count = count;
    return 0;
}

static float read_float_le(const uint8_t *p)
{
    uint32_t n = read_u32_le(p);
    float f;
    memcpy(&f, &n, 4);
    return f;
}
int cb_request_stereo_status(cb_t *host, uint64_t id)
{
    return send_empty(host, CB_STEREO_STATUS_REQ, id);
}
int cb_set_disparity_enabled(cb_t *host, int enabled, uint64_t id)
{
    const uint8_t b = enabled ? 1 : 0;
    return cb_internal_request(host, CB_STEREO_ENABLE_REQ, &b, 1, id);
}
int cb_set_disparity_config(cb_t *host, uint32_t mode, float focal, uint64_t id)
{
    uint8_t p[8];
    uint32_t bits;
    if (!mode || !isfinite(focal) || focal <= 0)
        return -EINVAL;
    memcpy(&bits, &focal, 4);
    for (int i = 0; i < 4; ++i)
    {
        p[i] = (uint8_t)(mode >> (8 * i));
        p[4 + i] = (uint8_t)(bits >> (8 * i));
    }
    return cb_internal_request(host, CB_STEREO_CONFIG_REQ, p, 8, id);
}
int cb_set_disparity_config_ex(cb_t *host, uint32_t mode, float focal, uint32_t projection,
                               float fov, uint64_t id)
{
    uint8_t p[16];
    uint32_t fields[4];
    if (!mode || !isfinite(focal) || focal <= 0 || projection > 1 || !isfinite(fov) || fov < 30 ||
        fov > 180)
        return -EINVAL;
    fields[0] = mode;
    memcpy(fields + 1, &focal, 4);
    fields[2] = projection;
    memcpy(fields + 3, &fov, 4);
    for (int j = 0; j < 4; ++j)
        for (int i = 0; i < 4; ++i)
            p[4 * j + i] = (uint8_t)(fields[j] >> (8 * i));
    return cb_internal_request(host, CB_STEREO_CONFIG_REQ, p, 16, id);
}
int cb_decode_stereo_status(const uint8_t *p, size_t n, cb_stereo_status *out)
{
    uint32_t count, version, stride, header;
    if (!p || !out || n < 25)
        return -EINVAL;
    version = read_u32_le(p);
    if (version < 1 || version > 3)
        return -EINVAL;
    stride = version == 3 ? 28u : version == 2 ? 24u : 20u;
    header = version == 3 ? 32u : 24u;
    count = read_u32_le(p + 20);
    if (count > CB_MAX_STEREO_MODES || n < header + 1 + count * stride ||
        n > header + count * stride + 256 || p[n - 1] != 0)
        return -EINVAL;
    memset(out, 0, sizeof(*out));
    out->flags = read_u32_le(p + 4);
    out->error = (int32_t)read_u32_le(p + 8);
    out->mode_id = read_u32_le(p + 12);
    out->focal_px = read_float_le(p + 16);
    out->mode_count = count;
    out->projection = version == 3 ? read_u32_le(p + 24) : 0;
    out->vertical_fov_deg = version == 3 ? read_float_le(p + 28) : 180;
    if (!isfinite(out->focal_px) || out->focal_px <= 0 || out->projection > 1 ||
        !isfinite(out->vertical_fov_deg) || out->vertical_fov_deg < 30 ||
        out->vertical_fov_deg > 180)
        return -EINVAL;
    for (uint32_t i = 0; i < count; ++i)
    {
        const uint8_t *m = p + header + i * stride;
        cb_stereo_mode *d = out->modes + i;
        d->id = read_u32_le(m);
        d->width = read_u32_le(m + 4);
        d->height = read_u32_le(m + 8);
        d->network_width = read_u32_le(m + 12);
        d->network_height = read_u32_le(m + 16);
        d->max_fps = version >= 2 ? read_u32_le(m + 20) : 20u;
        d->projection = version == 3 ? read_u32_le(m + 24) : 0;
        if (!d->id || !d->width || !d->height || d->width > CB_MAX_DISPARITY_WIDTH ||
            d->height > CB_MAX_DISPARITY_HEIGHT || d->network_width < d->width ||
            d->network_height < d->height || d->network_width > CB_MAX_DISPARITY_WIDTH ||
            d->network_height > CB_MAX_DISPARITY_HEIGHT || d->projection > 1 ||
            (d->max_fps != 20 && d->max_fps != 15 && d->max_fps != 10))
            return -EINVAL;
    }
    memcpy(out->message, p + header + count * stride, n - header - count * stride);
    return 0;
}
float cb_fp16_to_float(uint16_t half)
{
    uint32_t sign = (uint32_t)(half & 0x8000u) << 16;
    uint32_t exponent = (half >> 10) & 31u, fraction = half & 1023u, bits;
    if (exponent == 0)
    {
        bits = sign;
        if (fraction)
        {
            int e = -14;
            while (!(fraction & 1024u))
            {
                fraction <<= 1;
                --e;
            }
            bits |= (uint32_t)(e + 127) << 23;
            bits |= (fraction & 1023u) << 13;
        }
    }
    else if (exponent == 31)
    {
        bits = sign | 0x7f800000u | (fraction << 13);
    }
    else
    {
        bits = sign | ((exponent + 112u) << 23) | (fraction << 13);
    }
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}
int cb_decode_disparity(const uint8_t *p, size_t n, cb_disparity *out, const uint8_t **disparity,
                        const uint8_t **rgb)
{
    size_t pixels;
    uint32_t version, header;
    if (!p || !out || n < 64)
        return -EINVAL;
    version = read_u32_le(p);
    if (version != 2 && version != 3)
        return -EINVAL;
    header = version == 3 ? 80 : 64;
    if (n < header)
        return -EINVAL;
    memset(out, 0, sizeof(*out));
    out->generation = read_u32_le(p + 4);
    out->frame_id = read_u64_le(p + 8);
    out->timestamp_monotonic_ns = read_u64_le(p + 16);
    out->mode_id = read_u32_le(p + 24);
    out->width = read_u32_le(p + 28);
    out->height = read_u32_le(p + 32);
    out->focal_px = read_float_le(p + 36);
    out->cx = read_float_le(p + 40);
    out->cy = read_float_le(p + 44);
    out->baseline_metres = read_float_le(p + 48);
    out->disparity_format = read_u32_le(p + 52);
    if (!out->frame_id || !out->width || !out->height || out->width > CB_MAX_DISPARITY_WIDTH ||
        out->height > CB_MAX_DISPARITY_HEIGHT || out->disparity_format != CB_DISPARITY_FP16 ||
        !isfinite(out->focal_px) || out->focal_px <= 0 || !isfinite(out->cx) ||
        !isfinite(out->cy) || !isfinite(out->baseline_metres) || out->baseline_metres <= 0)
        return -EINVAL;
    out->projection = version == 3 ? read_u32_le(p + 64) : 0;
    out->fy = version == 3 ? read_float_le(p + 68) : out->focal_px;
    out->vertical_fov_deg = version == 3 ? read_float_le(p + 72) : 0;
    if (out->projection > 1 || !isfinite(out->fy) || out->fy <= 0 ||
        (version == 3 && read_u32_le(p + 76) != 80) || !isfinite(out->vertical_fov_deg) ||
        (out->projection == 1 && (out->vertical_fov_deg < 30 || out->vertical_fov_deg > 180)))
        return -EINVAL;
    pixels = (size_t)out->width * out->height;
    if (read_u32_le(p + 56) != pixels * 2 || read_u32_le(p + 60) != pixels * 2 ||
        n != header + pixels * 4)
        return -EINVAL;
    if (disparity)
        *disparity = p + header;
    if (rgb)
        *rgb = p + header + pixels * 2;
    return 0;
}

int cb_disparity_to_point(const cb_disparity *m, float u, float v, float d, float xyz[3])
{
    if (!m || !xyz || m->projection > 1 || !isfinite(u) || !isfinite(v) || !isfinite(d) || d <= 0 ||
        !isfinite(m->focal_px) || m->focal_px <= 0 || !isfinite(m->fy) || m->fy <= 0 ||
        !isfinite(m->cx) || !isfinite(m->cy) || !isfinite(m->baseline_metres) ||
        m->baseline_metres <= 0)
        return -EINVAL;
    const float r = m->focal_px * m->baseline_metres / d, b = (v - m->cy) / m->fy;
    xyz[0] = (u - m->cx) * r / m->focal_px;
    xyz[1] = m->projection == 1 ? r * sinf(b) : r * b;
    xyz[2] = m->projection == 1 ? r * cosf(b) : r;
    return isfinite(xyz[0]) && isfinite(xyz[1]) && isfinite(xyz[2]) && xyz[2] > 0 ? 0 : -EINVAL;
}
