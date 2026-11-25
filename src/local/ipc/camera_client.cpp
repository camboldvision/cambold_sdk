// SPDX-License-Identifier: Apache-2.0

#include "camera_client.h"

#include "camera_ipc.hpp"

#include <cerrno>
#include <cstring>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <vector>

struct camera_client
{
    CameraIpcConnection connection;
    CameraIpcRings rings;
    std::array<uint64_t, 2> rgb_cursor{};
    uint64_t imu_cursor = 0;
    uint64_t jpeg_cursor = 0;
    std::array<std::vector<uint8_t>, 2> rgb_buffers;
    std::vector<uint8_t> jpeg_buffer;
    std::vector<uint8_t> imu_buffer;
    CameraIpcRecord record{};
};

extern "C" camera_client_t *camera_client_open(const char *socket_path, uint32_t role,
                                               uint32_t stream_mask)
{
    if (!socket_path)
        return nullptr;
    auto *client = new camera_client;
    const int ret =
        client->connection.connect(socket_path, role, stream_mask, &client->rings, nullptr);
    if (ret)
    {
        delete client;
        return nullptr;
    }
    for (auto &buffer : client->rgb_buffers)
        buffer.resize(client->rings.payload_capacity(CAMERA_BROKER_RING_LEFT_RGB));
    client->imu_buffer.resize(sizeof(camera_broker_imu_sample));
    client->jpeg_buffer.resize(client->rings.payload_capacity(CAMERA_BROKER_RING_STEREO_JPEG));
    return client;
}

extern "C" void camera_client_close(camera_client_t *client)
{
    delete client;
}

extern "C" int camera_client_wait(camera_client_t *client, int timeout_ms)
{
    if (!client)
        return -EINVAL;
    pollfd pfd{client->rings.notification_fd(), POLLIN, 0};
    for (;;)
    {
        const int ret = poll(&pfd, 1, timeout_ms);
        if (ret < 0 && errno == EINTR)
            continue;
        if (ret < 0)
            return -errno;
        if (ret == 0)
            return 0;
        uint64_t value;
        while (read(pfd.fd, &value, sizeof(value)) == sizeof(value))
        {
        }
        return 1;
    }
}

extern "C" int camera_client_read_latest_image(camera_client_t *client, int camera_index,
                                             struct camera_client_frame *frame)
{
    if (!client || !frame || camera_index < 0 || camera_index > 1)
        return -EINVAL;
    const uint32_t ring_id =
        camera_index == 0 ? CAMERA_BROKER_RING_LEFT_RGB : CAMERA_BROKER_RING_RIGHT_RGB;
    if (!client->rings.read_latest(ring_id, &client->rgb_cursor[camera_index],
                                   client->rgb_buffers[camera_index].data(),
                                   client->rgb_buffers[camera_index].size(), &client->record))
        return -EAGAIN;
    frame->data = client->rgb_buffers[camera_index].data();
    frame->bytes = client->record.payload_size;
    frame->width = client->record.width;
    frame->height = client->record.height;
    frame->stride = client->record.stride;
    frame->format = client->record.format;
    frame->sequence = client->record.sequence;
    frame->timestamp_monotonic_ns = client->record.timestamp_monotonic_ns;
    frame->flags = client->record.flags;
    return 0;
}

extern "C" int camera_client_read_latest_rgb(camera_client_t *client, int camera_index,
                                             struct camera_client_frame *frame)
{
    const int result = camera_client_read_latest_image(client, camera_index, frame);
    if (result)
        return result;
    return frame->format == CAMERA_BROKER_FORMAT_RGB24 ? 0 : -EAGAIN;
}

extern "C" int camera_client_read_next_imu(camera_client_t *client,
                                           struct camera_client_imu *sample)
{
    if (!client || !sample)
        return -EINVAL;
    if (!client->rings.read_next(CAMERA_BROKER_RING_IMU, &client->imu_cursor,
                                 client->imu_buffer.data(), client->imu_buffer.size(),
                                 &client->record))
        return -EAGAIN;
    if (client->record.payload_size != sizeof(camera_broker_imu_sample))
        return -EPROTO;
    camera_broker_imu_sample raw{};
    std::memcpy(&raw, client->imu_buffer.data(), sizeof(raw));
    sample->sequence = raw.sequence;
    sample->timestamp_monotonic_ns = raw.timestamp_monotonic_ns;
    std::memcpy(sample->accel, raw.accel, sizeof(sample->accel));
    std::memcpy(sample->gyro, raw.gyro, sizeof(sample->gyro));
    sample->temperature = raw.temperature;
    sample->flags = raw.flags;
    return 0;
}

extern "C" int camera_client_read_latest_jpeg(camera_client_t *client,
                                              struct camera_client_frame *frame)
{
    if (!client || !frame)
        return -EINVAL;
    if (!client->rings.read_latest(CAMERA_BROKER_RING_STEREO_JPEG, &client->jpeg_cursor,
                                   client->jpeg_buffer.data(), client->jpeg_buffer.size(),
                                   &client->record))
        return -EAGAIN;
    frame->data = client->jpeg_buffer.data();
    frame->bytes = client->record.payload_size;
    frame->width = client->record.width;
    frame->height = client->record.height;
    frame->stride = client->record.stride;
    frame->format = client->record.format;
    frame->sequence = client->record.sequence;
    frame->timestamp_monotonic_ns = client->record.timestamp_monotonic_ns;
    frame->flags = client->record.flags;
    return 0;
}

extern "C" int camera_client_jpeg_get_timestamp(const uint8_t *jpeg, size_t bytes,
                                                uint64_t *sequence,
                                                uint64_t *timestamp_monotonic_ns, uint32_t *width,
                                                uint32_t *height)
{
    if (!jpeg || bytes < 2 || jpeg[0] != 0xff || jpeg[1] != 0xd8)
        return -EINVAL;
    for (size_t offset = 2; offset + 4 <= bytes;)
    {
        if (jpeg[offset] != 0xff)
            return -ENOENT;
        while (offset < bytes && jpeg[offset] == 0xff)
            ++offset;
        if (offset >= bytes)
            return -ENOENT;
        const uint8_t marker = jpeg[offset++];
        if (marker == 0xda || marker == 0xd9)
            return -ENOENT;
        if (marker == 0xd8 || (marker >= 0xd0 && marker <= 0xd7))
            continue;
        if (offset + 2 > bytes)
            return -ENOENT;
        const size_t length = (static_cast<size_t>(jpeg[offset]) << 8) | jpeg[offset + 1];
        if (length < 2 || offset + length > bytes)
            return -ENOENT;
        if (marker == 0xe1 && length >= 34 && std::memcmp(jpeg + offset + 2, "SCAM", 4) == 0 &&
            jpeg[offset + 6] == 1)
        {
            const uint8_t *payload = jpeg + offset + 2;
            auto get_le32 = [](const uint8_t *p)
            {
                return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                       (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
            };
            auto get_le64 = [](const uint8_t *p)
            {
                uint64_t value = 0;
                for (unsigned int i = 0; i < 8; ++i)
                    value |= static_cast<uint64_t>(p[i]) << (i * 8U);
                return value;
            };
            if (sequence)
                *sequence = get_le64(payload + 8);
            if (timestamp_monotonic_ns)
                *timestamp_monotonic_ns = get_le64(payload + 16);
            if (width)
                *width = get_le32(payload + 24);
            if (height)
                *height = get_le32(payload + 28);
            return (payload[5] & 1U) != 0 ? 0 : -ENOENT;
        }
        offset += length;
    }
    return -ENOENT;
}

static int send_subscription(camera_client_t *client, uint16_t type, uint32_t mask)
{
    if (!client || client->connection.fd() < 0)
        return -EINVAL;
    camera_broker_message message{};
    message.magic = CAMERA_BROKER_MAGIC;
    message.version = CAMERA_BROKER_VERSION;
    message.type = type;
    message.size = sizeof(message);
    message.stream_mask = mask;
    if (send(client->connection.fd(), &message, sizeof(message), MSG_NOSIGNAL) !=
        static_cast<ssize_t>(sizeof(message)))
        return -errno;
    return 0;
}

extern "C" int camera_client_subscribe(camera_client_t *client, uint32_t stream_mask)
{
    return send_subscription(client, CAMERA_BROKER_SUBSCRIBE, stream_mask);
}

extern "C" int camera_client_unsubscribe(camera_client_t *client, uint32_t stream_mask)
{
    return send_subscription(client, CAMERA_BROKER_UNSUBSCRIBE, stream_mask);
}

extern "C" int camera_client_fd(const camera_client_t *client)
{
    return client ? client->connection.fd() : -EINVAL;
}
