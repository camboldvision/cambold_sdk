// SPDX-License-Identifier: Apache-2.0

#include "camera_ipc.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/memfd.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <limits>

namespace
{

constexpr uint32_t kRgbPayloadCapacity = 4U * 1024U * 1024U;
constexpr uint32_t kJpegPayloadCapacity = 4U * 1024U * 1024U;
constexpr uint32_t kImuPayloadCapacity = sizeof(camera_broker_imu_sample);

int create_memfd(const char *name)
{
#ifdef SYS_memfd_create
    return static_cast<int>(syscall(SYS_memfd_create, name, MFD_CLOEXEC));
#else
    (void)name;
    errno = ENOSYS;
    return -1;
#endif
}

bool valid_ring_id(uint32_t id)
{
    return id < CAMERA_BROKER_RING_COUNT;
}

uint32_t ring_capacity(uint32_t id)
{
    switch (id)
    {
    case CAMERA_BROKER_RING_LEFT_RGB:
    case CAMERA_BROKER_RING_RIGHT_RGB:
        return kRgbPayloadCapacity;
    case CAMERA_BROKER_RING_STEREO_JPEG:
        return kJpegPayloadCapacity;
    case CAMERA_BROKER_RING_IMU:
        return kImuPayloadCapacity;
    default:
        return 0;
    }
}

size_t ring_mapping_size(uint32_t slot_count, uint32_t slot_size)
{
    const size_t header = sizeof(camera_broker_ring_header);
    if (slot_size == 0 || slot_count == 0 ||
        slot_count > (std::numeric_limits<size_t>::max() - header) / slot_size)
        return 0;
    return header + static_cast<size_t>(slot_count) * slot_size;
}

void log_errno(FILE *log, const char *what)
{
    if (log)
        std::fprintf(log, "camera IPC: %s: %s\n", what, strerror(errno));
}

} // namespace

struct CameraIpcRings::Ring
{
    int fd = -1;
    void *mapping = MAP_FAILED;
    size_t mapping_size = 0;
    uint32_t payload_capacity = 0;
    camera_broker_ring_header *header = nullptr;
};

CameraIpcRings::CameraIpcRings() = default;

CameraIpcRings::~CameraIpcRings()
{
    close_all();
}

void CameraIpcRings::close_all()
{
    for (Ring *&ring : rings_)
    {
        if (!ring)
            continue;
        if (ring->mapping != MAP_FAILED)
            munmap(ring->mapping, ring->mapping_size);
        if (ring->fd >= 0)
            close(ring->fd);
        delete ring;
        ring = nullptr;
    }
    if (notify_fd_ >= 0)
        close(notify_fd_);
    notify_fd_ = -1;
    owner_ = false;
    writer_ = false;
    valid_ = false;
}

int CameraIpcRings::create_owner(uint32_t rgb_slot_count, uint32_t jpeg_slot_count,
                                 uint32_t imu_slot_count, FILE *log)
{
    close_all();
    log_ = log;
    if (rgb_slot_count < 2 || jpeg_slot_count < 2 || imu_slot_count < 8)
        return -EINVAL;

    notify_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (notify_fd_ < 0)
    {
        log_errno(log, "eventfd");
        return -errno;
    }

    for (uint32_t id = 0; id < CAMERA_BROKER_RING_COUNT; ++id)
    {
        const uint32_t count = id == CAMERA_BROKER_RING_IMU           ? imu_slot_count
                               : id == CAMERA_BROKER_RING_STEREO_JPEG ? jpeg_slot_count
                                                                      : rgb_slot_count;
        const uint32_t capacity = ring_capacity(id);
        if (capacity == 0 || capacity > UINT32_MAX - sizeof(camera_broker_ring_slot))
        {
            close_all();
            return -EOVERFLOW;
        }
        auto *ring = new Ring;
        ring->payload_capacity = capacity;
        const uint32_t slot_size =
            static_cast<uint32_t>(sizeof(camera_broker_ring_slot) + capacity);
        ring->mapping_size = ring_mapping_size(count, slot_size);
        if (ring->mapping_size == 0)
        {
            delete ring;
            close_all();
            return -EOVERFLOW;
        }
        char name[64];
        std::snprintf(name, sizeof(name), "cambold-camera-%u", id);
        ring->fd = create_memfd(name);
        if (ring->fd < 0 || ftruncate(ring->fd, static_cast<off_t>(ring->mapping_size)) < 0)
        {
            log_errno(log, "memfd/ftruncate");
            delete ring;
            close_all();
            return -errno;
        }
        ring->mapping =
            mmap(nullptr, ring->mapping_size, PROT_READ | PROT_WRITE, MAP_SHARED, ring->fd, 0);
        if (ring->mapping == MAP_FAILED)
        {
            log_errno(log, "mmap ring");
            delete ring;
            close_all();
            return -errno;
        }
        std::memset(ring->mapping, 0, ring->mapping_size);
        ring->header = static_cast<camera_broker_ring_header *>(ring->mapping);
        ring->header->magic = CAMERA_BROKER_MAGIC;
        ring->header->version = CAMERA_BROKER_VERSION;
        ring->header->ring_id = static_cast<uint16_t>(id);
        ring->header->header_size = sizeof(camera_broker_ring_header);
        ring->header->slot_count = count;
        ring->header->slot_size = slot_size;
        ring->header->write_sequence = 0;
        ring->header->consumer_stream_mask = 0;
        ring->header->reserved = 0;
        rings_[id] = ring;
    }
    owner_ = true;
    writer_ = true;
    valid_ = true;
    return 0;
}

int CameraIpcRings::send_offer(int socket_fd, uint32_t role, uint32_t stream_mask,
                               int notify_fd_override) const
{
    (void)role;
    if (!valid_ || socket_fd < 0)
        return -EINVAL;
    camera_broker_fd_offer offer{};
    offer.message.magic = CAMERA_BROKER_MAGIC;
    offer.message.version = CAMERA_BROKER_VERSION;
    offer.message.type = CAMERA_BROKER_FD_OFFER;
    offer.message.size = sizeof(offer);
    offer.message.stream_mask = stream_mask;
    offer.ring_mask = (1U << CAMERA_BROKER_RING_COUNT) - 1U;
    offer.slot_count = rings_[0]->header->slot_count;
    offer.slot_size = rings_[0]->header->slot_size;

    int fds[CAMERA_BROKER_FD_COUNT];
    for (uint32_t i = 0; i < CAMERA_BROKER_RING_COUNT; ++i)
    {
        if (!rings_[i] || rings_[i]->fd < 0)
            return -EINVAL;
        fds[i] = rings_[i]->fd;
    }
    fds[CAMERA_BROKER_RING_COUNT] = notify_fd_override >= 0 ? notify_fd_override : notify_fd_;

    char control[CMSG_SPACE(sizeof(fds))]{};
    iovec iov{&offer, sizeof(offer)};
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control;
    msg.msg_controllen = sizeof(control);
    cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(fds));
    std::memcpy(CMSG_DATA(cmsg), fds, sizeof(fds));
    msg.msg_controllen = sizeof(control);
    const ssize_t sent = sendmsg(socket_fd, &msg, MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(sizeof(offer)))
        return -errno;
    return 0;
}

int CameraIpcRings::receive_offer(int socket_fd, bool writer, FILE *log)
{
    close_all();
    log_ = log;
    if (socket_fd < 0)
        return -EINVAL;
    camera_broker_fd_offer offer{};
    int fds[CAMERA_BROKER_FD_COUNT]{};
    char control[CMSG_SPACE(sizeof(fds))]{};
    iovec iov{&offer, sizeof(offer)};
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control;
    msg.msg_controllen = sizeof(control);
    const ssize_t received = recvmsg(socket_fd, &msg, MSG_CMSG_CLOEXEC);
    if (received != static_cast<ssize_t>(sizeof(offer)) ||
        offer.message.magic != CAMERA_BROKER_MAGIC || offer.message.type != CAMERA_BROKER_FD_OFFER)
    {
        if (log)
            std::fprintf(log, "camera IPC: invalid FD_OFFER\n");
        return -EPROTO;
    }

    size_t fd_count = 0;
    for (cmsghdr *cmsg = CMSG_FIRSTHDR(&msg); cmsg; cmsg = CMSG_NXTHDR(&msg, cmsg))
    {
        if (cmsg->cmsg_level != SOL_SOCKET || cmsg->cmsg_type != SCM_RIGHTS)
            continue;
        const size_t bytes = cmsg->cmsg_len - CMSG_LEN(0);
        if (bytes % sizeof(int) != 0)
            return -EPROTO;
        fd_count = std::min(bytes / sizeof(int), static_cast<size_t>(CAMERA_BROKER_FD_COUNT));
        std::memcpy(fds, CMSG_DATA(cmsg), fd_count * sizeof(int));
        break;
    }
    if (fd_count != CAMERA_BROKER_FD_COUNT)
        return -EPROTO;

    for (uint32_t id = 0; id < CAMERA_BROKER_RING_COUNT; ++id)
    {
        auto *ring = new Ring;
        ring->fd = fds[id];
        ring->mapping = mmap(nullptr, sizeof(camera_broker_ring_header),
                             writer ? PROT_READ | PROT_WRITE : PROT_READ, MAP_SHARED, ring->fd, 0);
        if (ring->mapping == MAP_FAILED)
        {
            log_errno(log, "mmap ring header");
            delete ring;
            close_all();
            return -errno;
        }
        // Read the ring dimensions before remapping the complete file. The
        // header is at offset zero and is always within the first page.
        const auto *header = static_cast<const camera_broker_ring_header *>(ring->mapping);
        if (header->magic != CAMERA_BROKER_MAGIC || header->ring_id != id ||
            header->header_size != sizeof(*header) || header->slot_count == 0 ||
            header->slot_size < sizeof(camera_broker_ring_slot))
        {
            munmap(ring->mapping, sizeof(camera_broker_ring_header));
            delete ring;
            close_all();
            return -EPROTO;
        }
        const uint32_t slot_count = header->slot_count;
        const uint32_t slot_size = header->slot_size;
        ring->mapping_size = ring_mapping_size(slot_count, slot_size);
        ring->payload_capacity = slot_size - sizeof(camera_broker_ring_slot);
        munmap(ring->mapping, sizeof(camera_broker_ring_header));
        ring->mapping = mmap(nullptr, ring->mapping_size,
                             writer ? PROT_READ | PROT_WRITE : PROT_READ, MAP_SHARED, ring->fd, 0);
        if (ring->mapping == MAP_FAILED)
        {
            delete ring;
            close_all();
            return -errno;
        }
        ring->header = static_cast<camera_broker_ring_header *>(ring->mapping);
        rings_[id] = ring;
    }
    notify_fd_ = fds[CAMERA_BROKER_RING_COUNT];
    owner_ = false;
    writer_ = writer;
    valid_ = true;
    return 0;
}

size_t CameraIpcRings::payload_capacity(uint32_t ring_id) const
{
    if (!valid_ring_id(ring_id) || !rings_[ring_id])
        return 0;
    return rings_[ring_id]->payload_capacity;
}

void CameraIpcRings::set_consumer_stream_mask(uint32_t stream_mask)
{
    if (!owner_ || !valid_)
        return;
    for (Ring *ring : rings_)
    {
        if (ring && ring->header)
            __atomic_store_n(&ring->header->consumer_stream_mask, stream_mask, __ATOMIC_RELEASE);
    }
}

uint32_t CameraIpcRings::consumer_stream_mask() const
{
    if (!valid_ || !rings_[0] || !rings_[0]->header)
        return 0;
    return __atomic_load_n(&rings_[0]->header->consumer_stream_mask, __ATOMIC_ACQUIRE);
}

static bool publish_record(CameraIpcRings::Ring *ring, int notify_fd, const void *payload,
                           size_t payload_size, uint32_t width, uint32_t height, uint32_t stride,
                           uint32_t format, uint32_t flags, uint64_t timestamp_ns,
                           uint64_t *sequence_out)
{
    if (!ring || !ring->header || !payload || payload_size > ring->payload_capacity)
        return false;
    // There is one capture producer. Publish the ring sequence only after the
    // payload and slot metadata have been committed, otherwise a reader can
    // observe the new global sequence while the slot is still being copied.
    const uint64_t sequence =
        __atomic_load_n(&ring->header->write_sequence, __ATOMIC_RELAXED) + 1ULL;
    const uint32_t slot_index = static_cast<uint32_t>(sequence % ring->header->slot_count);
    auto *slot = reinterpret_cast<camera_broker_ring_slot *>(
        static_cast<uint8_t *>(ring->mapping) + ring->header->header_size +
        static_cast<size_t>(slot_index) * ring->header->slot_size);
    uint8_t *dst = reinterpret_cast<uint8_t *>(slot) + sizeof(*slot);
    std::memcpy(dst, payload, payload_size);
    slot->timestamp_monotonic_ns = timestamp_ns;
    slot->payload_size = static_cast<uint32_t>(payload_size);
    slot->width = width;
    slot->height = height;
    slot->stride = stride;
    slot->format = format;
    slot->flags = flags;
    __atomic_store_n(&slot->sequence, sequence, __ATOMIC_RELEASE);
    __atomic_store_n(&ring->header->write_sequence, sequence, __ATOMIC_RELEASE);
    if (sequence_out)
        *sequence_out = sequence;
    if (notify_fd >= 0)
    {
        const uint64_t one = 1;
        (void)write(notify_fd, &one, sizeof(one));
    }
    return true;
}

bool CameraIpcRings::publish_rgb(int camera_index, const uint8_t *data, size_t bytes,
                                 uint32_t width, uint32_t height, uint32_t stride,
                                 uint64_t timestamp_ns, uint64_t *sequence_out)
{
    if (!writer_ || camera_index < 0 || camera_index > 1)
        return false;
    return publish_record(
        rings_[camera_index == 0 ? CAMERA_BROKER_RING_LEFT_RGB : CAMERA_BROKER_RING_RIGHT_RGB],
        notify_fd_, data, bytes, width, height, stride, CAMERA_BROKER_FORMAT_RGB24, 0, timestamp_ns,
        sequence_out);
}

bool CameraIpcRings::publish_jpeg(const uint8_t *data, size_t bytes, uint32_t width,
                                  uint32_t height, uint64_t timestamp_ns, uint64_t *sequence_out)
{
    if (!writer_)
        return false;
    return publish_record(rings_[CAMERA_BROKER_RING_STEREO_JPEG], notify_fd_, data, bytes, width,
                          height, 0, CAMERA_BROKER_FORMAT_MJPEG, 0, timestamp_ns, sequence_out);
}

bool CameraIpcRings::publish_imu(const camera_broker_imu_sample &sample, uint64_t *sequence_out)
{
    if (!writer_)
        return false;
    return publish_record(rings_[CAMERA_BROKER_RING_IMU], notify_fd_, &sample, sizeof(sample), 0, 0,
                          sizeof(sample), CAMERA_BROKER_FORMAT_IMU, 0,
                          sample.timestamp_monotonic_ns, sequence_out);
}

static bool read_ring_sequence(const CameraIpcRings::Ring *ring, uint64_t sequence, void *payload,
                               size_t payload_capacity, CameraIpcRecord *record)
{
    if (!ring || !ring->header || !payload || !record || sequence == 0)
        return false;
    const uint32_t slot_index = static_cast<uint32_t>(sequence % ring->header->slot_count);
    auto *slot = reinterpret_cast<camera_broker_ring_slot *>(
        static_cast<uint8_t *>(ring->mapping) + ring->header->header_size +
        static_cast<size_t>(slot_index) * ring->header->slot_size);
    const uint64_t published = __atomic_load_n(&slot->sequence, __ATOMIC_ACQUIRE);
    if (published != sequence || slot->payload_size > payload_capacity ||
        slot->payload_size > ring->payload_capacity)
        return false;
    const uint32_t bytes = slot->payload_size;
    const uint8_t *src = reinterpret_cast<const uint8_t *>(slot) + sizeof(*slot);
    std::memcpy(payload, src, bytes);
    if (__atomic_load_n(&slot->sequence, __ATOMIC_ACQUIRE) != sequence)
        return false;
    record->sequence = sequence;
    record->timestamp_monotonic_ns = slot->timestamp_monotonic_ns;
    record->payload_size = bytes;
    record->width = slot->width;
    record->height = slot->height;
    record->stride = slot->stride;
    record->format = slot->format;
    record->flags = slot->flags;
    return true;
}

static bool read_ring_metadata(const CameraIpcRings::Ring *ring, uint64_t sequence,
                               CameraIpcRecord *record)
{
    if (!ring || !ring->header || !record || sequence == 0)
        return false;
    const uint32_t slot_index = static_cast<uint32_t>(sequence % ring->header->slot_count);
    auto *slot = reinterpret_cast<const camera_broker_ring_slot *>(
        static_cast<const uint8_t *>(ring->mapping) + ring->header->header_size +
        static_cast<size_t>(slot_index) * ring->header->slot_size);
    if (__atomic_load_n(&slot->sequence, __ATOMIC_ACQUIRE) != sequence)
        return false;
    record->sequence = sequence;
    record->timestamp_monotonic_ns = slot->timestamp_monotonic_ns;
    record->payload_size = slot->payload_size;
    record->width = slot->width;
    record->height = slot->height;
    record->stride = slot->stride;
    record->format = slot->format;
    record->flags = slot->flags;
    return __atomic_load_n(&slot->sequence, __ATOMIC_ACQUIRE) == sequence;
}

bool CameraIpcRings::read_latest(uint32_t ring_id, uint64_t *cursor, void *payload,
                                 size_t payload_capacity, CameraIpcRecord *record) const
{
    if (!valid_ || !valid_ring_id(ring_id) || !rings_[ring_id] || !cursor || !payload || !record)
        return false;
    Ring *ring = rings_[ring_id];
    const uint64_t newest = __atomic_load_n(&ring->header->write_sequence, __ATOMIC_ACQUIRE);
    if (newest == 0 || newest <= *cursor)
        return false;
    if (!read_ring_sequence(ring, newest, payload, payload_capacity, record))
        return false;
    *cursor = newest;
    return true;
}

bool CameraIpcRings::read_next(uint32_t ring_id, uint64_t *cursor, void *payload,
                               size_t payload_capacity, CameraIpcRecord *record) const
{
    if (!valid_ || !valid_ring_id(ring_id) || !rings_[ring_id] || !cursor)
        return false;
    Ring *ring = rings_[ring_id];
    const uint64_t newest = __atomic_load_n(&ring->header->write_sequence, __ATOMIC_ACQUIRE);
    if (newest == 0 || newest <= *cursor)
        return false;
    uint64_t next = *cursor + 1U;
    const uint64_t oldest =
        newest >= ring->header->slot_count ? newest - ring->header->slot_count + 1U : 1U;
    if (next < oldest)
        next = oldest;
    if (!read_ring_sequence(ring, next, payload, payload_capacity, record))
    {
        // The producer may have advanced while the caller was copying. Keep the
        // cursor at the last observed sequence so the next call can resync.
        return false;
    }
    *cursor = next;
    return true;
}

bool CameraIpcRings::read_next_metadata(uint32_t ring_id, uint64_t *cursor,
                                        CameraIpcRecord *record) const
{
    if (!valid_ || !valid_ring_id(ring_id) || !rings_[ring_id] || !cursor || !record)
        return false;
    Ring *ring = rings_[ring_id];
    const uint64_t newest = __atomic_load_n(&ring->header->write_sequence, __ATOMIC_ACQUIRE);
    if (newest == 0 || newest <= *cursor)
        return false;
    uint64_t next = *cursor + 1U;
    const uint64_t oldest =
        newest >= ring->header->slot_count ? newest - ring->header->slot_count + 1U : 1U;
    if (next < oldest)
        next = oldest;
    if (!read_ring_metadata(ring, next, record))
        return false;
    *cursor = next;
    return true;
}

CameraIpcConnection::~CameraIpcConnection()
{
    close();
}

int CameraIpcConnection::connect(const std::string &socket_path, uint32_t role,
                                 uint32_t stream_mask, CameraIpcRings *rings, FILE *log)
{
    close();
    if (!rings || socket_path.empty())
        return -EINVAL;
    fd_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (fd_ < 0)
        return -errno;
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (socket_path.size() >= sizeof(address.sun_path))
        return -ENAMETOOLONG;
    std::memcpy(address.sun_path, socket_path.c_str(), socket_path.size() + 1);
    if (::connect(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
    {
        const int saved = errno;
        if (log)
            std::fprintf(log, "camera IPC connect %s: %s\n", socket_path.c_str(), strerror(saved));
        close();
        return -saved;
    }
    camera_broker_hello hello{};
    hello.message.magic = CAMERA_BROKER_MAGIC;
    hello.message.version = CAMERA_BROKER_VERSION;
    hello.message.type = CAMERA_BROKER_HELLO;
    hello.message.size = sizeof(hello);
    hello.message.stream_mask = stream_mask;
    hello.role = static_cast<uint16_t>(role);
    hello.pid = static_cast<uint32_t>(getpid());
    if (send(fd_, &hello, sizeof(hello), MSG_NOSIGNAL) != static_cast<ssize_t>(sizeof(hello)))
    {
        const int saved = errno;
        close();
        return -saved;
    }
    const int ret = rings->receive_offer(fd_, role == CAMERA_BROKER_ROLE_CAPTURE_PUBLISHER, log);
    if (ret)
    {
        close();
        return ret;
    }
    return 0;
}

void CameraIpcConnection::close()
{
    if (fd_ >= 0)
    {
        shutdown(fd_, SHUT_RDWR);
        ::close(fd_);
        fd_ = -1;
    }
}
