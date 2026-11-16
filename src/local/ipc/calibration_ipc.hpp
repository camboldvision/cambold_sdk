// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>
namespace cb_calib {
constexpr const char *socket_path = "/run/soc-camera/calibration.sock";
constexpr const char *lock_path = "/run/soc-camera/calibration.active";
constexpr size_t max_record = 300 * 1024;
inline void put(std::vector<uint8_t> &v, uint64_t n, int bytes) {
    for (int i = 0; i < bytes; ++i)
        v.push_back(uint8_t(n >> (8 * i)));
}
inline uint64_t get(const uint8_t *p, int bytes) {
    uint64_t n = 0;
    for (int i = 0; i < bytes; ++i)
        n |= uint64_t(p[i]) << (8 * i);
    return n;
}
// Internal SOCK_SEQPACKET records: type u16, reserved u16, request u64, data.
inline std::vector<uint8_t> packet(uint16_t type, uint64_t request, const uint8_t *p, size_t n) {
    std::vector<uint8_t> v;
    v.reserve(n + 12);
    put(v, type, 2);
    put(v, 0, 2);
    put(v, request, 8);
    if (n)
        v.insert(v.end(), p, p + n);
    return v;
}
inline bool active() {
    int fd = open(lock_path, O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return false;
    bool busy = flock(fd, LOCK_EX | LOCK_NB) < 0 && (errno == EAGAIN || errno == EWOULDBLOCK);
    if (!busy)
        flock(fd, LOCK_UN);
    close(fd);
    return busy;
}
inline int connect() {
    int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (fd < 0)
        return -1;
    int size = 1024 * 1024;
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path, sizeof(address.sun_path) - 1);
    if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 &&
        errno != EINPROGRESS) {
        close(fd);
        return -1;
    }
    return fd;
}
} // namespace cb_calib
