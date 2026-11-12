// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "svio/transport.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace cambold_ipc
{
constexpr const char *slam_socket = "/run/cambold-slam/control.sock";
inline sockaddr_un slam_address(const std::string &path)
{
    sockaddr_un a{};
    a.sun_family = AF_UNIX;
    if (path.size() >= sizeof(a.sun_path))
        throw std::runtime_error("SLAM socket path too long");
    std::memcpy(a.sun_path, path.c_str(), path.size() + 1);
    return a;
}
class SlamSocket final : public svio::Transport
{
    int fd_ = -1;

  public:
    explicit SlamSocket(int fd) : fd_(fd)
    {
        int size = 8192;
        setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
    }
    ~SlamSocket() override
    {
        close();
    }
    std::size_t read(std::uint8_t *p, std::size_t n, int timeout) override
    {
        pollfd item{fd_, POLLIN, 0};
        const int ready = poll(&item, 1, timeout);
        if (ready < 0 && errno == EINTR)
            return 0;
        if (ready < 0)
            throw std::runtime_error("SLAM socket poll failed");
        if (!ready)
            return 0;
        const auto count = recv(fd_, p, n, 0);
        if (count < 0 && (errno == EAGAIN || errno == EINTR))
            return 0;
        if (count <= 0)
            throw std::runtime_error("SLAM local link disconnected");
        return std::size_t(count);
    }
    std::size_t write(const std::uint8_t *p, std::size_t n, int timeout) override
    {
        pollfd item{fd_, POLLOUT, 0};
        const int ready = poll(&item, 1, timeout);
        if (ready < 0 && errno == EINTR)
            return 0;
        if (ready < 0)
            throw std::runtime_error("SLAM socket poll failed");
        if (!ready)
            return 0;
        const auto count = send(fd_, p, n, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (count < 0 && (errno == EAGAIN || errno == EINTR))
            return 0;
        if (count <= 0)
            throw std::runtime_error("SLAM local write failed");
        return std::size_t(count);
    }
    void close() override
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }
};
inline std::unique_ptr<svio::Transport> connect_slam(const std::string &path = slam_socket)
{
    const auto a = slam_address(path);
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0)
        throw std::runtime_error("Cannot allocate SLAM socket");
    if (connect(fd, reinterpret_cast<const sockaddr *>(&a), sizeof(a)))
    {
        ::close(fd);
        throw std::runtime_error("SLAM service unavailable");
    }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    return std::make_unique<SlamSocket>(fd);
}
class SlamListener
{
    int fd_ = -1;
    std::string path_;

  public:
    explicit SlamListener(std::string path = slam_socket) : path_(std::move(path))
    {
        const auto a = slam_address(path_);
        fd_ = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        unlink(path_.c_str());
        if (fd_ < 0 || bind(fd_, reinterpret_cast<const sockaddr *>(&a), sizeof(a)) ||
            listen(fd_, 2))
        {
            if (fd_ >= 0)
                ::close(fd_);
            throw std::runtime_error("Cannot listen on SLAM socket");
        }
        chmod(path_.c_str(), 0660);
    }
    ~SlamListener()
    {
        if (fd_ >= 0)
            ::close(fd_);
        unlink(path_.c_str());
    }
    std::unique_ptr<svio::Transport> accept(int timeout = 200)
    {
        pollfd p{fd_, POLLIN, 0};
        if (poll(&p, 1, timeout) <= 0)
            return {};
        const int peer = accept4(fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
        return peer < 0 ? nullptr : std::make_unique<SlamSocket>(peer);
    }
};
} // namespace cambold_ipc
