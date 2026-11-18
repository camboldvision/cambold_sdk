// SPDX-License-Identifier: Apache-2.0
#include "stereo_local.hpp"
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace cambold_ipc
{
namespace
{
struct Shared
{
    std::uint64_t guard;
    StereoFrameInfo info;
};
constexpr auto memory_size = sizeof(Shared) + stereo_capacity;
struct Reply
{
    std::int32_t status;
    std::uint32_t bytes;
    char message[128];
};
struct ControlRequest
{
    std::uint32_t magic = stereo_magic, version = stereo_version, operation = 1;
    StereoSettings settings;
};
struct ControlReply
{
    std::uint32_t magic = stereo_magic, version = stereo_version;
    std::int32_t status = 0;
    StereoConfiguration configuration;
    char message[256]{};
};
sockaddr_un address(const std::string &path)
{
    sockaddr_un a{};
    a.sun_family = AF_UNIX;
    if (path.size() >= sizeof(a.sun_path))
        throw std::runtime_error("Stereo IPC path too long");
    std::memcpy(a.sun_path, path.c_str(), path.size() + 1);
    return a;
}
void close_fd(int &fd)
{
    if (fd >= 0)
        ::close(fd);
    fd = -1;
}
std::uint64_t monotonic_ns()
{
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return std::uint64_t(t.tv_sec) * 1000000000ull + t.tv_nsec;
}
void reply(int socket, int status, int memory)
{
    Reply r{};
    r.status = status;
    r.bytes = std::uint32_t(memory_size);
    if (status)
        std::snprintf(r.message, sizeof(r.message),
                      "Capture stereo unavailable (%d): check calibration, model and frame rate",
                      status);
    iovec io{&r, sizeof(r)};
    msghdr msg{};
    msg.msg_iov = &io;
    msg.msg_iovlen = 1;
    alignas(cmsghdr) char control[CMSG_SPACE(sizeof(int))]{};
    if (!status)
    {
        msg.msg_control = control;
        msg.msg_controllen = sizeof(control);
        auto *c = CMSG_FIRSTHDR(&msg);
        c->cmsg_level = SOL_SOCKET;
        c->cmsg_type = SCM_RIGHTS;
        c->cmsg_len = CMSG_LEN(sizeof(int));
        std::memcpy(CMSG_DATA(c), &memory, sizeof(int));
    }
    (void)sendmsg(socket, &msg, MSG_NOSIGNAL);
}
} // namespace
StereoServer::StereoServer(Lease lease, Latest latest, const std::string &path, Configure configure)
    : lease_(std::move(lease)), latest_(std::move(latest)), path_(path),
      configure_(std::move(configure))
{
    try
    {
        memory_ = int(syscall(SYS_memfd_create, "cambold-rectified-depth", 1));
        if (memory_ < 0 || ftruncate(memory_, memory_size))
            throw std::runtime_error("Cannot allocate stereo shared memory");
        mapping_ = mmap(nullptr, memory_size, PROT_READ | PROT_WRITE, MAP_SHARED, memory_, 0);
        if (mapping_ == MAP_FAILED)
        {
            mapping_ = nullptr;
            throw std::runtime_error("Cannot map stereo shared memory");
        }
        listener_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
        auto a = address(path_);
        unlink(path_.c_str());
        if (listener_ < 0 || bind(listener_, reinterpret_cast<sockaddr *>(&a), sizeof(a)) ||
            listen(listener_, 4))
            throw std::runtime_error("Cannot listen on " + path_);
        chmod(path_.c_str(), 0666);
        thread_ = std::thread(&StereoServer::run, this);
    }
    catch (...)
    {
        if (mapping_)
            munmap(mapping_, memory_size);
        close_fd(memory_);
        close_fd(listener_);
        throw;
    }
}
StereoServer::~StereoServer()
{
    stop_ = true;
    if (thread_.joinable())
        thread_.join();
    close_fd(listener_);
    if (mapping_)
        munmap(mapping_, memory_size);
    close_fd(memory_);
    unlink(path_.c_str());
}
void StereoServer::run()
{
    std::vector<int> peers;
    std::uint64_t revision = 0, guard = 0;
    bool lease_dense = true;
    while (!stop_)
    {
        pollfd listen{listener_, POLLIN, 0};
        poll(&listen, 1, 5);
        // Release disconnected consumers before accepting a configuration request
        // (including an immediate restart after STOP).
        for (auto i = peers.begin(); i != peers.end();)
        {
            pollfd p{*i, POLLIN | POLLRDHUP, 0};
            poll(&p, 1, 0);
            if (p.revents & POLLIN)
            {
                std::uint32_t request[3]{};
                Reply response{};
                const auto size = recv(*i, request, sizeof(request), MSG_TRUNC);
                response.status = size != sizeof(request) || request[0] != stereo_magic ||
                                          request[1] != stereo_version || request[2] > 1
                                      ? -EINVAL
                                  : peers.size() > 1 && bool(request[2]) != lease_dense
                                      ? -EBUSY
                                      : lease_(true, request[2] != 0);
                if (!response.status)
                    lease_dense = request[2] != 0;
                (void)send(*i, &response, sizeof(response), MSG_NOSIGNAL);
            }
            if (p.revents & (POLLHUP | POLLRDHUP | POLLERR | POLLNVAL))
            {
                close(*i);
                i = peers.erase(i);
            }
            else
                ++i;
        }
        if (peers.empty() && demand_.exchange(false))
        {
            (void)lease_(false, true);
            revision = 0;
        }
        if (listen.revents & POLLIN)
        {
            int peer = accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC);
            if (peer >= 0)
            {
                pollfd p{peer, POLLIN, 0};
                ControlRequest request{};
                const auto bytes =
                    poll(&p, 1, 200) > 0 ? recv(peer, &request, sizeof(request), MSG_TRUNC) : -1;
                if (request.magic != stereo_magic || request.version != stereo_version)
                {
                    close(peer);
                }
                else if (bytes == sizeof(std::uint32_t) * 2 || bytes == sizeof(std::uint32_t) * 3)
                {
                    // The old two-word handshake requests dense RGB-D. A third word
                    // selects rectification-only (0) or dense (1) for the SLAM lease.
                    const bool dense = bytes == sizeof(std::uint32_t) * 2 || request.operation == 1;
                    const int result = bytes == sizeof(std::uint32_t) * 3 && request.operation > 1
                                           ? -EINVAL
                                       : peers.empty()        ? lease_(true, dense)
                                       : dense == lease_dense ? 0
                                                              : -EBUSY;
                    if (!result)
                        lease_dense = dense;
                    reply(peer, result, memory_);
                    if (!result)
                    {
                        peers.push_back(peer);
                        demand_ = true;
                    }
                    else
                        close(peer);
                }
                else if (bytes == sizeof(request))
                {
                    ControlReply response;
                    std::string error;
                    try
                    {
                        if (request.operation != 1 && request.operation != 2)
                        {
                            response.status = -EINVAL;
                            error = "Invalid camera configuration operation";
                        }
                        else if (!configure_)
                        {
                            response.status = -ENOTSUP;
                            error = "Capture service has no camera configuration support";
                        }
                        else
                            response.status =
                                configure_(request.operation == 2 ? &request.settings : nullptr,
                                           response.configuration, error);
                    }
                    catch (const std::exception &e)
                    {
                        response.status = -EIO;
                        error = e.what();
                    }
                    std::snprintf(response.message, sizeof(response.message), "%s", error.c_str());
                    (void)send(peer, &response, sizeof(response), MSG_NOSIGNAL);
                    close(peer);
                }
                else
                    close(peer);
            }
        }
        if (peers.empty())
            continue;
        auto frame = latest_();
        if (!frame || frame->info.sequence == revision ||
            frame->pixels.size() != frame->info.payload_bytes)
            continue;
        const auto extra =
            frame->gray_pixels.empty() ? 0 : sizeof(StereoGrayInfo) + frame->gray_pixels.size();
        if (frame->info.reserved != extra || frame->pixels.size() + extra > stereo_capacity)
            continue;
        auto *h = static_cast<Shared *>(mapping_);
        __atomic_store_n(&h->guard, ++guard, __ATOMIC_SEQ_CST);
        h->info = frame->info;
        std::memcpy(static_cast<char *>(mapping_) + sizeof(Shared), frame->pixels.data(),
                    frame->pixels.size());
        if (extra)
        {
            auto *tail = static_cast<char *>(mapping_) + sizeof(Shared) + frame->pixels.size();
            std::memcpy(tail, &frame->gray, sizeof(frame->gray));
            std::memcpy(tail + sizeof(frame->gray), frame->gray_pixels.data(),
                        frame->gray_pixels.size());
        }
        __atomic_store_n(&h->guard, ++guard, __ATOMIC_RELEASE);
        revision = frame->info.sequence;
    }
    for (int peer : peers)
        close(peer);
    if (demand_.exchange(false))
        (void)lease_(false, true);
}
StereoConfiguration stereo_configuration(const StereoSettings *desired, const std::string &path)
{
    int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    try
    {
        auto a = address(path);
        if (fd < 0 || connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)))
            throw std::runtime_error(
                "CAMERA_CONTROL_UNAVAILABLE: cannot connect to capture service");
        ControlRequest request;
        if (desired)
        {
            request.operation = 2;
            request.settings = *desired;
        }
        if (send(fd, &request, sizeof(request), MSG_NOSIGNAL) != sizeof(request))
            throw std::runtime_error(
                "CAMERA_CONTROL_UNAVAILABLE: camera configuration request failed");
        pollfd p{fd, POLLIN, 0};
        int ready;
        do
        {
            ready = poll(&p, 1, 20000);
        } while (ready < 0 && errno == EINTR);
        if (ready <= 0)
            throw std::runtime_error("CAMERA_CONTROL_UNAVAILABLE: camera configuration timed out");
        ControlReply response;
        if (recv(fd, &response, sizeof(response), MSG_TRUNC) != sizeof(response) ||
            response.magic != stereo_magic || response.version != stereo_version ||
            response.configuration.mode_count > response.configuration.modes.size())
            throw std::runtime_error("CAMERA_CONTROL_UNAVAILABLE: update stereo_imu_capture to "
                                     "enable automatic configuration");
        if (response.status)
            throw std::runtime_error(
                std::string(response.status == -EBUSY ? "CAMERA_BUSY: "
                                                      : "CAMERA_CONFIG_FAILED: ") +
                std::string(response.message, strnlen(response.message, sizeof(response.message))) +
                " (" + std::to_string(response.status) + ")");
        close_fd(fd);
        return response.configuration;
    }
    catch (...)
    {
        close_fd(fd);
        throw;
    }
}
StereoClient::StereoClient(const std::string &path, bool dense)
{
    try
    {
        minimum_timestamp_ = monotonic_ns();
        socket_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
        auto a = address(path);
        if (socket_ < 0 || connect(socket_, reinterpret_cast<sockaddr *>(&a), sizeof(a)))
            throw std::runtime_error("Cannot connect to rectified stereo IPC: " + path);
        std::uint32_t hello[3]{stereo_magic, stereo_version, dense ? 1u : 0u};
        if (send(socket_, hello, sizeof(hello), MSG_NOSIGNAL) != sizeof(hello))
            throw std::runtime_error("Stereo IPC handshake write failed");
        pollfd p{socket_, POLLIN, 0};
        if (poll(&p, 1, 20000) <= 0)
            throw std::runtime_error("Stereo IPC initialization timed out");
        Reply r{};
        iovec io{&r, sizeof(r)};
        alignas(cmsghdr) char control[CMSG_SPACE(sizeof(int))]{};
        msghdr msg{};
        msg.msg_iov = &io;
        msg.msg_iovlen = 1;
        msg.msg_control = control;
        msg.msg_controllen = sizeof(control);
        if (recvmsg(socket_, &msg, MSG_CMSG_CLOEXEC) != sizeof(r) || r.bytes != memory_size ||
            (msg.msg_flags & (MSG_TRUNC | MSG_CTRUNC)))
            throw std::runtime_error("Invalid stereo IPC reply");
        if (r.status)
            throw std::runtime_error(std::string(r.message, strnlen(r.message, sizeof(r.message))));
        auto *c = CMSG_FIRSTHDR(&msg);
        if (!c || c->cmsg_level != SOL_SOCKET || c->cmsg_type != SCM_RIGHTS ||
            c->cmsg_len != CMSG_LEN(sizeof(int)))
            throw std::runtime_error("Missing stereo memory descriptor");
        std::memcpy(&memory_, CMSG_DATA(c), sizeof(int));
        mapping_ = mmap(nullptr, memory_size, PROT_READ, MAP_SHARED, memory_, 0);
        if (mapping_ == MAP_FAILED)
        {
            mapping_ = nullptr;
            throw std::runtime_error("Cannot map rectified stereo frames");
        }
    }
    catch (...)
    {
        if (mapping_)
            munmap(mapping_, memory_size);
        close_fd(memory_);
        close_fd(socket_);
        throw;
    }
}
StereoClient::~StereoClient()
{
    if (mapping_)
        munmap(mapping_, memory_size);
    close_fd(memory_);
    close_fd(socket_);
}
void StereoClient::setDense(bool dense)
{
    const std::uint32_t request[3]{stereo_magic, stereo_version, dense ? 1u : 0u};
    if (send(socket_, request, sizeof(request), MSG_NOSIGNAL) != sizeof(request))
        throw std::runtime_error("CAMERA_CONFIG_FAILED: cannot change depth demand");
    pollfd p{socket_, POLLIN, 0};
    Reply response{};
    if (poll(&p, 1, 20000) <= 0 ||
        recv(socket_, &response, sizeof(response), MSG_TRUNC) != sizeof(response))
        throw std::runtime_error("CAMERA_CONFIG_FAILED: depth demand update timed out");
    if (response.status)
        throw std::runtime_error("CAMERA_CONFIG_FAILED: depth demand update rejected (" +
                                 std::to_string(response.status) + ")");
}
bool StereoClient::read(StereoFrame &frame)
{
    pollfd p{socket_, 0, 0};
    if (poll(&p, 1, 0) > 0 && (p.revents & (POLLHUP | POLLERR | POLLNVAL)))
        throw std::runtime_error("Rectified stereo publisher disconnected");
    auto *h = static_cast<const Shared *>(mapping_);
    for (int retry = 0; retry < 4; ++retry)
    {
        const auto before = __atomic_load_n(&h->guard, __ATOMIC_ACQUIRE);
        if (!before || (before & 1))
            return false;
        const auto info = h->info;
        const auto pixels = std::uint64_t(info.width) * info.height;
        if (info.magic != stereo_magic || info.version != stereo_version || info.projection > 1 ||
            !info.width || !info.height || info.width > 512 || info.height > 512 ||
            (info.payload_bytes != pixels * 10 && info.payload_bytes != pixels * 6) ||
            std::uint64_t(info.payload_bytes) + info.reserved > stereo_capacity)
        {
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            if (before != __atomic_load_n(&h->guard, __ATOMIC_ACQUIRE))
                continue;
            throw std::runtime_error("Invalid rectified stereo metadata");
        }
        if (info.sequence == cursor_ || info.timestamp_ns < minimum_timestamp_)
            return false;
        StereoGrayInfo gray{};
        bool valid_gray = true;
        if (info.reserved)
        {
            if (info.reserved < sizeof(gray))
                valid_gray = false;
            else
            {
                std::memcpy(&gray,
                            static_cast<const char *>(mapping_) + sizeof(Shared) +
                                info.payload_bytes,
                            sizeof(gray));
                valid_gray =
                    gray.magic == 0x59475243 && gray.version == 1 && gray.width && gray.height &&
                    gray.width <= 1024 && gray.height <= 1024 &&
                    info.reserved == sizeof(gray) + std::uint64_t(gray.width) * gray.height * 2 &&
                    gray.fx > 0 && gray.fy > 0;
            }
        }
        if (!valid_gray)
        {
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            if (before != __atomic_load_n(&h->guard, __ATOMIC_ACQUIRE))
                continue;
            throw std::runtime_error("Invalid rectified gray stereo metadata");
        }
        frame.info = info;
        frame.pixels.resize(info.payload_bytes);
        std::memcpy(frame.pixels.data(), static_cast<const char *>(mapping_) + sizeof(Shared),
                    info.payload_bytes);
        frame.gray = gray;
        frame.gray_pixels.resize(info.reserved ? info.reserved - sizeof(gray) : 0);
        if (info.reserved)
            std::memcpy(frame.gray_pixels.data(),
                        static_cast<const char *>(mapping_) + sizeof(Shared) + info.payload_bytes +
                            sizeof(gray),
                        frame.gray_pixels.size());
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (before == __atomic_load_n(&h->guard, __ATOMIC_ACQUIRE))
        {
            cursor_ = info.sequence;
            return true;
        }
    }
    return false;
}
} // namespace cambold_ipc
