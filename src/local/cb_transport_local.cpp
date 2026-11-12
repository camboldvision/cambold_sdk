/* SPDX-License-Identifier: Apache-2.0 */
#include "cb_transport.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <poll.h>
#include <pty.h>
#include <signal.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace
{
std::string product_name()
{
    std::ifstream product("/sys/kernel/config/usb_gadget/rockchip/strings/0x409/product");
    if (!product)
    {
        product.clear();
        product.open("/sys/kernel/config/usb_gadget/g1/strings/0x409/product");
    }
    std::string name;
    if (std::getline(product, name) && !name.empty())
        return name;
    return "CamBold";
}
} // namespace

std::unique_ptr<CbTransport> CbTransport::open(const std::string &requested, bool control,
                                               std::string *error)
{
    auto port = std::unique_ptr<CbTransport>(new CbTransport);
    if (!control)
    {
        // A local terminal owns its own PTY; it never opens the user's ACM or
        // the USB debug tty. The shell has the caller's existing privileges.
        const pid_t pid = forkpty(&port->local_fd_, nullptr, nullptr, nullptr);
        if (pid == 0)
        {
            execl("/bin/sh", "sh", "-i", static_cast<char *>(nullptr));
            _exit(127);
        }
        if (pid < 0)
        {
            if (error)
                *error = std::strerror(errno);
            return {};
        }
        port->child_pid_ = pid;
        port->name_ = "local-debug";
    }
    else
    {
        const std::string path = requested.empty() ? "/run/soc-camera/sdk.sock" : requested;
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        if (path.size() >= sizeof(address.sun_path))
        {
            if (error)
                *error = "Local SDK socket path is too long";
            return {};
        }
        std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
        port->local_fd_ = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if (port->local_fd_ < 0 ||
            connect(port->local_fd_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)))
        {
            if (error)
                *error = "Local camera service: " + std::string(std::strerror(errno));
            return {};
        }
        port->name_ = "local-control";
        port->device_name_ = product_name();
    }
    fcntl(port->local_fd_, F_SETFL, O_NONBLOCK);
    fcntl(port->local_fd_, F_SETFD, FD_CLOEXEC);
    return port;
}

CbTransport::~CbTransport()
{
    close();
}
void CbTransport::close()
{
    if (local_fd_ >= 0)
    {
        ::close(local_fd_);
        local_fd_ = -1;
    }
    if (child_pid_ > 0)
    {
        kill(child_pid_, SIGHUP);
        pid_t result = 0;
        for (int i = 0; i < 10 && result == 0; ++i)
        {
            result = waitpid(child_pid_, nullptr, WNOHANG);
            if (!result)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!result)
        {
            kill(child_pid_, SIGKILL);
            while (waitpid(child_pid_, nullptr, 0) < 0 && errno == EINTR)
            {
            }
        }
        child_pid_ = -1;
    }
}

int CbTransport::resize_terminal(uint16_t columns, uint16_t rows)
{
    if (child_pid_ <= 0 || local_fd_ < 0)
        return -ENOTCONN;
    winsize size{};
    size.ws_col = columns;
    size.ws_row = rows;
    return ioctl(local_fd_, TIOCSWINSZ, &size) ? -errno : 0;
}

size_t CbTransport::read(uint8_t *data, size_t capacity, int timeout_ms)
{
    pollfd item{local_fd_, POLLIN, 0};
    const int ready = poll(&item, 1, timeout_ms);
    if (ready < 0 && errno == EINTR)
        return 0;
    if (ready < 0)
        throw std::runtime_error(std::strerror(errno));
    if (!ready)
        return 0;
    const auto count = ::read(local_fd_, data, capacity);
    if (count < 0 && (errno == EAGAIN || errno == EINTR))
        return 0;
    if (count <= 0)
        throw std::runtime_error("Local camera channel disconnected");
    return static_cast<size_t>(count);
}

size_t CbTransport::write(const std::vector<uint8_t> &data)
{
    size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (offset < data.size())
    {
        if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error("Local camera write timed out");
        pollfd item{local_fd_, POLLOUT, 0};
        const int ready = poll(&item, 1, 20);
        if (ready < 0 && errno == EINTR)
            continue;
        if (ready < 0)
            throw std::runtime_error(std::strerror(errno));
        if (!ready)
            continue;
        const auto count =
            child_pid_ > 0
                ? ::write(local_fd_, data.data() + offset, data.size() - offset)
                : send(local_fd_, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
        if (count < 0 && (errno == EAGAIN || errno == EINTR))
            continue;
        if (count <= 0)
            throw std::runtime_error("Local camera write failed");
        offset += static_cast<size_t>(count);
    }
    return offset;
}
