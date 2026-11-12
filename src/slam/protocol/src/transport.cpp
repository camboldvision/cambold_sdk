/* SPDX-License-Identifier: Apache-2.0 */
#include "svio/transport.hpp"
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>
#ifdef SVIO_USE_SERIAL_LIBRARY
#include <serial/serial.h>
#endif
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace svio
{
namespace
{
#ifdef SVIO_USE_SERIAL_LIBRARY
class Serial final : public Transport
{
  public:
    explicit Serial(const std::string &path)
    {
        port_.setPort(path);
        port_.setBaudrate(115200);
        port_.setTimeout(0, 5, 0, 5, 0);
        port_.open();
        port_.flushInput();
    }
    ~Serial() override
    {
        try
        {
            close();
        }
        catch (...)
        {
        }
    }
    std::size_t read(std::uint8_t *data, std::size_t size, int timeout) override
    {
        set_timeout(timeout);
        auto available = port_.available();
        if (!available && timeout > 0)
        {
            port_.waitReadable();
            available = port_.available();
        }
        // ttyGS can signal readable with no bytes. Avoid the library's
        // zero-byte disconnect exception and a busy loop while USB is idle.
        if (!available)
        {
            if (timeout > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return 0;
        }
        // ACM is a byte stream. Read only what is already available, without
        // the serial library's baud-derived fixed-length/inter-byte wait.
        port_.setTimeout(0, 0, 0, std::max(0, timeout), 0);
        timeout_ = -1;
        return port_.read(data, std::min(size, available));
    }
    std::size_t write(const std::uint8_t *data, std::size_t size, int timeout) override
    {
        set_timeout(timeout);
        return port_.write(data, size);
    }
    void close() override
    {
        if (port_.isOpen())
            port_.close();
    }

  private:
    void set_timeout(int timeout)
    {
        timeout = std::max(0, timeout);
        if (timeout != timeout_)
        {
            port_.setTimeout(0, timeout, 0, timeout, 0);
            timeout_ = timeout;
        }
    }
    serial::Serial port_;
    int timeout_ = 5;
};
#elif defined(_WIN32)
std::runtime_error system_error(const char *what)
{
    return std::runtime_error(std::string(what) + ": Windows error " +
                              std::to_string(GetLastError()));
}
class Serial final : public Transport
{
  public:
    explicit Serial(std::string path)
    {
        if (path.rfind("\\\\.\\", 0) != 0)
            path = "\\\\.\\" + path;
        handle_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                              FILE_FLAG_OVERLAPPED, nullptr);
        if (handle_ == INVALID_HANDLE_VALUE)
            throw system_error("Open serial");
        try
        {
            DCB dcb{};
            dcb.DCBlength = sizeof(dcb);
            if (!GetCommState(handle_, &dcb))
                throw system_error("GetCommState");
            dcb.BaudRate = CBR_115200;
            dcb.ByteSize = 8;
            dcb.Parity = NOPARITY;
            dcb.StopBits = ONESTOPBIT;
            dcb.fBinary = TRUE;
            dcb.fParity = FALSE;
            dcb.fOutxCtsFlow = FALSE;
            dcb.fOutxDsrFlow = FALSE;
            dcb.fDtrControl = DTR_CONTROL_ENABLE;
            dcb.fRtsControl = RTS_CONTROL_DISABLE;
            dcb.fOutX = FALSE;
            dcb.fInX = FALSE;
            dcb.fDsrSensitivity = FALSE;
            dcb.fAbortOnError = FALSE;
            if (!SetCommState(handle_, &dcb))
                throw system_error("SetCommState");
            COMMTIMEOUTS t{};
            t.ReadIntervalTimeout = MAXDWORD;
            t.ReadTotalTimeoutMultiplier = MAXDWORD;
            t.ReadTotalTimeoutConstant = 20;
            if (!SetCommTimeouts(handle_, &t))
                throw system_error("SetCommTimeouts");
            PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
        }
        catch (...)
        {
            close();
            throw;
        }
    }
    ~Serial() override
    {
        close();
    }
    std::size_t read(std::uint8_t *p, std::size_t n, int timeout) override
    {
        return io(p, n, timeout, false);
    }
    std::size_t write(const std::uint8_t *p, std::size_t n, int timeout) override
    {
        return io(const_cast<std::uint8_t *>(p), n, timeout, true);
    }
    void close() override
    {
        if (handle_ != INVALID_HANDLE_VALUE)
        {
            CancelIoEx(handle_, nullptr);
            CloseHandle(handle_);
            handle_ = INVALID_HANDLE_VALUE;
        }
    }

  private:
    std::size_t io(std::uint8_t *p, std::size_t n, int timeout, bool writing)
    {
        OVERLAPPED op{};
        op.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
        if (!op.hEvent)
            throw system_error("CreateEvent");
        DWORD count = 0;
        BOOL ok = writing ? WriteFile(handle_, p, static_cast<DWORD>(n), &count, &op)
                          : ReadFile(handle_, p, static_cast<DWORD>(n), &count, &op);
        if (!ok && GetLastError() != ERROR_IO_PENDING)
        {
            CloseHandle(op.hEvent);
            throw system_error("Serial I/O");
        }
        if (!ok)
        {
            auto result = WaitForSingleObject(op.hEvent, static_cast<DWORD>(std::max(0, timeout)));
            if (result != WAIT_OBJECT_0)
            {
                CancelIoEx(handle_, &op);
                WaitForSingleObject(op.hEvent, INFINITE);
            }
            ok = GetOverlappedResult(handle_, &op, &count, FALSE);
            auto error = GetLastError();
            CloseHandle(op.hEvent);
            if (!ok && error != ERROR_OPERATION_ABORTED)
                throw std::runtime_error("Serial I/O: Windows error " + std::to_string(error));
            return count;
        }
        CloseHandle(op.hEvent);
        return count;
    }
    HANDLE handle_ = INVALID_HANDLE_VALUE;
};
#else
class Serial final : public Transport
{
  public:
    explicit Serial(const std::string &path)
    {
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error("Open serial " + path + ": " + std::strerror(errno));
        termios t{};
        if (tcgetattr(fd_, &t) < 0)
        {
            auto error = std::string(std::strerror(errno));
            close();
            throw std::runtime_error("tcgetattr: " + error);
        }
        cfmakeraw(&t);
        cfsetispeed(&t, B115200);
        cfsetospeed(&t, B115200);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
        t.c_cflag |= CS8;
#ifdef CRTSCTS
        t.c_cflag &= ~CRTSCTS;
#endif
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &t) < 0)
        {
            auto error = std::string(std::strerror(errno));
            close();
            throw std::runtime_error("tcsetattr: " + error);
        }
        tcflush(fd_, TCIOFLUSH);
    }
    ~Serial() override
    {
        close();
    }
    std::size_t read(std::uint8_t *p, std::size_t n, int timeout) override
    {
        return io(p, n, timeout, false);
    }
    std::size_t write(const std::uint8_t *p, std::size_t n, int timeout) override
    {
        return io(const_cast<std::uint8_t *>(p), n, timeout, true);
    }
    void close() override
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

  private:
    std::size_t io(std::uint8_t *p, std::size_t n, int timeout, bool writing)
    {
        pollfd poller{fd_, static_cast<short>(writing ? POLLOUT : POLLIN), 0};
        int result;
        do
        {
            result = ::poll(&poller, 1, timeout);
        } while (result < 0 && errno == EINTR);
        if (result < 0)
            throw std::runtime_error("Serial poll failed");
        if (!result)
            return 0;
        if (poller.revents & POLLNVAL)
            throw std::runtime_error("Serial disconnected");
        auto count = writing ? ::write(fd_, p, n) : ::read(fd_, p, n);
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
            return 0;
        if (count < 0 || (!count && (poller.revents & (POLLHUP | POLLERR))))
            throw std::runtime_error("Serial disconnected or I/O error");
        return static_cast<std::size_t>(count);
    }
    int fd_ = -1;
};
#endif

} // namespace
std::unique_ptr<Transport> open_serial(const std::string &path)
{
    return std::make_unique<Serial>(path);
}
void Transmitter::begin(Frame frame)
{
    if (!empty())
        throw std::logic_error("Frame already in flight");
    frame.sequence = sequence_++;
    bytes_ = encode_frame(frame);
    offset_ = 0;
}
std::size_t Transmitter::flush(Transport &port, int timeout)
{
    if (empty())
        return 0;
    auto count = port.write(bytes_.data() + offset_, bytes_.size() - offset_, timeout);
    offset_ += count;
    if (offset_ == bytes_.size())
    {
        bytes_.clear();
        offset_ = 0;
    }
    return count;
}
} // namespace svio
