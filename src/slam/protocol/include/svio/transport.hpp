/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "protocol.hpp"
#include <memory>

namespace svio
{
class Transport
{
  public:
    virtual ~Transport() = default;
    virtual std::size_t read(std::uint8_t *, std::size_t, int timeout_ms) = 0;
    virtual std::size_t write(const std::uint8_t *, std::size_t, int timeout_ms) = 0;
    virtual void close() = 0;
};
std::unique_ptr<Transport> open_serial(const std::string &path);
// One outstanding byte frame at a time: priority changes never split a frame.
class Transmitter
{
  public:
    bool empty() const
    {
        return bytes_.empty();
    }
    void begin(Frame frame);
    std::size_t flush(Transport &, int timeout_ms = 10);
    void reset()
    {
        bytes_.clear();
        offset_ = 0;
        sequence_ = 0;
    }

  private:
    Bytes bytes_;
    std::size_t offset_ = 0;
    std::uint32_t sequence_ = 0;
};
} // namespace svio
