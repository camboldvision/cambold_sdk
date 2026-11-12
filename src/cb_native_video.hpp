/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

class CbVideoSession
{
  public:
    struct Frame
    {
        std::vector<uint8_t> jpeg;
        uint64_t sequence = 0;
        uint64_t timestamp_ns = 0;
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t flags = 0;
        uint32_t exposure_us[2] = {0, 0};
    };

    using FrameCallback = std::function<void(Frame &&)>;

    explicit CbVideoSession(FrameCallback callback);
    ~CbVideoSession();

    CbVideoSession(const CbVideoSession &) = delete;
    CbVideoSession &operator=(const CbVideoSession &) = delete;

    bool start(std::string *error);
    void stop();
    bool running() const;

  private:
    struct Impl;
    Impl *impl_ = nullptr;
};

// Internal parser shared by USB capture and board shared-memory capture.
bool cb_parse_jpeg_metadata(const uint8_t *, size_t, CbVideoSession::Frame *);
