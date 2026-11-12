// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "cb_sdk.h"
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>
// USB framing and IMU decoding stay on the transport thread. Assembly is
// bounded to one image, consecutive offsets, one generation and 500 ms.
class CbDisparityAssembler
{
    std::vector<uint8_t> bytes_;
    uint64_t id_ = 0;
    uint32_t generation_ = 0, total_ = 0;
    std::chrono::steady_clock::time_point start_;
    static uint32_t u32(const uint8_t *p)
    {
        return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
    }
    static uint64_t u64(const uint8_t *p)
    {
        return u32(p) | (uint64_t(u32(p + 4)) << 32);
    }

  public:
    static uint32_t crc32(const uint8_t *p, size_t n)
    {
        static const auto table = []()
        {
            std::vector<uint32_t> t(256);
            for (uint32_t i = 0; i < 256; ++i)
            {
                uint32_t c = i;
                for (int b = 0; b < 8; ++b)
                    c = (c >> 1) ^ (0xedb88320U & (0U - (c & 1)));
                t[i] = c;
            }
            return t;
        }();
        uint32_t c = 0xffffffffU;
        for (size_t i = 0; i < n; ++i)
            c = (c >> 8) ^ table[(c ^ p[i]) & 255];
        return c ^ 0xffffffffU;
    }
    void reset()
    {
        bytes_.clear();
        id_ = 0;
        generation_ = total_ = 0;
    }
    std::shared_ptr<std::vector<uint8_t>> accept(const uint8_t *p, size_t n)
    {
        if (!p || n <= 24 || n > 24 + 4096)
        {
            reset();
            return {};
        }
        const uint64_t id = u64(p);
        const uint32_t gen = u32(p + 8), total = u32(p + 12), offset = u32(p + 16);
        const size_t length = n - 24;
        const auto now = std::chrono::steady_clock::now();
        if (!id || total < 64 ||
            total > 80 + 4 * CB_MAX_DISPARITY_WIDTH * CB_MAX_DISPARITY_HEIGHT || offset > total ||
            length > total - offset || crc32(p + 24, length) != u32(p + 20))
        {
            reset();
            return {};
        }
        if (offset == 0)
        {
            reset();
            id_ = id;
            generation_ = gen;
            total_ = total;
            start_ = now;
            bytes_.reserve(total);
        }
        if (id != id_ || gen != generation_ || total != total_ || offset != bytes_.size() ||
            now - start_ > std::chrono::milliseconds(500))
        {
            reset();
            return {};
        }
        bytes_.insert(bytes_.end(), p + 24, p + n);
        if (bytes_.size() != total_)
            return {};
        if ((u32(bytes_.data()) != 2 && u32(bytes_.data()) != CB_DISPARITY_FRAME_VERSION) ||
            u32(bytes_.data() + 52) != CB_DISPARITY_FP16 || u32(bytes_.data() + 4) != generation_ ||
            u64(bytes_.data() + 8) != id_)
        {
            reset();
            return {};
        }
        cb_disparity metadata{};
        if (cb_decode_disparity(bytes_.data(), bytes_.size(), &metadata, nullptr, nullptr) != 0)
        {
            reset();
            return {};
        }
        auto result = std::make_shared<std::vector<uint8_t>>(std::move(bytes_));
        reset();
        return result;
    }
};
