// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace cambold_ipc
{
// USB uses the existing SC header once. This logical SLAM record has no
// nested serial header/CRC. Both directions use explicitly little-endian fields.
constexpr std::uint16_t slam_packet = 0x0080, slam_link = 0x0081, slam_link_status = 0x0082;
constexpr std::size_t slam_record_header = 16, slam_record_limit = 16 + 65536;
struct SlamRecord
{
    std::uint16_t type = 0, flags = 0;
    std::uint32_t request = 0;
    std::uint64_t timestamp = 0;
    std::vector<std::uint8_t> payload;
};
inline std::vector<std::uint8_t> pack_slam(const SlamRecord &r)
{
    if (r.payload.size() > 65536)
        throw std::runtime_error("SLAM record too large");
    std::vector<std::uint8_t> b;
    b.reserve(16 + r.payload.size());
    auto put = [&](std::uint64_t value, int n)
    {
        for (int i = 0; i < n; ++i)
            b.push_back(std::uint8_t(value >> (8 * i)));
    };
    put(r.type, 2);
    put(r.flags, 2);
    put(r.request, 4);
    put(r.timestamp, 8);
    b.insert(b.end(), r.payload.begin(), r.payload.end());
    return b;
}
inline SlamRecord unpack_slam(const std::uint8_t *p, std::size_t size)
{
    if (!p || size < 16 || size > slam_record_limit)
        throw std::runtime_error("Invalid SLAM record");
    auto get = [&](int n)
    {
        std::uint64_t value = 0;
        for (int i = 0; i < n; ++i)
            value |= std::uint64_t(*p++) << (8 * i);
        return value;
    };
    SlamRecord r;
    r.type = std::uint16_t(get(2));
    r.flags = std::uint16_t(get(2));
    r.request = std::uint32_t(get(4));
    r.timestamp = get(8);
    if (r.flags > 1)
        throw std::runtime_error("Invalid SLAM flags");
    r.payload.assign(p, p + size - 16);
    return r;
}
} // namespace cambold_ipc
