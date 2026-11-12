/* SPDX-License-Identifier: Apache-2.0 */
#include "cb_native_video.hpp"
#include <cstring>

namespace
{
uint32_t le32(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

uint64_t le64(const uint8_t *p)
{
    uint64_t value = 0;
    for (unsigned int i = 0; i < 8; ++i)
        value |= static_cast<uint64_t>(p[i]) << (i * 8U);
    return value;
}

} // namespace
bool cb_parse_jpeg_metadata(const uint8_t *jpeg, size_t bytes, CbVideoSession::Frame *frame)
{
    if (!jpeg || bytes < 2 || jpeg[0] != 0xff || jpeg[1] != 0xd8)
        return false;
    for (size_t offset = 2; offset + 4 <= bytes;)
    {
        if (jpeg[offset] != 0xff)
            return false;
        while (offset < bytes && jpeg[offset] == 0xff)
            ++offset;
        if (offset >= bytes)
            return false;
        const uint8_t marker = jpeg[offset++];
        if (marker == 0xda || marker == 0xd9)
            return false;
        if (marker == 0xd8 || (marker >= 0xd0 && marker <= 0xd7))
            continue;
        if (offset + 2 > bytes)
            return false;
        const size_t length = (static_cast<size_t>(jpeg[offset]) << 8) | jpeg[offset + 1];
        if (length < 2 || offset + length > bytes)
            return false;
        if (marker == 0xe1 && length >= 34 && std::memcmp(jpeg + offset + 2, "SCAM", 4) == 0 &&
            jpeg[offset + 6] == 1)
        {
            const uint8_t *payload = jpeg + offset + 2;
            if ((payload[5] & 6U) && length < 42)
                return false;
            frame->flags = payload[5] & 7U;
            frame->sequence = le64(payload + 8);
            frame->timestamp_ns = (frame->flags & 1U) ? le64(payload + 16) : 0;
            frame->width = le32(payload + 24);
            frame->height = le32(payload + 28);
            if (frame->flags & 2U)
                frame->exposure_us[0] = le32(payload + 32);
            if (frame->flags & 4U)
                frame->exposure_us[1] = le32(payload + 36);
            return true;
        }
        offset += length;
    }
    return false;
}
