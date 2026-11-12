/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CB_TRANSPORT_HPP
#define CB_TRANSPORT_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/* A small byte-stream adapter over one vendor-specific USB interface.  The
 * board's generic serial gadget exposes exactly one bulk-IN and one bulk-OUT
 * endpoint per logical channel; this class deliberately hides USB packet
 * boundaries from the existing framed protocol. */
class CbTransport
{
  public:
    static std::unique_ptr<CbTransport> open(const std::string &requested, bool control,
                                             std::string *error);
    ~CbTransport();

    CbTransport(const CbTransport &) = delete;
    CbTransport &operator=(const CbTransport &) = delete;

    bool is_open() const
    {
        return handle_ != nullptr || local_fd_ >= 0;
    }
    const std::string &name() const
    {
        return name_;
    }
    const std::string &device_name() const
    {
        return device_name_;
    }
    size_t write(const std::vector<uint8_t> &data);
    /* Read at most one bulk transfer.  A timeout of zero is treated as a
     * short non-blocking poll by the platform backends; the default keeps the
     * control-channel worker's existing bounded wait. */
    size_t read(uint8_t *data, size_t capacity, int timeout_ms = 100);
    int resize_terminal(uint16_t columns, uint16_t rows);
    void close();

  private:
    CbTransport() = default;

    struct libusb_context *context_ = nullptr;
    struct libusb_device_handle *handle_ = nullptr;
    int interface_number_ = -1;
    uint8_t endpoint_in_ = 0;
    uint8_t endpoint_out_ = 0;
    std::string name_;
    std::string device_name_;
    int local_fd_ = -1;
    int child_pid_ = -1;
#ifdef _WIN32
    void *winusb_file_ = nullptr;
    void *winusb_handle_ = nullptr;
#endif
};

#endif
