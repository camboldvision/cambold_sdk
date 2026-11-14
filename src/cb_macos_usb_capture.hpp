/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <memory>
#include <string>

// macOS captures a whole USB device, not one interface. All SDK interfaces
// share a lease so preview start/stop cannot detach active bulk handles.
class CbMacUsbCapture
{
  public:
    static std::shared_ptr<CbMacUsbCapture> acquire(bool required, std::string *error);
    ~CbMacUsbCapture();

    CbMacUsbCapture(const CbMacUsbCapture &) = delete;
    CbMacUsbCapture &operator=(const CbMacUsbCapture &) = delete;

  private:
    CbMacUsbCapture() = default;
    struct Device;
    struct Registry;
    static Registry &registry();
    Device *device_ = nullptr;
};
