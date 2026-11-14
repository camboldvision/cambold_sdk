# Third-party sources

The public SDK uses a C ABI; its implementation is C11/C++17. Bundled libraries
are built statically. Their source and notices remain in `third_party/`:

| Dependency | Purpose | License |
| --- | --- | --- |
| libusb | USB control/debug transfer | LGPL-2.1-or-later; `third_party/libusb/COPYING` |
| libuvc | Linux/macOS USB MJPEG capture | BSD; `third_party/libuvc/LICENSE.txt` |
| nlohmann/json | Internal SLAM protocol and map metadata | MIT; `third_party/nlohmann_json/LICENSE.MIT` |

The libusb and libuvc directories contain only the sources and headers used by
`cmake/usb.cmake` on Windows, Linux and macOS, plus their license notices.
Examples, tests, upstream build files and other platform backends are omitted.
The libuvc interface claim/release path is patched on macOS to avoid capturing
and re-enumerating the entire composite device. Upstream: [libusb](https://github.com/libusb/libusb)
and [libuvc](https://github.com/libuvc/libuvc).

The LOCAL backend builds no libusb or libuvc code. Windows video capture uses
Media Foundation. Linux board video capture reads the capture service's shared
memory. The SDK has no GUI, ROS, OpenCV or NPU runtime dependency.

When distributing applications that statically link libusb, follow its LGPL
requirements, including source and relinking obligations. CamBold's Apache-2.0 license
does not replace the bundled dependencies' licenses.
