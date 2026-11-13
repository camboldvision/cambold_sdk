# CamBold SDK

CamBold SDK 为电脑端和相机板端提供同一套 C API。应用包含 `cb_sdk.h`，链接
`CamBold::SDK`，即可控制相机并获取 JPEG、IMU、同帧视差与颜色、SLAM 位姿和地图。
公开接口使用 `cb_*` 前缀，常量使用 `CB_*` 前缀。

## 目录

```text
cambold_sdk/
  include/             公共 C 头文件
  src/                 公共运行时、解码、USB 传输和视频接入
  src/local/           板端 socket、共享内存和终端实现
  src/slam/            SLAM 协议、客户端与增量地图缓存
  third_party/         随源码提供的依赖及其许可证
  cmake/               USB 依赖构建配置
  examples/camera.c    两端共用的 C 示例
```

SDK 目录可独立复制给用户，不需要桌面程序、板端算法源码或外部下载步骤。
SDK 实现使用 C11/C++17；公开头文件可由 C 或 C++ 编译器包含。CMake 会处理静态
链接的依赖。第三方许可证见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 引入工程

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_camera_app LANGUAGES C CXX)
set(CB_SDK_BACKEND USB CACHE STRING "") # 板端使用 LOCAL
add_subdirectory(path/to/cambold_sdk sdk)
add_executable(my_camera_app main.c)
target_link_libraries(my_camera_app PRIVATE CamBold::SDK)
```

`CB_SDK_BACKEND` 选择传输实现：

| 配置 | 运行位置 | 控制、IMU、视差和 SLAM | 图像 |
| --- | --- | --- | --- |
| `USB` | Windows、Linux、macOS 电脑 | USB bulk | Windows Media Foundation；Linux/macOS libuvc |
| `LOCAL` | 相机内的 Linux 应用 | `/run/soc-camera/sdk.sock` | 采集服务共享内存中的同源 JPEG |

Windows/macOS 默认使用 USB，Linux AArch64 默认使用 LOCAL。Linux AArch64 电脑
作为外接主机时，显式选择 USB。LOCAL 需要安装本版本的 `stereo_imu_capture` 和
`soc-uvc-broker` 服务；仅更换应用程序不能为旧采集服务增加本地控制入口。

SDK 的构建目录由调用方指定。通过 `cross_compile` 构建时，可执行文件仍输出到
`cross_compile/bin`。SDK 本身不设置全局编译器、交叉库路径或可执行文件目录。

## 编译示例

电脑端（示例使用 Ninja；先准备本机 C/C++ 编译器）：

```sh
cmake -S cambold_sdk -B build/usb -G Ninja -DCB_SDK_BACKEND=USB -DCB_SDK_BUILD_EXAMPLES=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/usb
```

板端使用本机 AArch64 交叉工具链构建。也可将 SDK 复制到相机后使用板端编译器：

```sh
cmake -S cambold_sdk -B build/local -G Ninja -DCMAKE_TOOLCHAIN_FILE=/path/to/aarch64-toolchain.cmake -DCB_SDK_BACKEND=LOCAL -DCB_SDK_BUILD_EXAMPLES=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/local
```

示例命令的参数依次为运行秒数、启用 IMU、启用视频、启用深度：

```sh
cb_camera_example 10 1 1 0
cb_camera_example 10 0 1 1
```

不传运行秒数时，按 Ctrl+C 退出。程序打印真实设备名、支持的帧率、数据尺寸、源
时间戳和曝光时长。开启深度前请准备设备标定和模型。示例不修改投影模式或焦距；
开启深度可能按当前模式上限降低并保存相机帧率。

## 接口和数据约定

1. 将 `cb_config` 清零，设置回调和 `user`，调用 `cb_open()`。
2. 等待连接事件或 `cb_connected()`。打开成功表示异步工作线程已启动。
3. 用 `cb_next_request_id()` 分配请求号，提交设置请求。
4. 用 `CB_EVENT_RESULT` 的同一请求号判断设备执行结果；返回 0 仅表示请求已排队。
5. 应用退出时调用 `cb_video_close()` 和 `cb_close()`。

| 功能 | 主要接口 |
| --- | --- |
| 名称和连接 | `cb_get_device_name`、`cb_connected`、`cb_refresh_connection` |
| 参数和能力 | `cb_request_device_config`、`cb_decode_device_config`、`cb_set_frame_rate` |
| JPEG | `cb_video_open/start/stop/close`、`cb_video_frame` 回调 |
| IMU | `cb_set_imu_output`、`CB_EVENT_IMU_BATCH`、`cb_decode_imu_batch` |
| 深度和颜色 | `cb_set_disparity_enabled`、`cb_get_disparity` |
| 投影和三维坐标 | `cb_set_disparity_config_ex`、`cb_disparity_to_point` |
| SLAM | `cb_slam_start/stop`、`cb_slam_get_pose`、`cb_slam_poll_json` |
| 系统与终端 | `cb_request_metrics`、`cb_debug_open/read/write/close` |

图像、IMU、深度和位姿使用板端 `CLOCK_MONOTONIC` 纳秒时间戳。图像时间为曝光
中点；深度及同帧 RGB 使用源图像时间，不使用推理完成或主机收到时间。视差为
FP16 位模式，颜色为 RGB565；`cb_get_disparity` 原子地复制同一帧的两个平面。
针孔和圆柱模式的三维计算统一调用 `cb_disparity_to_point`。

SLAM 完成初始化后，位姿按 IMU 的 200 Hz 采样节奏更新，USB 和 LOCAL 后端默认
均订阅 200 Hz。`cb_slam_get_pose()` 返回最新有效位姿；需要按此频率读取时，应在
独立线程中轮询，并用 `timestamp_monotonic_ns` 区分样本。界面绘制频率独立于接收频率。

回调数据只在回调返回前有效。视频、控制和深度回调可能并发；将耗时处理交给应用
线程，并自行保护共享状态。禁止在回调中关闭 SDK 或同步等待另一回调。JSON 返回
的内存使用 `cb_slam_free_json` 释放；其他输出缓冲由调用方管理。

负数返回值遵循 `-errno`；`-EAGAIN` 表示暂无新数据，`-ENOSPC` 表示缓冲不足。
标定和模型错误分别为 `CB_STEREO_CALIBRATION_ERROR`、`CB_STEREO_MODEL_ERROR`。
断开后 SDK 自动重连；设置命令不自动重放，应用收到新连接事件后按需重新订阅。
USB 后台每两秒发送心跳。应用异常退出或拔线后，服务在十秒无请求时释放该 USB
连接的订阅与 SLAM 通信会话；本地连接在 socket 断开后释放。释放通信会话不会删除
地图，也不会向 SLAM 算法发送停止命令。

SDK 也可通过 `cmake --install build/usb --prefix /path/to/sdk` 安装。使用已安装库时，
设置 `CMAKE_PREFIX_PATH`，然后调用 `find_package(CamBoldSDK CONFIG REQUIRED)` 并链接
`CamBold::SDK`。安装库对应构建时选择的平台与后端；跨平台分发请提供本目录源码。

## 多应用使用

板端本地连接各自维护 IMU、深度订阅和帧缓存。关闭一个本地连接只释放该连接的
需求；采集服务保留 USB 或其他本地连接仍需要的功能。视频订阅由 broker 单独管理。
帧率、投影、焦距和天线属于全设备配置，修改会影响全部应用。

同一时刻由一个 SDK 连接控制 SLAM 会话和地图增量同步。其他连接可继续使用相机、
IMU 和深度；其 SLAM 会话在当前控制连接释放后重试。USB 后端在稠密 SLAM 启动时
自动关闭本连接的 IMU 上传，运行期间开启请求返回 `-EBUSY`；停止或失败后解除限制，
不自动恢复上传。LOCAL 后端没有此限制，可同时接收稠密地图和 IMU。算法内部 IMU
持续运行，其他本地连接的 IMU 订阅不受 USB 策略影响。稀疏 SLAM 与独立深度预览
可同时使用。

## 从旧接口迁移

原 `host_sdk_app` 改名为 `cambold_viewer`，原 SDK 源码已迁入本目录。新应用包含
`cb_sdk.h`，链接 `CamBold::SDK`。旧 `sc_host_*` 接口改为 `cb_*`，原
`sc_host_uvc_*` 接口改为传输无关的 `cb_video_*`，原 `SC_HOST_*` 常量改为 `CB_*`。
不保留旧符号的别名。请同步更新头文件和库，并重新编译调用方。设备的 USB 协议
编号、JPEG 元数据标识和数据单位保持兼容。

`camera_client`、`camera_ipc`、`stereo_local` 属于采集与算法之间的内部 IPC。
它们仍供现有算法使用，不属于公开 SDK 的稳定 API。

## 许可

CamBold SDK 的第一方代码使用 [Apache License 2.0](LICENSE)，版权归
CamBold contributors 所有。第三方代码保留各自的许可与版权声明，详见
[THIRD_PARTY.md](THIRD_PARTY.md)。
