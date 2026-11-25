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
| `USB` | Windows、Linux、macOS 电脑 | USB bulk | UVC 原始 MJPEG：Windows Media Foundation；Linux/macOS libuvc + libusb |
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

### macOS USB 权限

macOS 的 UVC 原始视频采集需要管理员权限。请使用 sudo 启动调用 SDK 的程序：

```sh
sudo ./build/usb/examples/cb_camera_example 10 1 1 0
```

SDK 通过 libuvc/libusb 接收 UVC 原始 MJPEG，解析 SCAM APP1 中的板端时间戳、
帧序号和曝光参数。`cb_video_start` 收到带有效时间戳的首帧后返回成功；无帧或
元数据无效时返回具体错误。

macOS 的驱动捕获作用于整个复合 USB 设备。捕获期间，MTP、系统相机应用和用户 ACM
等系统驱动接口可能暂时不可用；最后释放设备时可能发生一次 USB 重新枚举。请先关闭
其他占用相机的应用。

权限和设备捕获行为参见 [libusb 官方讨论](https://github.com/libusb/libusb/issues/1014) 与
[Apple USB 捕获说明](https://developer.apple.com/documentation/iousbhost/iousbhostobjectinitoptions/devicecapture)。

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
| SLAM | `cb_slam_start/stop`、`cb_slam_set_processing`、`cb_slam_set_elevator`、`cb_slam_get_pose`、`cb_slam_poll_json` |
| 相机标定 | `cb_calibration_command`、`cb_get_calibration_status`、`cb_get_calibration_preview`、`cb_request_calibration_report` |
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

## 板端标定任务

USB 和 LOCAL 后端使用同一组 `cb_calibration_*` 接口，需要固件提供
`CB_CAP_CALIBRATION` 能力。旧固件返回 `-ENOTSUP`。

`cb_calibration_command()` 是异步请求。返回 0 表示已排队，应用应等待
`CB_EVENT_CALIBRATION_STATUS` 中相同 `request_id` 的 JSON 回复，并检查 `ok/error`。
无请求号的事件是周期状态，包含 `version=1`、`task`、`phase`、`step`、
`hardware_ready`、`quiet_elapsed_s`、`remaining_s`、`coverage_percent`、
`iteration/max_iterations`、`cost` 和 `heartbeat_ns` 等字段。
`cb_get_calibration_status()` 复制最近状态，尚未收到状态时返回 `{}`。

正常流程为 `ENTER` → 等待 `hardware_ready` → `START_IMU` 或 `START_CAMERA` →
`SOLVE` → 等待 `task=completed` → `LEAVE`。使用 `CANCEL` 取消采集或求解，
收到完成/取消状态后再 `LEAVE`；文件写入与回读阶段不接受取消。
`START_CAMERA` 需要填写 `cb_calibration_options` 中实际标定板参数，并要求共享
YAML 中已有有效 IMU 噪声。`START_IMU` 需要连续静置至少 7200 秒才可 `SOLVE`。

标定模式暂停普通 IMU、深度和 SLAM 的 bulk 数据，原始 UVC 预览保持可用。
临时 30fps 配置在 `LEAVE` 后恢复。应用应先停止正在进行的录制和 SLAM；
切换页面或 SDK 断线不会取消板端标定任务，重新连接后先查询状态。

标定预览通过 `CB_EVENT_CALIBRATION_PREVIEW` 或
`cb_get_calibration_preview()` 获取。事件 payload 的前 32 字节为小端元数据：
`version/width/height/camera_count`（各 4 字节），随后为 `sequence/timestamp_ns`
（各 8 字节），其后为 JPEG。Getter 单独填充 `cb_calibration_preview` 并复制 JPEG。
预览包含两目原始画面和历史角点覆盖，标定计算使用原始分辨率坐标。
预览回调与控制回调可能并发，耗时图像处理应交给应用自己的线程。

`cb_request_calibration_report()` 异步读取报告，不要求 `ENTER`，USB 和 LOCAL 行为相同。
类型为 `CB_CALIBRATION_REPORT_CURRENT`（当前 YAML）、`CB_CALIBRATION_REPORT_JOINT`
（双目与相机–IMU）或 `CB_CALIBRATION_REPORT_IMU`（静态噪声）。相同请求号的
`CB_EVENT_CALIBRATION_STATUS` 回复包含 `ok/error`；成功时包含：

```json
{"ok":true,"report_chunk":{"kind":"joint","revision":123,"offset":0,"total":20000,"data":"..."}}
```

首次传入 `revision=0, offset=0`，后续携带返回的 `revision` 与累计数据长度。
每片最多 8192 字节，报告最多 2 MiB。`data` 为 ASCII JSON 文本片段，中文用 JSON 转义表示。
按 `offset` 顺序拼接到 `total`，验证整个 ASCII 文本的 FNV-1a-32 等于 `revision` 后，
再解析为 JSON。报告变化或连接中断时从零重新读取。分片回复不会替换周期状态缓存。

当前参数随请求读取 YAML；两种详细报告由对应阶段完成后分别持久化，包含当时的 YAML 快照。
报告字段 `version=1`，`kind=current/joint/imu`。历史 YAML 不含误差曲线，尚未生成详细报告时
请求会返回明确错误。状态的 `reports.current/joint/imu` 可用于判断文件是否存在。

## 多应用使用

板端本地连接各自维护 IMU、深度订阅和帧缓存。关闭一个本地连接只释放该连接的
需求；采集服务保留 USB 或其他本地连接仍需要的功能。视频订阅由 broker 单独管理。
帧率、投影、焦距和天线属于全设备配置，修改会影响全部应用。

同一时刻由一个 SDK 连接控制 SLAM 会话和地图增量同步。其他连接可继续使用相机、
IMU 和深度；其 SLAM 会话在当前控制连接释放后重试。USB 后端在稠密 SLAM 开启时
自动关闭本连接的 IMU 上传，运行期间开启请求返回 `-EBUSY`；停止或失败后解除限制，
不自动恢复上传。LOCAL 后端没有此限制，可同时接收稠密地图和 IMU。算法内部 IMU
持续运行，其他本地连接的 IMU 订阅不受 USB 策略影响。稀疏 SLAM 与独立深度预览
可同时使用。

`cb_slam_set_processing(camera, backend_enabled, dense_reconstruction)` 在空闲时设置
下次启动的选项，在运行中切换处理模式。关闭后端时自动关闭稠密重建，前端位姿
继续输出；再次开启后端会继续使用已有地图。调用成功表示命令已排队，完成结果
及实际的 `backend_enabled`、`dense_reconstruction`、`processing_options_pending`
由 SLAM JSON 返回。稠密重建仅处理开启后的新帧，不补算稀疏阶段未保存的深度。

电梯运动估计默认关闭，无配置文件开关。调用 `cb_slam_set_elevator(camera, 1)`
开启，传入 `0` 关闭；空闲时设置下次启动的状态，运行中在帧边界生效，无需重启
SLAM。关闭时保留已估计的世界位置与速度，并恢复普通定位和建图；再次开启会重新
积累启停检测证据。调用返回 `0` 表示已排队，实际状态与切换进度通过 SLAM JSON 的
`elevator_enabled`、`elevator_options_pending` 和操作完成结果确认。

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
