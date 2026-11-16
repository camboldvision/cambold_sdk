/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CB_PROTOCOL_H
#define CB_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CB_PROTOCOL_MAGIC ((uint16_t)0x5343u)
#define CB_PROTOCOL_MAJOR ((uint8_t)1u)
#define CB_PROTOCOL_MINOR ((uint8_t)0u)
#define CB_PROTOCOL_HEADER_SIZE ((uint16_t)40u)
#define CB_PROTOCOL_MAX_PAYLOAD (1024u * 1024u)
#define CB_CAP_STEREO_DISPARITY (1u << 7)
#define CB_CAP_SLAM (1u << 8)
#define CB_CAP_CALIBRATION (1u << 9)

    enum cb_message_type
    {
        CB_HELLO_REQ = 0x0001,
        CB_HELLO_RSP = 0x8001,
        CB_GET_CAPS_REQ = 0x0002,
        CB_GET_CAPS_RSP = 0x8002,
        CB_HEARTBEAT = 0x0003,
        CB_CONTROL_ACCEPTED_EVT = 0x0004,
        CB_JOB_STATUS_EVT = 0x0005,
        CB_CONTROL_RESULT_EVT = 0x0006,
        CB_FRAME_META = 0x0010,
        CB_IMU_BATCH = 0x0011,
        CB_SENSOR_STATUS = 0x0012,
        CB_STREAM_STATUS_REQ = 0x0020,
        CB_STREAM_STATUS_EVT = 0x0021,
        CB_IMU_STREAM_SET_REQ = 0x0022,
        CB_IMU_STREAM_STATUS_EVT = 0x0023,
        CB_FRAME_RATE_SET_REQ = 0x0024,
        CB_DEVICE_CONFIG_REQ = 0x0025,
        CB_DEVICE_CONFIG_EVT = 0x0026,
        CB_SYSTEM_METRICS_REQ = 0x0030,
        CB_SYSTEM_METRICS_EVT = 0x0031,
        CB_ANTENNA_GET_REQ = 0x0040,
        CB_ANTENNA_SET_REQ = 0x0041,
        CB_ANTENNA_STATUS_EVT = 0x0042,
        /* 0x0050..0x0052 are reserved for retired commands. */
        CB_DEVICE_INFO_REQ = 0x0060,
        CB_RESET_REQ = 0x0061,
        CB_TERMINAL_SIZE_SET_REQ = 0x0062,
        CB_STEREO_STATUS_REQ = 0x0070,
        CB_STEREO_ENABLE_REQ = 0x0071,
        CB_STEREO_CONFIG_REQ = 0x0072,
        CB_STEREO_STATUS_EVT = 0x0073,
        CB_DISPARITY_CHUNK = 0x0074,
        CB_SLAM_PACKET = 0x0080,
        CB_SLAM_LINK = 0x0081,
        CB_SLAM_LINK_STATUS = 0x0082,
        CB_CALIBRATION_COMMAND = 0x0090,
        CB_CALIBRATION_STATUS = 0x0091,
        CB_CALIBRATION_PREVIEW_CHUNK = 0x0092
    };

    enum cb_result_code
    {
        CB_RESULT_OK = 0,
        CB_RESULT_INVALID = -22,
        CB_RESULT_NOT_READY = -11,
        CB_RESULT_IO = -5,
        CB_RESULT_BUSY = -16
    };

    enum cb_sensor_error_code
    {
        CB_SENSOR_ERROR_NONE = 0,
        CB_SENSOR_ERROR_RK_FSYNC_SOURCE_MISSING = 1001,
        CB_SENSOR_ERROR_IMU_FSYNC_INPUT_MISSING = 1002,
        CB_SENSOR_ERROR_FSYNC_PAIRING_UNAVAILABLE = 1003
    };

#ifdef __cplusplus
}
#endif

#endif
