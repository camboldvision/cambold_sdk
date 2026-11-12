/* SPDX-License-Identifier: Apache-2.0 */
/* This exact source builds for a USB host and for an on-camera application. */
#include "cb_sdk.h"
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#endif

static volatile sig_atomic_t running = 1;
static void interrupt(int signal_number)
{
    (void)signal_number;
    running = 0;
}

static void on_event(const cb_event *event, void *user)
{
    (void)user;
    switch (event->type)
    {
    case CB_EVENT_CONNECTED:
        puts("Camera connected");
        break;
    case CB_EVENT_DISCONNECTED:
        puts("Camera disconnected");
        break;
    case CB_EVENT_DEVICE_CONFIG:
    {
        cb_device_config config;
        if (cb_decode_device_config(event->payload, event->payload_size, &config) == 0)
        {
            printf("Frame rate: %u; options:", config.frame_rate);
            for (uint32_t i = 0; i < config.frame_rate_count; ++i)
                printf(" %u", config.frame_rates[i]);
            putchar('\n');
        }
        break;
    }
    case CB_EVENT_IMU_BATCH:
    {
        cb_imu_sample samples[32];
        size_t count = 32;
        if (cb_decode_imu_batch(event->payload, event->payload_size, samples, &count) == 0 && count)
            printf("IMU: %zu samples, last timestamp: %llu ns\n", count,
                   (unsigned long long)samples[count - 1].timestamp_monotonic_ns);
        break;
    }
    case CB_EVENT_RESULT:
        if (event->result)
            fprintf(stderr, "Request %llu: %d %s\n", (unsigned long long)event->request_id,
                    event->result, event->text ? event->text : "");
        break;
    case CB_EVENT_ERROR:
        fprintf(stderr, "%s\n", event->text ? event->text : "Camera I/O error");
        break;
    default:
        break;
    }
}

static void on_video(const cb_video_frame *frame, void *user)
{
    (void)user;
    printf("JPEG %ux%u: %zu bytes, timestamp: %llu ns, exposure L/R: %u/%u us\n", frame->width,
           frame->height, frame->jpeg_size, (unsigned long long)frame->timestamp_monotonic_ns,
           frame->exposure_us[0], frame->exposure_us[1]);
}

int main(int argc, char **argv)
{
    /* Optional duration makes the same example useful for deployment checks.
       Zero means run until Ctrl+C. IMU/depth remain off unless requested. */
    const unsigned duration = argc > 1 ? (unsigned)strtoul(argv[1], NULL, 10) : 0;
    const int imu = argc > 2 && atoi(argv[2]);
    const int video = argc > 3 && atoi(argv[3]);
    const int depth = argc > 4 && atoi(argv[4]);
    cb_config config = {0};
    cb_t *camera = NULL;
    config.callback = on_event;
    if (cb_open(&config, &camera))
        return 1;
    signal(SIGINT, interrupt);
    signal(SIGTERM, interrupt);
    const time_t start = time(NULL);
    int initialized = 0;
    uint64_t last_depth_id = 0;
    cb_video_t *stream = NULL;
    while (running && (!duration || (unsigned)(time(NULL) - start) < duration))
    {
        if (cb_connected(camera) && !initialized)
        {
            char name[512] = {0};
            if (cb_get_device_name(camera, name, sizeof(name)) >= 0)
                puts(name);
            cb_request_metrics(camera, cb_next_request_id(camera));
            if (imu)
                cb_set_imu_output(camera, 1, cb_next_request_id(camera));
            if (depth)
                cb_set_disparity_enabled(camera, 1, cb_next_request_id(camera));
            if (video)
            {
                cb_video_config options = {on_video, NULL};
                char error[256];
                if (!cb_video_open(&options, &stream) &&
                    cb_video_start(stream, error, sizeof(error)))
                    fprintf(stderr, "Video: %s\n", error);
            }
            initialized = 1;
        }
        if (!cb_connected(camera))
        {
            initialized = 0;
            last_depth_id = 0;
            cb_video_close(stream);
            stream = NULL;
        }
        if (depth)
        {
            cb_disparity info;
            static uint16_t disparity[CB_MAX_DISPARITY_WIDTH * CB_MAX_DISPARITY_HEIGHT];
            static uint16_t rgb[CB_MAX_DISPARITY_WIDTH * CB_MAX_DISPARITY_HEIGHT];
            if (!cb_get_disparity(camera, last_depth_id, &info, disparity, sizeof(disparity) / 2, rgb,
                                  sizeof(rgb) / 2))
            {
                last_depth_id = info.frame_id;
                printf("Depth %ux%u, timestamp: %llu ns\n", info.width, info.height,
                       (unsigned long long)info.timestamp_monotonic_ns);
            }
        }
        /* Standard C sleep is intentionally outside callbacks. */
#ifdef _WIN32
        Sleep(20);
#else
        struct timespec delay = {0, 20000000};
        nanosleep(&delay, NULL);
#endif
    }
    cb_video_close(stream);
    cb_close(camera);
    return 0;
}
