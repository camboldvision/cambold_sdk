/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CB_INTERNAL_H
#define CB_INTERNAL_H

#include "cb_sdk.h"

#ifdef __cplusplus
extern "C"
{
#endif

    int cb_internal_open(const cb_config *config, cb_t **out_host);
    void cb_internal_close(cb_t *host);
    int cb_internal_connected(const cb_t *host);
    int cb_internal_refresh_connection(cb_t *host);
    uint64_t cb_internal_next_request_id(cb_t *host);
    int cb_internal_request(cb_t *host, uint16_t message_type, const uint8_t *payload,
                            size_t payload_size, uint64_t request_id);
    int cb_internal_debug_open(cb_t *host, const char *port, uint32_t baudrate);
    void cb_internal_debug_close(cb_t *host);
    int cb_internal_debug_write(cb_t *host, const uint8_t *data, size_t size);
    int cb_internal_debug_read(cb_t *host, uint8_t *data, size_t size, int timeout_ms);
    int cb_internal_copy_control_port(const cb_t *host, char *out, size_t capacity);
    int cb_internal_copy_debug_port(const cb_t *host, char *out, size_t capacity);
    int cb_internal_copy_device_name(const cb_t *host, char *out, size_t capacity);
    int cb_internal_copy_metrics(const cb_t *host, char *out, size_t capacity);
    int cb_internal_copy_error(const cb_t *host, char *out, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
