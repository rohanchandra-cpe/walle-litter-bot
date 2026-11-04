#pragma once

#include <stddef.h>
#include <stdint.h>

#include "lwip/ip_addr.h"
#include "lwip/udp.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"

#define PICOW_UDP_RECEIVE_QUEUE_DEPTH 4

typedef struct {
    const char *agent_ip;
    uint16_t agent_port;
    struct udp_pcb *pcb;
    ip_addr_t agent_address;
    uint8_t receive_buffers[PICOW_UDP_RECEIVE_QUEUE_DEPTH]
                           [UXR_CONFIG_CUSTOM_TRANSPORT_MTU];
    size_t receive_lengths[PICOW_UDP_RECEIVE_QUEUE_DEPTH];
    size_t receive_head;
    size_t receive_tail;
    size_t receive_count;
} picow_udp_transport_context_t;

bool picow_udp_transport_open(struct uxrCustomTransport *transport);
bool picow_udp_transport_close(struct uxrCustomTransport *transport);
size_t picow_udp_transport_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buffer,
    size_t length,
    uint8_t *error_code);
size_t picow_udp_transport_read(
    struct uxrCustomTransport *transport,
    uint8_t *buffer,
    size_t length,
    int timeout,
    uint8_t *error_code);
