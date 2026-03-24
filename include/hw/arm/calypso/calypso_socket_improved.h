/*
 * calypso_socket.h — Calypso Socket device for communication with transceiver
 *
 * Version améliorée sans threads - utilise le système d'événements QEMU
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_CALYPSO_SOCKET_H
#define HW_ARM_CALYPSO_SOCKET_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_CALYPSO_SOCKET "calypso-socket"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoSocketState, CALYPSO_SOCKET)

/* Device state */
struct CalypsoSocketState {
    /*< private >*/
    SysBusDevice parent_obj;
    
    /*< public >*/
    /* Socket handling */
    int socket_fd;              /* Listening socket */
    int client_fd;              /* Connected client */
    struct sockaddr_un socket_addr;
    
    /* Buffer for incoming data */
    uint8_t rx_buffer[1024];
    uint32_t rx_len;
    
    /* Configuration */
    char *socket_path;          /* QOM property: path to UNIX socket */
    
    /* Device status */
    uint32_t status;
    
    /* Memory region */
    MemoryRegion mmio;
};

/* Register offsets */
#define CALYPSO_SOCKET_CTRL   0x00
#define CALYPSO_SOCKET_STATUS 0x04
#define CALYPSO_SOCKET_DATA   0x08

/* Control bits */
#define CALYPSO_SOCKET_CTRL_START  (1 << 0)
#define CALYPSO_SOCKET_CTRL_STOP   (1 << 1)
#define CALYPSO_SOCKET_CTRL_RESET  (1 << 2)

/* Status bits */
#define CALYPSO_SOCKET_STATUS_READY (1 << 0)  /* Data available or connected */
#define CALYPSO_SOCKET_STATUS_ERROR (1 << 1)  /* Error occurred */
#define CALYPSO_SOCKET_STATUS_TX    (1 << 2)  /* TX completed */

/* Reset function */
void calypso_socket_reset(DeviceState *dev);

#endif /* HW_ARM_CALYPSO_SOCKET_H */
