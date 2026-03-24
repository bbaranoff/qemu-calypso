/*
 * calypso_socket.c — Calypso Socket device for communication with transceiver
 *
 * Version améliorée utilisant le système d'événements QEMU au lieu de threads.
 * Plus propre et mieux intégré avec QEMU.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"
#include "hw/arm/calypso/calypso_socket.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include "qemu/cutils.h"

/* Forward declarations */
static void calypso_socket_read_handler(void *opaque);
static void calypso_socket_accept_handler(void *opaque);

void calypso_socket_reset(DeviceState *dev)
{
    CalypsoSocketState *s = CALYPSO_SOCKET(dev);

    if (s->socket_fd >= 0) {
        qemu_set_fd_handler(s->socket_fd, NULL, NULL, NULL);
        close(s->socket_fd);
        s->socket_fd = -1;
    }
    
    if (s->client_fd >= 0) {
        qemu_set_fd_handler(s->client_fd, NULL, NULL, NULL);
        close(s->client_fd);
        s->client_fd = -1;
    }
    
    s->status = 0;
    s->rx_len = 0;
}

/* Called when there's data to read from the client connection */
static void calypso_socket_read_handler(void *opaque)
{
    CalypsoSocketState *s = (CalypsoSocketState *)opaque;
    uint8_t buffer[1024];
    
    int bytes_received = recv(s->client_fd, buffer, sizeof(buffer), 0);
    
    if (bytes_received > 0) {
        /* Store in RX buffer if there's space */
        if (s->rx_len + bytes_received < sizeof(s->rx_buffer)) {
            memcpy(&s->rx_buffer[s->rx_len], buffer, bytes_received);
            s->rx_len += bytes_received;
            s->status |= CALYPSO_SOCKET_STATUS_READY;
        } else {
            s->status |= CALYPSO_SOCKET_STATUS_ERROR;
        }
    } else if (bytes_received == 0) {
        /* Client disconnected */
        qemu_set_fd_handler(s->client_fd, NULL, NULL, NULL);
        close(s->client_fd);
        s->client_fd = -1;
        s->status &= ~CALYPSO_SOCKET_STATUS_READY;
    } else {
        /* Error */
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            s->status |= CALYPSO_SOCKET_STATUS_ERROR;
            qemu_set_fd_handler(s->client_fd, NULL, NULL, NULL);
            close(s->client_fd);
            s->client_fd = -1;
        }
    }
}

/* Called when there's a new connection to accept */
static void calypso_socket_accept_handler(void *opaque)
{
    CalypsoSocketState *s = (CalypsoSocketState *)opaque;
    struct sockaddr_un client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    /* Close existing client if any */
    if (s->client_fd >= 0) {
        qemu_set_fd_handler(s->client_fd, NULL, NULL, NULL);
        close(s->client_fd);
    }
    
    /* Accept new connection */
    s->client_fd = accept(s->socket_fd, (struct sockaddr *)&client_addr, &client_len);
    
    if (s->client_fd < 0) {
        error_report("calypso-socket: accept failed: %s", strerror(errno));
        return;
    }
    
    /* Set non-blocking */
    int flags = fcntl(s->client_fd, F_GETFL, 0);
    fcntl(s->client_fd, F_SETFL, flags | O_NONBLOCK);
    
    /* Register read handler for this client */
    qemu_set_fd_handler(s->client_fd, calypso_socket_read_handler, NULL, s);
    
    s->status |= CALYPSO_SOCKET_STATUS_READY;
}

static uint64_t calypso_socket_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoSocketState *s = (CalypsoSocketState *)opaque;
    
    switch (offset) {
    case CALYPSO_SOCKET_STATUS:
        return s->status;
    
    case CALYPSO_SOCKET_DATA:
        if (s->rx_len > 0) {
            uint8_t data = s->rx_buffer[0];
            s->rx_len--;
            memmove(&s->rx_buffer[0], &s->rx_buffer[1], s->rx_len);
            
            /* Clear ready flag if buffer now empty */
            if (s->rx_len == 0) {
                s->status &= ~CALYPSO_SOCKET_STATUS_READY;
            }
            
            return data;
        }
        return 0;
    
    default:
        return 0;
    }
}

static void calypso_socket_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    CalypsoSocketState *s = (CalypsoSocketState *)opaque;
    
    switch (offset) {
    case CALYPSO_SOCKET_CTRL:
        if (value & CALYPSO_SOCKET_CTRL_START) {
            if (s->socket_fd >= 0 && s->client_fd < 0) {
                /* Already have listening socket, just wait for connection */
                s->status |= CALYPSO_SOCKET_STATUS_READY;
            }
        }
        
        if (value & CALYPSO_SOCKET_CTRL_STOP) {
            if (s->client_fd >= 0) {
                qemu_set_fd_handler(s->client_fd, NULL, NULL, NULL);
                close(s->client_fd);
                s->client_fd = -1;
            }
            s->status &= ~CALYPSO_SOCKET_STATUS_READY;
        }
        
        if (value & CALYPSO_SOCKET_CTRL_RESET) {
            calypso_socket_reset(DEVICE(&s->parent_obj));
        }
        break;
    
    case CALYPSO_SOCKET_DATA:
        /* Send data to client if connected */
        if (s->client_fd >= 0) {
            uint8_t byte = value & 0xFF;
            ssize_t sent = send(s->client_fd, &byte, 1, 0);
            
            if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                s->status |= CALYPSO_SOCKET_STATUS_ERROR;
            } else if (sent > 0) {
                s->status |= CALYPSO_SOCKET_STATUS_TX;
            }
        }
        break;
    
    default:
        break;
    }
}

static const MemoryRegionOps calypso_socket_ops = {
    .read = calypso_socket_read,
    .write = calypso_socket_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void calypso_socket_realize(DeviceState *dev, Error **errp)
{
    CalypsoSocketState *s = CALYPSO_SOCKET(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->socket_path[0] == '\0') {
        error_setg(errp, "socket-path property not set");
        return;
    }

    /* Create UNIX socket */
    s->socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (s->socket_fd < 0) {
        error_setg(errp, "Failed to create UNIX socket: %s", strerror(errno));
        return;
    }

    /* Set non-blocking */
    int flags = fcntl(s->socket_fd, F_GETFL, 0);
    fcntl(s->socket_fd, F_SETFL, flags | O_NONBLOCK);

    /* Remove existing socket file if present */
    unlink(s->socket_path);

    /* Set up socket address */
    memset(&s->socket_addr, 0, sizeof(s->socket_addr));
    s->socket_addr.sun_family = AF_UNIX;
    pstrcpy(s->socket_addr.sun_path,
            sizeof(s->socket_addr.sun_path),
            s->socket_path);

    /* Bind to socket path */
    if (bind(s->socket_fd, (struct sockaddr *)&s->socket_addr,
             sizeof(s->socket_addr)) < 0) {
        error_setg(errp, "Failed to bind to socket: %s", strerror(errno));
        close(s->socket_fd);
        s->socket_fd = -1;
        return;
    }

    /* Listen on socket */
    if (listen(s->socket_fd, 1) < 0) {
        error_setg(errp, "Failed to listen on socket: %s", strerror(errno));
        close(s->socket_fd);
        s->socket_fd = -1;
        return;
    }

    /* Register accept handler with QEMU event loop */
    qemu_set_fd_handler(s->socket_fd, calypso_socket_accept_handler, NULL, s);

    /* Set up MMIO region */
    memory_region_init_io(&s->mmio, OBJECT(dev), &calypso_socket_ops, s,
                          TYPE_CALYPSO_SOCKET, 0x10);
    sysbus_init_mmio(sbd, &s->mmio);
}

static void calypso_socket_instance_init(Object *obj)
{
    CalypsoSocketState *s = CALYPSO_SOCKET(obj);
    
    s->socket_fd = -1;
    s->client_fd = -1;
    s->status = 0;
    s->rx_len = 0;
}

static void calypso_socket_finalize(Object *obj)
{
    CalypsoSocketState *s = CALYPSO_SOCKET(obj);
    
    if (s->socket_fd >= 0) {
        qemu_set_fd_handler(s->socket_fd, NULL, NULL, NULL);
        close(s->socket_fd);
        unlink(s->socket_path);
        s->socket_fd = -1;
    }
    
    if (s->client_fd >= 0) {
        qemu_set_fd_handler(s->client_fd, NULL, NULL, NULL);
        close(s->client_fd);
        s->client_fd = -1;
    }
}

static Property calypso_socket_properties[] = {
    DEFINE_PROP_STRING("socket-path", CalypsoSocketState, socket_path),
    DEFINE_PROP_END_OF_LIST(),
};

static void calypso_socket_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    
    dc->realize = calypso_socket_realize;
    device_class_set_legacy_reset(dc, calypso_socket_reset);
    dc->desc = "Calypso UNIX domain socket interface";
    device_class_set_props(dc, calypso_socket_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo calypso_socket_info = {
    .name = TYPE_CALYPSO_SOCKET,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoSocketState),
    .instance_init = calypso_socket_instance_init,
    .instance_finalize = calypso_socket_finalize,
    .class_init = calypso_socket_class_init,
};

static void calypso_socket_register_types(void)
{
    type_register_static(&calypso_socket_info);
}

type_init(calypso_socket_register_types)
