/*
* uhdwrap.h — protos C only pour calypso-ipc-device.
*
* Fork de osmo-trx/.../ipc/uhdwrap.h : on garde uniquement la partie #else
* (interface C), la classe C++ uhd_wrap est exclue parce que notre backend
* device est UDP 6702 vers QEMU, pas UHD.
*
* L'implémentation de ces 11 hooks vit dans qemu_wrap.c.
*/
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "shm.h"  /* struct ipc_sk_if, struct ipc_sk_if_open_req */

void   *uhdwrap_open(struct ipc_sk_if_open_req *open_req);
int32_t uhdwrap_get_bufsizerx(void *dev);
int32_t uhdwrap_get_timingoffset(void *dev);
int32_t uhdwrap_read(void *dev, uint32_t num_chans);
int32_t uhdwrap_write(void *dev, uint32_t num_chans, bool *underrun);
double  uhdwrap_set_freq(void *dev, double f, size_t chan, bool for_tx);
double  uhdwrap_set_gain(void *dev, double f, size_t chan, bool for_tx);
int32_t uhdwrap_start(void *dev, int chan);
int32_t uhdwrap_stop(void *dev, int chan);
void    uhdwrap_fill_info_cnf(struct ipc_sk_if *ipc_prim);
double  uhdwrap_set_txatt(void *dev, double a, size_t chan);
