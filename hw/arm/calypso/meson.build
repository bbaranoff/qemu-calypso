# hw/arm/calypso/meson.build

# libosmocoding for RACH/NB UL burst encoding (gsm0503_rach_ext_encode etc).
# Needed by calypso_bsp.c::calypso_bsp_tx_rach_burst.
osmocoding = dependency('libosmocoding', required: false)

arm_ss.add(when: 'CONFIG_CALYPSO', if_true: [
  files(
    'calypso_mb.c',
    'calypso_soc.c',
    'calypso_trx.c',
    'calypso_c54x.c',
    'calypso_fbsb.c',
    'calypso_bsp.c',
    'calypso_dbg.c',
    'calypso_iota.c',
    'l1ctl_sock.c',
    'sercomm_gate.c',
    'calypso_tint0.c',
    'fw_console.c',
    'calypso_sim.c',
  ),
  osmocoding,
])
