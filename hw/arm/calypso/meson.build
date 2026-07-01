# hw/arm/calypso/meson.build

# libosmocoding for RACH/NB UL burst encoding (gsm0503_rach_ext_encode etc).
# Needed by calypso_bsp.c::calypso_bsp_tx_rach_burst — required:true so meson
# configure fails fast if the dev package is missing, instead of failing at
# link time with undefined gsm0503_rach_ext_encode.
osmocoding = dependency('libosmocoding', required: true)

arm_ss.add(when: 'CONFIG_CALYPSO', if_true: [
  files(
    'calypso_mb.c',
    'calypso_soc.c',
    'calypso_trx.c',
    'calypso_c54x.c',
    'calypso_layer1.c',
    'calypso_fbsb.c',
    'calypso_bsp.c',
    'calypso_iota.c',
    'calypso_twl3025.c',
    'l1ctl_sock.c',
    'sercomm_gate.c',
    'calypso_tint0.c',
    'fw_console.c',
    'calypso_sim.c',
    'calypso_full_pcb.c',
    'calypso_debug.c',
    'calypso_dsp_shunt.c',
  ),
  osmocoding,
])
