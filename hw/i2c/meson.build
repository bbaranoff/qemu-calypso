# hw/i2c/meson.build
# Ajouter cette ligne au meson.build existant dans hw/i2c/ de votre arbre QEMU
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_i2c.c'))
