
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

BSPOBJS += sentosa_init.o sentosa_devs.o cpu1cmds.o

ifeq ($(strip ${CFG_PCI}),1)
BSPOBJS += sentosa_pci.o
endif

SMBDEVS = 24lc128 m41t81clock x1240eeprom x1241clock

ALLOBJS += ui_sentosa.o
