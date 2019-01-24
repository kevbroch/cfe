
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

BSPOBJS += bmw_devs.o bmw_init.o
ifeq ($(strip ${CFG_PCI}),1)
BSPOBJS += mpc824x_pci_machdep.o bmw_pci.o
endif

PCIDEVS += bcm5700 tulip

SMBDEVS = 

CFLAGS += -DINCLUDE_FLASH_DRIVERS

ALLOBJS += ui_bmw.o ui_vxboot.o

