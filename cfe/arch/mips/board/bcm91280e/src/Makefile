
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

BSPOBJS += bcm91280e_init.o bcm91280e_devs.o cpu1cmds.o cpu1test.o

ifeq ($(strip ${CFG_PCI}),1)
BSPOBJS += bcm91280e_pci.o
endif

PCIDEVS =

SMBDEVS = 24lc128 m41t81clock

ALLOBJS += ui_bcm91280e.o 

