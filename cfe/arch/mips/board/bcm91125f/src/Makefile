
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

BSPOBJS += bcm91125f_init.o bcm91125f_devs.o

ifeq ($(strip ${CFG_PCI}),1)
BSPOBJS += bcm91125f_pci.o
endif

PCIDEVS = sb1250_pcihost

SMBDEVS = 24lc128 m41t81clock at24c02

ALLOBJS += ui_bcm91125f.o bcm91125f_tests.o
