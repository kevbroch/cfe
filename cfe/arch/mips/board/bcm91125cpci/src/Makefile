
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

BSPOBJS += bcm91125cpci_init.o bcm91125cpci_devs.o ui_vxboot.o

ifeq ($(strip ${CFG_PCI}),1)
BSPOBJS += bcm91125cpci_pci.o
endif

PCIDEVS = 

SMBDEVS = 24lc128 m41t81clock
