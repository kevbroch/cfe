
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

ALLOBJS += mousse_devs.o mousse_init.o ui_mousse.o dev_m48txx.o
ifeq ($(strip ${CFG_PCI}),1)
ALLOBJS += mpc824x_pci_machdep.o mousse_pci.o
ALLOBJS += mpc824x_pci_devs.o
endif

PCIDEVS = bcm4401 
# On-board PCI
PCIDEVS += tulip

SMBDEVS = 
