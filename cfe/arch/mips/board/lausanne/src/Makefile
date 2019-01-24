
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

BSPOBJS = lausanne_init.o lausanne_devs.o

ifeq ($(strip ${CFG_PCI}),1)
BSPOBJS += lausanne_pci.o
endif

SMBDEVS = 24lc128

ALLOBJS += cpu1test.o cpu1cmds.o ui_lausanne.o
