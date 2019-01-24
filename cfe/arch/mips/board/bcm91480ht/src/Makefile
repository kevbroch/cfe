
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

BSPOBJS += bcm91480ht_init.o bcm91480ht_devs.o cpu1cmds.o cpu1test.o

ifeq ($(strip ${CFG_PCI}),1)
BSPOBJS += bcm91480ht_pci.o
endif

SMBDEVS = 24lc128 m41t81clock switch

ALLOBJS += ui_bcm91480ht.o 

ifeq ($(strip ${CFG_LDT}),1)
BSPOBJS += ui_ccncmds.o cc_numa.o
endif
