
CFG_LITTLE ?= 0
CFG_RELOC ?= 1
CFG_UNCACHED ?= 0
CFG_VAPI ?= 0
CFG_BOOTRAM ?= 0
CFG_BOARDNAME = "SIM_SWARM"
CFG_PCI = 1
CFG_LDT_REV_017 = 1
CFG_RAMAPP = 0
ifeq ($(strip ${CFG_RELOC}),STATIC)
  CFG_TEXT_START = 0x81000000
endif
CFG_ZLIB = 0
CFG_ZIPPED_CFE = 0

TOP = ../../../cfe
ARCH = mips
CPU = sb1250
CHIPSET = sibyte
BOARD = swarm

include ${TOP}/main/cfe.mk

CFLAGS += -DSIBYTE_HDR_FEATURES="(SIBYTE_HDR_FMASK_1250_ALL|SIBYTE_HDR_FMASK_112x_ALL)"

BSPOBJS = swarm_init.o swarm_devs.o diag_null.o
ZSOBJS += swarm_init.o

ifeq ($(strip ${CFG_PCI}),1)
  BSPOBJS += swarm_pci.o
endif

#
# These settings speed up the DRAM init shortening the delays and
# by not doing too much of it.  This helps boot time in the sim.
# It also adjusts timers and other hardware stuff for running
# in the functional simulator (mostly changes CPU speed to 
# 500Khz).  Remove this before using on real hardware.
#
#CFLAGS +=  -D_UNICPU_ 
CFLAGS += -D_FASTEMUL_ -D_FUNCSIM_


ALL : cfe cfe.flash
	echo done

include ${TOP}/main/cfe_link.mk


