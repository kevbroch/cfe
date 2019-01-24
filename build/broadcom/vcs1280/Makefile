
CFG_LITTLE ?= 0
CFG_RELOC ?= 1
CFG_UNCACHED ?= 0
CFG_VAPI ?= 0
CFG_BOOTRAM ?= 0
CFG_BOARDNAME = "VCS1280"
CFG_PCI = 0

TOP = ../../../cfe
ARCH = mips
CPU = bcm1480
CHIPSET = sibyte
BOARD = vcs1280

include ${TOP}/main/cfe.mk

BSPOBJS = vcs1280_init.o vcs1280_devs.o

#
# These settings speed up the cache init by not doing too much of it.
# This helps boot time in the sim.
# They also adjust timers and other hardware stuff 
# for running in the functional simulator (mostly changes CPU speed to 
# 500Khz).  Remove this before using on real hardware.
#
CFLAGS += -D_FASTINIT_ 
CFLAGS += -D_VERILOG_
# -D_FASTEMUL_ -D_FUNCSIM_
CFLAGS += -D_DENALI_
CFLAGS += -DSIBYTE_HDR_FEATURES="(SIBYTE_HDR_FMASK_1480_ALL)"


ALL : cfe cfe.flash
	sb1-elf-objdump -d -l cfe > cfe.dis
	echo done

include ${TOP}/main/cfe_link.mk
