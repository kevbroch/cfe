
CFG_LITTLE ?= 0
CFG_RELOC ?= 1
CFG_UNCACHED ?= 0
CFG_VAPI ?= 0
CFG_BOOTRAM ?= 0
CFG_BOARDNAME = "SWARM"
CFG_ZLIB ?= 1
CFG_PCI ?= 1
CFG_LDT_REV_017 ?= 1
CFG_VGACONSOLE ?= 0
ifeq ($(strip ${CFG_RELOC}),STATIC)
  CFG_TEXT_START = 0x8F000000
endif
CFG_ZIPPED_CFE = 1

TOP = ../../../cfe
ARCH = mips
CPU = sb1250
CHIPSET = sibyte
BOARD = swarm

include ${TOP}/main/cfe.mk
#
# Don't clean these object files if building bi-endian.
# To build a clean bi-endian flash image, use "gmake clean cfe-biendian.flash"
#

ifeq ($(strip ${CFG_BIENDIAN}),0)
CLEANOBJS += cfe.bin.le cfe.bin.be
endif

#CFLAGS += -D_UNICPU_
CFLAGS += -DSIBYTE_HDR_FEATURES="(SIBYTE_HDR_FMASK_1250_ALL|SIBYTE_HDR_FMASK_112x_ALL)"

BSPOBJS = swarm_init.o swarm_devs.o
ZSOBJS += swarm_init.o diag_l2cache.o

ifeq ($(strip ${CFG_PCI}),1)
BSPOBJS += swarm_pci.o
endif

#
# Main target for normal builds
#

ALL : cfe cfe.flash
	echo done

#
# Special targets for bi-endian builds
#

cfe.bin.le :
	${MAKE} clean all CFG_BIENDIAN=1 CFG_LITTLE=1 CFG_RELOC=1 
	cp cfe.bin cfe.bin.le

cfe.bin.be :
	${MAKE} clean all CFG_BIENDIAN=1 CFG_LITTLE=0 CFG_RELOC=1 
	cp cfe.bin cfe.bin.be

biend : cfe-biendian.flash cfe-biendian.srec
	echo done

cfe-biendian.flash : cfe.bin.le cfe.bin.be mkflashimage
	./mkflashimage -v -EX -B ${CFG_BOARDNAME} -V ${CFE_VER_MAJ}.${CFE_VER_MIN}.${CFE_VER_ECO} cfe.bin.be cfe.bin.le cfe-biendian.flash

cfe-biendian.srec : cfe-biendian.flash
	dd if=cfe-biendian.flash of=cfe-biendian.bin bs=64 skip=1
	$(OBJCOPY) --input-target=binary --output-target=srec cfe-biendian.bin cfe-biendian.srec

boot : cfe.flash
	relay swarm:on ; romload cfe.flash ; relay swarm:off

#
# Linker script
#

include ${TOP}/main/cfe_link.mk


