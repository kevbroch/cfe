CFG_LITTLE ?= 0
CFG_RELOC ?= 1
CFG_UNCACHED ?= 0
CFG_VAPI ?= 0 
CFG_BOOTRAM ?= 0
CFG_BOARDNAME = "BCM95836CPCI"
CFG_PCI ?= 1
CFG_LDT ?= 0

TOP = ../../../cfe
ARCH = mips
CPU = bcmcore
CHIPSET = bcmsb
BOARD = bcm95836cpci

include ${TOP}/main/cfe.mk

#LDLIBS += -lgcc

ALL : cfe cfe.flash 
	echo done

include ${TOP}/main/cfe_link.mk
