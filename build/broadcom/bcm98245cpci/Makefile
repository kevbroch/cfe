
CFG_LITTLE ?= 0
CFG_RELOC ?= 1
CFG_UNCACHED ?= 0
CFG_VAPI ?= 0
CFG_BOOTRAM ?= 0
CFG_BOARDNAME = "BCM98245CPCI"
CFG_PCI = 1
CFG_LDT = 0
CFG_MSYS ?= 0

TOP = ../../../cfe
ARCH = ppc
CHIPSET = mpc824x
CPU = mpc8245
BOARD = bcm98245cpci

include ${TOP}/main/cfe.mk


ALL : cfe cfe.flash
	echo done

include ${TOP}/main/cfe_link.mk
