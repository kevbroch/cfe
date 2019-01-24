
CFG_LITTLE ?= 0
CFG_RELOC ?= 1
CFG_UNCACHED ?= 0
CFG_VAPI ?= 0
CFG_BOOTRAM ?= 0
CFG_BOARDNAME = "MOUSSE"
CFG_PCI = 1
CFG_LDT = 0

TOP = ../../../cfe
ARCH = ppc
CHIPSET = mpc824x
CPU = mpc8240
BOARD = mousse

include ${TOP}/main/cfe.mk


ALL : cfe cfe.flash
	echo done

include ${TOP}/main/cfe_link.mk
