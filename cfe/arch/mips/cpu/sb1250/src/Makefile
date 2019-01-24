
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#
#
#  Macro               Contains
#  CRT0OBJS            Files that must be linked first for CFE
#  ALLOBJS             Other files for CFE
#  ZCRTO0OBJS          Files that must be linked first for ZIPSTART
#  ZSOBJS              ZIPSTART object files
#

#
# These files are needed by ZIPSTART
#

ZSOBJS += sb1250_cpu.o sb1_cpuinit.o sb1250_altcpu.o sb1250_l1cache.o \
	sb1250_l2cache.o sb1250_draminit.o diag_null.o

#
# These files are needed by normal CFE
#

ALLOBJS += sb1250_cpu.o sb1_cpuinit.o sb1250_altcpu.o sb1250_l1cache.o \
           sb1250_l2cache.o sb1250_draminit.o sb1250_utils.o


ifeq ($(strip ${CFG_PCI}),1)
ALLOBJS += sb1250_pci_machdep.o
endif

ALLOBJS += diag_l2cache.o

#
# Enable 1250/112x-specific code sections
#
CFLAGS += -DSIBYTE_SB1250

#
# Deal with pass1's little quirks
#
#CFLAGS += -D_SB1250_PASS1_WORKAROUNDS_
