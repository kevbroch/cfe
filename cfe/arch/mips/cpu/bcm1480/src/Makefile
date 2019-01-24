
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

ALLOBJS += bcm1480_cpu.o sb1_cpuinit.o bcm1480_altcpu.o bcm1480_l1cache.o \
           bcm1480_l2cache.o bcm1480_draminit.o bcm1480_utils.o bcm1480_hsp_utils.o

ifeq ($(strip ${CFG_PCI}),1)
ALLOBJS += bcm1480_pci_machdep.o
endif

#
# Enable 1480-family-specific code sections
#
CFLAGS += -DSIBYTE_BCM1480

#
# Deal with pass1's little quirks
#
CFLAGS += -D_BCM1480_PASS1_WORKAROUNDS_=1

#
# Deal with pass2's little quirks
#
CFLAGS += -D_BCM1480_PASS2_WORKAROUNDS_=1

#
# If you have a BCM1480 S0 (first preproduction sample), the 
# define below must be enabled to work around erratum SOC-81
# which restricts the available divisors in the memory controller.
# Revision An samples may have fuse problems, reporting incorrect
# chip revisions at high temperatures, so this particular fix must
# be made at compile time, not run-time.
#
# CFLAGS += -D_BCM1480_S0_WORKAROUNDS_=1


