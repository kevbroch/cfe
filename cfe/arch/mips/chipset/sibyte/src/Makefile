
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS"
#

ALLOBJS += sb1250_arena.o

ALLOBJS += dev_sb1250_uart.o dev_sb1250_ethernet.o dev_sb1250_pcmcia.o dev_jtag.o

ALLOBJS += dev_sb1250_pcihost.o cfe_device_ldr.o

ALLOBJS += sb1250_smbus.o sb1250_mii.o

ALLOBJS += ui_soccmds.o ui_corecmds.o ui_tempsensor.o 

ALLOBJS += cfe_tests.o 

ALLOBJS += ui_reset.o

ALLOBJS += ui_cpuinfo.o

ALLOBJS += ui_spdcmds.o

ALLOBJS += ui_memtest.o 

# only include below ui for bcm1480 cpu types
ifeq ($(strip ${CPU}), bcm1480)
ALLOBJS += ui_swtrace.o ui_pmcmds.o

ALLOBJS += ui_hspcmds.o
endif

ui_soccmds.o : ${CHIPSET_SRC}/ui_soccmds.c ${CPU}_socregs.inc


# Pick compiler flags based on what the compiler supports.  -mcpu=...
# is the old way; the current compiler does not allow it.  The old
# compiler, unfortunately, didn't do the right thing with -march=...
# without a reasonable -mipsN flag.
ifeq (0,$(shell $(GCC) -mcpu=sb1 -mips4 -S -xc -o /dev/null /dev/null > /dev/null 2>&1 ; echo $$?))
CFLAGS += -mcpu=sb1 -mips4
else
CFLAGS += -march=sb1
endif

#
# Deal with pass1's little quirks
#
#CFLAGS += -D_SB1250_PASS1_WORKAROUNDS_
