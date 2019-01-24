
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS",
# but since it is mean to be linked *first*, it will append
# modules to "CRT0OBJS"
#
#
#  Macro               Contains
#  CRT0OBJS            Files that must be linked first for CFE
#  ALLOBJS             Other files for CFE
#  ZCRTO0OBJS          Files that must be linked first for ZIPSTART
#  ZSOBJS              ZIPSTART object files
#

#
# Allow makefile to override init_mips.S.  If not overridden, it's
# either 'init_mips.S' or 'init_ram.o' for RAMAPP and ZIPSTART versions.
#

ifndef INIT_MIPS
  INIT_MIPS = init_mips.o

  ifeq ($(strip ${CFG_RAMAPP}),1)
    INIT_MIPS = init_ram.o
  endif

  ifeq ($(strip ${CFG_ZIPSTART}),1)
    INIT_MIPS = init_ram.o
  endif
endif

FLASH_ENGINE ?= dev_flashop_engine.o

CRT0OBJS += $(INIT_MIPS) apientry.o exception.o

ZCRT0OBJS += zipstart_init.o zipstart_entry.o

ALLOBJS += lib_setjmp.o disasm.o mips_arena.o exchandler.o
ALLOBJS += dev_flash_all.o ${FLASH_ENGINE}
ALLOBJS += lib_hssubr.o

CLEANOBJS += cpu_socregs.inc

makereg : ${TOP}/hosttools/makereg.c
	$(HOST_CC) $(HOST_CFLAGS) -o makereg ${TOP}/hosttools/makereg.c

%.inc : %.regdef makereg
	./makereg $< $@ ${CPU}_regs.h
	rm -f ./cpu_socregs.inc
	ln -s $@ ./cpu_socregs.inc

vapi.o : ${TOP}/verif/vapi.S ${CPU}_socregs.inc



