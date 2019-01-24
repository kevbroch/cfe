
#
# This is just a Makefile fragment -- it is included by the master
# makefile, cfe.mk
#
# This file should just append object file names to "ALLOBJS",
# but since it is mean to be linked *first*, it will append
# modules to "CRT0OBJS"
#

ifndef INIT_PPC
INIT_PPC = init_ppc.o
endif

ifeq ($(strip ${CFG_RAMAPP}),1)
CRT0OBJS += init_ram.o exception.o
else
CRT0OBJS += $(INIT_PPC) exception.o
endif

ALLOBJS += lib_setjmp.o disasm.o ppc_arena.o exchandler.o
LDLIBS += -L$(dir $(LIBGCC)) -lgcc

ALLOBJS += dev_generic_flashop.o

ifeq ($(strip ${CFG_PPCBOOT}),1)
ALLOBJS += ppcboot.o
CFLAGS += -DCFG_PPCBOOT=1
endif


makereg : ${TOP}/hosttools/makereg.c
	$(HOST_CC) $(HOST_CFLAGS) -o makereg ${TOP}/hosttools/makereg.c

%.inc : %.regdef makereg
	./makereg $< $@ 




