#
makereg : ${TOP}/hosttools/makereg.c
	$(HOST_CC) $(HOST_CFLAGS) -o makereg ${TOP}/hosttools/makereg.c

%.inc : %.regdef makereg
	./makereg $< $@ 

LDLIBS +=  -L$(dir $(LIBGCC)) -lgcc
CRT0OBJS = init_x86.o altcpu.o
ALLOBJS += promice.o lib_setjmp.o  disasm.o exchandler.o x86_arena.o
ALLOBJS += misc_x86.o reset_vec.o protmode.o 




