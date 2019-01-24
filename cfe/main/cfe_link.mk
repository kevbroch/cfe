#
# This Makefile snippet takes care of linking the firmware.
#

pci : $(PCICOMMON) $(PCIMACHDEP)
	echo done

.PHONY: $(CFE_LDSCRIPT) $(ZIPSTART_LDSCRIPT)

$(CFE_LDSCRIPT) : $(CFE_LDSCRIPT_TEMPLATE) 
	$(GCPP) -P $(GENLDFLAGS) $(CFE_LDSCRIPT_TEMPLATE) -o $(CFE_LDSCRIPT)


$(ZIPSTART_LDSCRIPT) : $(ZIPSTART_LDSCRIPT_TEMPLATE) 
	$(GCPP) -P $(GENLDFLAGS) $(ZIPSTART_LDSCRIPT_TEMPLATE) -o $(ZIPSTART_LDSCRIPT)


#
# If the relocation type is STATIC, we need ZIPSTART.
#
ifeq ($(strip ${CFG_RELOC}),STATIC)
  CFETARGET = ramcfe
  ZIPSTART  = cfe
else
  CFETARGET = cfe
  ZIPSTART  = zipstart
endif

#
# ZIPSTART linker stuff
#

ZZSOBJS = $(patsubst %,ZS_%,$(ZSOBJS))
ZZCRT0OBJS = $(patsubst %,ZS_%,$(ZCRT0OBJS))

$(LIBZIPSTART) : $(ZZSOBJS)
	rm -f $(LIBZIPSTART)
	$(GAR) cr $(LIBZIPSTART) $(ZZSOBJS)
	$(RANLIB) $(LIBZIPSTART)

$(ZIPSTART) : $(ZZCRT0OBJS) $(LIBZIPSTART) $(ZIPSTART_LDSCRIPT) ramcfe.bin.o
	$(GLD) $(ENDIAN) -o $@ -Map $@.map -g --script $(ZIPSTART_LDSCRIPT) $(ZZCRT0OBJS) ramcfe.bin.o -L. -lzipstart 
	$(OBJDUMP) -d $@ > $@.dis
	$(OBJCOPY) $(OCBINFLAGS) --output-target=binary $@ $@.bin
	$(OBJCOPY) --input-target=binary --output-target=srec $@.bin $@.srec

#
# CFE linker stuff
#


$(CFETARGET) : $(CRT0OBJS) $(BSPOBJS) $(LIBCFE) $(CFE_LDSCRIPT)
	$(GLD) $(ENDIAN) -o $(CFETARGET) -Map $(CFETARGET).map $(CFE_LDFLAGS) $(CRT0OBJS) $(BSPOBJS) -L. -lcfe $(LDLIBS)

$(CFETARGET).bin : $(CFETARGET)
	$(OBJCOPY) $(OCBINFLAGS) --output-target=binary $(CFETARGET) $(CFETARGET).bin
	$(OBJCOPY) --input-target=binary --output-target=srec $(CFETARGET).bin $(CFETARGET).srec

OFMT = $(shell $(OBJDUMP) -i | head -2 | grep elf)

ramcfe.bin.o : $(CFETARGET).bin
ifeq ($(strip ${CFG_ZIPPED_CFE}),1)
	gzip -c $(CFETARGET).bin > $(CFETARGET).bin.gz
	$(GLD) $(ENDIAN) -T ${ARCH_SRC}/binobj.lds -b binary --oformat $(OFMT) -o $(CFETARGET).bin.o ramcfe.bin.gz
else
	$(GLD) $(ENDIAN) -T ${ARCH_SRC}/binobj.lds -b binary --oformat $(OFMT) -o ramcfe.bin.o $(CFETARGET).bin
endif

#
# Make the disassembly listing only on request
#

.PHONY : $(CFETARGET).dis

$(CFETARGET).dis :
	if [ ! -f $(CFETARGET) ]; then \
	    echo "*** no $(CFETARGET)"; \
	elif [ -f $(CFETARGET).dis ] && [ ! $(CFETARGET) -nt $(CFETARGET).dis ]; then \
	    echo "\`$(CFETARGET).dis' is up to date."; \
	else \
	    $(OBJDUMP) -d $(CFETARGET) > $(CFETARGET).dis; \
	fi

#
# Build the flash image
#

cfe.flash : cfe.bin mkflashimage swapflashimage
	./mkflashimage -v ${ENDIAN} -B ${CFG_BOARDNAME} -V ${CFE_VER_MAJ}.${CFE_VER_MIN}.${CFE_VER_ECO} cfe.bin cfe.flash
	$(OBJCOPY) --input-target=binary --output-target=srec cfe.flash cfe.flash.srec

#
# Housecleaning
#

clean :
	rm -f *.o *~ cfe cfe.bin cfe.dis cfe.map cfe.srec cfe.lds ramcfe ramcfe.*
	rm -f makereg ${CPU}_socregs.inc cpu_socregs.inc
	rm -f mkpcidb pcidevs*.h mkflashimage swapflashimage
	rm -f build_date.c
	rm -f zipstart.lds zipstart zipstart.srec
	rm -f libcfe.a libzipstart.a
	rm -f cfe.flash cfe.flash.srec $(CLEANOBJS)

distclean : clean
