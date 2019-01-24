#
# PCI bus related files and rules
#

# Predefined driver collections for PCI devices (see board Makefile's). */

ALLPCIDEVS = ide_pci ide_frodo tulip dp83815 bcm4401 bcm5700 i82559 ns16550_pci
CPCIDEVS = tulip
MINIPCIDEVS = 

PCIDEVS ?= ${ALLPCIDEVS}


# Rules for converting the master table of vendor/device names

pcidevs_data.h pcidevs.h : ${TOP}/pci/pcidevs
	$(AWK) -f ${TOP}/pci/devlist2h.awk < ${TOP}/pci/pcidevs

mkpcidb : ${TOP}/hosttools/mkpcidb.c pcidevs.h
	$(HOST_CC) $(HOST_CFLAGS) -I. -o mkpcidb ${TOP}/hosttools/mkpcidb.c

pcidevs_data2.h : mkpcidb ./pcidevs_data.h
	./mkpcidb > pcidevs_data2.h

pciconf.o : ${TOP}/pci/pciconf.c pcidevs.h

pci_subr.o : ${TOP}/pci/pci_subr.c pcidevs_data2.h


# Rules for building PCI management support and device drivers.

SRCDIRS += ${TOP}/pci
PCIOBJS  = pciconf.o ldtinit.o pci_subr.o
PCIOBJS += pci_devs.o cpci_devs.o
DRVOBJS += $(patsubst %,dev_%.o,${PCIDEVS})
CFE_CFLAGS  += -DCFG_PCI=1
ifeq ($(strip ${CFG_LDT}),1)
  DRVOBJS += dev_sp1011.o dev_ht7520.o
  CFE_CFLAGS  += -DCFG_LDT=1
  ifeq ($(strip ${CFG_LDT_REV_017}),1)
    CFE_CFLAGS  += -DCFG_LDT_REV_017=1
  endif
else
  CFE_CFLAGS  += -DCFG_LDT=0
endif
ifeq ($(strip ${CFG_DOWNLOAD}),1)
  DRVOBJS += dev_bcm1250.o download.data
  CFE_CFLAGS += -DCFG_DOWNLOAD=1
endif
ALLOBJS += ${PCIOBJS} ${DRVOBJS}
