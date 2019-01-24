
BSPOBJS += bcm95836cpci_devs.o bcm95836cpci_init.o
ifeq ($(strip ${CFG_PCI}),1)
BSPOBJS += bcm95836cpci_pci.o
endif

PCIDEVS = ${CPCIDEVS}

SMBDEVS =

# From CFE
ALLOBJS += sbmemc.o dev_sb_mac.o \
           ui_bcm95836cpci.o ui_test_flash.o
ALLOBJS += bcmnvram.o ui_nvramcmds.o

# Finally define some things peculiar to this board.

CFLAGS += -DINCLUDE_FLASH_DRIVERS
