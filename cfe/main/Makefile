
#
# Macros that expand to the list of arch-independent files
#

CFEOBJS = env_subr.o cfe_attach.o cfe_iocb_dispatch.o cfe_devfuncs.o \
          nvram_subr.o cfe_console.o cfe_main.o cfe_mem.o cfe_timer.o \
          cfe_background.o cfe_error.o build_date.o \
	  cfe_rawfs.o cfe_zlibfs.o cfe_xreq.o cfe_fatfs.o cfe_httpfs.o cfe_xmodem.o cfe_filesys.o cfe_boot.o \
          cfe_autoboot.o cfe_ldr_elf.o cfe_ldr_raw.o cfe_ldr_srec.o cfe_ldr_uboot.o cfe_loader.o url.o \
	  cfe_savedata.o cfe_smbus.o cfe_spi.o cfe_mii.o ptable.o

#
# Parts of ZIPstart
#

ZSOBJS += zipstart_main.o 

ifeq ($(strip ${CFG_ZIPPED_CFE}),1)
  ZSOBJS += zipstart_readz.o
endif



