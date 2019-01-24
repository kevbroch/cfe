
# 
# Here are our CFE-specific object files
#

CFLAGS += -DCFG_MSYS=1
ALLOBJS += dev_msys.o
SRCDIRS += ${TOP}/msys
CFE_INC += ${TOP}/msys

#
# BDK directory files.  These are relative to MSYS_TOP.
#

SRCDIRS += ${MSYS_TOP}/bdk
CFE_INC += ${MSYS_TOP}/bdk

ifeq ($(strip ${CFG_LITTLE}),0)
CFLAGS += -DFL_BIG_ENDIAN
endif

#
# BDK source files
# 

ALLOBJS += diskonc.o docbdk.o docsys.o flbase.o mdocplus.o mtdsa.o \
           protectp.o reedsol.o



