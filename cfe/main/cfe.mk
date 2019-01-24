
#
# CFE's version number
#

include ${TOP}/main/cfe_version.mk

#
# Default values for certain parameters
#

CFG_MLONG64 ?= 0
CFG_LITTLE  ?= 0
CFG_RELOC ?= 0
CFG_UNCACHED ?= 0
CFG_BOOTRAM ?= 0
CFG_PCI ?= 1
CFG_LDT ?= 1
CFG_LDT_REV_017 ?= 0
CFG_PCIDEVICE ?= 0 
CFG_DOWNLOAD ?= 0
CFG_USB ?= 0
CFG_MSYS ?= 0
CFG_ZLIB ?= 0
CFG_VGACONSOLE ?= 0
CFG_BIENDIAN ?= 0
CFG_RAMAPP ?= 0
CFG_USB ?= 0
CFG_ZIPSTART ?= 0
CFG_ZIPPED_CFE ?= 1

#
# Override some settings based on the value of CFG_RELOC.
#
# 'STATIC' relocation means no biendian, no bootram, ZIPstart
# '1' is SVR4 Relocation,  no RAMAPP, no BOOTRAM
# '0' (no relocation) : no changes
#

ifeq ($(strip ${CFG_RELOC}),1)
  CFG_RAMAPP = 0
  CFG_BOOTRAM = 0
endif

ifeq ($(strip ${CFG_RELOC}),STATIC)
  CFG_RAMAPP = 0
  CFG_BIENDIAN = 0
  CFG_BOOTRAM = 0
  CFG_ZIPSTART = 1
  CFE_CFLAGS += -DCFG_ZIPSTART=1
endif

ifeq ($(strip ${CFG_PCIDEVICE}),1)
CFLAGS += -DCFG_PCIDEVICE=1
endif
#
# Default goal.
#

all : ALL

#
# Paths to other parts of the firmware.  Everything's relative to ${TOP}
# so that you can actually do a build anywhere you want.
#

ARCH_TOP   = ${TOP}/arch/${ARCH}
ARCH_SRC   = ${ARCH_TOP}/common/src
ARCH_INC   = ${ARCH_TOP}/common/include
CPU_SRC    = ${ARCH_TOP}/cpu/${CPU}/src
CPU_INC    = ${ARCH_TOP}/cpu/${CPU}/include
MAIN_SRC   = ${TOP}/main
MAIN_INC   = ${TOP}/include

#
# We have an optional "chipset" directory that goes under
# "arch".  Include that if requested.
#

ifneq ("$(strip ${CHIPSET})","")
CHIPSET_SRC  = ${ARCH_TOP}/chipset/${CHIPSET}/src
CHIPSET_INC  = ${ARCH_TOP}/chipset/${CHIPSET}/include
endif

#
# It's actually optional to have a 'board'
# directory.  If you don't specify BOARD,
# don't include the files.
#

ifneq ("$(strip ${BOARD})","")
  BOARD_SRC  = ${ARCH_TOP}/board/${BOARD}/src
  BOARD_INC  = ${ARCH_TOP}/board/${BOARD}/include
endif

#
# Preprocessor defines for CFE's version number
#

VDEF = -DCFE_VER_MAJ=${CFE_VER_MAJ} -DCFE_VER_MIN=${CFE_VER_MIN} -DCFE_VER_ECO=${CFE_VER_ECO}

ifneq ("$(strip ${CFE_VER_SDK})","")
  VDEF += -DCFE_VER_SDK=${CFE_VER_SDK}
endif

#
# Construct the list of paths that will eventually become the include
# paths and VPATH.
#

SRCDIRS = ${BOARD_SRC} $(CHIPSET_SRC) ${CPU_SRC} ${ARCH_SRC} ${TOP}/main ${TOP}/vendor ${TOP}/include ${TOP}/net ${TOP}/dev ${TOP}/ui ${TOP}/lib

CFE_INC = ${TOP}/include ${TOP}/net ${TOP}/dev


#
# Some device drivers currently require PCI headers
# to compile even when PCI is not configured.
#

CFE_INC += ${TOP}/pci

#
# Configure tools and basic tools flags.  This include sets up
# macros for calling the C compiler, basic flags,
# and linker scripts.
#

include ${ARCH_SRC}/tools.mk

#
# Add some common flags that are used on any architecture.
#

CFLAGS += -I. $(INCDIRS)
CFLAGS += -D_CFE_ ${VDEF} -DCFG_BOARDNAME=\"${CFG_BOARDNAME}\"

#
# Gross - allow more options to be supplied from command line
#

ifdef CFG_OPTIONS
OPTFLAGS = $(patsubst %,-D%,$(subst :, ,$(CFG_OPTIONS)))
CFLAGS += ${OPTFLAGS}
endif


#
# Decide whether we need to compile zlib or not. 
#
# Basically, we need ZLIB if it is required for either CFE
# or ZipStart.
#

ifeq ($(strip ${CFG_ZLIB}),1)
  SRCDIRS += ${TOP}/zlib
  CFE_INC += ${TOP}/zlib
else
  ifeq ($(strip ${CFG_ZIPSTART}),1)
    ifeq ($(strip ${CFG_ZIPPED_CFE}),1)
      CFG_ZLIB = 1
      SRCDIRS += ${TOP}/zlib
      CFE_INC += ${TOP}/zlib
    endif
  endif
endif


#
# Include the makefiles for the architecture-common, cpu-specific,
# and board-specific directories.  Each of these will supply
# some files to "ALLOBJS".  The BOARD and CHIPSET directories are optional
# as some ports are so simple they don't need boad-specific stuff.
#

include ${ARCH_SRC}/Makefile
include ${CPU_SRC}/Makefile

ifneq ("$(strip ${CHIPSET})","")
include ${CHIPSET_SRC}/Makefile
endif

ifneq ("$(strip ${BOARD})","")
  include ${BOARD_SRC}/Makefile
endif

#
# Pull in the common directories
#

include ${MAIN_SRC}/Makefile
include ${TOP}/dev/Makefile
include ${TOP}/ui/Makefile
include ${TOP}/net/Makefile
include ${TOP}/lib/Makefile

#
# Add more object files if we're supporting PCI
#

ifeq ($(strip ${CFG_PCI}),1)
  include ${TOP}/pci/Makefile
endif


#
# Add the common object files here.
#

ALLOBJS += $(LIBOBJS) $(CFEOBJS) $(UIOBJS) $(DEVOBJS) $(NETOBJS)

#
# Add optional code.  Each option will add to ALLOBJS with its Makefile,
# and some append to SRCDIRS and/or CFE_INC.
#

#
# USB support
#

ifeq ($(strip ${CFG_USB}),1)
 SRCDIRS += ${TOP}/usb
  CFE_INC += ${TOP}/usb
  include ${TOP}/usb/Makefile
endif

#
# M-Systems DoC support
#

ifeq ($(strip ${CFG_MSYS}),1)
  ifndef ${MSYS_TOP}
  # Supply a stub directory by default
    MSYS_TOP = ${TOP}/msys
  endif
  include ${MSYS_TOP}/Makefile
endif

#
# If we're including ZLIB, then add its makefile.
#

ifeq ($(strip ${CFG_ZLIB}),1)
  SRCDIRS += ${TOP}/zlib
  CFE_INC += ${TOP}/zlib
  include ${TOP}/zlib/Makefile
endif

#
# If we're doing the VGA console thing, pull in the x86 emulator
# and the pcconsole subsystem
#

ifeq ($(strip ${CFG_VGACONSOLE}),1)
  SRCDIRS += ${TOP}/x86emu ${TOP}/pccons
  CFE_INC += ${TOP}/x86emu ${TOP}/pccons
  include ${TOP}/x86emu/Makefile
  include ${TOP}/pccons/Makefile
endif


ifeq ($(strip ${CFG_VAPI}),1)
  SRCDIRS += ${TOP}/verif
  CFE_INC += ${TOP}/verif
  include ${TOP}/verif/Makefile
endif

#
# Vendor extensions come next - they live in their own directory.
#

include ${TOP}/vendor/Makefile

#
# Bi-endian support: If we're building the little-endian
# version, use a different linker script so we can locate the
# ROM at a higher address.  You'd think we could do this with
# normal linker command line switches, but there appears to be no
# command-line way to override the 'AT' qualifier in the linker script.
#
# Also add the compiler switch to change
# the way the vectors are generated.  These switches are
# only added to the big-endian portion of the ROM,
# which is located at the real boot vector.
#

CFG_TEXTAT1MB=0
ifeq ($(strip ${CFG_BIENDIAN}),1)
  ifeq ($(strip ${CFG_LITTLE}),1)
    CFG_TEXT_START = 0x9fd00000
    CFG_ROM_START  = 0xbfd00000
    CFG_TEXTAT1MB=1
  endif
  ifeq ($(strip ${CFG_LITTLE}),0)
    CFLAGS += -DCFG_BIENDIAN=1
  endif
endif


#
# Make the paths
#

INCDIRS = $(patsubst %,-I%,$(subst :, ,$(BOARD_INC) $(CHIPSET_INC) $(CPU_INC) $(ARCH_INC) $(CFE_INC)))

VPATH = $(SRCDIRS)

#
# This is the makefile's main target.  Note that we actually
# do most of the work in 'ALL' (from the build Makefile) not 'all'.
#

all : build_date.c makereg ALL

.PHONY : all 
.PHONY : ALL
.PHONY : build_date.c

#
# Build the local tools that we use to construct other source files
#

HOST_CC = gcc
HOST_CFLAGS = -g -Wall -Werror -Wstrict-prototypes -Wmissing-prototypes

# For building the PCI device table
AWK ?= gawk

memconfig : ${TOP}/hosttools/memconfig.c
	$(HOST_CC) $(HOST_CFLAGS) -o memconfig -D_MCSTANDALONE_ -D_MCSTANDALONE_NOISY_ -I${TOP}/arch/mips/chipset/sibyte/include -I${TOP}/arch/mips/cpu/sb1250/include -I${TOP}/include ${TOP}/hosttools/memconfig.c ${TOP}/arch/${ARCH}/cpu/${CPU}/src/sb1250_draminit.c

memconfig1480 : ${TOP}/hosttools/memconfig1480.c
	$(HOST_CC) $(HOST_CFLAGS) -o memconfig1480 -D_MCSTANDALONE_ -D_MCSTANDALONE_NOISY_ -I${TOP}/arch/mips/chipset/sibyte/include -I${TOP}/arch/mips/cpu/bcm1480/include -I${TOP}/include ${TOP}/hosttools/memconfig1480.c ${TOP}/arch/${ARCH}/cpu/${CPU}/src/bcm1480_draminit.c

mkflashimage : ${TOP}/hosttools/mkflashimage.c
	$(HOST_CC) $(HOST_CFLAGS) -o mkflashimage -I${TOP}/include ${TOP}/hosttools/mkflashimage.c

swapflashimage : ${TOP}/hosttools/swapflashimage.c
	$(HOST_CC) $(HOST_CFLAGS) -o swapflashimage -I${TOP}/include ${TOP}/hosttools/swapflashimage.c

build_date.c :
	echo "const char *builddate = \"`date`\";" > build_date.c
	echo "const char *builduser = \"`whoami`@`hostname`\";" >> build_date.c

#
# Make a define for the board name
#

CFLAGS += -D_$(patsubst "%",%,${CFG_BOARDNAME})_

#
# Rules for building normal CFE files
#

LIBCFE = libcfe.a

%.o : %.c
	$(GCC) $(ENDIAN) $(CFE_CFLAGS) $(CFLAGS) -o $@ $<

%.o : %.S
	$(GCC) $(ENDIAN) $(CFE_CFLAGS) $(CFLAGS) -o $@ $<

#
# Rules for building ZIPSTART
#

LIBZIPSTART = libzipstart.a

ZS_%.o : %.c
	$(GCC) $(ENDIAN) $(ZIPSTART_CFLAGS) -D_ZIPSTART_ $(CFLAGS) -o $@ $<

ZS_%.o : %.S
	$(GCC) $(ENDIAN) $(ZIPSTART_CFLAGS) -D_ZIPSTART_ $(CFLAGS) -o $@ $<


#
# This rule constructs "libcfe.a" which contains most of the object
# files.
#

$(LIBCFE) : $(ALLOBJS)
	rm -f $(LIBCFE)
	$(GAR) cr $(LIBCFE) $(ALLOBJS)
	$(RANLIB) $(LIBCFE)



