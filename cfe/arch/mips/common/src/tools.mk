#
# Addresses of things unless overridden
#

ifeq ($(strip ${CFG_UNCACHED}),1)
  CFG_TEXT_START ?= 0xBFC00000
  CFG_DATA_START ?= 0xA1F00000
  CFG_ROM_START  ?= 0xBFC00000
else
  CFG_TEXT_START ?= 0x9FC00000
  CFG_DATA_START ?= 0x81F00000
  CFG_ROM_START  ?= 0xBFC00000
endif

#
# BOOTRAM mode (runs from ROM vector assuming ROM is writable)
# implies no relocation.
#

ifeq ($(strip ${CFG_BOOTRAM}),1)
  CFG_RELOC = 0
endif


#
# Basic compiler options and preprocessor flags.  By placing
# these in "CFLAGS" they will be applied to both CFE and ZipStart
#
# There are three macros with flags:
#
#  CFLAGS -- flags common to everything we build
#  ZIPSTART_CFLAGS - flags unique to ZIPSTART
#  CFE_CFLAGS - flags unique to CFE
#

CFLAGS += -g -c  -ffreestanding 
CFLAGS += -O1 -Wall -Werror -Wstrict-prototypes -Wmissing-prototypes 

ZIPSTART_CFLAGS += -D_ZIPSTART_

#
# Tools locations
#
TOOLPREFIX ?= sb1-elf-
GCC        ?= $(TOOLS)$(TOOLPREFIX)gcc
GCPP       ?= $(TOOLS)$(TOOLPREFIX)cpp
GLD        ?= $(TOOLS)$(TOOLPREFIX)ld
GAR        ?= $(TOOLS)$(TOOLPREFIX)ar
OBJDUMP    ?= $(TOOLS)$(TOOLPREFIX)objdump
OBJCOPY    ?= $(TOOLS)$(TOOLPREFIX)objcopy
RANLIB     ?= $(TOOLS)$(TOOLPREFIX)ranlib

#
# Check for 64-bit mode.  ZipStart is always built as 32-bit, so 
# this one only goes in CFE_CFLAGS
#

ifeq ($(strip ${CFG_MLONG64}),1)
  CFE_CFLAGS += -mlong64 -D__long64
endif

#
# Determine parameters for the linker script, which is generated
# using the C preprocessor.
#
# Supported combinations:
#
#  CFG_RAMAPP   CFG_UNCACHED   CFG_RELOC   Description
#    Yes        YesOrNo        MustBeNo    CFE as a separate "application"
#    No         YesOrNo        Yes         CFE relocates to RAM as firmware
#    No         YesOrNo        No          CFE runs from flash as firmware
#

CFE_LDSCRIPT = ./cfe.lds
ZIPSTART_LDSCRIPT = ./zipstart.lds
ZIPSTART_LDSCRIPT_TEMPLATE = ${ARCH_SRC}/zipstart_ldscript.template

ifeq ($(strip ${CFG_RELOC}),1)
  CFE_LDFLAGS += -Bshareable -Bsymbolic 
  CFE_LDFLAGS += --no-undefined
  CFE_LDFLAGS += --script $(CFE_LDSCRIPT)
  CFE_LDSCRIPT_TEMPLATE = ${ARCH_SRC}/cfe_ldscript.svr4pic
  OCBINFLAGS += -R .dynstr -R .hash -R .compact_rel
else
  CFE_LDFLAGS += -g --script $(CFE_LDSCRIPT)
  CFE_LDSCRIPT_TEMPLATE = ${ARCH_SRC}/cfe_ldscript.template
endif

ifeq ($(strip ${CFG_UNCACHED}),1)
  GENLDFLAGS += -DCFG_RUNFROMKSEG0=0
else
  GENLDFLAGS += -DCFG_RUNFROMKSEG0=1
endif

ifeq ($(strip ${CFG_ZIPSTART}),1)
   GENLDFLAGS += -DCFG_ZIPSTART=1
endif

ifeq ($(strip ${CFG_RAMAPP}),1)
   GENLDFLAGS += -DCFG_RAMAPP=1
else 
  #
  # RELOC=0 is used for no relocation (run in place)
  #
  ifeq ($(strip ${CFG_RELOC}),0)
    ifeq ($(strip ${CFG_BOOTRAM}),1)
      GENLDFLAGS += -DCFG_BOOTRAM=1
    else
      GENLDFLAGS += -DCFG_BOOTRAM=0
    endif
  endif
  #
  # RELOC=1 is "SVR4 PIC"
  #
  ifeq ($(strip ${CFG_RELOC}),1)
    CFLAGS += -G0 -fpic -mabicalls -Wa,-KPIC 
    GENLDFLAGS += -DCFG_RELOC=1
    CFG_TEXT_START = 0x9FC00000
    CFG_DATA_START = 0x81F00000
    CFG_ROM_START  = 0xBFC00000
  endif
  #
  # RELOC=STATIC is "move to static location"
  #
endif

#
# Add GENLDFLAGS to CFLAGS (we need this stuff in the C code as well)
#

CFLAGS += ${GENLDFLAGS}

#
# Determine target endianness.  We use this switch everywhere,
# in the C compiler, linker, and assembler, and for both CFE
# and ZipStart.
#

ifeq ($(strip ${CFG_LITTLE}),1)
  ENDIAN = -EL
else
  ENDIAN = -EB
endif

#
# Add the text/data/ROM addresses to the GENLDFLAGS so they
# will make it into the linker script.
#

GENLDFLAGS += -DCFE_ROM_START=${CFG_ROM_START} 
GENLDFLAGS += -DCFE_TEXT_START=${CFG_TEXT_START} 
GENLDFLAGS += -DCFE_DATA_START=${CFG_DATA_START} 

