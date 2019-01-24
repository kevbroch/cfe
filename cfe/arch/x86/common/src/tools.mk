#
# Addresses of things unless overridden
#

CFG_ROM_START  ?= 0xFFF80000

#
# BOOTRAM mode (runs from ROM vector assuming ROM is writable)
# implies no relocation.
#

ifeq ($(strip ${CFG_BOOTRAM}),1)
  CFG_RELOC = 0
  GENLDFLAGS += -DCFG_BOOTRAM=1
  CFLAGS += -DCFG_BOOTRAM=1
endif

ifeq ($(strip ${CFG_RELOC}),1)
  GENLDFLAGS += -DCFG_RELOC=1
  CFLAGS += -DCFG_RELOC=1
  CFG_DATA_START ?= xxxxx
  # Allow override from Makefile 
  CFG_TEXT_START ?= 0x00110000
else
  CFLAGS += -DCFG_RELOC=0
  CFG_DATA_START ?= 0x00080000
  CFG_TEXT_START = 0xFFF80000
endif



#
# Basic compiler options and preprocessor flags
#

CFLAGS += -g -c -ffreestanding -D__SIZE_TYPE__="unsigned int" -D__SSIZE_TYPE__="int" -D__long32
CFLAGS += -O2 -Wall -Werror -Wstrict-prototypes -Wmissing-prototypes 

#
# Tools locations
#

GCC     = $(TOOLS)gcc
GCPP    = $(TOOLS)cpp
GLD     = $(TOOLS)ld
GAR     = $(TOOLS)ar
OBJDUMP = $(TOOLS)objdump
OBJCOPY = $(TOOLS)objcopy
RANLIB  = $(TOOLS)ranlib

LIBGCC = $(shell $(GCC) -print-libgcc-file-name)


#
# Figure out which linker script to use
#

CFE_LDSCRIPT = ./cfe.lds
CFE_LDFLAGS += -g --script $(CFE_LDSCRIPT)
CFE_LDSCRIPT_TEMPLATE = ${ARCH_SRC}/cfe_ldscript.template


#
# Add the text/data/ROM addresses to the GENLDFLAGS so they
# will make it into the linker script.
#

GENLDFLAGS += -DCFE_ROM_START=${CFG_ROM_START} 
GENLDFLAGS += -DCFE_TEXT_START=${CFG_TEXT_START} 
GENLDFLAGS += -DCFE_DATA_START=${CFG_DATA_START} 

