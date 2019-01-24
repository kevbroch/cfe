/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  nvram environment manipulation		File: bcmnvram.h
    *  
    *  Definitions and prototypes for nvram variable subroutines.
    *
    *  The functionality is roughly parallel to env_subr.h but the
    *  inteface is designed to match the HND "nvram" functions.  This
    *  file replaces their nvram.h and defines a subset of the same
    *  interface.
    *  
    *********************************************************************  
    *
    *  Copyright 2003,2004
    *  Broadcom Corporation. All rights reserved.
    *  
    *  This software is furnished under license and may be used and 
    *  copied only in accordance with the following terms and 
    *  conditions.  Subject to these conditions, you may download, 
    *  copy, install, use, modify and distribute modified or unmodified 
    *  copies of this software in source and/or binary form.  No title 
    *  or ownership is transferred hereby.
    *  
    *  1) Any source code used, modified or distributed must reproduce 
    *     and retain this copyright notice and list of conditions 
    *     as they appear in the source file.
    *  
    *  2) No right is granted to use any trade name, trademark, or 
    *     logo of Broadcom Corporation.  The "Broadcom Corporation" 
    *     name may not be used to endorse or promote products derived 
    *     from this software without the prior written permission of 
    *     Broadcom Corporation.
    *  
    *  3) THIS SOFTWARE IS PROVIDED "AS-IS" AND ANY EXPRESS OR
    *     IMPLIED WARRANTIES, INCLUDING BUT NOT LIMITED TO, ANY IMPLIED
    *     WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
    *     PURPOSE, OR NON-INFRINGEMENT ARE DISCLAIMED. IN NO EVENT 
    *     SHALL BROADCOM BE LIABLE FOR ANY DAMAGES WHATSOEVER, AND IN 
    *     PARTICULAR, BROADCOM SHALL NOT BE LIABLE FOR DIRECT, INDIRECT,
    *     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
    *     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
    *     GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
    *     BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
    *     OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
    *     TORT (INCLUDING NEGLIGENCE OR OTHERWISE), EVEN IF ADVISED OF 
    *     THE POSSIBILITY OF SUCH DAMAGE.
    ********************************************************************* */


#ifndef _BCMNVRAM_H
#define _BCMNVRAM_H

/*  *********************************************************************
    *  Conventions
    ********************************************************************* */

/*
 * The representation of nvram variable bindings is effectively a
 * public interface, since other versions of the firmware and various
 * operating systems interpret the same data structures.
 *
 * There is a binary header of type nvram_header_t (see below).
 * The flash/nvram region following the header stores a sequence
 * of <name,value> tuples.  The binding of each environment variable
 * is stored as a null-terminated ASCII string with the format:
 *                   <name>=<value>
 * The strings are contiguous in memory; the end of the sequence is marked
 * by an empty string (two consecutive null characeters).
 *
 * The header is word oriented, in match-bits order.  The characters
 * of the tuple strings are stored in little-endian byte order when
 * in non-volatile memory and must be converted to/from host byte
 * order internally.
 */

/*  *********************************************************************
    *  Data structures.
    ********************************************************************* */

typedef struct nvram_header_s {
    uint32_t  magic;
    uint32_t  len;
    uint32_t  crc_ver_init;
    uint32_t  config_refresh;
    uint32_t  config_ncdl;
} nvram_header_t;

typedef struct {
    const char *envname;
    const char *nvname;
    const char *def;
} nvram_import_t;


#define NVRAM_CRC_SHIFT        0
#define NVRAM_CRC_MASK         (0xFF << NVRAM_CRC_SHIFT)
#define NVRAM_VER_SHIFT        8
#define NVRAM_VER_MASK         (0xFF << NVRAM_VER_SHIFT)
#define NVRAM_INIT_SHIFT       16
#define NVRAM_INIT_MASK        (0xFFF << NVRAM_INIT_SHIFT)
#define NVRAM_MEMTEST_MASK     (0x1 << 28)

#define NVRAM_CONFIG_SHIFT     0
#define NVRAM_CONFIG_MASK      (0xFFFF << NVRAM_CONFIG_SHIFT)
#define NVRAM_REFRESH_SHIFT    16
#define NVRAM_REFRESH_MASK     (0xFFFF << NVRAM_REFRESH_MASK)

/*  *********************************************************************
    *  Constants
    ********************************************************************* */

#define NVRAM_MAGIC            0x48534C46     /* 'FLSH' (little-endian) */
#define NVRAM_VERSION          1

#define NVRAM_SPACE            0x8000         /* Header plus tuples */

/*  *********************************************************************
    *  Globals
    ********************************************************************* */

extern nvram_import_t *nvram_import_data;

/*  *********************************************************************
    *  Prototypes
    ********************************************************************* */

/* Initialize from non-volatile storage. */
int         nvram_init (uint8_t *base, size_t size);

/* Enumerate all tuples in arbitrary order.  Calls of set and unset
   have unpredictable effect. */
int         nvram_enum (int (*proc)(const char *tuple));

/* Lookup, add, delete variables. */
const char *nvram_get(const char *name);
int         nvram_set(const char *name, const char *value);
int         nvram_unset(const char *name);

/* Commit to non-volatile storage. */
int         nvram_commit(int device);

#endif /* _BCMNVRAM_H */
