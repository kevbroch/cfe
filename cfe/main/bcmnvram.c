/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  nvram management subroutines		File: bcmnvram.c
    *  
    *  This module contains routines to muck with environment variables
    *  as defined by HND.  The functionality is approximately parallel
    *  to env_subr.c but uses a format that is bit-compatible with
    *  HND conventions.
    *  
    *********************************************************************  
    *
    *  Copyright 2004
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

#include "cfe.h"
#include "bcmnvram.h"

#if ((ENDIAN_BIG + ENDIAN_LITTLE) != 1)
#error "bcmnvram.c: system endian not set"
#endif

/*  *********************************************************************
    *  Types
    ********************************************************************* */

/* Hash chains are linked lists of tuples, where each tuple has the
   ASCII representation of the binding appended.  Note that the
   basic declaration provides space for the null-termination only. */

typedef struct nvram_tuple_s {
    struct nvram_tuple_s *next;
    char binding[1];
} nvram_tuple_t;


/*  *********************************************************************
    *  Globals
    ********************************************************************* */

#define HASH_SIZE   257

static nvram_header_t nv_header;
static nvram_tuple_t *hashtab[HASH_SIZE];
static nvram_tuple_t *nv_pending;


/*  *********************************************************************
    *  Helper functions
    ********************************************************************* */

#if ENDIAN_BIG
static uint32_t
htol4(uint32_t x)
{
    uint32_t t;

    t = ((x & 0xFF00FF00) >> 8) | ((x & 0x00FF00FF) << 8);
    return (t >> 16) | ((t & 0xFFFF) << 16);
}

#define ltoh4(x) htol4(x)
#else
#define htol4(x) (x)
#define ltoh4(x) (x)
#endif


/*  *********************************************************************
    * crc8(pdata, len, crc)
    *
    * Computes a crc8 over the input data using the polynomial:
    *       x^8 + x^7 +x^6 + x^4 + x^2 + 1
    ********************************************************************* */

static const uint8_t crc8_table[256] = {
    0x00, 0xF7, 0xB9, 0x4E, 0x25, 0xD2, 0x9C, 0x6B,
    0x4A, 0xBD, 0xF3, 0x04, 0x6F, 0x98, 0xD6, 0x21,
    0x94, 0x63, 0x2D, 0xDA, 0xB1, 0x46, 0x08, 0xFF,
    0xDE, 0x29, 0x67, 0x90, 0xFB, 0x0C, 0x42, 0xB5,
    0x7F, 0x88, 0xC6, 0x31, 0x5A, 0xAD, 0xE3, 0x14,
    0x35, 0xC2, 0x8C, 0x7B, 0x10, 0xE7, 0xA9, 0x5E,
    0xEB, 0x1C, 0x52, 0xA5, 0xCE, 0x39, 0x77, 0x80,
    0xA1, 0x56, 0x18, 0xEF, 0x84, 0x73, 0x3D, 0xCA,
    0xFE, 0x09, 0x47, 0xB0, 0xDB, 0x2C, 0x62, 0x95,
    0xB4, 0x43, 0x0D, 0xFA, 0x91, 0x66, 0x28, 0xDF,
    0x6A, 0x9D, 0xD3, 0x24, 0x4F, 0xB8, 0xF6, 0x01,
    0x20, 0xD7, 0x99, 0x6E, 0x05, 0xF2, 0xBC, 0x4B,
    0x81, 0x76, 0x38, 0xCF, 0xA4, 0x53, 0x1D, 0xEA,
    0xCB, 0x3C, 0x72, 0x85, 0xEE, 0x19, 0x57, 0xA0,
    0x15, 0xE2, 0xAC, 0x5B, 0x30, 0xC7, 0x89, 0x7E,
    0x5F, 0xA8, 0xE6, 0x11, 0x7A, 0x8D, 0xC3, 0x34,
    0xAB, 0x5C, 0x12, 0xE5, 0x8E, 0x79, 0x37, 0xC0,
    0xE1, 0x16, 0x58, 0xAF, 0xC4, 0x33, 0x7D, 0x8A,
    0x3F, 0xC8, 0x86, 0x71, 0x1A, 0xED, 0xA3, 0x54,
    0x75, 0x82, 0xCC, 0x3B, 0x50, 0xA7, 0xE9, 0x1E,
    0xD4, 0x23, 0x6D, 0x9A, 0xF1, 0x06, 0x48, 0xBF,
    0x9E, 0x69, 0x27, 0xD0, 0xBB, 0x4C, 0x02, 0xF5,
    0x40, 0xB7, 0xF9, 0x0E, 0x65, 0x92, 0xDC, 0x2B,
    0x0A, 0xFD, 0xB3, 0x44, 0x2F, 0xD8, 0x96, 0x61,
    0x55, 0xA2, 0xEC, 0x1B, 0x70, 0x87, 0xC9, 0x3E,
    0x1F, 0xE8, 0xA6, 0x51, 0x3A, 0xCD, 0x83, 0x74,
    0xC1, 0x36, 0x78, 0x8F, 0xE4, 0x13, 0x5D, 0xAA,
    0x8B, 0x7C, 0x32, 0xC5, 0xAE, 0x59, 0x17, 0xE0,
    0x2A, 0xDD, 0x93, 0x64, 0x0F, 0xF8, 0xB6, 0x41,
    0x60, 0x97, 0xD9, 0x2E, 0x45, 0xB2, 0xFC, 0x0B,
    0xBE, 0x49, 0x07, 0xF0, 0x9B, 0x6C, 0x22, 0xD5,
    0xF4, 0x03, 0x4D, 0xBA, 0xD1, 0x26, 0x68, 0x9F
};

#define CRC8_INIT_VALUE    0xFF
#define CRC8_GOOD_VALUE    0x9F   /* not used here */

static uint8_t
nvram_crc (nvram_header_t *hdr)
{
    uint8_t crc;
    uint8_t *base = (uint8_t *)hdr;
    unsigned int offset;
    const unsigned bytesel = (ENDIAN_BIG ? 0x3 : 0x0);

    /* CRC covers last 11 bytes of header plus tuples, all LE order.
       Note that hdr is at least word aligned. */
    crc = CRC8_INIT_VALUE;
    for (offset = 9; offset < hdr->len; offset++)
	crc = crc8_table[crc ^ base[offset ^ bytesel]];

    return crc;
}


/* Local hash function (simplistic for now) */
static unsigned int
nvram_hash(const char *name)
{
    const char *p;
    unsigned int sum;

    p = name;
    sum = 0;
    while (*p && *p != '=') sum = 7*sum + *p++;
    return sum % HASH_SIZE;
}

static nvram_tuple_t *
nvram_lookup(const char *name, size_t len)
{
    nvram_tuple_t *p;
    unsigned int hash = nvram_hash(name);

    for (p = hashtab[hash]; p != NULL; p = p->next) {
	if (memcmp(name, p->binding, len) == 0 && p->binding[len] == '=')
	    return p;
    }
    return NULL;
}
    
static void
nvram_insert(unsigned int hash, nvram_tuple_t *tuple)
{
    tuple->next = hashtab[hash];
    hashtab[hash] = tuple;
}

static int
nvram_delete(unsigned int hash, nvram_tuple_t *tuple)
{
    nvram_tuple_t *p, *prev;

    prev = NULL;
    for (p = hashtab[hash]; p != NULL; p = p->next) {
	if (p == tuple) {
	    if (prev == NULL)
		hashtab[hash] = p->next;
	    else
		prev->next = p->next;
	    return 1;
	    }
	prev = p;
	}
      return 0;          /* not found */
}


static nvram_header_t *
nvram_find(uint8_t *base, uint32_t size)
{
    nvram_header_t *hdr;
    uint32_t crc_ver_init;
    uint8_t crc;

    hdr = (nvram_header_t *)(base + size - NVRAM_SPACE);

    if (hdr->magic != NVRAM_MAGIC)
	hdr = NULL;
    else {
	crc_ver_init = hdr->crc_ver_init;

	crc = nvram_crc(hdr);
	/* XXX HND code doesn't seem to do a real check here. */
        if (crc != ((crc_ver_init & NVRAM_CRC_MASK) >> NVRAM_CRC_SHIFT)) {
            xprintf("nvram CRC: computed %02x, stored %02x\n",
                    crc, (crc_ver_init & NVRAM_CRC_MASK) >> NVRAM_CRC_SHIFT);
        }
    }

    return hdr;
}

static int
nvram_internalize(nvram_header_t *hdr)
{
    void *buffer;
    uint32_t *src, *dst;
    int i;
    unsigned int hash;
    nvram_tuple_t *tuple;
    const char *base;
    unsigned int offset;
    size_t len;

    buffer = KMALLOC(hdr->len, 4);

    src = (uint32_t *)hdr;
    dst = (uint32_t *)buffer;

    /* The header is read/written as words (match bits) */
    for (i = 0; i < sizeof(nvram_header_t); i += 4)
	*dst++ = *src++;
    /* The tuples are stored LE; convert to host byte order */
    for (i = sizeof(nvram_header_t); i < hdr->len; i += 4)
	*dst++ = ltoh4(*src++);

    base = (const char *)buffer;
    offset = sizeof(nvram_header_t);

    while (offset < hdr->len && *(base+offset) != 0) {
        len = strlen(base+offset);
	tuple = (nvram_tuple_t *)KMALLOC(sizeof(nvram_tuple_t) + len, 4);
	if (tuple == NULL)     /* Out of memory */
	    return -1;   
	strcpy(tuple->binding, base+offset);
	hash = nvram_hash(base + offset);
	/* Check for duplicates here? */
	nvram_insert(hash, tuple);
	offset += len + 1;
	}

    KFREE(buffer);
    return 0;
}


/*  *********************************************************************
    *  Functions
    ********************************************************************* */

/*  *********************************************************************
    *  nvram_init(base, size)
    *  
    *  Initialize the a-list from flash.
    *  
    *  Input parameters: 
    *  	   base   -  physical address of nv memory (e.g., flash)
    *      size   -  size of nv memory
    *  	   
    *  Return value:
    *  	   error code (0 for success)
    ********************************************************************* */

int
nvram_init (uint8_t *nvram_base, size_t nvram_size)
{
    int i;
    nvram_header_t *header;

    for (i = 0; i < HASH_SIZE; i++)
	hashtab[i] = NULL;
    nv_pending = NULL;

    header = nvram_find(nvram_base, nvram_size);

    if (header != NULL) {
	nv_header = *header;                /* Save fixed fields */
	nvram_internalize(header);
	}
    else {
	memset(&nv_header, 0, sizeof(nvram_header_t));
	nv_header.magic = NVRAM_MAGIC;
	nv_header.len = sizeof(nvram_header_t);
	nv_header.crc_ver_init = (NVRAM_VERSION << NVRAM_VER_SHIFT);
#ifdef SDRAM_INIT
	nv_header.crc_ver_init |= (SDRAM_INIT << NVRAM_INIT_SHIFT);
#endif
	}
    return 0;
}


int
nvram_enum(int (*proc)(const char *tuple))
{
    int i;
    nvram_tuple_t *p;

    for (i = 0; i < HASH_SIZE; i++) {
	for (p = hashtab[i]; p != NULL; p = p->next) {
	    if ((*proc)(p->binding) == 0)
		return 0;
	    }
	}
    return 1;
}


const char *
nvram_get(const char *name)
{
    unsigned int hash;
    size_t len = strlen(name);
    nvram_tuple_t *p;

    hash = nvram_hash(name);
    p = nvram_lookup(name, len);
    if (p != NULL)                       /* "name=value" */
	return &p->binding[len + 1];     /* "value" */
    else
	return NULL;
}

int
nvram_set(const char *name, const char *value)
{
    unsigned int hash;
    size_t len = strlen(name);
    nvram_tuple_t *tuple, *p;

    tuple = (nvram_tuple_t *)KMALLOC(
	      sizeof(nvram_tuple_t) + len + 1 + strlen(value), 4);
    if (tuple == NULL)     /* Out of memory */
        return -1;   
    strcpy(tuple->binding, name);
    strcat(tuple->binding, "=");
    strcat(tuple->binding, value);

    hash = nvram_hash(name);
    p = nvram_lookup(name, len);
    if (p != NULL) {
	nvram_delete(hash, p);
	p->next = nv_pending;
	nv_pending = p;
	}

    nvram_insert(hash, tuple);
    return 0;
}

int
nvram_unset(const char *name)
{
    unsigned int hash;
    size_t len = strlen(name);
    nvram_tuple_t *p;

    hash = nvram_hash(name);
    p = nvram_lookup(name, len);
    if (p == NULL)
      return -1;            /* Not currently bound */

    nvram_delete(hash, p);
    p->next = nv_pending;
    nv_pending = p;

    return 0;
}

int
nvram_commit(int nvram_handle)
{
    int i;
    nvram_tuple_t *p;
    size_t total;
    nvram_header_t *hdr;
    uint8_t *cp;
    uint32_t *wp;
    int res;

    /* XXX Delete nv_pending here? */

    total = sizeof(nvram_header_t);
    for (i = 0; i < HASH_SIZE; i++)
	for (p = hashtab[i]; p != NULL; p = p->next)
	    total += strlen(p->binding) + 1;
    total += 2;                      /* final null strings (2 for compat) */

    total = (total + 3) & ~3;        /* word align */
    if (total > NVRAM_SPACE)
	return -1;

    hdr = (nvram_header_t *)KMALLOC(total, 4);
    if (hdr == NULL)
	return -1;

    *hdr = nv_header;
    hdr->len = total;
    hdr->crc_ver_init &= ~NVRAM_CRC_MASK;

    cp = (uint8_t *)hdr + sizeof(nvram_header_t);
    wp = (uint32_t *)cp;

    for (i = 0; i < HASH_SIZE; i++) {
	for (p = hashtab[i]; p != NULL; p = p->next) {
	    size_t n = strlen(p->binding) + 1;
	    memcpy(cp, p->binding, n);
	    cp += n;
	}
    }
    while (cp < (uint8_t *)hdr + hdr->len)
	*cp++ = 0;                       /* terminate and pad */

    if (ENDIAN_BIG) {
	/* Must put the byte-oriented strings into LE order. */
	for (i = sizeof(nvram_header_t); i < hdr->len; i += 4) {
	    *wp = htol4(*wp);
	    wp++;
	}
    }

    hdr->crc_ver_init |= (nvram_crc(hdr) << NVRAM_CRC_SHIFT);

    xprintf("nvram_commit: will write %x bytes from %p\n", total, hdr);
    res = cfe_writeblk(nvram_handle, 0, PTR2HSADDR(hdr), hdr->len);
    xprintf(" result %x (%d)\n", res, res);

    nv_header = *hdr;
    /* KFREE(hdr) */

    return 0;
}
