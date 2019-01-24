/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  UBOOT Program Loader			File: cfe_ldr_uboot.c
    *  
    *  This program reads raw binaries into memory.
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2005
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

#include "cfe_fileops.h"

#include "cfe_boot.h"
#include "cfe_bootblock.h"

#include "cfe_loader.h"

#include "uboot.h"

#if CFG_ZLIB
#include "zlib.h"
#endif

/*  *********************************************************************
    *  Externs
    ********************************************************************* */

extern const cfe_loader_t rawloader;

static int cfe_ubootload(cfe_loadargs_t *la);

const cfe_loader_t ubootloader = {
    "uboot",
    cfe_ubootload,
    0};


#if CFG_ZLIB

/* gzip flag byte */
#define ASCII_FLAG   0x01 /* bit 0 set: file probably ascii text */
#define HEAD_CRC     0x02 /* bit 1 set: header CRC present */
#define EXTRA_FIELD  0x04 /* bit 2 set: extra field present */
#define ORIG_NAME    0x08 /* bit 3 set: original file name present */
#define COMMENT      0x10 /* bit 4 set: file comment present */
#define RESERVED     0xE0 /* bits 5..7: reserved */

static uint8_t gz_magic[2] = {0x1f, 0x8b}; /* gzip magic header */

/*  *********************************************************************
    *  gunzip(dst,dstlen,src.srclen)
    *  
    *  Decompress gzip'ed image in memory.
    *
    *  Input parameters: 
    *  	   dst - destination address for the uncompressed image
    *  	   dstlen - size of destination buffer (OUT: actual size)
    *  	   src - address of compressed image
    *  	   srclen - size of compressed image
    *  	   
    *  Return value:
    *  	   0 if ok, else error code
    ********************************************************************* */

static int gunzip(void *dst,int *dstlen,unsigned char *src,int srclen)
{
    z_stream s;
    int r, i, flags;

    /* Check the gzip magic header */
    for (i = 0; i < 2; i++) {
	if (src[i] != gz_magic[i]) {
            s.msg = "bad magic in in header";
	    return Z_DATA_ERROR;
	}
    }
    /* Skip header */
    i = 10;
    flags = src[3];
    if (src[2] != Z_DEFLATED || (flags & RESERVED) != 0) {
        s.msg = "unknown compression method";
        return Z_DATA_ERROR;
    }
    if ((flags & EXTRA_FIELD) != 0) {
        i = 12 + src[10] + (src[11] << 8);
    }
    if ((flags & ORIG_NAME) != 0) {
        while (src[i++] != 0)
            ;
    }
    if ((flags & COMMENT) != 0) {
        while (src[i++] != 0)
            ;
    }
    if ((flags & HEAD_CRC) != 0) {
        i += 2;
    }
    if (i >= srclen) {
        s.msg = "out of data in header";
        return Z_DATA_ERROR;
    }

    s.zalloc = Z_NULL;
    s.zfree = Z_NULL;

    r = inflateInit2(&s,-MAX_WBITS);
    if (r != Z_OK) {
        return r;
    }
    s.next_in = src + i;
    s.avail_in = srclen - i;
    s.next_out = dst;
    s.avail_out = *dstlen;
    r = inflate(&s,Z_FINISH);
    if (r != Z_OK && r != Z_STREAM_END) {
        return r;
    }
    *dstlen = s.next_out - (unsigned char *) dst;
    inflateEnd(&s);

    return 0;
}

#endif

/*  *********************************************************************
    *  cfe_ubootload(la)
    *  
    *  Read a U-Boot image file
    *  
    *  Input parameters: 
    *      la - loader args
    *  	   
    *  Return value:
    *  	   0 if ok, else error code
    ********************************************************************* */
static int cfe_ubootload(cfe_loadargs_t *la)
{
    int imglen;
    image_header_t *hdr;
    unsigned long data;
    unsigned long load_addr;
    int srclen;

    /* Set up reasonable defaults for load address and max size */
    la->la_flags |= LOADFLG_SPECADDR;
    if (la->la_address == 0) {
        la->la_address = ((unsigned long)la & ~0x0FFFFFFF) + 0x800000;
    }
    if (la->la_maxsize == 0) {
        la->la_maxsize = 0x400000;
    }

    /* Load raw image file */
    if ((imglen = rawloader.loader(la)) < 0) {
        return imglen;
    }

    /*
     * Unpack image file.
     */

    hdr = (image_header_t *)HSADDR2PTR(la->la_address);		/* XXX not 64-bit compatible */
    if (hdr->ih_magic != IH_MAGIC) {
        if (la->la_flags & LOADFLG_NOISY) {
            xprintf("Invalid U-Boot image\n");
        }
        return -1;
    }
    if (la->la_flags & LOADFLG_NOISY) {
        xprintf("Unpacking U-Boot image at 0x%08lx .... ",hdr->ih_load);
    }
    
    data = la->la_address + sizeof(image_header_t);
    srclen  = hdr->ih_size;

    if (!(strcmp(CPUCFG_ARCHNAME, "PPC") == 0 && hdr->ih_arch == IH_CPU_PPC) &&
        !(strcmp(CPUCFG_ARCHNAME, "MIPS") == 0 && hdr->ih_arch == IH_CPU_MIPS)) {
        if (la->la_flags & LOADFLG_NOISY) {
            xprintf("Unsupported Architecture\n");
        }
        return -1;
    }
    if (hdr->ih_type != IH_TYPE_KERNEL) {
        if (la->la_flags & LOADFLG_NOISY) {
            xprintf("Unknown Image Type\n");
        }
        return -1;
    }
    switch (hdr->ih_comp) {
    case IH_COMP_NONE:
        if (hdr->ih_load != la->la_address) {
            load_addr = (unsigned long)hdr->ih_load;
            memcpy((void *)load_addr,(void *)data,srclen);
        }
        imglen = srclen;
        break;
#if CFG_ZLIB
    case IH_COMP_GZIP:
        imglen = 0x400000;
        load_addr = (unsigned long)hdr->ih_load;
        if (gunzip((void *)load_addr,&imglen,(void *)data,srclen) != 0) {
            if (la->la_flags & LOADFLG_NOISY) {
                xprintf ("GUNZIP Failed\n");
            }
            return -1;
        }
        break;
#endif
    default:
        if (la->la_flags & LOADFLG_NOISY) {
            xprintf("Unimplemented Compression Type %d\n",hdr->ih_comp);
        }
        return -1;
    }
    xprintf ("%d bytes\n", imglen);

    /* Adjust entry point */
    la->la_entrypt = hdr->ih_ep;

    return imglen;
}
