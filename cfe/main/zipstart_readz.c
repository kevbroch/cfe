/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  "Compressed" file system			File: zipstart_readz.c
    *  
    *  This module calls ZLIB to read a compressed file so we
    *  can boot compressed versions of CFE.
    *  
    *  Author:  Mitch Lichtenberg (mpl@broadcom.com)
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003
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

#if CFG_ZLIB

#include "cfe.h"
#include "zipstart.h"
#include "zlib.h"

/*  *********************************************************************
    *  ZLIBFS context
    ********************************************************************* */

/*
 * File context - describes an open file on the file system.
 * For raw devices, this is pretty meaningless, but we do
 * keep track of where we are.
 */

#define ZLIBFS_BUFSIZE	1024
typedef struct zlibfs_file_s {
    int zlibfs_fileoffset;
    z_stream zlibfs_stream;
    uint8_t *zlibfs_inbuf;
    uint8_t *zlibfs_outbuf;
    int zlibfs_outlen;
    uint8_t *zlibfs_outptr;
    int zlibfs_eofseen;

    /* Pointers to zip data we are reading */
    uint8_t *zlibfs_inptr;
    int zlibfs_inlen;
} zlibfs_file_t;

/*  *********************************************************************
    *  Globals
    ********************************************************************* */

zlibfs_file_t rz_file;
uint8_t rz_inbuf[ZLIBFS_BUFSIZE];
uint8_t rz_outbuf[ZLIBFS_BUFSIZE];

/*  *********************************************************************
    *  Prototypes
    ********************************************************************* */

voidpf zcalloc(voidpf opaque,unsigned items, unsigned size);
void zcfree(voidpf opaque,voidpf ptr);

/*  *********************************************************************
    *  ZLIB fileio dispatch table
    ********************************************************************* */


static uint8_t gz_magic[2] = {0x1f, 0x8b}; /* gzip magic header */


/* gzip flag byte */
#define ASCII_FLAG   0x01 /* bit 0 set: file probably ascii text */
#define HEAD_CRC     0x02 /* bit 1 set: header CRC present */
#define EXTRA_FIELD  0x04 /* bit 2 set: extra field present */
#define ORIG_NAME    0x08 /* bit 3 set: original file name present */
#define COMMENT      0x10 /* bit 4 set: file comment present */
#define RESERVED     0xE0 /* bits 5..7: reserved */


static void rz_memcpy(uint8_t *dst,uint8_t *src,int len)
{
    while (len > 0) {
	*dst++ = *src++;
	len--;
	}
}

static void rz_memset(uint8_t *dst,int c,int cnt)
{
    while (cnt > 0) {
	*dst++ = (uint8_t) c;
	cnt--;
	}
}

/*
 * Utility functions needed by the ZLIB routines
 */
voidpf zcalloc(voidpf opaque,unsigned items, unsigned size)
{
    void *ptr;

    ptr = zs_malloc(items*size);
    if (ptr) rz_memset(ptr,0,items*size);
    return ptr;
}

void zcfree(voidpf opaque,voidpf ptr)
{
    return zs_free(ptr);
}



static int get_byte(zlibfs_file_t *file,uint8_t *ch)
{
    if (file->zlibfs_inlen > 0) {
	*ch = *(file->zlibfs_inptr++);
	file->zlibfs_inlen--;
	return 1;
	}
    else {
	return 0;
	}

}


static int check_header(zlibfs_file_t *file)
{
    int method; /* method byte */
    int flags;  /* flags byte */
    uInt len;
    uint8_t c;
    int res;

    /* Check the gzip magic header */
    for (len = 0; len < 2; len++) {
	res = get_byte(file,&c);
	if (c != gz_magic[len]) {
	    return -1;
	}
    }

    get_byte(file,&c); method = (int) c;
    get_byte(file,&c); flags = (int) c;

    if (method != Z_DEFLATED || (flags & RESERVED) != 0) {
	return -1;
	}

    /* Discard time, xflags and OS code: */
    for (len = 0; len < 6; len++) (void)get_byte(file,&c);

    if ((flags & EXTRA_FIELD) != 0) { /* skip the extra field */
	get_byte(file,&c);
	len = (uInt) c;
	get_byte(file,&c);
	len += ((uInt)c)<<8;
	/* len is garbage if EOF but the loop below will quit anyway */
	while ((len-- != 0) && (get_byte(file,&c) == 1)) ;
    }
    if ((flags & ORIG_NAME) != 0) { /* skip the original file name */
	while ((get_byte(file,&c) == 1) && (c != 0)) ;
    }
    if ((flags & COMMENT) != 0) {   /* skip the .gz file comment */
	while ((get_byte(file,&c) == 1) && (c != 0)) ;
    }
    if ((flags & HEAD_CRC) != 0) {  /* skip the header crc */
	for (len = 0; len < 2; len++) (void)get_byte(file,&c);
    }

    return 0;
}

int rz_init(uint8_t *inptr,int len)
{
    zlibfs_file_t *file = &rz_file;
    int err;


    file->zlibfs_fileoffset = 0;
    file->zlibfs_inbuf = NULL;
    file->zlibfs_outbuf = NULL;
    file->zlibfs_eofseen = 0;

    file->zlibfs_inptr = inptr;
    file->zlibfs_inlen = len;

    /* Open the zstream */

    file->zlibfs_inbuf = rz_inbuf;
    file->zlibfs_outbuf = rz_outbuf;

    file->zlibfs_stream.next_in = NULL;
    file->zlibfs_stream.avail_in = 0;
    file->zlibfs_stream.next_out = file->zlibfs_outbuf;
    file->zlibfs_stream.avail_out = ZLIBFS_BUFSIZE;
    file->zlibfs_stream.zalloc = (alloc_func)0;
    file->zlibfs_stream.zfree = (free_func)0;

    file->zlibfs_outlen = 0;	
    file->zlibfs_outptr = file->zlibfs_outbuf;

    err = inflateInit2(&(file->zlibfs_stream),-15);
    if (err != Z_OK) {
	return -1;
	}

    check_header(file);

    return 0;
}

int rz_read(uint8_t *buf,int len)
{
    zlibfs_file_t *file = &rz_file;
    int res = 0;
    int err;
    int amtcopy;
    int ttlcopy = 0;

    if (len == 0) return 0;

    while (len) {

	/* Figure the amount to copy.  This is the min of what we
	   have left to do and what is available. */
	amtcopy = len;
	if (amtcopy > file->zlibfs_outlen) {
	    amtcopy = file->zlibfs_outlen;
	    }

	/* Copy the data. */

	if (buf) {
	    rz_memcpy(buf,file->zlibfs_outptr,amtcopy);
	    buf += amtcopy;
	    }

	/* Update the pointers. */
	file->zlibfs_outptr += amtcopy;
	file->zlibfs_outlen -= amtcopy;
	len -= amtcopy;
	ttlcopy += amtcopy;

	/* If we've eaten all of the output, reset and call inflate
	   again. */

	if (file->zlibfs_outlen == 0) {
	    /* If no input data to decompress, get some more if we can. */
	    if (file->zlibfs_eofseen) break;
	    if (file->zlibfs_stream.avail_in == 0) {
		int amtcopy;
		uint8_t *src,*dst;

		amtcopy = file->zlibfs_inlen;
		if (amtcopy > ZLIBFS_BUFSIZE) amtcopy = ZLIBFS_BUFSIZE;
		res = amtcopy;

		if (amtcopy > 0) {
		    src = file->zlibfs_inptr;
		    dst = file->zlibfs_inbuf;
		    file->zlibfs_inptr += amtcopy;
		    file->zlibfs_inlen -= amtcopy;
		    file->zlibfs_stream.next_in = file->zlibfs_inbuf;
		    file->zlibfs_stream.avail_in = amtcopy;
		    rz_memcpy(dst,src,amtcopy);
		    }
		else break;
		}

	    /* inflate the input data. */
	    file->zlibfs_stream.next_out = file->zlibfs_outbuf;
	    file->zlibfs_stream.avail_out = ZLIBFS_BUFSIZE;
	    file->zlibfs_outptr = file->zlibfs_outbuf;
	    err = inflate(&(file->zlibfs_stream),Z_SYNC_FLUSH);
	    if (err == Z_STREAM_END) {
		/* We can get a partial buffer fill here. */
	        file->zlibfs_eofseen = 1;
		}
	    else if (err != Z_OK) {
		putstr("rz_read error"); putdec(0-err);

		res = -1;
		break;
		}
	    file->zlibfs_outlen = file->zlibfs_stream.next_out -
		file->zlibfs_outptr;
	    }

	}

    file->zlibfs_fileoffset += ttlcopy;

    return (res < 0) ? res : ttlcopy;
}


void rz_close(void)
{
    zlibfs_file_t *file = &rz_file;

    inflateEnd(&(file->zlibfs_stream));
}


#endif
