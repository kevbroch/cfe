/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  Flash Image Swapper			File: swapflashimage.c
    *  
    *  This program updates the header and swaps the data bytes
    *  in files generated for use with the 'flash' command
    *  in CFE.  It is only used when in certain cases of flashing
    *  a file intended for the opposite endian of the CFE executing
    *  the flash command, typically when the target flash device
    *  has a bus interface wider than 8 bits.
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

#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>

#ifndef _SYS_INT_TYPES_H
typedef unsigned char uint8_t;
typedef unsigned long uint32_t;
#endif

#include "cfe_flashimage.h"
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#ifndef O_BINARY
#define O_BINARY 0
#endif

static void usage(void)
{
    fprintf(stderr,"usage: swapflashimage [-(b|h|w)] flashfile outfile\n");
    fprintf(stderr,"\n");
    exit(1);
}


#define     CRC32_POLY        0xEDB88320UL    /* CRC-32 Poly */

static unsigned int
crc32(const unsigned char *databuf, unsigned int  datalen) 
{       
    unsigned int idx, bit, data, crc = 0xFFFFFFFFUL;
 
    for (idx = 0; idx < datalen; idx++) {
	for (data = *databuf++, bit = 0; bit < 8; bit++, data >>= 1) {
	    crc = (crc >> 1) ^ (((crc ^ data) & 1) ? CRC32_POLY : 0);
	    }
	}

    return crc;
}


static void stuff_be32(uint8_t *dest,unsigned int src)
{
    *dest++ = (src >> 24) & 0xFF;
    *dest++ = (src >> 16) & 0xFF;
    *dest++ = (src >>  8) & 0xFF;
    *dest++ = (src >>  0) & 0xFF;
}

static uint32_t get_be32(uint8_t *src)
{
    return (((uint32_t) (src[0] << 24)) | \
	    ((uint32_t) (src[1] << 16)) | \
	    ((uint32_t) (src[2] << 8)) |  \
	    ((uint32_t) (src[3] << 0)));
}

static int flash_validate(cfe_flashimage_t *hdr, uint8_t *code, int insize)
{
    uint32_t size;
    uint32_t flags;
    uint32_t hdrcrc;
    uint32_t calccrc;

    if (memcmp(hdr->seal,CFE_IMAGE_SEAL,sizeof(hdr->seal)) != 0) {
	printf("Invalid header seal.  This is not a CFE flash image.\n");
	return -1;
	}

    printf("Flash image contains CFE version %d.%d.%d for board '%s'\n",
	   hdr->majver,hdr->minver,hdr->ecover,hdr->boardname);

    size = get_be32(hdr->size);
    flags = get_be32(hdr->flags);
    hdrcrc = get_be32(hdr->crc);
    printf("Flash image is %d bytes, flags %08X, CRC %08X\n",
	   (unsigned int)size,(unsigned int)flags,(unsigned int)hdrcrc);

    if (size != insize) {
	printf("Flash image size is bogus!\n");
	return -1;
	}

    calccrc = crc32(code,size);

    if (calccrc != hdrcrc) {
	printf("CRC is incorrect. Calculated CRC is %08X\n",
	       (unsigned int)calccrc);
	return -1;
	}
    return 0;
}

int main(int argc, char *argv[])
{
    int swapsize;
    int fh;    
    int flashsize;
    uint8_t *flashcode;
    cfe_flashimage_t header;
    uint32_t crc;
    char *outfile;
    int outsize;
    uint8_t *ptr;
    int i, j, k;
    uint8_t t;

    swapsize = 0;
    while ((argc > 1) && (argv[1][0] == '-')) {
	if (strcmp(argv[1],"-B") == 0 || strcmp(argv[1],"-b") == 0) {
	    fprintf(stderr,"[Bytes will be not be swapped]\n");
	    swapsize = 1;
	    }
	else if (strcmp(argv[1],"-H") == 0 || strcmp(argv[1],"-h") == 0) {
	    fprintf(stderr,"[Bytes within pairs will be swapped]\n");
	    swapsize = 2;
	    }
	else if (strcmp(argv[1],"-W") == 0 || strcmp(argv[1],"-w") == 0) {
	    fprintf(stderr,"[Bytes within quads will be swapped]\n");
	    swapsize = 4;
	    }
	else {
	    fprintf(stderr,"Invalid switch: %s\n",argv[1]);
	    exit(1);
	    }
	argv++;
	argc--;
	}

    if (swapsize == 0) {
	fprintf(stderr,"[Bytes within quads will be swapped]\n");
	swapsize = 4;
	}

    /*
     * We need to swap things around if the host and 
     * target are different endianness
     */

    if (argc != 3) {
	usage();
	}

    /*
     * Read and validate the header first.
     */

    fh = open(argv[1],O_RDONLY|O_BINARY);
    if (fh < 0) {
	perror(argv[1]);
	}

    flashsize = lseek(fh,0L,SEEK_END);
    lseek(fh,0L,SEEK_SET);

    if (read(fh,&header,sizeof(header)) != sizeof(header)) {
	perror("header");
	exit(1);
	}

    /*
     * Force the data to be a multiple of the swap unit (or error?).
     */

    flashsize -= sizeof(header);
    outsize = (flashsize + (swapsize-1)) & ~(swapsize-1);
      
    flashcode = malloc(outsize);
    if (flashcode == NULL) {
	perror("malloc");
	exit(1);
	}

    /*
     * Read in the boot code
     */

    if (read(fh,flashcode,flashsize) != flashsize) {
	perror("read");
	exit(1);
	}

    close(fh);

    if (flash_validate(&header, flashcode, flashsize) < 0) {
	exit(1);
	}

    if (flashsize < outsize)
	memset(flashcode+flashsize,0xFF,outsize-flashsize);
    flashsize = outsize;

    outfile = argv[2];

    /*
     * Perform the requested swap.
     */

    ptr = flashcode;
    for (i = 0; i < flashsize; i += swapsize) {
	for (j = 0,k = (swapsize-1); j < swapsize/2; j++,k--) {
	    t = *(ptr+j);
	    *(ptr+j) = *(ptr + k);
	    *(ptr+k) = t;
	    }
	ptr += swapsize;
	}

    /*
     * Update the size and CRC in the flash header
     */

    crc = crc32(flashcode,flashsize);

    stuff_be32(header.size,flashsize);
    stuff_be32(header.crc,crc);

    /*
     * Now write the output file
     */

    fh = open(outfile,O_RDWR|O_CREAT|O_TRUNC,S_IREAD|S_IWRITE|S_IRGRP|S_IWGRP|S_IROTH|O_BINARY);
    if  (fh < 0) {
	perror(outfile);
	exit(1);
	}
    if (write(fh,&header,sizeof(header)) != sizeof(header)) {
	perror(outfile);
	exit(1);
	}
    if (write(fh,flashcode,flashsize) != flashsize) {
	perror(outfile);
	exit(1);
	}

    fprintf(stderr,"Wrote %d bytes to %s\n",
	    (int)(sizeof(header)+flashsize),outfile);

    close(fh);

    exit(0);
}
