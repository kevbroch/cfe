/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Flash partition manipulation		File: ptable.h
    *  
    *  Definitions and prototypes for flash partitioning.
    *
    *********************************************************************  
    *
    *  Copyright 2005
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


#ifndef _PTABLE_H
#define _PTABLE_H

/*  *********************************************************************
    *  Constants
    ********************************************************************* */

#define PTABLE_MAGIC           0x5054424C     /* 'PTBL' (big-endian) */

#define PTABLE_PNAME_MAX       16
#define PTABLE_MAX_PARTITIONS  8

/*  *********************************************************************
    *  Data structures.
    ********************************************************************* */

typedef struct partition_s {
    char        name[PTABLE_PNAME_MAX];
    uint32_t    offset;
    uint32_t    size;
    uint32_t    type;
    uint32_t    flags;
} partition_t;

typedef struct ptable_s {
    uint32_t    magic;
    uint32_t    version;
    uint32_t    chksum;
    uint32_t    reserved;
    partition_t part[PTABLE_MAX_PARTITIONS];
} ptable_t;

typedef struct ptable_drv_s {
    int         (*pt_read)(ptable_t *ptbl);
    int         (*pt_write)(ptable_t *ptbl);
} ptable_drv_t;

/*  *********************************************************************
    *  Prototypes
    ********************************************************************* */

/* Initialize and provide access functions */
int         ptable_init(ptable_drv_t *ptbl_drv);

/* Calculate checksum across entire partition table */
uint32_t    ptable_check(ptable_t *ptbl);

/* Read/write partition table */
int         ptable_read(ptable_t *ptbl);
int         ptable_write(ptable_t *ptbl);

#endif /* _PTABLE_H */
