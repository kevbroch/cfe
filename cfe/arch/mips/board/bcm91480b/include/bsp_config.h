/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  BSP Configuration file			File: bsp_config.h
    *  
    *  This module contains global parameters and conditional
    *  compilation settings for building CFE.
    *  
    *  Author:  Mitch Lichtenberg
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


#ifndef CFG_L2_RAM         /* Set this to run from L2 */
#define CFG_L2_RAM 0 
#endif

#if CFG_RUNFROMKSEG0
#define CFG_INIT_L1		1	/* initialize the L1 cache */
#define CFG_INIT_L2		1	/* initialize the L2 cache */
#else
#define CFG_INIT_L1		1	/* initialize the L1 cache */
#define CFG_INIT_L2		1	/* initialize the L2 cache */
#endif

#if CFG_BOOTRAM
#define CFG_INIT_DRAM		1	/* initialize DRAM controller */
#define CFG_DRAM_SIZE		xxx	/* size of DRAM if you don't initialize */
#elif CFG_L2_RAM
#define CFG_INIT_DRAM		1	/* ignore DRAM */
#define CFG_DRAM_SIZE		xxx	/* reassure init_mips */
#else
#define CFG_INIT_DRAM		1	/* initialize DRAM controller */
#define CFG_DRAM_SIZE		xxx	/* size of DRAM if you don't initialize */
#endif

#define CFG_NETWORK		1	/* define to include network support */
#define CFG_TCP 1
#define CFG_FATFS 1
#define CFG_UI			1	/* Define to enable user interface */

#ifndef _UNICPU_
#define CFG_MULTI_CPUS		1	/* Define to include multiple CPU support */
#endif

#if (CFG_BOOTRAM || CFG_L2_RAM)
#define CFG_HEAP_SIZE		384	/* heap size in kilobytes */
#else
/* 4MB needed for PM (packet manager) testing */
#define CFG_HEAP_SIZE		4096	/* heap size in kilobytes  */
#endif

#define CFG_STACK_SIZE		8192	/* stack size (bytes, rounded up to K) */

/*
 * These parameters control the flash driver's sector buffer.  
 * If you write environment variables or make small changes to
 * flash sectors from user applications, you
 * need to have the heap big enough to store a temporary sector
 * for merging in small changes to flash sectors, so you
 * should set CFG_FLASH_ALLOC_SECTOR_BUFFER in that case.
 * Otherwise, you can provide an address in unallocated memory
 * of where to place the sector buffer.
 */

#define CFG_FLASH_ALLOC_SECTOR_BUFFER 0	/* '1' to allocate sector buffer from the heap */
#define CFG_FLASH_SECTOR_BUFFER_ADDR (100*1024*1024-128*1024)	/* 100MB - 128K */
#define CFG_FLASH_SECTOR_BUFFER_SIZE (128*1024)

/*
 * The flash staging buffer is where we store a flash image before we write
 * it to the flash.  It's too big for the heap.
 */

#define CFG_FLASH_STAGING_BUFFER_ADDR (100*1024*1024)
#define CFG_FLASH_STAGING_BUFFER_SIZE (4*1024*1024)

/*
 * MC_NOCHANINTLV
 * MC_01CHANINTLV		
 * MC_23CHANINTLV      	Valid in 32-bit channels only 
 * MC_01_23CHANINTLV	Valid in 32-bit channels only 
 * MC_FULLCHANINTLV 	Valid in 32-bit channels only 
 */
#define CFG_DRAM_INTERLEAVE	MC_01CHANINTLV

/*
 * NOCSINTLV
 * CSINTLV_2CS
 * CSINTLV_4CS
 * CSINTLV_8CS	Valid in 64-bit channels only
 */
#define CFG_DRAM_CSINTERLEAVE	CSINTLV_4CS 	/* Max is 4 CS interleave */
                                                /* CS 0,2,4,6 for DDR2 	  */

#if CFG_RELOC
#define CFG_DRAM_ECC		1			/* Turn on to enable ECC */
#else
#define CFG_DRAM_ECC		0			/* Turn on to enable ECC */
#endif


#define CFG_SERIAL_BAUD_RATE	115200	/* normal console speed */

#define CFG_VENDOR_EXTENSIONS   0

/*
 * Include board-specific stuff
 */

#include "bcm91480b.h"

