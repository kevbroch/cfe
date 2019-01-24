/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Physical Memory (arena) manager		File: x86_arena.c
    *  
    *  This module describes the physical memory available to the 
    *  firmware.
    *  
    *  Author:  Mitch Lichtenberg 
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001
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
#include "cfe_mem.h"
#include "lib_arena.h"

#include "initdata.h"

#define _NOPROTOS_
#include "cfe_boot.h"
#undef _NOPROTOS_

/*  *********************************************************************
    *  Constants
    ********************************************************************* */


#define MEG	(1024*1024)
#define KB      1024
#define PAGESIZE 4096
#define CFE_BOOTAREA_SIZE (256*KB)
#define CFE_BOOTAREA_ADDR 0x20000000


/*  *********************************************************************
    *  Globals
    ********************************************************************* */

extern arena_t cfe_arena;

void cfe_bootarea_init(void);

void x86_arena_init(void);

/*  *********************************************************************
    *  X86_ARENA_INIT()
    *  
    *  Initialize the arena for ARM processors
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
void x86_arena_init(void)
{
    /*
     * 32-bit physical address space (yeah, wrong, but...)
     * XXX extend for 36-bit PA's when we understand impact of PAE.
     */

    arena_init(&cfe_arena,0,0x100000000);

    /*
     * Mark DRAM we have
     */
    arena_markrange(&cfe_arena,0x00000000,(mem_totalsize << 20),MEMTYPE_DRAM_AVAILABLE,NULL);

    /*
     * Leave 640KB -> 1MB as "reserved"
     */

    arena_markrange(&cfe_arena,0x000A0000,0x60000,MEMTYPE_RESERVED,NULL);

    /*
     * 512K for the boot ROM.  Probably needs to be defined elsewhere.
     */
    arena_markrange(&cfe_arena,0xFFF80000,0x80000,MEMTYPE_BOOTROM,NULL);
}

/*  *********************************************************************
    *  CFE_BOOTAREA_INIT()
    *  
    *  Initialize the page table and map our boot program area.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void cfe_bootarea_init(void)
{
    /* set up page table for boot area here */
}

