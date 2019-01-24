/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Physical Memory (arena) manager		File: ppc_arena.c
    *  
    *  This module describes the physical memory available to the 
    *  firmware.
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
void ppc_arena_init(void);


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

}


/*  *********************************************************************
    *  ppc_arena_init()
    *  
    *  Create the initial map of physical memory
    *
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void ppc_arena_init(void)
{
    uint64_t memamt;

    arena_init(&cfe_arena,0x0,0x100000000ULL);

#if 0
    /*
     * Mark the ranges from the SB1250's memory map
     */

    ARENA_RANGE(0x0000000000,0x000FFFFFFF,MEMTYPE_DRAM_NOTINSTALLED);
    ARENA_RANGE(0x0080000000,0x009FFFFFFF,MEMTYPE_DRAM_NOTINSTALLED);
    ARENA_RANGE(0x00C0000000,0x00CFFFFFFF,MEMTYPE_DRAM_NOTINSTALLED);
    ARENA_RANGE(0x0100000000,0x7FFFFFFFFF,MEMTYPE_DRAM_NOTINSTALLED);
    /*
     * Mark LDT and PCI regions
     */

    ARENA_RANGE(0x0040000000,0x005FFFFFFF,MEMTYPE_LDT_PCI);
    ARENA_RANGE(0x0060000000,0x007FFFFFFF,MEMTYPE_LDT_PCI);

    ARENA_RANGE(0x00D8000000,0x00DFFFFFFF,MEMTYPE_LDT_PCI);

    ARENA_RANGE(0x00F8000000,0x00FFFFFFFF,MEMTYPE_LDT_PCI);
    ARENA_RANGE(0xF800000000,0xF9FFFFFFFF,MEMTYPE_LDT_PCI);

    ARENA_RANGE(0xFD00000000,0xFFFFFFFFFF,MEMTYPE_LDT_PCI);

    /*
     * System IO registers
     */

    ARENA_RANGE(0x0010000000,0x001002FFFF,MEMTYPE_IOREGISTERS);
#endif

    /*
     * Now, fix up the map with what is known about *this* system.
     *
     * Do each 256MB chunk.
     */

    memamt = ((int64_t) mem_totalsize) << 20;
    arena_markrange(&cfe_arena,0x00000000,memamt,MEMTYPE_DRAM_AVAILABLE,NULL);

#if 0
    memleft = ((int64_t) mem_totalsize) << 20;

    memamt = (memleft > mem256) ? mem256 : memleft;

    arena_markrange(&cfe_arena,0x00000000,memamt,MEMTYPE_DRAM_AVAILABLE,NULL);
    memleft -= memamt;

    if (memleft) {
	memamt = (memleft > mem256*2) ? mem256*2 : memleft;
	arena_markrange(&cfe_arena,0x80000000,memamt,MEMTYPE_DRAM_AVAILABLE,NULL);
	memleft -= memamt;
	}

    if (memleft) {
	memamt = (memleft > mem256) ? mem256 : memleft;
	arena_markrange(&cfe_arena,0xC0000000,memamt,MEMTYPE_DRAM_AVAILABLE,NULL);
	memleft -= memamt;
	}

    if (memleft) {
	arena_markrange(&cfe_arena,0x100000000,memleft,MEMTYPE_DRAM_AVAILABLE,NULL);
	}
	

    /*
     * Do the boot ROM
     */
    
    arena_markrange(&cfe_arena,0x1FC00000,2*1024*1024,MEMTYPE_BOOTROM,NULL);
#endif

}


