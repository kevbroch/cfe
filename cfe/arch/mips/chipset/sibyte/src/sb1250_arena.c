/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Physical Memory (arena) manager		File: sb1250_arena.c
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
#include "sbmips.h"

#include "lib_arena.h"

#include "cfe_mem.h"

#include "initdata.h"

#include "sb1250_regs.h"
#include "sb1250_scd.h"

#define _NOPROTOS_
#include "cfe_boot.h"
#undef _NOPROTOS_


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#define ARENA_RANGE(bottom,top,type) arena_markrange(&cfe_arena,(uint64_t)(bottom), \
                                     (uint64_t)(top)-(uint64_t)bottom+1,(type),NULL)

#define MEG	(1024*1024)
#define KB      1024
#define PAGESIZE 4096
#define CFE_BOOTAREA_SIZE (256*KB)
#define CFE_BOOTAREA_ADDR 0x20000000

/*  *********************************************************************
    *  Globals
    ********************************************************************* */

extern arena_t cfe_arena;

unsigned int mem_bootarea_start;
unsigned int mem_bootarea_size;

void sb1250_arena_init(void);
void sb1250_pagetable_init(uint64_t *ptaddr,unsigned int physaddr);


/*  *********************************************************************
    *  sb1250_arena_init()
    *  
    *  Create the initial map of physical memory
    *
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void sb1250_arena_init(void)
{
    int64_t memleft;
    int64_t mem256 = (256*1024*1024);		/* 256 MBytes */
    int64_t memamt;
    uint64_t sysrev;

    arena_init(&cfe_arena,0x0,0x10000000000LL);

    /*
     * Mark the ranges from the SB1250's memory map
     */

    ARENA_RANGE(0x0000000000LL,0x000FFFFFFFLL,MEMTYPE_DRAM_NOTINSTALLED);
    ARENA_RANGE(0x0080000000LL,0x009FFFFFFFLL,MEMTYPE_DRAM_NOTINSTALLED);
    ARENA_RANGE(0x00C0000000LL,0x00CFFFFFFFLL,MEMTYPE_DRAM_NOTINSTALLED);
    
    sysrev = SBREADCSR(A_SCD_SYSTEM_REVISION);

    if (SYS_SOC_TYPE(sysrev) >= K_SYS_SOC_TYPE_BCM1x80)
      ARENA_RANGE(0x0140000000LL,0x0FFFFFFFFFLL,MEMTYPE_DRAM_NOTINSTALLED);
    else
      ARENA_RANGE(0x0100000000LL,0x7FFFFFFFFFLL,MEMTYPE_DRAM_NOTINSTALLED);

    
    /*
     * Mark LDT and PCI regions
     */

    ARENA_RANGE(0x0040000000LL,0x005FFFFFFFLL,MEMTYPE_LDT_PCI);
    ARENA_RANGE(0x0060000000LL,0x007FFFFFFFLL,MEMTYPE_LDT_PCI);

    ARENA_RANGE(0x00D8000000LL,0x00DFFFFFFFLL,MEMTYPE_LDT_PCI);

    ARENA_RANGE(0x00F8000000LL,0x00FFFFFFFFLL,MEMTYPE_LDT_PCI);
    ARENA_RANGE(0xF800000000LL,0xF9FFFFFFFFLL,MEMTYPE_LDT_PCI);

    ARENA_RANGE(0xFD00000000LL,0xFFFFFFFFFFLL,MEMTYPE_LDT_PCI);

    /*
     * System IO registers
     */

    ARENA_RANGE(0x0010000000LL,0x001002FFFFLL,MEMTYPE_IOREGISTERS);

    /*
     * Now, fix up the map with what is known about *this* system.
     *
     * Do each 256MB chunk.
     */

    memleft = ((int64_t) mem_totalsize) << 20;

    memamt = (memleft > mem256) ? mem256 : memleft;

    arena_markrange(&cfe_arena,0x00000000,memamt,MEMTYPE_DRAM_AVAILABLE,NULL);
    memleft -= memamt;

    if (memleft) {
	memamt = (memleft > mem256*2) ? mem256*2 : memleft;
	arena_markrange(&cfe_arena,0x80000000LL,memamt,MEMTYPE_DRAM_AVAILABLE,NULL);
	memleft -= memamt;
	}

    if (memleft) {
	memamt = (memleft > mem256) ? mem256 : memleft;
	arena_markrange(&cfe_arena,0xC0000000LL,memamt,MEMTYPE_DRAM_AVAILABLE,NULL);
	memleft -= memamt;
	}

    if (memleft) {
      if (SYS_SOC_TYPE(sysrev) >= K_SYS_SOC_TYPE_BCM1x80)
	arena_markrange(&cfe_arena,0x140000000LL,memleft,MEMTYPE_DRAM_AVAILABLE,NULL);
      else
	arena_markrange(&cfe_arena,0x100000000LL,memleft,MEMTYPE_DRAM_AVAILABLE,NULL);
	}
	

    /*
     * Do the boot ROM
     */
    
    arena_markrange(&cfe_arena,0x1FC00000,2*1024*1024,MEMTYPE_BOOTROM,NULL);

}


/*  *********************************************************************
    *  SB1250_PAGETABLE_INIT(ptaddr,physaddr)
    *  
    *  This routine constructs the page table.  256KB is mapped
    *  starting at physical address 'physaddr' - the resulting
    *  table entries are placed at 'ptaddr'
    *  
    *  Input parameters: 
    *  	   ptaddr - base of page table
    *  	   physaddr - starting physical addr of area to map
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void sb1250_pagetable_init(uint64_t *ptaddr,unsigned int physaddr)
{
    int idx;

    for (idx = 0; idx < (CFE_BOOTAREA_SIZE/PAGESIZE); idx++) {
	ptaddr[idx] = (physaddr >> 6) | 
	    V_TLBLO_CALG(K_CALG_COH_SHAREABLE) | 
	    M_TLBLO_V |
	    M_TLBLO_D;
	physaddr += PAGESIZE;
	}

}

