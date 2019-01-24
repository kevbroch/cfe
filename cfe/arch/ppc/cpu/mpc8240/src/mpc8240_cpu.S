/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU Initialization			File: mpc8240_cpu.S
    *  
    *  Motorola 8240 PowerPC init
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2003
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


#include "ppcdefs.h"
#include "ppcmacros.h"
#include "bsp_config.h"
#include "cpu_config.h"
#include "mpc824x.h"
#include "init_engine.h"


/*
 * Duplicates from cfe_iocb.h -- warning!
 */

#define CFE_CACHE_FLUSH_D	1
#define CFE_CACHE_INVAL_I	2
#define CFE_CACHE_INVAL_D	4
#define CFE_CACHE_INVAL_L2	8
#define CFE_CACHE_FLUSH_L2	16
#define CFE_CACHE_INVAL_RANGE	32
#define CFE_CACHE_FLUSH_RANGE	64


/*  *********************************************************************
    *  Global Offset Table
    ********************************************************************* */

#if CFG_RELOC
      BEGIN_GOT()
GOT_ENTRY(mpc8240_cpuinittab)
GOT_ENTRY(_ftext)
GOT_ENTRY(_etext)
      END_GOT()
#endif


	 .text


/*  *********************************************************************
    *  CPU initialization table
    *  
    *  The table below is fed to the init_engine to set the 8240
    *  into a known state.
    ********************************************************************* */


mpc8240_cpuinittab:

	/*
	 * First, clear out a few things
	 */

	IET_SPR(SPR_SPRG0,0)
	IET_SPR(SPR_SPRG1,0)
	IET_SPR(SPR_SPRG2,0)
	IET_SPR(SPR_SPRG3,0)
	IET_SPR(SPR_SPRG4,0)
	IET_SPR(SPR_SPRG5,0)
	IET_SPR(SPR_SPRG6,0)
	IET_SPR(SPR_SPRG7,0)

	/*
	 * Enable the FPU 
	 */

	IET_MSR(M_MSR_IP|M_MSR_ME|M_MSR_RI|M_MSR_FP)

	/*
	 * Clear out segment registers.
	 */

	IET_SR(0,0)
	IET_SR(1,0)
	IET_SR(2,0)
	IET_SR(3,0)
	IET_SR(4,0)
	IET_SR(5,0)
	IET_SR(6,0)
	IET_SR(7,0)

	/*
	 * Disable (zero) all the BATs to avoid possible clashes below.
	 */
	
	IET_IBAT(0,0,0)
	IET_IBAT(1,0,0)
	IET_IBAT(2,0,0)
	IET_IBAT(3,0,0)

	IET_DBAT(0,0,0)
	IET_DBAT(1,0,0)
	IET_DBAT(2,0,0)
	IET_DBAT(3,0,0)

	
	/*
	 * Enable the ICache
	 */
	IET_SPRX(SPR_HID0,0,(M_HID0_ICE|M_HID0_ICFI))	
	IET_SPRX(SPR_HID0,0,M_HID0_ICE)	

	/*
	 * Set up basic PCI host bridge registers and controls
	 * XXX Might be better to defer this until pciconfig
	 */

	IET_PCI8(MPC_LATTMR,0x20) 
	IET_PCI8(MPC_CLSIZE,0x08)
	IET_PCI16(MPC_PCICMD,(M_PCICMD_MEMENA | M_PCICMD_BUSMASTER))

	IET_PCI16(MPC_PCISTS,0xffff)

	IET_PCI8(MPC_PMCR2,V_PMCR2_PCI_HOLD(0x4))     /* 33 MHz */
	IET_PCI16(MPC_PACR,M_PAC_ARB_EN)
	
	/*
	 * Set up the processor interface config registers 
	 */

	IET_PCI32(MPC_PICR1,(V_PIC1_PROC_TYPE(2) | M_PIC1_FLASH_WR_EN | M_PIC1_MCP_EN |
			M_PIC1_ST_GATH_EN | M_PIC1_RD_SPEC_EN | M_PIC1_CF_APARK |
			M_PIC1_MBO))
	IET_PCI32(MPC_PICR2,M_PIC2_CF_FF0_LOCAL)

	IET_EOT()




/*  *********************************************************************
    *  mpc8240_cpuinit
    *  
    *  Initialize the 8240 CPU and its resources.
    *
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(mpc8240_cpuinit)

	mflr	r29			/* save return address */

	/*
	 * Clear the interrupt cause register (read it) 
	 */

	mfspr  r1,SPR_ICR

	/*
	 * Run CPU init table to reset internal registers
	 */

	/* LOAD_GOT trashes LR, but we saved it above. */
	LOAD_GOT(GOT_REG)
	LDADDR(r6,mpc8240_cpuinittab)

	bl	init_engine

	/* Initialize floating point registers */
	mtfsfi	7,0x0
	mtfsfi	6,0x0
	mtfsfi	5,0x0
	mtfsfi	4,0x0
	mtfsfi	3,0x0
	mtfsfi	2,0x0
	mtfsfi	1,0x0
	mtfsfi	0,0x0
	isync


	/*
	 * Clear out the TLB
	 */

        lis     r3,0
        mtctr   r3
        lis     r5,0x04

1:      tlbie   r3
        sync
        addi    r3,r3,0x1000
        cmplw   r3,r5
        blt     1b
        sync

	mtlr	r29
	blr

END(mpc8240_cpuinit)

/*  *********************************************************************
    *  mpc8240_restart
    *  
    *  Called when we want to reset/restart an 8240.  Does not return.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(mpc8240_cpurestart)
	LDCONST(r3,0xfff00100)
	mtlr	r3
	blr
END(mpc8240_cpurestart)


/*  *********************************************************************
    *  mpc8240_cacheops
    *  
    *  Called when we want to do cache operations on the 8240.
    *  
    *  Input parameters: 
    *  	   r3 - cacheop flags
    *  	   r4,r5 - might be used if we are invalidating a range
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(mpc8240_cacheops)

	mflr  r7
	SAVE_GOT(r8)
	LOAD_GOT(GOT_REG)

	andi. r0,r3,CFE_CACHE_FLUSH_D
	beq   22f

    /*
     * PPC doesn't seem to have an index-invalidate instruction - the only
     * way to flush the DCache is to read lots of data.
     */

        LDADDR(r4,_ftext)
	LDADDR(r5,_etext)

1:	lwz    r0,0(r4)
	addi   r4,r4,CPUCFG_CACHELINESIZE
	cmplw  r4,r5
	ble    1b

22:	andi. r0,r3,CFE_CACHE_INVAL_I
	beq   22f

	li     r5,M_HID0_ICFI
	mfspr  r4,SPR_HID0
	or     r4,r4,r5
	mtspr  SPR_HID0,r4
	isync
	isync
	andc   r4,r4,r5
	mtspr  SPR_HID0,r4
	isync
	isync

	andi.  r0,r3,CFE_CACHE_INVAL_RANGE
	beq    22f

	mr     r9,r4
1:	dcbf   0,r9
	addi   r9,r9,CPUCFG_CACHELINESIZE
	cmplw  r9,r5
	blt    1b

	andi.  r0,r3,CFE_CACHE_FLUSH_RANGE
	beq    22f

	mr     r9,r4
1:	icbi   0,r9
	addi   r9,r9,CPUCFG_CACHELINESIZE
	cmplw  r9,r5
	blt    1b

22:	RESTORE_GOT(r8)
	mtlr   r7
	blr

END(mpc8240_cacheops)



