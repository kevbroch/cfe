/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU Initialization			File: mpc8245_cpu.S
    *  
    *  Motorola 8245 PowerPC init
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
#include "mpc824x.h"
#include "init_engine.h"
#include "cpu_config.h"

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
GOT_ENTRY(mpc8245_cpuinittab)
GOT_ENTRY(_ftext)
GOT_ENTRY(_etext)
      END_GOT()
#endif

	 .text


/*  *********************************************************************
    *  CPU initialization table
    *  
    *  The table below is fed to the init_engine to set the 8245
    *  into a known state.
    ********************************************************************* */


mpc8245_cpuinittab:


	IET_SPR(SPR_HID0,0)

	/*
	 * First, clear out a few things
	 */

	IET_SPR(SPR_SPRG0,0)
	IET_SPR(SPR_SPRG1,0)
	IET_SPR(SPR_SPRG2,0)
	IET_SPR(SPR_SPRG3,0)

	IET_IBAT(0,0,0)
	IET_IBAT(1,0,0)
	IET_IBAT(2,0,0)
	IET_IBAT(3,0,0)

	IET_DBAT(0,0,0)
	IET_DBAT(1,0,0)
	IET_DBAT(2,0,0)
	IET_DBAT(3,0,0)

	IET_SR(0,0)
	IET_SR(1,0)
	IET_SR(2,0)
	IET_SR(3,0)
	IET_SR(4,0)
	IET_SR(5,0)
	IET_SR(6,0)
	IET_SR(7,0)
	IET_SR(8,0)
	IET_SR(9,0)
	IET_SR(10,0)
	IET_SR(11,0)
	IET_SR(12,0)
	IET_SR(13,0)
	IET_SR(14,0)
	IET_SR(15,0)


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
	 * Enable the ICache 
	 */


	IET_ENABICACHE()

	/*
	 * Set up basic PCI host bridge registers and controls
	 * XXX Might be better to defer this until pciconfig
	 */

	IET_PCI8(MPC_LATTMR,0x20) 
	IET_PCI8(MPC_CLSIZE,0x08)
	IET_PCI16(MPC_PCICMD,(M_PCICMD_MEMENA | M_PCICMD_BUSMASTER))

	IET_PCI16(MPC_PCISTS,0xffff)

	IET_PCI8(MPC_PMCR2,V_PMCR2_PCI_HOLD(0x2))     /* 33 MHz */
	IET_PCI16(MPC_PACR,M_PAC_ARB_EN)
	
	/*
	 * Set up the processor interface config registers 
	 */

	IET_PCI32(MPC_PICR1,(V_PIC1_PROC_TYPE(2) | M_PIC1_FLASH_WR_EN | M_PIC1_MCP_EN |
			M_PIC1_DEC | M_PIC1_CF_APARK | M_PIC1_CF_DPARK |
			M_PIC1_RSVD | M_PIC1_RCS0 | M_PIC1_MBO))
	IET_PCI32(MPC_PICR2,M_PIC2_NO_SERIAL_CFG)

	IET_PCI8(MPC_MIOCR1,(M_MIOCR1_DLL_MAX_DELAY))
	IET_PCI8(MPC_MIOCR2,V_MIOCR2_SDRAM_DSCD(1))

	
	/*
	 * reset the DLL
	 */

//
// XXX for some reason, IET_PCI8X doesn't work here.  Macro expands
// to the right stuff, code in init_engine looks fine, but it doesn't work.
// CPU reset values for AMBOR are reflected below, should have the same effect.
// It may have something to do with the read cycles that the IET_PCI8X macro
// will do to the register to do the r/m/w each time.  The register
// may expect a r/w/w/w pattern to do the dllreset properly.
//
//	IET_PCI8X(MPC_AMBOR,~M_AMBOR_DLLRESET,0)
//	IET_PCI8X(MPC_AMBOR,0xFFFFFFFF,M_AMBOR_DLLRESET)
//	IET_PCI8X(MPC_AMBOR,~M_AMBOR_DLLRESET,0)

	IET_PCI8(MPC_AMBOR,(M_AMBOR_PCI_FD_ALIAS | M_AMBOR_CPU_FD_ALIAS))
	IET_PCI8(MPC_AMBOR,(M_AMBOR_PCI_FD_ALIAS | M_AMBOR_CPU_FD_ALIAS | M_AMBOR_DLLRESET))
	IET_PCI8(MPC_AMBOR,(M_AMBOR_PCI_FD_ALIAS | M_AMBOR_CPU_FD_ALIAS))

	IET_EOT()


/*  *********************************************************************
    *  mpc8245_cpuinit
    *  
    *  Initialize the 8245 CPU and its resources.
    *
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(mpc8245_cpuinit)

	mflr	r29

	/*
	 * Clear the interrupt cause register (read it) 
	 */

	mfspr  r1,SPR_ICR


	/* Set up FPU */

	LDCONST(r3,M_MSR_FP)
	sync
	mtmsr	r3
	isync

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
	 * Run CPU init table to reset internal registers
	 */

	/* LOAD_GOT trashes LR, but we saved it above. */
	LOAD_GOT(GOT_REG)
	LDADDR(r6,mpc8245_cpuinittab)
	bl	init_engine

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

END(mpc8245_cpuinit)

/*  *********************************************************************
    *  mpc8245_restart
    *  
    *  Called when we want to reset/restart an 8245.  Does not return.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(mpc8245_cpurestart)
	LDCONST(r3,0xfff00100)
	mtlr	r3
	blr
END(mpc8245_cpurestart)


/*  *********************************************************************
    *  mpc8245_cacheops
    *  
    *  Called when we want to do cache operations on the 8245.
    *  
    *  Input parameters: 
    *  	   r3 - cacheop flags
    *  	   r4,r5 - might be used if we are invalidating a range
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(mpc8245_cacheops)

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

END(mpc8245_cacheops)



