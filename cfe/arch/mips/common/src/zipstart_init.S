/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU init module			File: zipstart_init.S
    *
    *  This module contains the vectors and lowest-level CPU startup
    *  functions for CFE.
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


#include "sbmips.h"
#include "exception.h"

#include "bsp_config.h"
#include "cpu_config.h"

#ifdef _CFE_
#include "cfe_devfuncs.h"
#else

#define CFE_EPTSEAL 0x43464531

#endif


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#include "mipsmacros.h"


/*  *********************************************************************
    *  SETLEDS(a,b,c,d)
    *  SETLEDS1(a,b,c,d)
    *  
    *  Sets the on-board LED display (if present).  Two variants
    *  of this routine are provided.  If you're running KSEG1,
    *  call the SETLEDS1 variant, else call SETLEDS.
    *  
    *  Input parameters: 
    *  	   a,b,c,d - four ASCII characters (literal constants)
    *  	   
    *  Return value:
    *  	   a0,k1,ra trashed
    ********************************************************************* */


#define SETLEDS(a,b,c,d)                     \
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
       la     k1,board_setleds ;			 \
       jalr   k1 ;

#define SETLEDS1(a,b,c,d)                     \
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
       la     k1,board_setleds ;			 \
       or     k1,K1BASE ; \
       jalr   k1 ;


/*  *********************************************************************
    *  Other constants
    ********************************************************************* */

/*
 * This is the size of the stack, rounded to KByte boundaries.
 */

#ifndef CFG_STACK_SIZE
#error "CFG_STACK_SIZE not defined"
#else
#define STACK_SIZE	((CFG_STACK_SIZE+1023) & ~1023)
#endif

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
    *  Names of registers used in this module
    ********************************************************************* */

#define MEMTOP		t8			/* $24 (t8) */

		.sdata

#include "initdata.h"		/* declare variables we use here */

		.extern	_fdata
		.extern	_edata
		.extern	_etext

/*  *********************************************************************
    *  uninitialized data
    ********************************************************************* */

		.bss

		.comm	__junk,4

/*  *********************************************************************
    *  Exception Vectors
    ********************************************************************* */

		.text
	
		.set noreorder

/*
 * Declare the actual vectors.  This expands to code that
 * must be at the very beginning of the text segment.
 */

DECLARE_VECTOR(0x0000,vec_reset,cpu_reset)
DECLARE_XVECTOR(0x0200,vec_tlbfill,cpu_tlbfill,XTYPE_TLBFILL)
DECLARE_XVECTOR(0x0280,vec_xtlbfill,cpu_xtlbfill,XTYPE_XTLBFILL)
DECLARE_XVECTOR(0x0300,vec_cacheerr,cpu_cacheerr,XTYPE_CACHEERR)
DECLARE_XVECTOR(0x0380,vec_exception,cpu_exception,XTYPE_EXCEPTION)
DECLARE_XVECTOR(0x0400,vec_interrupt,cpu_interrupt,XTYPE_INTERRUPT)
DECLARE_XVECTOR(0x0480,vec_ejtag,cpu_ejtag,XTYPE_EJTAG)


/*
 * New location of CFE seal.  Will eventually phase out the seal at
 * offset 0x508
 */
		.org	0x4E0
cfe_seal:	.word	CFE_EPTSEAL
		.word	CFE_EPTSEAL


		.set reorder

/*  *********************************************************************
    *  CFE Entry Point (used by OS boot loaders and such)
    ********************************************************************* */

                .set  noreorder

DECLARE_VECTOR(0x0500,vec_apientry,cpu_apientry)

		.org	0x508
		.word   CFE_EPTSEAL
		.word	CFE_EPTSEAL

		.set   reorder


/*  *********************************************************************
    *  Segment Table.
    *
    *  Addresses of data segments and of certain routines we're going
    *  to call from KSEG1.  These are here mostly for the embedded
    *  PIC case, since we can't count on the 'la' instruction to
    *  do the expected thing (the assembler expands it into a macro
    *  for doing GP-relative stuff, and the code is NOT GP-relative.
    *  So, we (relocatably) get the offset of this table and then
    *  index within it.  
    *
    *  Pointer values in this segment will be relative to KSEG0 for 
    *  cached versions of CFE, so we need to OR in K1BASE in the
    *  case of calling to a uncached address.
    ********************************************************************* */


#include "segtable.h"

		.org	0x580			# move past exception vectors

		.globl segment_table
segment_table:
		_LONG_	_etext			# [  0] End of text (R_SEG_ETEXT)
		_LONG_	_fdata			# [  1] Beginning of data (R_SEG_FDATA)
		_LONG_	_edata			# [  2] End of data (R_SEG_EDATA)
		_LONG_	_end			# [  3] End of BSS (R_SEG_END)
		_LONG_	_ftext			# [  4] Beginning of text (R_SEG_FTEXT)
		_LONG_	_fbss			# [  5] Beginning of BSS (R_SEG_FBSS)


	
/*  *********************************************************************
    *  CPU Startup Code
    ********************************************************************* */

cpu_reset:

	/*
	 * Test the CAUSE and STATUS registers for why we
         * are here.  Cold reset, Warm reset, and NMI all
         * use this vector.
         */

		mfc0	t0,C0_SR
		and	t0,M_SR_NMI
		beq	t0,zero,not_nmi

		li	t0,PHYS_TO_K1(CFE_LOCORE_GLOBAL_NMIEPT)
		LR	t0,0(t0)
		beq	t0,zero,not_nmi

		j	t0

not_nmi:


	/*
	 * Test to see if we're on the secondary CPU.  If so,
	 * go do the initialization for that CPU.
	 */

#if CFG_MULTI_CPUS

		mfc0	t0,C0_PRID		# get CPU PRID register
		and	t0,t0,0xe000000		# determine cpu number
		beq	t0,zero,is_cpu0		# go if  on CPU0

		li	t0,PHYS_TO_K1(CFE_LOCORE_GLOBAL_CPUEPT)
		LR	t0,0(t0)
		j	t0

is_cpu0:

		/* does not return if on CPU1 */
#endif

#------------------------------------------------------------------------------

	/*
	 * Do low-level board initialization.  This is our first
	 * chance to customize the startup sequence.
	 */

		JAL_KSEG1(board_earlyinit)

		SETLEDS1('H','E','L','O')

		JAL_KSEG1(CPUCFG_CPUINIT)

	/*
	 * Run some diagnostics
	 */

#if !CFG_MINIMAL_SIZE
		SETLEDS1('T','S','T','1')

		JAL_KSEG1(CPUCFG_DIAG_TEST1)
#endif


#------------------------------------------------------------------------------
#if 0
#if CFG_MULTI_CPUS
	/*
	 * Spin up secondary CPU core(s)
	 */

		CALLINIT_KSEG1(init_table,R_INIT_ALTCPU_START1) 
#endif
#endif

	/*
	 * Now, switch from KSEG1 to KSEG0
	 */


#if CFG_RUNFROMKSEG0
		bal	cpu_kseg0_switch
#endif

#------------------------------------------------------------------------------
	/*
	 * Now running on cpu0 in K0SEG.
	 */

#if CFG_INIT_DRAM
		SETLEDS('D','R','A','M')

		JAL(board_draminfo)

		move   a0,v0			# pass these params
		JAL(CPUCFG_DRAMINIT)
		move   k0,v0			# Save in k0 for now
#else
		li	k0,(CFG_DRAM_SIZE * 1024)
#endif

#------------------------------------------------------------------------------

		bne    k0,zero,have_ram

		SETLEDS('R','A','M','X')	# die here if no ram

die1:		b      die1


have_ram:
	 /*
	  * If this is the 64-bit version, turn on the KX bit
	  * to allow 64-bit accesses.
	  */

#if CPUCFG_REGS64
		mfc0	t0,C0_SR
		or	t0,t0,M_SR_KX
		mtc0	t0,C0_SR
#endif

#------------------------------------------------------------------------------
	/*
	 * K0 contains the RAM size (and therefore the top of RAM 
	 * offset).  Start there, and subtract the amount of memory
	 * we expect to use.  If we have more than 256MB of
	 * physical memory, work backwards from the 256MB 
	 * boundary.
	 */  

__CalcMemTop:   li	MEMTOP,256		# 256MB boundary
		bgt	k0,MEMTOP,1f		# use 256MB if k0 is greater
		move	MEMTOP,k0		# otherwise keep top
1:		sll	MEMTOP,20		# make into byte amount


	/*
	 * DRAM is now running, and we're alive in cacheable memory
	 * on cpu0 in K0SEG.  Set up GP.
	 */

		la	gp,_gp

#------------------------------------------------------------------------------
	/*
	 * Zero BSS
         */

		SETLEDS('Z','B','S','S')

		la	a0,segment_table
__ZeroBss:

		LR	v0,R_SEG_FBSS(a0)
		LR	v1,R_SEG_END(a0)

1:		SR	zero,0(v0)		# Zero one cacheline at a time
		SR	zero,(REGSIZE*1)(v0)
		SR	zero,(REGSIZE*2)(v0)
		SR	zero,(REGSIZE*3)(v0)
		add	v0,REGSIZE*4
		blt	v0,v1,1b

#------------------------------------------------------------------------------

	/*
	 * Copy initialized data
         */


		SETLEDS('D','A','T','A')

		la	a0,segment_table

__CopyData:
		LR	t1,R_SEG_ETEXT(a0)
		li	t0,15
		add	t1,t0
		not	t0
		and	t1,t0		# t1 = _etext rounded up to 16-byte boundary
			
		LR	t2,R_SEG_FDATA(a0)
		LR	t3,R_SEG_EDATA(a0)

1:		LR	t4,0(t1)	# read one cache line
		LR	t5,(REGSIZE*1)(t1)
		LR	t6,(REGSIZE*2)(t1)
		LR	t7,(REGSIZE*3)(t1)
		SR	t4,0(t2)	# write one cache line
		SR	t5,(REGSIZE*1)(t2)
		SR	t6,(REGSIZE*2)(t2)
		SR	t7,(REGSIZE*3)(t2)
		add	t1,(REGSIZE*4)
		add	t2,(REGSIZE*4)
		bltu	t2,t3,1b


#------------------------------------------------------------------------------



	/*
         * Remember total amount of memory.  This is *still* in k0
	 * after all this time.  Hopefully.
	 */

__MemVars:
		SR	k0,mem_totalsize

		move	v0,zero

		la	a0,segment_table	# trashed by l2 cache flush
		LR	v0,R_SEG_FDATA(a0)
		LR	v1,R_SEG_END(a0)

		ADD	v1,7			# Realign _END so it 
		and	v1,~7			# is on a 64-bit boundary.

		SR	v0,mem_bottomofmem
		SR	v1,mem_heapstart

		add	v1,(CFG_HEAP_SIZE*1024)	# Otherwise
		add	v1,STACK_SIZE
		SR	v1,mem_topofmem


		LR	t1,R_SEG_FTEXT(a0)
		LR	t0,R_SEG_ETEXT(a0)
		sub	t0,t0,t1
		SR	t0,mem_textsize
		SR	t1,mem_textbase


#------------------------------------------------------------------------------

#if CFG_MULTI_CPUS
	/*
	 * Let secondary CPU(s) run their idle loops.  Set the 
	 * mailbox register to our relocation factor so we can read
	 * it out of the mailbox register and relocate GP properly.
	 */

		move	a0,zero
/*		CALLINIT_KSEG0(init_table,R_INIT_ALTCPU_START2) */
#endif


#------------------------------------------------------------------------------

	/*
	 * Set up the "C" stack and jump to the main routine.
         */

		SETLEDS('Z','L','D','R')

		LR	sp,mem_heapstart
		ADD	sp,((CFG_HEAP_SIZE*1024)+STACK_SIZE - 8)
		li	a0,0			# call as "cfe_main(0,0)"
		li	a1,0
		li	a2,0
		la	a3,_elf_start

__LaunchMain:

		jal	zs_main


	/*
	 * Terminate the simulator.
	 */

crash_sim:      li $2,1
                li $4,0
                syscall	0xCA
		b	cpu_reset



/*  *********************************************************************
    *  CPU_KSEG0_SWITCH
    *  
    *  Hack the return address so we will come back in KSEG0
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(cpu_kseg0_switch)

		and	ra,(K0SIZE-1)
		or	ra,K0BASE
		jr	ra

END(cpu_kseg0_switch)



/*  *********************************************************************
    *  CFE_LAUNCH
    *  
    *  Start the user program.  The program is passed a handle
    *  that must be passed back when calling the firmware.
    *
    *  Parameters passed to the called program are as follows:
    *
    *      a0 - memory size
    *      a1 - 
    *      a2 - 
    *      a3 - 
    *  
    *  Input parameters: 
    *  	   a0 - entry vector
    *  	   
    *  Return value:
    *  	   does not return
    ********************************************************************* */

LEAF(cfe_launch)

		sub	sp,8
		SR	a0,0(sp)


	/*
	 * Mask all interrupts.
	 */
		mfc0	v0,C0_SR		# Get current interrupt flag
		li	v1,M_SR_IE		# master interrupt control
		not	v1			# disable interrupts
		and	v0,v1			# SR now has IE=0
		mtc0	v0,C0_SR		# put back into CP0


	/*
	 * Flush the D-Cache, since the program we loaded is "data".
	 * Invalidate the I-Cache, so that addresses in the program
	 * region will miss and need to be filled from the data we 
	 * just flushed above.
	 */

		SETLEDS('L','1','2','F')
		li	a0,CFE_CACHE_FLUSH_D | CFE_CACHE_FLUSH_L2
		JAL(CPUCFG_CACHEOPS)
		li	a0,CFE_CACHE_INVAL_I
		JAL(CPUCFG_CACHEOPS)

#ifdef _SB1250_PASS1_WORKAROUNDS_
		SETCCAMODE(v0,K_CFG_K0COH_COHERENT) /* cacheable coherent */
#endif

		
	/*
	 * Launch the firmware.
	 */

		LR	a0,mem_totalsize
		move	a1,zero
		move	a2,zero
		move	a3,zero

		LR	t0,0(sp)

__Banzai:
		jr	t0

END(cfe_launch)



/*  *********************************************************************
    *  TLB Fill Exeption Handler
    ********************************************************************* */

cpu_tlbfill:	
		j	_exc_entry

/*  *********************************************************************
    *  XTLB Fill Exception Handler
    ********************************************************************* */

cpu_xtlbfill:	
		j	_exc_entry

/*  *********************************************************************
    *  Cache Error Exception Handler
    ********************************************************************* */

cpu_cacheerr:

	/*
	 * Force stack pointer to be uncached to discourage
	 * future use of the cache.
	 */
		or	sp,K1BASE

		j	_exc_entry


/*  *********************************************************************
    *  API entry
    ********************************************************************* */

		.globl cpu_apientry
cpu_apientry:
		li	t0,PHYS_TO_K0(CFE_LOCORE_GLOBAL_APIEPT)
		LR	t0,0(t0)
		beq	t0,zero,1f
		jr	t0			# Jump to API vector

1:		li	v0,-1			# No entry point installed
		j	ra

/*  *********************************************************************
    *  General Exception Handler
    ********************************************************************* */

cpu_exception:
		j	_exc_entry


/*  *********************************************************************
    *  General Interrupt Handler
    ********************************************************************* */

cpu_interrupt:
		j	_exc_entry


/*  *********************************************************************
    *  EJTAG Debug Exception Handler
    ********************************************************************* */

cpu_ejtag:
		.set push
		.set mips64
		deret
		.set pop
		nop
		j	cpu_reset



/*  *********************************************************************
    *  End
    ********************************************************************* */



