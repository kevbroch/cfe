/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU init module				File: init_ram.S
    *
    *  This module contains the vectors and lowest-level CPU startup
    *  functions for CFE.
    * 
    *  This is very similar to "init_mips.S" but is used when
    *  you want to locate CFE in DRAM, loading it like an
    *  application program.
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


#include "sbmips.h"
#include "exception.h"

#include "bsp_config.h"
#include "cpu_config.h"

#include "cfe_devfuncs.h"

/*  *********************************************************************
    *  Check some stuff
    ********************************************************************* */

#if (CFG_RELOC)
#error "RAM version is not compatible with relocation."
#endif
#if !(CFG_RUNFROMKSEG0) && !(defined(JTAG_RAM_BOOT))
#error "RAM version should be run cached"
#endif

#if !(CFG_ZIPSTART)
#if CFG_MULTI_CPUS
#error "Multiple CPUs not compatible with RAM version"
#endif
#endif


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#include "mipsmacros.h"


/*  *********************************************************************
    *  SETLEDS(a,b,c,d)
    *  
    *  Sets the on-board LED display (if present). 
    *  
    *  Input parameters: 
    *  	   a,b,c,d - four ASCII characters (literal constants)
    *  	   
    *  Return value:
    *  	   a0,k1,ra trashed
    ********************************************************************* */


#define SETLEDS(a,b,c,d)                     \
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
       jal    board_setleds ;


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


/*
 * To make life easier reading this code, define "KSEGBASE" 
 * to either K0BASE or K1BASE depending on whether we're running
 * uncached.
 */
#ifdef JTAG_RAM_BOOT
#define KSEGBASE        K1BASE   /* JTAG RAM version always uncached */
#else
#define KSEGBASE        K0BASE   /* RAM version always cached */
#endif /* JTAG_RAM_BOOT */

/*  *********************************************************************
    *  Names of registers used in this module
    ********************************************************************* */

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

		.text
	
		.set noreorder


/*  *********************************************************************
    *  CFE Entry Point (used by OS boot loaders and such)
    ********************************************************************* */

                .set  noreorder

                .globl vec_reset

vec_reset:	b      cpu_reset
		nop

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

#------------------------------------------------------------------------------
	/*
	 * Set up GP.
	 */

	        la	gp,_gp

#------------------------------------------------------------------------------

#if CFG_ZIPSTART
	 /*
	  * The boot loader passes the memory size to us
	  * in A0.
	  */
		SR	a0,mem_totalsize

	 /*
	  * We need to set up certain pointers in locore for ourselves to use
	  * later on.  The fragment of the ROM-based entry points vector
	  * through these locore pointers.
	  */
		li	t1,K0BASE		# write into both cached
		li	t2,K1BASE		# and uncached space.

		la	t3,CPUCFG_ALTCPU_RESET
		or	t3,K1BASE
		SR	t3,CFE_LOCORE_GLOBAL_CPUEPT(t2)
		SR	t3,CFE_LOCORE_GLOBAL_CPUEPT(t1)

		la	t3,cpu_apientry
		SR	t3,CFE_LOCORE_GLOBAL_APIEPT(t2)
		SR	t3,CFE_LOCORE_GLOBAL_APIEPT(t1)

#if CFG_MULTI_CPUS
	   /*
	    * Run uncacheable for a bit to ensure coherency while we
	    * pull the other core out of reset
	    *
	    * Hack: We can use "CALLINIT_KSEG1" to do this, since it
	    * makes sure the things we call are in KSEG1.
	    */

		JAL_KSEG1(CPUCFG_ALTCPU_START1)

	   /*
	    * Now, we're back in KSEG0 again.  We can just finish the startup
	    * and let cpu1 go to its idle loop.
	    */

	        move	a0,zero			/* not relocating */
		JAL(CPUCFG_ALTCPU_START2)
#endif

#endif

#------------------------------------------------------------------------------

#if (!CFG_ZIPSTART)
	/*
	 * Do low-level board initialization.  This is our first
	 * chance to customize the startup sequence.
	 * 
	 * Don't do this for ZIPSTART, since the boot loader
	 * already did it.
	 */

		JAL(board_earlyinit)

		SETLEDS('H','E','L','O')
#endif

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



#if CPUCFG_REGS64
		mfc0	t0,C0_SR
		or	t0,t0,M_SR_KX
		mtc0	t0,C0_SR
#endif

#------------------------------------------------------------------------------

__MemVars:

#if (!CFG_ZIPSTART)
		li	k0,256			# memory size in megabytes
		SR	k0,mem_totalsize
#endif
		SR	zero,mem_datareloc

		move	v0,zero

		la	a0,segment_table	# trashed by l2 cache flush
		LR	v0,R_SEG_FTEXT(a0)	# bottom = beginning of text
		LR	v1,R_SEG_END(a0)

		SR	v0,mem_bottomofmem
		SR	v1,mem_heapstart

		add	v1,(CFG_HEAP_SIZE*1024)	# Otherwise
		add	v1,STACK_SIZE
		SR	v1,mem_topofmem

		SR	zero,mem_textreloc


		LR	t1,R_SEG_FTEXT(a0)
		LR	t0,R_SEG_ETEXT(a0)
		sub	t0,t0,t1
		SR	t0,mem_textsize
		SR	t1,mem_textbase


#------------------------------------------------------------------------------

#if 0
#if CFG_MULTI_CPUS
	/*
	 * Let secondary CPU(s) run their idle loops.  Set the 
	 * mailbox register to our relocation factor so we can read
	 * it out of the mailbox register and relocate GP properly.
	 */

		move	a0,zero
		CALLINIT_KSEG0(init_table,R_INIT_ALTCPU_START2)
#endif
#endif
		
	/*
	 * Stash away some config register stuff
         */

		mfc0	v0,C0_PRID
		SR	v0,cpu_prid		


#------------------------------------------------------------------------------

	/*
	 * Set up the "C" stack and jump to the main routine.
         */

		SETLEDS('M','A','I','N')

		LR	sp,mem_heapstart
		ADD	sp,((CFG_HEAP_SIZE*1024)+STACK_SIZE - 8)
		li	a0,0			# call as "cfe_main(0,0)"
		li	a1,0

		JAL(cfe_main)


	/*
	 * Terminate the simulator.
	 */

crash_sim:      li $2,1
                li $4,0
                syscall	0xCA
		b	cpu_reset





/*  *********************************************************************
    *  CFE_LAUNCH
    *  
    *  Start the user program.  The program is passed a handle
    *  that must be passed back when calling the firmware.
    *
    *  Parameters passed to the called program are as follows:
    *
    *      a0 - CFE handle
    *      a1 - entry vector
    *      a2 - reserved, will be 0
    *      a3 - entrypoint signature.
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

#ifndef JTAG_RAM_BOOT
	/*
	 * Flush the D-Cache, since the program we loaded is "data".
	 * Invalidate the I-Cache, so that addresses in the program
	 * region will miss and need to be filled from the data we 
	 * just flushed above.
	 */
		li	a0,CFE_CACHE_FLUSH_D|CFE_CACHE_INVAL_I
		JAL(CPUCFG_CACHEOPS)
#endif
	/*
	 * Set things up for launching the program.  Pass the
	 * handle in A0 - apps need to remember that and pass it
	 * back.
	 */

		j	RunProgram

END(cfe_launch)

	/*
 	 * This is a nice place to set a breakpoint.
	 */
LEAF(RunProgram)

		la	a2,cpu_apientry # A2 = API entry
		move	t0,a0		# 
		move	a1,zero		# A1 = 0
		move	a0,gp		# A0 = handle
		li	a3,CFE_EPTSEAL  # A3 = entrypoint signature
		LR	t0,0(sp)	# entry point
		j	t0		# go for it.
END(RunProgram)




/*  *********************************************************************
    *  CFE_LEDS
    *  
    *  Set the on-board LEDs.
    *  
    *  Input parameters: 
    *  	   a0 - LEDs
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(cfe_leds)

		j	board_setleds		# jump to BSP routine

END(cfe_leds)


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
    *  CPU_KSEG1_SWITCH
    *  
    *  Hack the return address so we will come back in KSEG1
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(cpu_kseg1_switch)

		and	ra,(K0SIZE-1)
		or	ra,K1BASE
		jr	ra

END(cpu_kseg1_switch)




/*  *********************************************************************
    *  End
    ********************************************************************* */


