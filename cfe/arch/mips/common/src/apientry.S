/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  API entry module				File: apientry.S
    *
    *  Low-level API entry point routines and some other misc stuff.
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

#if (CFG_BIENDIAN) && defined(__MIPSEB)
#define CFE_EPTSEAL_REV 0x31454643
#endif
#define CFE_EPTSEAL 0x43464531

#define cfe_command_restart 0
#endif

#ifndef CFG_STACK_SIZE
#error "CFG_STACK_SIZE not defined"
#else
#define STACK_SIZE	((CFG_STACK_SIZE+1023) & ~1023)
#endif

#include "segtable.h"

#if defined(_ZIPSTART_)
#error "This should not be part of zipstart."
#endif

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#include "mipsmacros.h"

/*  *********************************************************************
    *  Data
    ********************************************************************* */

#if CFG_MULTI_CPUS

		.sdata
		.globl	cfe_spinlock
cfe_spinlock:	.word 0
#endif


/*  *********************************************************************
    *  Code starts here
    ********************************************************************* */


		.text


/*  *********************************************************************
    *  cpu_apientry(handle,iocb)
    *
    *  API entry point for external apps.
    *  
    *  Input parameters: 
    *  	   a0 - firmware handle (used to determine the location of
    *  	        our relocated data)
    *  	   a1 - pointer to IOCB to execute
    *  	   
    *  Return value:
    *  	   v0 - return code, 0 if ok
    ********************************************************************* */

#define _regidx(x)    ((x)*8)

#define CAE_SRSAVE     _regidx(0)
#define CAE_GPSAVE     _regidx(1)
#define CAE_RASAVE     _regidx(2)
#define CAE_S0SAVE     _regidx(3)
#define CAE_S1SAVE     _regidx(4)
#define CAE_S2SAVE     _regidx(5)
#define CAE_S3SAVE     _regidx(6)
#define CAE_S4SAVE     _regidx(7)
#define CAE_S5SAVE     _regidx(8)
#define CAE_S6SAVE     _regidx(9)
#define CAE_S7SAVE     _regidx(10)
#define CAE_K0SAVE     _regidx(11)
#define CAE_K1SAVE     _regidx(12)

#define CAE_STKSIZE    _regidx(14)

#if defined(__MIPSEB)
#define ENDIANOFFSET	4
#else
#define ENDIANOFFSET	0      
#endif

#define R_XIOCB_FCODE  (8*0+ENDIANOFFSET)
#define R_XIOCB_FLAGS  (8*3+ENDIANOFFSET)
#define R_XIOCB_XSTAT  (8*5+ENDIANOFFSET)

#define CFE_CMD_FW_RESTART	1
#define CFE_FLG_WARMSTART     0x00000001

LEAF(cpu_apientry)

	/*	
	 * Gross: Make an explicit check here for a warm firmware restart,
	 * to avoid setting up the stack and doing other nasty things
	 * when we're just going to return to the firmware anyway.
	 */

		lw	t0,R_XIOCB_FCODE(a1)
		bne	t0,CFE_CMD_FW_RESTART,notwarm
		lw	t0,R_XIOCB_FLAGS(a1)
		and	t0,CFE_FLG_WARMSTART
		bne	t0,CFE_FLG_WARMSTART,notwarm

	 /*
	  * Disable interrupts before warm restart.  Don't bother
	  * to save the results on the stack, we aren't going back.
	  */

		mfc0	t0,C0_SR		# Get current interrupt flag
		and	t0,~M_SR_IE
		mtc0	t0,C0_SR
		HAZARD

	 /*
	  * Transfer control back to CFE, passing exit status.
	  */

		move	gp,a0			# Reset our GP
		lw	a0,R_XIOCB_XSTAT(a1)	# Exit status
		b	_cfe_warmstart

	 /*  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  */

  	 /*
	  * Not a warm restart, run on the caller's stack.
	  */

notwarm:	sub	sp,CAE_STKSIZE		# Make room for our stuff

		mfc0	v0,C0_SR		# Get current interrupt flag
		SREG	v0,CAE_SRSAVE(sp)	# save on stack
		li	t0,M_SR_IE		# master interrupt control
		not	t0			# disable interrupts
		and	v0,t0			# SR now has IE=0
#if CPUCFG_REGS64
		or	v0,M_SR_KX
#endif
		mtc0	v0,C0_SR		# put back into CP0
		HAZARD

		SREG	gp,CAE_GPSAVE(sp)	# save GP
		SREG	ra,CAE_RASAVE(sp)	# and old RA

		SREG	s0,CAE_S0SAVE(sp)
		SREG	s1,CAE_S1SAVE(sp)
		SREG	s2,CAE_S2SAVE(sp)
		SREG	s3,CAE_S3SAVE(sp)
		SREG	s4,CAE_S4SAVE(sp)
		SREG	s5,CAE_S5SAVE(sp)
		SREG	s6,CAE_S6SAVE(sp)
		SREG	s7,CAE_S7SAVE(sp)
		SREG	k0,CAE_K0SAVE(sp)
		SREG	k1,CAE_K1SAVE(sp)

		move	gp,a0			# set up new GP
		move	a0,a1			# A0 points at IOCB

#if CFG_MULTI_CPUS
		SPIN_LOCK(cfe_spinlock,t0,t1)
#endif

		JAL(cfe_doxreq)

#if CFG_MULTI_CPUS
		SPIN_UNLOCK(cfe_spinlock,t0)
#endif

		#
		# Restore the saved registers.
		#

		LREG	k1,CAE_K1SAVE(sp)
		LREG	k0,CAE_K0SAVE(sp)
		LREG	s7,CAE_S7SAVE(sp)
		LREG	s6,CAE_S6SAVE(sp)
		LREG	s5,CAE_S5SAVE(sp)
		LREG	s4,CAE_S4SAVE(sp)
		LREG	s3,CAE_S3SAVE(sp)
		LREG	s2,CAE_S2SAVE(sp)
		LREG	s1,CAE_S1SAVE(sp)
		LREG	s0,CAE_S0SAVE(sp)

		LREG	ra,CAE_RASAVE(sp)	# unwind the stack
		LREG	gp,CAE_GPSAVE(sp)

		LREG	t0,CAE_SRSAVE(sp)	# old interrupt mask

		add	sp,CAE_STKSIZE		# restore old stack pointer

		mtc0	t0,C0_SR		# restore interrupts
		HAZARD
		j	ra
		nop
END(cpu_apientry)



/*  *********************************************************************
    *  CFE_WARMSTART
    *  
    *  Restart the command interpreter
    *  
    *  Input parameters: 
    *      A0 - command status
    *  	   nothing (GP has already been set up for us)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(cfe_warmstart)

_cfe_warmstart:
	/*
	 * Reset the stack pointer.
	 */

		LR	sp,mem_heapstart
		ADD	sp,((CFG_HEAP_SIZE*1024)+STACK_SIZE - 8)


	/*
	 * Undo any CP0 setup the calling program left behind.
	 */

		SR	a0,0(sp)	
		JAL(CPUCFG_CPURESTART)
		LR	a0,0(sp)

	 /*
	  * If this is the 64-bit version, turn on the KX bit
	  * to allow 64-bit accesses. 
	  */

#if CPUCFG_REGS64
		mfc0	t0,C0_SR
		or	t0,t0,M_SR_KX
		mtc0	t0,C0_SR
		HAZARD
#endif



#ifdef notused
		LR	v0,cfe_pagetable		# reestablish 
		dsll	v0,v0,13			# see mips_arena.c for this
		dmtc0	v0,C0_CTEXT			# boot area TLBs

		la	t9,sb1_cp0_init
		jalr	t9
#endif

	/*
	 * If someone called the API to do a warm start, clear the
	 * spin lock, since the call will never return.
	 */

#if CFG_MULTI_CPUS
		SPIN_UNLOCK(cfe_spinlock,t0)
#endif

	 /*
	  * Switch back to using RAM vectors.
	  */

		mfc0	t0,C0_SR
		and	t0,t0,~M_SR_BEV
		mtc0	t0,C0_SR
		HAZARD

		JAL(cfe_command_restart)

END(cfe_warmstart)



/*  *********************************************************************
    *  _GETSTATUS()
    *  
    *  Read the STATUS register into v0
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   v0 - Status register
    ********************************************************************* */

LEAF(_getstatus)

		mfc0	v0,C0_SR
		j	ra
END(_getstatus)

/*  *********************************************************************
    *  _SETSTATUS()
    *  
    *  Set the STATUS register to the value in a0
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   v0 - Status register
    ********************************************************************* */

LEAF(_setstatus)

		mtc0	a0,C0_SR
		j	ra
END(_setstatus)


/*  *********************************************************************
    *  _GETCAUSE()
    *  
    *  Read the CAUSE register into v0
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   v0 - Cause register
    ********************************************************************* */

LEAF(_getcause)

		mfc0	v0,C0_CAUSE
		j	ra
END(_getcause)


/*  *********************************************************************
    *  _GETTICKS()
    *  
    *  Read the COUNT register into v0
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   v0 - count register
    ********************************************************************* */

LEAF(_getticks)

		mfc0	v0,C0_COUNT
		j	ra
END(_getticks)


/*  *********************************************************************
    *  _SETALARM(ticks)
    *  
    *  Set the C0_Compare register from a0
    *  
    *  Input parameters: 
    *  	   a0 - compare register
    *  	   
    *  Return value:
    *  	   none
    ********************************************************************* */

LEAF(_setalarm)

		mtc0	a0,C0_COMPARE
		j	ra
END(_setalarm)


/*  *********************************************************************
    *  _SETCONTEXT()
    *  
    *  Set the CONTEXT register.
    *  
    *  Input parameters: 
    *  	   a0 - context
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(_setcontext)

		MTC0	a0,C0_CTEXT
		j	ra
END(_setcontext)

/*  *********************************************************************
    *  _GETSEGTBL()
    *  
    *  Return the address of the segment table.  We use this
    *  to display the startup messages.
    *  
    *  You can't just address the table from C because it lives
    *  in the text segment.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *      address of table
    ********************************************************************* */


LEAF(_getsegtbl)
		la	v0,segment_table
		j	ra
END(_getsegtbl)


/*  *********************************************************************
    *  _wbflush()
    *  
    *  Flush the write buffer.  This is probably not necessary
    *  on SiByte CPUs, but we have it for completeness.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(_wbflush)

		sync			/* drain the buffers */
		la	t0,__junk	/* do an uncached read to force it out */
		or	t0,K1BASE
		lw	zero,0(t0)
		j	ra	

END(_wbflush)


/*  *********************************************************************
    *  CFE_FLUSHCACHE
    *  
    *  Perform certain cache operations
    *  
    *  Input parameters: 
    *  	   a0 - flags (CFE_CACHE_xxx flags, or zero for a default)
    *      a1,a2 - start/end of range for "range invalidate" operations
    *      (not used otherwise)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(_cfe_flushcache)

		sub	sp,56
		SREG	ra,0(sp)
		SREG	a0,8(sp)
		SREG	s0,16(sp)
		SREG	v1,24(sp)
		SREG	s1,32(sp)
		SREG	s2,40(sp)
		SREG	s3,48(sp)
		SREG	s4,56(sp)

		JAL(CPUCFG_CACHEOPS)

		LREG	s4,56(sp)
		LREG	s3,48(sp)
		LREG	s2,40(sp)
		LREG	s1,32(sp)
		LREG	v1,24(sp)
		LREG	s0,16(sp)
		LREG	a0,8(sp)
		LREG	ra,0(sp)
		add	sp,56
		j	ra

END(_cfe_flushcache)


/*  *********************************************************************
    *  End
    ********************************************************************* */
