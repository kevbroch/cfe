/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  API entry module				File: zipstart_entry.S
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


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#include "mipsmacros.h"


#if CPUCFG_REGS32
#define LREG	lw
#define SREG	sw
#define SRL	srl
#define SLL	sll
#else
#define LREG	ld
#define SREG	sd
#define SRL	dsrl
#define SLL	dsll
#endif


#include "exception.h"



/*  *********************************************************************
    *  _exc_entry(k0)
    *  
    *  Main exception entry point.
    *  
    *  Input parameters: 
    *  	   k0 - exception type
    *  	   
    *  Return value:
    *  	   ...
    ********************************************************************* */

LEAF(_exc_entry)

		.set noreorder
		.set noat

		subu	k1,sp,EXCEPTION_SIZE
		SRL	k1,3
		SLL	k1,3

                SREG    zero,XGR_ZERO(k1)
		SREG 	AT,XGR_AT(k1)

		SREG	v0,XGR_V0(k1)
		SREG	v1,XGR_V1(k1)

		SREG	a0,XGR_A0(k1)
		SREG	a1,XGR_A1(k1)
		SREG	a2,XGR_A2(k1)
		SREG	a3,XGR_A3(k1)

		SREG	t0,XGR_T0(k1)
		SREG	t1,XGR_T1(k1)
		SREG	t2,XGR_T2(k1)
		SREG	t3,XGR_T3(k1)
		SREG	t4,XGR_T4(k1)
		SREG	t5,XGR_T5(k1)
		SREG	t6,XGR_T6(k1)
		SREG	t7,XGR_T7(k1)

		SREG	s0,XGR_S0(k1)
		SREG	s1,XGR_S1(k1)
		SREG	s2,XGR_S2(k1)
		SREG	s3,XGR_S3(k1)
		SREG	s4,XGR_S4(k1)
		SREG	s5,XGR_S5(k1)
		SREG	s6,XGR_S6(k1)
		SREG	s7,XGR_S7(k1)

		SREG	t8,XGR_T8(k1)
		SREG	t9,XGR_T9(k1)

		SREG	gp,XGR_GP(k1)
		SREG	sp,XGR_SP(k1)
		SREG	fp,XGR_FP(k1)
		SREG	ra,XGR_RA(k1)

		mfc0	t0,C0_CAUSE
		mfc0	t1,C0_SR
		MFC0	t2,C0_BADVADDR
		MFC0	t3,C0_EPC
		mfc0    t4,C0_PRID
		mflo    t5
		mfhi    t6	
		SREG	t0,XCP0_CAUSE(k1)
		SREG	t1,XCP0_SR(k1)
		SREG	t2,XCP0_VADDR(k1)
		SREG	t3,XCP0_EPC(k1)
		SREG	t4,XCP0_PRID(k1)
		SREG	t5,XGR_LO(k1)
		SREG	t6,XGR_HI(k1)

		la	gp,_gp			# Load up GP, not relocated so it's easy

		move	a0,k0			# Pass exception type 
		move	a1,k1			# Pass frame to exception handler
		move	sp,k1			# "C" gets fresh stack area

		jal	zs_exception
		 nop

		move    k1, sp
		LREG  	AT,XGR_AT(k1)

		LREG	t0,XGR_LO(k1)
		LREG	t1,XGR_HI(k1)
		mtlo    t0
		mthi    t1

		LREG	a0,XGR_A0(k1)
		LREG	a1,XGR_A1(k1)
		LREG	a2,XGR_A2(k1)
		LREG	a3,XGR_A3(k1)

		LREG	t0,XGR_T0(k1)
		LREG	t1,XGR_T1(k1)
		LREG	t2,XGR_T2(k1)
		LREG	t3,XGR_T3(k1)
		LREG	t4,XGR_T4(k1)
		LREG	t5,XGR_T5(k1)
		LREG	t6,XGR_T6(k1)
		LREG	t7,XGR_T7(k1)

		LREG	s0,XGR_S0(k1)
		LREG	s1,XGR_S1(k1)
		LREG	s2,XGR_S2(k1)
		LREG	s3,XGR_S3(k1)
		LREG	s4,XGR_S4(k1)
		LREG	s5,XGR_S5(k1)
		LREG	s6,XGR_S6(k1)
		LREG	s7,XGR_S7(k1)

		LREG	t8,XGR_T8(k1)
		LREG	t9,XGR_T9(k1)

		LREG	gp,XGR_GP(k1)
		LREG	sp,XGR_SP(k1)
		LREG	fp,XGR_FP(k1)
		LREG	ra,XGR_RA(k1)

/* do any CP0 cleanup here */

		LREG	v0,XGR_V0(k1)
		LREG	v1,XGR_V1(k1)
	
		ERET

		.set at
		.set reorder


END(_exc_entry)



/*  *********************************************************************
    *  End
    ********************************************************************* */
