/*  *********************************************************************
    *  PowerPC CPU support
    *  
    *  Exception Handler			File: exception.S
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


#include "ppcdefs.h"
#include "exception.h"
#include "ppcmacros.h"
#include "cpu_config.h"		/* for definition of HAZARD and ERET */
#include "bsp_config.h"

/*  *********************************************************************
    *  Macros
    ********************************************************************* */


/*  *********************************************************************
    *  Data
    ********************************************************************* */

                .data


/*  *********************************************************************
    *  Code
    ********************************************************************* */

		.text


/*  *********************************************************************
    *  _exc_restart()
    *  
    *  Restart the firmware at the boot address
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(_exc_restart)

		b	CPUCFG_CPURESTART

END(_exc_restart)

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

	/*
	 * For now, we're going to build the exception frame
	 * on the stack.  This is ugly, since the stack might be
	 * the problem in the first place...
	 *
	 * r1 is our stack pointer.
	 */

		subi	r1,r1,EXCEPTION_SIZE

		stw	r0,XGR_R0(r1)

		mfmsr	r0 
		stw	r0,XGR_MSR(r1)
		mflr	r0
		stw	r0,XGR_LR(r1)
		mfctr	r0
		stw	r0,XGR_CTR(r1)
		mfxer	r0
		stw	r0,XSPR_XER(r1)
		mfspr	r0,SPR_SRR0
		stw	r0,XSPR_SRR0(r1)
		mfspr	r0,SPR_SRR1
		stw	r0,XSPR_SRR1(r1)
		mfspr	r0,SPR_DSISR
		stw	r0,XSPR_DSISR(r1)
		
		stw	r1,XGR_R1(r1)
		stw	r2,XGR_R2(r1)
		stw	r3,XGR_R3(r1)
		stw	r4,XGR_R4(r1)
		stw	r5,XGR_R5(r1)
		stw	r6,XGR_R6(r1)
		stw	r7,XGR_R7(r1)
		stw	r8,XGR_R8(r1)
		stw	r9,XGR_R9(r1)
		stw	r10,XGR_R10(r1)
		stw	r11,XGR_R11(r1)
		stw	r12,XGR_R12(r1)
		stw	r13,XGR_R13(r1)
		stw	r14,XGR_R14(r1)
		stw	r15,XGR_R15(r1)
		stw	r16,XGR_R16(r1)
		stw	r17,XGR_R17(r1)
		stw	r18,XGR_R18(r1)
		stw	r19,XGR_R19(r1)
		stw	r20,XGR_R20(r1)
		stw	r21,XGR_R21(r1)
		stw	r22,XGR_R22(r1)
		stw	r23,XGR_R23(r1)
		stw	r24,XGR_R24(r1)
		stw	r25,XGR_R25(r1)
		stw	r26,XGR_R26(r1)
		stw	r27,XGR_R27(r1)
		stw	r28,XGR_R28(r1)
		stw	r29,XGR_R29(r1)
		stw	r30,XGR_R30(r1)
		stw	r31,XGR_R31(r1)

	/*
	 * Okay, the exception code was stored in r0 and the original r0
	 * was stored in SPRG1.  The original LR was stored
	 * in SPRG0.  Fix things up.
	 */

		mfspr   r0,SPR_SPRG0
		mtlr	r0
		stw	r0,XGR_LR(r1)
		lwz	r3,XGR_R0(r1)
		mfspr	r0,SPR_SPRG1
		stw	r0,XGR_R0(r1)
		mr	r4,r1

	/*
	 * Reenable address translation, was disabled by the
	 * exception mechanism.
	 */

		mfmsr r0
		ori   r0,r0,(M_MSR_IR|M_MSR_DR)
		mtmsr r0

	/*
	 * R3 (arg1) is the exception vector number, and
	 * R4 (arg2) is the exception frame (stack pointer).
	 */

		bl	cfe_exception


	/* 
	 * Stack pointer should still be valid.  Restore everything.
	 */		


		lwz	r0,XGR_MSR(r1)
		mtmsr	r0 

		lwz	r0,XGR_LR(r1)
		mtlr	r0

		lwz	r0,XGR_CTR(r1)
		mtctr	r0

		lwz	r0,XSPR_XER(r1)
		mtxer	r0

		lwz	r1,XGR_R1(r1)
		lwz	r2,XGR_R2(r1)
		lwz	r3,XGR_R3(r1)
		lwz	r4,XGR_R4(r1)
		lwz	r5,XGR_R5(r1)
		lwz	r6,XGR_R6(r1)
		lwz	r7,XGR_R7(r1)
		lwz	r8,XGR_R8(r1)
		lwz	r9,XGR_R9(r1)
		lwz	r10,XGR_R10(r1)
		lwz	r11,XGR_R11(r1)
		lwz	r12,XGR_R12(r1)
		lwz	r13,XGR_R13(r1)
		lwz	r14,XGR_R14(r1)
		lwz	r15,XGR_R15(r1)
		lwz	r16,XGR_R16(r1)
		lwz	r17,XGR_R17(r1)
		lwz	r18,XGR_R18(r1)
		lwz	r19,XGR_R19(r1)
		lwz	r20,XGR_R20(r1)
		lwz	r21,XGR_R21(r1)
		lwz	r22,XGR_R22(r1)
		lwz	r23,XGR_R23(r1)
		lwz	r24,XGR_R24(r1)
		lwz	r25,XGR_R25(r1)
		lwz	r26,XGR_R26(r1)
		lwz	r27,XGR_R27(r1)
		lwz	r28,XGR_R28(r1)
		lwz	r29,XGR_R29(r1)
		lwz	r30,XGR_R30(r1)
		lwz	r31,XGR_R31(r1)

		lwz	r0,XGR_R0(r1)

		rfi


END(_exc_entry)



LEAF(_exc_continue)

		mfmsr	r0			/* set the RI bit in MSR to make us recoverable */
		LDCONST(r4,M_MSR_RI)
		or	r4,r4,r0
		mtmsr	r4

		mtspr	SPR_SRR0,r3
		rfi

END(_exc_continue)

	

/*  *********************************************************************
    *  End
    ********************************************************************* */


