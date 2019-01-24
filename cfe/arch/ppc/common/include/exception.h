/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Exception/trap handler defs		File: exception.h
    *  
    *  This module describes the exception handlers, exception
    *  trap frames, and dispatch.
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

#ifdef __ASSEMBLER__
#define _XAIDX(x)	(4*(x))
#else
#define _XAIDX(x)	(x)
#endif



/*  *********************************************************************
    *  Fixed locations of other low-memory objects.  We stuff some
    *  important data in the spaces between the vectors.
    ********************************************************************* */


/*  *********************************************************************
    *  Exception frame definitions.
    ********************************************************************* */

/*
 * The exception frame is divided up into pieces, representing the different
 * parts of the processor that the data comes from:
 *
 * Int Regs:	Words 0..31
 * non-SPRs:	Words 32..34
 * SPRs:	Words 35..39
 * Total size:  40 words
 */

#define XSPR_BASE	35
#define XGR_BASE	0

#define _XGRIDX(x)	_XAIDX((x)+XGR_BASE)
#define XGR_R0       	_XGRIDX(0)
#define XGR_R1		_XGRIDX(1)
#define XGR_R2		_XGRIDX(2)
#define XGR_R3		_XGRIDX(3)
#define XGR_R4		_XGRIDX(4)
#define XGR_R5		_XGRIDX(5)
#define XGR_R6		_XGRIDX(6)
#define XGR_R7		_XGRIDX(7)
#define XGR_R8		_XGRIDX(8)
#define XGR_R9		_XGRIDX(9)
#define XGR_R10		_XGRIDX(10)
#define XGR_R11		_XGRIDX(11)
#define XGR_R12		_XGRIDX(12)
#define XGR_R13		_XGRIDX(13)
#define XGR_R14		_XGRIDX(14)
#define XGR_R15		_XGRIDX(15)
#define XGR_R16		_XGRIDX(16)
#define XGR_R17		_XGRIDX(17)
#define XGR_R18		_XGRIDX(18)
#define XGR_R19		_XGRIDX(19)
#define XGR_R20		_XGRIDX(20)
#define XGR_R21		_XGRIDX(21)
#define XGR_R22		_XGRIDX(22)
#define XGR_R23		_XGRIDX(23)
#define XGR_R24		_XGRIDX(24)
#define XGR_R25		_XGRIDX(25)
#define XGR_R26		_XGRIDX(26)
#define XGR_R27		_XGRIDX(27)
#define XGR_R28		_XGRIDX(28)
#define XGR_R29		_XGRIDX(29)
#define XGR_R30		_XGRIDX(30)
#define XGR_R31		_XGRIDX(31)

#define XGR_LR		_XGRIDX(32)
#define XGR_CTR		_XGRIDX(33)
#define XGR_MSR		_XGRIDX(34)

#define _XSPRIDX(x)	_XAIDX((x)+XSPR_BASE)
#define XSPR_XER	_XSPRIDX(0)
#define XSPR_SRR0	_XSPRIDX(1)
#define XSPR_SRR1	_XSPRIDX(2)
#define XSPR_DAR	_XSPRIDX(3)
#define XSPR_DSISR	_XSPRIDX(4)

#define EXCEPTION_SIZE	_XSPRIDX(5)


#ifdef __ASSEMBLER__
#define DECLARE_VECTOR(vec,target) \
     .org (vec) ; \
     mtspr SPR_SPRG1,r0 ; \
     mflr  r0 ; \
     mtspr SPR_SPRG0,r0 ; \
     li r0,(vec) ; \
     b target
#endif


#ifndef __ASSEMBLER__
extern void _exc_restart(void);
extern void _exc_continue(uint32_t addr);
void cfe_exception(int code,uint32_t *info);
void cfe_setup_exceptions(void);
#endif





