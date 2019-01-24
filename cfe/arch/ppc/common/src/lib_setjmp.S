/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Simple setjmp/longjmp			File: lib_setjmp.S
    *  
    *  A very simple SETJMP and LONGJMP
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
#include "lib_setjmp.h"

#define SREG stw
#define LREG lwz

	.text

LEAF(lib_setjmp)

        SREG	31,JMPB_R31(3)
        SREG	1,JMPB_R1(3)
        SREG	14,JMPB_R14(3)
        SREG	15,JMPB_R15(3)
        SREG	16,JMPB_R16(3)
        SREG	17,JMPB_R17(3)
        SREG	18,JMPB_R18(3)
        SREG	19,JMPB_R19(3)
        SREG	20,JMPB_R20(3)
        SREG	21,JMPB_R21(3)
        SREG	22,JMPB_R22(3)
        SREG	23,JMPB_R23(3)
        SREG	24,JMPB_R24(3)
        SREG	25,JMPB_R25(3)
        SREG	26,JMPB_R26(3)
        SREG	27,JMPB_R27(3)
        SREG	28,JMPB_R28(3)
        SREG	29,JMPB_R29(3)
        SREG	30,JMPB_R30(3)
        mfcr	0
        SREG	0,JMPB_CR(3)
        mflr	0
        SREG	0,JMPB_LR(3)
        mfctr	0
        SREG	0,JMPB_CTR(3)
        mfxer	0
        SREG	0,JMPB_XER(3)
        li	3,0
        blr

END(lib_setjmp)

LEAF(lib_longjmp)

        LREG	31,JMPB_R31(3)
        LREG	1,JMPB_R1(3)
        LREG	14,JMPB_R14(3)
        LREG	15,JMPB_R15(3)
        LREG	16,JMPB_R16(3)
        LREG	17,JMPB_R17(3)
        LREG	18,JMPB_R18(3)
        LREG	19,JMPB_R19(3)
        LREG	20,JMPB_R20(3)
        LREG	21,JMPB_R21(3)
        LREG	22,JMPB_R22(3)
        LREG	23,JMPB_R23(3)
        LREG	24,JMPB_R24(3)
        LREG	25,JMPB_R25(3)
        LREG	26,JMPB_R26(3)
        LREG	27,JMPB_R27(3)
        LREG	28,JMPB_R28(3)
        LREG	29,JMPB_R29(3)
        LREG	30,JMPB_R30(3)
        LREG	0,JMPB_CR(3)
        mtcr	0
        LREG	0,JMPB_LR(3)
        mtlr	0
        LREG	0,JMPB_CTR(3)
        mtctr	0
        LREG	0,JMPB_XER(3)
        mtxer	0
        mr	3,4
        blr

END(lib_longjmp)

