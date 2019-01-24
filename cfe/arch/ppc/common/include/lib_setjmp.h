/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  setjmp/longjmp defs			File: lib_setjmp.h
    *  
    *  Description of the jmp_buf structure for setjmp and longjmp
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

#ifndef _LIB_SETJMP_H
#define _LIB_SETJMP_H

/*
 * Note that while lib_setjmp() and lib_longjmp() behave like setjmp()
 * and longjmp() normally do, gcc 3.1.x (and later) assumes things about
 * how setjmp() and longjmp() should work (even with -fno-builtins).  We
 * don't want it to do those, so these functions must be named differently.
 */

#ifdef __ASSEMBLER__
#define _JBIDX(x)	(8*(x))
#else
#define _JBIDX(x)	(x)
#endif

#define JMPB_R1  	_JBIDX(0)
#define JMPB_R14 	_JBIDX(1)
#define JMPB_R15 	_JBIDX(2)
#define JMPB_R16 	_JBIDX(3)
#define JMPB_R17 	_JBIDX(4)
#define JMPB_R18 	_JBIDX(5)
#define JMPB_R19 	_JBIDX(6)
#define JMPB_R20 	_JBIDX(7)
#define JMPB_R21 	_JBIDX(8)
#define JMPB_R22 	_JBIDX(9)
#define JMPB_R23 	_JBIDX(10)
#define JMPB_R24 	_JBIDX(11)
#define JMPB_R25 	_JBIDX(12)
#define JMPB_R26 	_JBIDX(13)
#define JMPB_R27 	_JBIDX(14)
#define JMPB_R28 	_JBIDX(15)
#define JMPB_R29 	_JBIDX(16)
#define JMPB_R30 	_JBIDX(17)
#define JMPB_R31 	_JBIDX(18)
#define JMPB_LR  	_JBIDX(20)
#define JMPB_CR  	_JBIDX(21)
#define JMPB_CTR 	_JBIDX(22)
#define JMPB_XER 	_JBIDX(23)
#define JMPB_SIG 	_JBIDX(24)


#define JMPB_SIZE       _JBIDX(25)

#ifndef __ASSEMBLER__
typedef long long jmp_buf[JMPB_SIZE];
extern int lib_setjmp(jmp_buf);
extern void lib_longjmp(jmp_buf,int val);
#endif

#endif
