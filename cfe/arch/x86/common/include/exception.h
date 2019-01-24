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

#define XTYPE_DIVBYZERO	0
#define XTYPE_DEBUG	1
#define XTYPE_NMI	2
#define XTYPE_BREAKPOINT 3
#define XTYPE_OVERFLOW	4
#define XTYPE_BOUND	5
#define XTYPE_INVOPCODE 6
#define XTYPE_DEVNOTAVL	7
#define XTYPE_DBLFAULT	8
#define XTYPE_COPOVERRUN 9
#define XTYPE_INVTSS	10
#define XTYPE_SEGNOTPRES 11
#define XTYPE_STACKFAULT 12
#define XTYPE_GENPROT	13
#define XTYPE_PAGEFAULT 14
#define XTYPE_RESERVED	15
#define XTYPE_FPU	16
#define XTYPE_ALIGN	17
#define XTYPE_MACHINECHECK 18
#define XTYPE_SIMD	19


#define XGR_EDI 	_XAIDX(0)
#define XGR_ESI 	_XAIDX(1)
#define XGR_EBP 	_XAIDX(2)
#define XGR_ESP 	_XAIDX(3)
#define XGR_EBX 	_XAIDX(4)
#define XGR_EDX 	_XAIDX(5)
#define XGR_ECX 	_XAIDX(6)
#define XGR_EAX 	_XAIDX(7)

#define XGR_EIP		_XAIDX(8)
/*#define XGR_CODE	_XAIDX(9)*/



/*#define XGR_ESP		_XAIDX(0)*/


#ifndef __ASSEMBLER__
void cfe_exception(int code,uint32_t *info);
void cfe_setup_exceptions(void);
void _exc_restart(void);
#endif
