 /*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Intel (X86) processor startup		File: protmode.h
    *  
    *  macros and functions particular to Intel X86 protected-mode
    *  features.
    *  
    *  Author:  Mitch Lichtenberg 
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2003
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

/*  *********************************************************************
    *  Descriptor stuff
    ********************************************************************* */

#define GDT_NULL_DESCR()  .word 0,0,0,0
#define GDT_CODE_DESCR()  .word 0xFFFF,0x0000,0x9B00,0x00CF
#define GDT_DATA_DESCR()  .word 0xFFFF,0x0000,0x9300,0x00CF

#define GDT_GDT_DESCR(ptr,cnt) .word ((cnt)*8-1) ; .long ptr

#define GDT_DESCR_SIZE	8

/*  *********************************************************************
    *  Selectors for our GDT
    ********************************************************************* */

#define SEG_NULL	0x0000
#define SEG_CODE	0x0008
#define SEG_DATA	0x0010

#define SEG_COUNT	3

/*  *********************************************************************
    *  Macros to construct instructions
    ********************************************************************* */

#define OP32 .byte 0x66 ;

#define JMPFAR32(x) OP32 ; .byte 0xEA ; .long x ; .word SEG_CODE

/*  *********************************************************************
    *  Intel control registers
    ********************************************************************* */

/*
 * CR0 - Protected Mode control
 */

#define CR0_PG		0x80000000		/* Paging */
#define CR0_CD		0x40000000		/* Cache Disabled */
#define CR0_NW		0x20000000		/* Not WriteThrough */
#define CR0_AM		0x00040000		/* Alignment Check */
#define CR0_WP		0x00010000		/* Write Protect */
#define CR0_MBO		0x00000010		/* always 1 */
#define CR0_TS		0x00000008		/* Task Switch */
#define CR0_EM		0x00000004		/* x87 FPU trapped */
#define CR0_MP		0x00000002		/* WAIT/FWAIT trapped */
#define CR0_PE		0x00000001		/* Protection Enable */


