 /*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Intel (X86) processor startup		File: protmode.S
    *  
    *  This module contains code to switch to protected mode.
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

#include "protmode.h"

	 .section .pminit_text,"xr"


	.align 32

/*  *********************************************************************
    *  ROM GDT
    *  
    *  This GDT is used while we're in protected mode but have not
    *  relocated to RAM yet.  
    ********************************************************************* */

rom_gdt:       
	GDT_NULL_DESCR()		/* null descriptor */
	GDT_CODE_DESCR()		/* Code segment (CS=0x0008) */
	GDT_DATA_DESCR()		/* Data/stack/extra (DS/ES/SS=0x0010) */

rom_gdt_descr: 
	GDT_GDT_DESCR(rom_gdt,SEG_COUNT) /* descriptor for GDT itself */


/*  *********************************************************************
    *  16-bit startup code.  This code switches us into
    *  protected mode.  It's kind of fragile, due to GCC's
    *  limited support for 16-bit code segments.   If this
    *  code is modified, pay particular attention to any
    *  32-bit constants or offsets you use and be sure that
    *  the OP32 prefix is applied to get the correct CPU
    *  addressing mode from a 16-bit segment.
    ********************************************************************* */
	

	.align 16   /* Align on paragraph boundary for 16-bit code*/
	.code16

	.global cpu_prot32
cpu_prot32:

	/*
	 * Set up the Global Descriptor Table (GDT)
	 */


        OP32 lgdt    %cs:rom_gdt_descr

	/* 
	 * Switch on the PE bit in CR0 to enable protected mode
	 */

        movl    %cr0,%eax   
        orb     $CR0_PE,%al 
        movl    %eax,%cr0   

	/*
	 * Do a FAR jump to the next instruction to load the
	 * new code segment selector into CS, then a near
	 * jump to flush the prefetch queue.
	 */

	JMPFAR32(1f)
1:	jmp	2f
2:

	/*
	 * Okay, we're in protected mode.  Set up the
	 * data and stack segments and jump into the new world.
	 */

	.code32

	movl	$SEG_DATA,%eax
	movw	%ax,%ds
	movw	%ax,%es
	movw	%ax,%fs
	movw	%ax,%gs
	movw	%ax,%ss


	jmp	cfe_init32


