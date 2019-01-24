 /*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Intel (X86) processor startup		File: altcpu.S
    *  
    *  This module contains code to start the other CPU(s).  The basic
    *  procedure used here is outlined in the Intel System Software
    *  Writer's Guide (SSWG) section 7.5.
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
#include "x86macros.h"


/*  *********************************************************************
    *  Constants
    ********************************************************************* */

#define APIC_ID		0xFEE00020
#define APIC_ICR_LOW	0xFEE00300
#define APIC_SVR	0xFEE000F0
#define APIC_ENABLED	0x00000100
#define APIC_LVT3	0xFEE00370

#define ALTCPU_MAX_CPUS	8
#define ALTCPU_STACK_SIZE 256

/*
 * If you change the vector below, you also need to change the
 * linker script's load address for the ALTCPU_TEXT section
 * to be whatever the (vector << 12) value is.
 */

#define AP_START_VECTOR	0x22


/*  *********************************************************************
    *  Part of this code lives in a special segment, since we
    *  need to move it into the low 1MB of memory where it can
    *  be run in real-mode.  After this part runs, we can go back
    *  to the real text segment again.
    ********************************************************************* */

	.data

	.extern ram_gdt_descr

	.comm altcpu_stacks,(ALTCPU_MAX_CPUS*ALTCPU_STACK_SIZE)

	.section .altcpu_text,"xr"


/*  *********************************************************************
    *  16-bit startup code.  This code switches us into
    *  protected mode.  It's kind of fragile, due to GCC's
    *  limited support for 16-bit code segments.   If this
    *  code is modified, pay particular attention to any
    *  32-bit constants or offsets you use and be sure that
    *  the OP32 prefix is applied to get the correct CPU
    *  addressing mode from a 16-bit segment.
    ********************************************************************* */

#define ALTCPU_GDT_OFFSET   0x40
	

	.align 16   /* Align on paragraph boundary for 16-bit code*/

	.code16

	.global altcpu_initvec
altcpu_initvec:

	/*
	 * Set up the Global Descriptor Table (GDT)
	 */

	xorl    %esi,%esi
	movw	$ALTCPU_GDT_OFFSET,%si
	lgdt %cs:(%si)


	/* 
	 * Switch on the PE bit in CR0 to enable protected mode
	 */

	 movb	$0xFF,%al
	 outb	%al,$0x80

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

	jmp	altcpu_init32




/*  *********************************************************************
    *  ALTCPU ROM GDT
    *  
    *  It is located at a _magic_ place,
    *  so we can predict where it is relative to our current
    *  code segment.  
    *
    *  Unfortunately, the actual location of this module
    *  lives in the linker script, and is also dependent on
    *  the vector we choose to start up our CPUs (currently 0x22)
    ********************************************************************* */

	.org    ALTCPU_GDT_OFFSET

/*
 * Descriptor for GDT 
 */

altcpurom_gdt_descr: 
	GDT_GDT_DESCR(altcpurom_gdt,SEG_COUNT) /* descriptor for GDT itself */

/*
 * GDT itself
 */

	.align 16

altcpurom_gdt:       
	GDT_NULL_DESCR()		/* null descriptor */
	GDT_CODE_DESCR()		/* Code segment (CS=0x0008) */
	GDT_DATA_DESCR()		/* Data/stack/extra (DS/ES/SS=0x0010) */


	.byte 0xEE,0xAA,0xEE,0xAA	/* stuff to find in the S-records */

/*  *********************************************************************
    *  Data used by our startup code
    ********************************************************************* */

        .data

cpu_sem:
	.long	0			/* Startup semaphore */

cpu_count:
	.long	1			/* start with ourselves */

cpu_curstack:
	.long	altcpu_stacks

	.globl cpu_ids
cpu_ids:
        .long	0			/* Filled in with APIC IDs */
	.long	0
	.long	0
	.long	0
	.long	0
	.long	0
	.long	0
	.long	0

/*  *********************************************************************
    *  Here is the rest of the startup for the other CPUs,
    *  this part goes in with the rest of the relocated CFE code.
    ********************************************************************* */

        .text


altcpu_init32:

	/*
	 * Reload the GDT from the "real" place.
	 *
	 * Also, load the IDT so we can take exceptions.
	 */

	lgdt %ds:ram_gdt_descr
	lidt %ds:ram_idt_ptr

	/*
	 * Wait on the startup semaphore so we can be the only
	 * one to update the CPU table.
	 */

altcpu_wait:
	movl	$1,%eax			/* Test-and-set semaphore */
	xchgl	%eax,cpu_sem		/* See if we got it. */
	or	%eax,%eax		/* Was old value zero? */
	jne	altcpu_wait		/* Nope, go back and keep waiting */

	/*
	 * Initialize MTRRs, but only once per CPU cluster.  The
	 * threads on an HT Xeon share the MTRRs, but the
	 * individual chips do not (of course).
	 *
	 * (is this check necessary? --- yes, it seems to be.  Boot
	 * of secondary CPUs is unreliable without it)
	 */

	movl	$APIC_ID,%esi
	movl	(%esi),%eax		/* get our APIC ID */
	movl	(%esi),%eax		/* get our APIC ID */
	shrl	$24,%eax		/* move APIC ID to low bit */
	test	$1,%eax			/* second CPU on HT chip? */
	jnz	is_ht_cpu1		/* Don't bother with MTRR init */

	BAL_ESP(cpu_init)

       /*
        * Load processor microcode  (belongs somewhere else).  The
        * processor microcode lives at a fixed place in the flash so
        * we can stuff it in *after* CFE is built, this way we don't
        * have to distribute the microcode with CFE itself, avoiding
        * possible Intel proprietary stuff (vendor must supply
        * microcode themselves)
        */

	movl	$0xFFFF0000,%esi	/* this is where it is */
	movl	$(16*2048),%eax		/* 16 blocks max */
	BAL_ESP(cpu_load_ucode)


is_ht_cpu1:

	/*
	 * Init the caches.  We do this on both threads of HT
	 * gear, since once the CPU is out of reset, both have
	 * to say that the cache is enabled for it to really
	 * be enabled.
	 */

#if CFG_INIT_L1
	movl	%cr0,%eax
	andl	$(~(CR0_CD|CR0_NW)),%eax
	movl	%eax,%cr0
	invd

#endif



	/*
	 * Okay, we have the lock.  Add us to the table.
	 */

	movl	$APIC_ID,%esi
	movl	(%esi),%eax
	andl	$0xFF000000,%eax	/* save only APIC ID info */

	movl	cpu_count,%ebx		/* Get current count */
	movl	$cpu_ids,%esi
	movl	%eax,(%esi,%ebx,4)	/* Write at current offset */
	incl	%ebx
	movl	%ebx,cpu_count		/* Write back new index */

	/*
	 * Initialize local APIC
	 */

	/*
	 * Initialize SMI environment
	 */

	/*
	 * Initialize stack - we will wake up via
	 * an NMI and need somewhere to push the 
	 * return address.
	 */

	addl	$ALTCPU_STACK_SIZE,cpu_curstack
	movl	cpu_curstack,%esp

	/*
	 * Release semaphore.
	 */

	 movl	$0,%eax
	 xchgl	%eax,cpu_sem

	/*
	 * Now, wait for soemone to hit us with an INIT IPI or an NMI
	 */

1:	cli
	hlt
	nop
	nop
	jmp	1b


	
/*  *********************************************************************
    *  altcpu_startup
    *  
    *  This routine kicks off the CPU startup.  It is called after
    *  DRAM is initialized and the stack set up, so we can
    *  do fancy stuff like push and pop :-).
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   eax - number of CPUs detected
    ********************************************************************* */

         .globl altcpu_startup

altcpu_startup:

	pushl	%esi
	pushl	%edi
	pushl	%ebx
	pushl	%ecx
	pushl	%edx

	movl	$1,cpu_count
	movl	$altcpu_stacks,%eax
	movl	%eax,cpu_curstack

	/*
	 * Copy the code into lower DRAM.
	 */

	movl	$0xFFFFD000,%esi
	movl	$(AP_START_VECTOR << 12),%edi
	movl	$256,%ecx
	cld
	rep	movsb

	/*
	 * Flush cache to ensure code is in memory.
	 */

	wbinvd

	/*
	 * Enable the local APIC
	 */

	movl	$APIC_SVR,%esi
	movl	(%esi),%eax
	orl	$APIC_ENABLED,%eax
	movl	%eax,(%esi)

	movl	$APIC_LVT3,%esi
	movl	(%esi),%eax
	andl	$0xFFFFFF00,%eax
	orl	$0x66,%eax
	movl	%eax,(%esi)


	/*
	 * Whack the APIC to get the APs going
	 */	

	movl	$APIC_ICR_LOW,%esi
	movl	$0x000C4500,%eax
	movl	%eax,(%esi)

	/*
	 * Delay (200ms)
	 */

	pushl	$2
	call	cfe_sleep
	addl	$4,%esp

	/*
	 * Send out SIPI IPI
	 */

	movl	$APIC_ICR_LOW,%esi
	movl	$(0x000C4600 | AP_START_VECTOR),%eax
	movl	%eax,(%esi)

	pushl	$1
	call	cfe_sleep
	addl	$4,%esp

	/*
	 * Send out SIPI IPI
	 */

	movl	$APIC_ICR_LOW,%esi
	movl	$(0x000C4600 | AP_START_VECTOR),%eax
	movl	%eax,(%esi)

	/*
	 * Wait 200msec
	 */

	pushl	$5
	call	cfe_sleep
	addl	$4,%esp

	popl	%edx
	popl	%ecx
	popl	%ebx
	popl	%edi
	popl	%esi

	movl	cpu_count,%eax

        ret

/* 
 * Make a reference to the altcpu_text segment, to force the 
 * linker to bring it in.  
 */

	movl	$altcpurom_gdt,%eax	

/*  *********************************************************************
    *  End
    ********************************************************************* */
