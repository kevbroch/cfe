 /*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Intel (X86) processor startup		File: init_x86.S
    *  
    *  This module contains the stuff that gets CFE up and running
    *  on an X86 processor.
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

#include "segtable.h"
#include "promice.h"
#include "bsp_config.h"
#include "x86macros.h"
#include "protmode.h"

		.text
		.align 0

		.data

/*  *********************************************************************
    *  Segment table.  Mostly a carryover from the MIPS embedded PIC
    *  world, but this holds some interesting addresses for the rest
    *  of CFE.
    ********************************************************************* */

		.globl segtable
segtable:
		.long	_etext		/* R_SEG_ETEXT */
		.long	_fdata		/* R_SEG_FDATA */
		.long	_edata		/* R_SEG_EDATA */
		.long	_end		/* R_SEG_END */
		.long	_ftext		/* R_SEG_FTEXT */
		.long	_fbss		/* R_SEG_FBSS */
		.long	0		/* R_SEG_APIENTRY */


/*  *********************************************************************
    *  GDT we'll use once we move to DRAM
    *
    *  It is exactly the same as the ROM one, except located 
    *  in DRAM (no expensive cycles to load descriptors).
    ********************************************************************* */

	.align 32

ram_gdt:       
	GDT_NULL_DESCR()		/* null descriptor */
	GDT_CODE_DESCR()		/* Code segment (CS=0x0008) */
	GDT_DATA_DESCR()		/* Data/stack/extra (DS/ES/SS=0x0010) */

	.globl ram_gdt_descr
ram_gdt_descr: 
	GDT_GDT_DESCR(ram_gdt,SEG_COUNT) /* descriptor for GDT itself */


/*  *********************************************************************
    *  Globals
    ********************************************************************* */

#define GLOBAL_WORD(x) \
	.global x ; x: .long 0
#define GLOBAL_WORD_DEFAULT(x,y) \
	.global x ; x: .long y

GLOBAL_WORD_DEFAULT(cpu_prid,       0)
GLOBAL_WORD_DEFAULT(mem_totalsize,  128)
GLOBAL_WORD_DEFAULT(mem_bottomofmem,0x00000000)
GLOBAL_WORD_DEFAULT(mem_topofmem,   0x007fffff)
GLOBAL_WORD_DEFAULT(mem_datareloc,  0)
GLOBAL_WORD_DEFAULT(mem_heapstart,  0x00180000)
GLOBAL_WORD_DEFAULT(mem_textbase,   0x00000000)
GLOBAL_WORD_DEFAULT(mem_textsize,   0x00100000)
GLOBAL_WORD_DEFAULT(mem_textreloc,  0)

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#define OUTLEDS(x) \
	movb $x,%al ; \
	outb %al,$0x80


/* #define _DUMPIT_ */

/*  *********************************************************************
    *  Code
    ********************************************************************* */
		

		.text

/*
 * Skip the first 32 bytes of the flash.  We'll put the PromICE
 * AI2 port there and maybe other stuff.
 */


		.org	0
		.long 0,0,0,0,0,0,0,0

/*  *********************************************************************
    *  Switch to the ".pminit_text" section where our 
    *  processor and memory init code lives
    ********************************************************************* */

		.section .pminit_text,"xr"

		.extern cfe_main


/*  *********************************************************************
    *  cfe_init32
    *  
    *  This is the 32-bit CPU entry point.  Execeution comes
    *  here as soon as we're in protected mode.
    *  
    *  Input parameters: 
    *  	   nothing - processor is in protected mode
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */


		.globl cfe_init32
cfe_init32:

    /*
     * Jump into board_earlyinit to do some early stuff.  That routine will
     * jump back here.
     */

	jmp	board_earlyinit
	.globl board_earlyinit_ret
board_earlyinit_ret:



    /*
     * Initialize memory.- memory size is returned in 
     * megabytes in %eax.
     */

	jmp	board_draminit
	.global board_draminit_ret
board_draminit_ret:

    /*
     * Save amount of memory returned - eventually we'll write this
     * into a global variable.
     */

	movl	%eax,%ebp			/* Hopefully nobody uses this 
						   between here and initvars*/

	OUTSTR("Memory init completed\r\n")

    /*
     * Reload the GDT from our RAM-based version.  Then,
     * read/write CR0 to flush the on-chip TLBs.
     */

	lgdt	%ds:ram_gdt_descr
	movl	%cr0,%eax
	movl	%eax,%cr0


	BAL_ESP(cpu_init)
	OUTSTR("CPU init complete\r\n")


    /*
     * Move stuff into RAM, as follows:
     *
     * If "BOOTRAM" mode, we're running from PromICE space (writable ROM)
     * so don't do anything.
     * 
     * If "RELOC" mode, copy both code and data to RAM.
     *
     * If non-RELOC, the code is in the flash and the data is in the RAM
     * so copy just the data.
     */


#if (!CFG_BOOTRAM) && (!CFG_RELOC)
CopyDataSeg:
	movl    $_datarom,%esi
	movl	$_fdata,%edi
	movl	$_edata,%ecx
	sub	$_fdata,%ecx
	cld
	rep	movsb

	OUTSTR("Data segment copied to RAM\r\n")

    /* 
     * As we're paranoid about memory init, compare what
     * we just wrote and make sure it's really there.
     */

	movl    $_datarom,%esi
	movl	$_fdata,%edi
	movl	$_edata,%ecx
	sub	$_fdata,%ecx
	cld
	rep	cmpsb

	jz	goodcopy

#ifdef _DUMPIT_
        jmp	dump_memory
#else
	jmp	badcopy
#endif

goodcopy:
#endif

    /*
     * Normal case - move code and data into
     * DRAM.
     */


#if (CFG_RELOC)	/* implied !CFG_BOOTRAM */
__CopyAll:
	movl    $_textrom,%esi		/* ROM address, 0xFFF80000 */
	movl	$_textram,%edi		/* RAM address, from makefile */
	movl	$_edata,%ecx		/* Compute # of bytes to copy */
	subl	$_textram,%ecx		/* end of initdata minus RAM addr */

    /*
     * Our PromICE area is at the beginning of the
     * flash.  Don't disturb it here or we'll screw up
     * the AI2 port.
     */

#ifdef _PROMICE_AI2_
	addl	$32,%esi
	addl	$32,%edi
	subl	$32,%ecx
#endif

    /*
     * Move code and data to memory.
     */

	cld
	rep	movsb

	OUTSTR("Code and data copied to RAM\r\n")

    /*
     * Flush the cache to push the instructions into
     * memory.  This may not really be necssary on X86
     * processors but do it anyway.
     */

	wbinvd
#endif


    /*
     * Clear BSS
     */
__ClearBSS:
	movl	$_fbss,%edi		/* BSS is from "_fbss" */
	movl	$_end,%ecx		/* to the end of the image. */
	subl	%edi,%ecx
	xorl	%eax,%eax		/* Just write zeroes */
	cld
	rep	stosb

    /*
     * Set up heap and other pointers to things.
     */

__InitVars:

	movl	$1,%eax
	cpuid
	movl	%eax,cpu_prid		/* Basic processor type */

        movl    %ebp,mem_totalsize
	movl	$_end,%eax

	movl	%eax,mem_heapstart	/* Heap is just after _end */
	movl	$_etext,%eax

	subl	$_ftext,%eax
	movl	%eax,mem_textsize	/* Text size is difference of _ftext, _etext */

	movl	$_ftext,%eax
	movl	%eax,mem_textbase	/* Text base is just _ftext */

#if (CFG_RELOC)
	movl	$_ftext,%eax
#else
	movl	$_fdata,%eax
#endif
	movl	%eax,mem_bottomofmem	/* Bottom of memory is either beginning of
					   code (reloc) or data (nonreloc) */

#if CFG_BOOTRAM
	movl	$0xFFFFFC00,%esp	/* point stack/heap into this code area */
	movl	$0xFFFFFC00,%ebp	/* since we don't need it anymore. */
#else
	movl	$_end,%esp
	addl	$(CFG_HEAP_SIZE*1024+CFG_STACK_SIZE),%esp
	andl	$0xFFFFFFF0,%esp
	movl	%esp,%ebp
#endif

	movl	%esp,mem_topofmem	/* Top of stack is top of CFE's mem */

    /*
     * Silly test: call a local routine to make sure
     * memory is working.
     */

 	call	testret 
	OUTLEDS(0x03)

    /*
     * initialize the IDT, needed for handling exceptions.
     */

        call	__init_idt
	OUTLEDS(0x07)

 
    /*
     * It's time to jump to "C" code.
     */

	jmp	cfe_main


testret:
	OUTLEDS(0)	
	ret

badcopy:
	OUTSTR("Memory init failure: Data segment does not compare\r\n")
	OUTLEDS(0xFD)

forever: jmp	forever


/*  *********************************************************************
    *  cpu_init
    *  
    *  Initialize CPU-specific stuff
    *  
    *  Input parameters: 
    *  	   nothing
    *      esp - return address
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

        .global cpu_init
cpu_init:

    /*
     * Initialize the MTRRs to zero.
     */

	xorl	%eax,%eax
	xorl	%edx,%edx
	xorl	%ecx,%ecx
	movl	$mtrrtable,%esi		/* Write zeroes to each of these */
1:	movw	(%esi),%cx
	jcxz	2f
	wrmsr
	add	$2,%esi
	jmp	1b
2:

    /*
     * Initialize MTRRs for our normal running system.
     */

#if CFG_INIT_L1    
	movl	$0x06060606,%eax		/* Use fixed MTRRs to set */
	movl	$0x06060606,%edx		/* Lower 640K to "writeback" */
	movl	$0x250,%ecx
	wrmsr
	movl	$0x258,%ecx			/* two MTRRs control lower 640K */
	wrmsr

	/* 
	 * Set a variable MTRR to handle the rest of RAM above 1MB 
	 * XXX Note that this needs to change when RAM size changes!
	 */
	movl	$0,%edx		  /* Physical base 0x00000000 */
	movl	$0x06,%eax	  /* Write-back mode */
	movl	$0x200,%ecx
	wrmsr

	movl	$0,%edx
	movl	%ebp,%eax	  /* EDX:EAX = memory size in MB */
	decl	%eax		  /* convert to mask */
	not	%edx
	not	%eax
	shld	$20,%eax,%edx	  /* Shift into place */
	shll	$20,%eax	  /* shift lower half too */
	orl	$(1<<11),%eax	  /* Set 'valid' bit */
	and	$0x0F,%edx	  /* we only have 36-bit PAs */
	inc	%ecx
	wrmsr


    /*
     * Invalidate TLBs.
     */
	invd
	movl    %cr3,%eax
	movl	%eax,%cr3

    /* 
     * Enable the MTRRs and set the default memory type to "uncached" 
     */

	movl	$((1<<10)|(1<<11)|(0)),%eax	/* turn on MTRRs */
	xorl	%edx,%edx
	movl	$0x2ff,%ecx
	wrmsr

#endif



    /*
     * Now enable the cache.  We don't need to initialize it
     * on an Intel processor, just clear the bits in CR0.
     */

#if CFG_INIT_L1
	movl	%cr0,%eax
	andl	$(~(CR0_CD|CR0_NW)),%eax
	movl	%eax,%cr0
	invd

#endif

	jmp	*%esp


/*  *********************************************************************
    *  MTRR table - this is the list of MSRs to zero as we get the
    *  CPU ready to go.  It is a zero-terminated list of 16-bit words.
    *  It's read-only, so we'll stick it in with the code.
    ********************************************************************* */

mtrrtable:
	.word 0x200,0x201,0x202,0x203,0x204,0x205,0x206,0x207
	.word 0x208,0x209,0x20a,0x20b,0x20c,0x20d,0x20e,0x20f
	.word 0x250,0x258,0x259,0x268,0x269,0x26a,0x26b,0x26c
	.word 0x26d,0x26e,0x26f,0x2ff
	.word 0



/*  *********************************************************************
    *  end
    ********************************************************************* */
