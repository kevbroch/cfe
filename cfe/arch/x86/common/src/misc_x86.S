/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Intel (X86) Misc Rotines			File: misc_x86.S
    *  
    *  Miscellaneous assembly routines that are used in the X86
    *  port of CFE.
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

#include "bsp_config.h"

#include "promice.h"

/*  *********************************************************************
    *  Constants
    ********************************************************************* */

#define IDT_ADDRESS   0

                .data

/*  *********************************************************************
    *  IDT descriptor
    ********************************************************************* */

#define IDT_MAX_VECTORS     40		/* reserve space for 40 vectors */
#define IDT_EXIT_VECTOR	    0x20	/* Fast exit to CFE, no args */
#define IDT_USERREQ_VECTOR  0x21	/* same as DOS -- I just couldn't resist! */

		.globl ram_idt_ptr
ram_idt_ptr:
	.word (IDT_MAX_VECTORS*8-1)
	.long IDT_ADDRESS

/*  *********************************************************************
    *  Exception Frame
    ********************************************************************* */

		.comm __exc_frame,(40*4),4

		.text

/*  *********************************************************************
    *  Interrupt Dispatch(es)
    ********************************************************************* */

#define EXHANDLER(num) \
__exception##num: ; \
	pushal ; \
	movl  $##num,%eax ; \
	jmp   __exc_common ; \

/*
 * XXX this isn't really right - we need to do something different
 * for exceptions that have error codes. 
 */


EXHANDLER(0)
EXHANDLER(1)
EXHANDLER(2)
EXHANDLER(3)
EXHANDLER(4)
EXHANDLER(5)
EXHANDLER(6)
EXHANDLER(7)
EXHANDLER(8)
EXHANDLER(9)
EXHANDLER(10)
EXHANDLER(11)
EXHANDLER(12)
EXHANDLER(13)
EXHANDLER(14)
EXHANDLER(15)
EXHANDLER(16)
EXHANDLER(17)
EXHANDLER(18)
EXHANDLER(19)

/*
 * Exception handler -- we've pushed all the register data onto
 * the stack - the handler is expected to either restart CFE
 * or longjmp to another place, which will clean up the mess
 * we've made on the stack.
 */

__exc_common:
#ifdef _LED_DEBUG_
	outb  %al,$0x80
#endif
	movl  %esp,%edx
	pushl %edx		/* argument to exc handler */
	pushl %eax		/* argument to exc handler */
	pushl $__exc_fail	/* Fake return address */
	pushf			/* Pass current flags */
	push  %cs
	pushl $cfe_exception
	iretl			/* Undo interrupt condition,
				   jump to exception handler */

__exc_fail:

#ifdef _LED_DEBUG_
	movb  $0x99,%al		/* Catch for returning exception handlers */
	outb  %al,$0x80
#endif

1:	jmp   1b


/*  *********************************************************************
    *  Interrupt Handler Table
    *  (used for constructing IDT)
    ********************************************************************* */

__inthandlers:
        .long __exception0,__exception1,__exception2,__exception3
        .long __exception4,__exception5,__exception6,__exception7
        .long __exception8,__exception9,__exception10,__exception11
        .long __exception12,__exception13,__exception14,__exception15
        .long __exception16,__exception17,__exception18,__exception19
	.long 0

/*  *********************************************************************
    *  __init_idt()
    *  
    *  Initialize the IDT.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

        .globl __init_idt
__init_idt:
	pushl	%esi
	pushl	%edi
	pushl	%ecx

	movl	$0,%edi
	movl	$(IDT_MAX_VECTORS*8),%ecx
	xor	%al,%al
	cld
	rep	stosb			/* Zero IDT */

	movl	$__inthandlers,%esi
	movl	$IDT_ADDRESS,%edi

2:	movl	(%esi),%eax		/* Loop through vector table */
	addl	$4,%esi
	or	%eax,%eax
	jz	1f

	movl	%eax,%edx		/* Construct descriptor */
	andl	$0xFFFF,%eax
	andl	$0xFFFF0000,%edx
	orl	$0x80000,%eax		/* Segment Selector */
	orl	$0x8F00,%edx		/* Trap descriptor */

	movl	%eax,(%edi)
	movl	%edx,4(%edi)
	addl	$8,%edi
	jmp	2b
1:

	/*
	 * Now install the userreq vector.
	 */

	movl	$__cfe_userreq,%eax
	movl	$(IDT_ADDRESS+(IDT_USERREQ_VECTOR*8)),%edi

	movl	%eax,%edx		/* Construct descriptor */
	andl	$0xFFFF,%eax
	andl	$0xFFFF0000,%edx
	orl	$0x80000,%eax		/* Segment Selector */
	orl	$0x8F00,%edx		/* Trap descriptor */

	movl	%eax,(%edi)
	movl	%edx,4(%edi)

	/*
	 * Install the exit vector
	 */

	movl	$cfe_warmstart2,%eax
	movl	$(IDT_ADDRESS+(IDT_EXIT_VECTOR*8)),%edi

	movl	%eax,%edx		/* Construct descriptor */
	andl	$0xFFFF,%eax
	andl	$0xFFFF0000,%edx
	orl	$0x80000,%eax		/* Segment Selector */
	orl	$0x8F00,%edx		/* Trap descriptor */

	movl	%eax,(%edi)
	movl	%edx,4(%edi)

	

	/*
	 * finally install the new IDT
	 */

	lidt	%cs:ram_idt_ptr	

	popl	%ecx
	popl	%edi
	popl	%esi
	ret


/*  *********************************************************************
    *  _getsegtbl()
    *  
    *  Returns the address of the "segment table".  Mostly historical.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   eax - segment table address
    ********************************************************************* */

	.globl	_getsegtbl
_getsegtbl:

	movl	$segtable,%eax
	ret

/*  *********************************************************************
    *  _getticks()
    *  
    *  Returns the value of the CPU tick counter.  Used for the 
    *  system timer.  This value may be related to the actual
    *  CPU speed.  On intel we use the RDTSC instruction, which
    *  returns the system timestamp counter.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   eax - lower 32 bits of the counter
    ********************************************************************* */


	.globl _getticks
_getticks:
	pushl   %edx
	rdtsc
	popl	%edx
	ret


/*  *********************************************************************
    *  __cfe_userreq()
    *  
    *  Entry point for external applications.
    *  
    *  Input parameters: 
    *  	   EAX - IOCB to execute
    *  	   
    *  Return value:
    *  	   return code
    ********************************************************************* */

/* We dig right into the IOCB here.  Danger! */
#define ENDIANOFFSET   0      /* always little-endian  */
#define R_XIOCB_FCODE  (8*0+ENDIANOFFSET)
#define R_XIOCB_FLAGS  (8*3+ENDIANOFFSET)
#define R_XIOCB_XSTAT  (8*5+ENDIANOFFSET)

#define CFE_CMD_FW_RESTART      1
#define CFE_FLG_WARMSTART     0x00000001

	.globl __cfe_userreq
__cfe_userreq:

        /*
         * Gross: Make an explicit check here for a warm firmware restart,
         * to avoid setting up the stack and doing other nasty things
         * when we're just going to return to the firmware anyway.
         */

	cmpl	$CFE_CMD_FW_RESTART,R_XIOCB_FCODE(%eax)
	jne	notwarm
	testl	$CFE_FLG_WARMSTART,R_XIOCB_FLAGS(%eax)
	jz	notwarm
	movl	R_XIOCB_XSTAT(%eax),%eax

	jmp	cfe_warmstart

 	/*
	 * Okay, regular API call.
	 */

notwarm:
	pushl	%eax
	call	cfe_doxreq
	addl	$4,%esp

	iretl


/*  *********************************************************************
    *  cfe_leds(v)
    *  
    *  Write to the system LEDs, normally at I/O port 0x80
    *  
    *  Input parameters: 
    *  	   v - value (in %al for now)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

	.globl cfe_leds
cfe_leds:
#ifdef _LED_DEBUG_
	outb   %al,$0x80
#endif
	ret


/*  *********************************************************************
    *  cfe_launch(...)
    *  
    *  Launch an external program.
    *  
    *  Input parameters: 
    *  	   startaddr - entry address
    *  	   
    *  Return value:
    *  	   does not return
    ********************************************************************* */
	.globl cfe_launch
cfe_launch:	
	movl	4(%esp),%ecx
	wbinvd
	jmp	*%ecx
	ret


/*  *********************************************************************
    *  _exc_restart
    *  
    *  Reset the system.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   does not return
    ********************************************************************* */

	.globl _exc_restart
_exc_restart:	

	jmp	board_reset


/*  *********************************************************************
    *  _cfe_flushcache
    *  
    *  Main entry point to the cache flushing routines.
    *  
    *  Input parameters: 
    *  	   (used to accept flags here, how to do this?)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

	.globl _cfe_flushcache
_cfe_flushcache:

	wbinvd
	ret
		

/*  *********************************************************************
    *  cfe_warmstart
    *  
    *  Execution comes here when someone wants to reenter CFE
    *  from an application.
    *  
    *  Input parameters: 
    *  	   edx:eax - exit status
    *  	   
    *  Return value:
    *  	   does not return
    ********************************************************************* */

	.globl  cfe_warmstart2
cfe_warmstart2:

	xorl	%eax,%eax
	xorl	%edx,%edx

	/* fall through */

	.globl	cfe_warmstart
cfe_warmstart:

	movl	$_end,%esp
	addl	$(CFG_HEAP_SIZE*1024+CFG_STACK_SIZE),%esp
	andl	$0xFFFFFFF0,%esp
	movl	%esp,%ebp

1:	pushl	%edx		/* Pass 64-bit status */
	pushl	%eax
	call	cfe_command_restart
	add	$8,%esp		/* should never get here */
	xorl	%eax,%eax
	xorl	%edx,%edx
	jmp	1b		/* should never get here */


/*  *********************************************************************
    *  CPUID
    *  
    *  Return the results of the CPUID instruction
    *  
    *  Input parameters: 
    *  	   EAX value
    *      Pointer to 4 values for results
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

        .globl cpuid
cpuid:	

	pushl	%ebp
	movl	%esp,%ebp

	pushl	%edi
	pushl	%ebx
	pushl	%ecx
	pushl	%edx

	movl	8(%ebp),%eax
	cpuid

	movl	12(%ebp),%edi
	movl	%eax,0(%edi)
	movl	%ebx,4(%edi)
	movl	%ecx,8(%edi)
	movl	%edx,12(%edi)

	popl	%edx
	popl	%ecx
	popl	%ebx
	popl	%edi

	leave
	ret



/*  *********************************************************************
    *  End
    ********************************************************************* */
