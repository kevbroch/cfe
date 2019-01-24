/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PromICE debug routines			File: promice.S
    *  
    *  Some stackless, memoryless routines for writing debug messages
    *  to the PromICE AI2 port
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

#include "promice.h"
	 
		.text

#ifdef _PROMICE_AI2_

#define ZERO_OFFSET	(0)
#define ONE_OFFSET	(1)
#define DATA_OFFSET	(2)
#define STATUS_OFFSET	(3)
#define TDA 0x01 	/* Target data available */
#define HDA 0x02 	/* Host data available */
#define OVR 0x04 	/* Host data overflow */

/*
 * Trashes: ESI, AX
 */

		.section .pminit_text,"xr"

/*  *********************************************************************
    *  AI2_OUTSTR
    *  
    *  Write a string to the PromICE serial port.  We use this
    *  routine in assembly code before the memory is working
    *  so it really, really tries to conserve registers.
    *  In particular, the null-terminated string it reads
    *  is placed at the return address of this routine!
    *  So: in memory you will see:
    *  
    *               movl $message,%esi
    *               jmp  ai2_outstr
    *  message:     .asciz "your message goes here"
    *  
    *               more code goes here.
    *  
    *  AI2_OUTSTR jumps to the byte immediately following the 
    *  null byte in the string.
    *  
    *  Input parameters: 
    *  	   ESI - points to message to display
    *  	   
    *  Return value:
    *  	   nothing (ESI will contain return address)
    *  	   
    *  Registers used:
    *  	   ESI - modified
    *  	   AX
    ********************************************************************* */


		.globl	ai2_outstr
ai2_outstr:

wait1:		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al

		test	$TDA,%al
		jnz	wait1

		movb	(%esi),%ah
		inc	%esi
		or	%ah,%ah
		jz	ai2_outdone

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		jmp	wait1


ai2_outdone:	jmp	*%esi



/*  *********************************************************************
    *  AI2_OUTWORD
    *  
    *  Writes an 8-digit hex number to the AI2 port.  This routine
    *  is used when memory is not available, so it's ugly.
    *  
    *  Input parameters: 
    *  	   EDX - number to display
    *      EBX - return address (routine jumps here when done)
    *  	   
    *  Return value:
    *  	   nothing
    *  	   
    *  Registers used:
    *  	   EDX
    *  	   EAX
    ********************************************************************* */


		.globl	ai2_outword
ai2_outword:


		rol	$4,%edx
		movl	%edx,%eax
		andl	$0x0F,%eax
		add	$0x30,%al
		cmpb	$0x39,%al
		jbe	2f
		add	$7,%al
2:		movb	%al,%ah
		

1:		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al

		test	$TDA,%al
		jnz	1b

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		movb	(PROMICE_LOC+ONE_OFFSET),%al

/* ---------------------------------------------- */


		rol	$4,%edx
		movl	%edx,%eax
		andl	$0x0F,%eax
		add	$0x30,%al
		cmpb	$0x39,%al
		jbe	2f
		add	$7,%al
2:		movb	%al,%ah
		

1:		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al

		test	$TDA,%al
		jnz	1b

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		movb	(PROMICE_LOC+ONE_OFFSET),%al

/* ---------------------------------------------- */


		rol	$4,%edx
		movl	%edx,%eax
		andl	$0x0F,%eax
		add	$0x30,%al
		cmpb	$0x39,%al
		jbe	2f
		add	$7,%al
2:		movb	%al,%ah
		

1:		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al

		test	$TDA,%al
		jnz	1b

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		movb	(PROMICE_LOC+ONE_OFFSET),%al

/* ---------------------------------------------- */


		rol	$4,%edx
		movl	%edx,%eax
		andl	$0x0F,%eax
		add	$0x30,%al
		cmpb	$0x39,%al
		jbe	2f
		add	$7,%al
2:		movb	%al,%ah
		

1:		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al

		test	$TDA,%al
		jnz	1b

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		movb	(PROMICE_LOC+ONE_OFFSET),%al

/* ---------------------------------------------- */


		rol	$4,%edx
		movl	%edx,%eax
		andl	$0x0F,%eax
		add	$0x30,%al
		cmpb	$0x39,%al
		jbe	2f
		add	$7,%al
2:		movb	%al,%ah
		

1:		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al

		test	$TDA,%al
		jnz	1b

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		movb	(PROMICE_LOC+ONE_OFFSET),%al

/* ---------------------------------------------- */


		rol	$4,%edx
		movl	%edx,%eax
		andl	$0x0F,%eax
		add	$0x30,%al
		cmpb	$0x39,%al
		jbe	2f
		add	$7,%al
2:		movb	%al,%ah
		

1:		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al

		test	$TDA,%al
		jnz	1b

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		movb	(PROMICE_LOC+ONE_OFFSET),%al

/* ---------------------------------------------- */


		rol	$4,%edx
		movl	%edx,%eax
		andl	$0x0F,%eax
		add	$0x30,%al
		cmpb	$0x39,%al
		jbe	2f
		add	$7,%al
2:		movb	%al,%ah
		

1:		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al

		test	$TDA,%al
		jnz	1b

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		movb	(PROMICE_LOC+ONE_OFFSET),%al

/* ---------------------------------------------- */


		rol	$4,%edx
		movl	%edx,%eax
		andl	$0x0F,%eax
		add	$0x30,%al
		cmpb	$0x39,%al
		jbe	2f
		add	$7,%al
2:		movb	%al,%ah
		

1:		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al
		movb	(PROMICE_LOC+STATUS_OFFSET),%al

		test	$TDA,%al
		jnz	1b

		movb	(PROMICE_LOC+ONE_OFFSET),%al

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		rcr	$1,%ah
		jc	2f
		movb	(PROMICE_LOC+ZERO_OFFSET),%al
		jmp	1f
2:		movb	(PROMICE_LOC+ONE_OFFSET),%al
1:

		movb	(PROMICE_LOC+ONE_OFFSET),%al

/* ---------------------------------------------- */



		jmp	*%ebx
#endif
