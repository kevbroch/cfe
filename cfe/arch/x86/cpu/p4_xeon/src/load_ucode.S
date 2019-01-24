 /*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  P4/Xeon Microcode Loader			File: load_ucode.S
    *  
    *  Routine to load processor updates into the CPU.  Information	
    *  about this is in the Intel System Software Writer's Guide (SSWG)
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

/*  *********************************************************************
    *  Constants
    ********************************************************************* */

#define UCODE_HDR_VER	0x00
#define UCODE_UPD_REV	0x04
#define UCODE_DATE	0x08
#define UCODE_PROCESSOR	0x0C
#define UCODE_CKSUM	0x10
#define UCODE_LDR_REV	0x14
#define UCODE_PROC_FLG	0x18
#define UCODE_RESERVED	0x1C
#define UCODE_DATA	0x30
#define UCODE_SIZE	2048

#define UCODE_CUR_HDR_VER	0x00000001
#define UCODE_CUR_LDR_REV	0x00000001

#define MSR_UCODE_UPDATE	0x79
#define MSR_UCODE_SIGNATURE	0x8B

/*  *********************************************************************
    *  Code - this code lives in the special "pminit" section
    *  since it is run before relocation.
    ********************************************************************* */

	 .section .pminit_text,"xr"


/*  *********************************************************************
    *  cpu_load_ucode
    *  
    *  Load processor microcode.  This is a "stackless" call
    *  
    *  Input parameters: 
    *  	   esi - pointer to microcode block
    *	   eax - size of microcode block (bytes)
    *      esp - return address
    *  	   
    *  Return value:
    *  	   CY - set if failure
    *  	   CY - clear if microcode loaded
    *  	   
    *  Registers used:
    *  	   EAX, EBX, ECX, EDX, ESI, EDI
    ********************************************************************* */


        .globl cpu_load_ucode

cpu_load_ucode:

	/*
	 * compute max address of table
	 */

	movl	%esi,%edi
	addl	%eax,%edi

	/*
	 * See SSWG Page B-5:  This register contains the Microcode
	 * update signature - clear to zero first, then do a CPUID.
	 * If it remains nonzero, there is no microcode loaded.
	 */

	movl	$MSR_UCODE_SIGNATURE,%ecx	
	xor	%eax,%eax
	xor	%edx,%edx
	wrmsr

	movl	$1,%eax	
	cpuid

	movl	$MSR_UCODE_SIGNATURE,%ecx
	rdmsr	

	/*
	 * EDX should now contain the current microcode signature.
	 */

	 
	/*
	 * Get our CPUID so we can see which block we ought to be
	 * loading.
	 */

	movl	$1,%eax			/* get our CPUID */
	cpuid				/* EAX contains processor ID */

	/*
	 * Scan table looking for an appropriate update.  The table
	 * ends when we hit something that's not a microcode update.
	 * So, check for zeroes, FF's, and 0xCC's (CC = int3 instruction)
	 * for typical ends of the table.  I suppose we could do better
	 * about this and keep track of how many records there are.
	 */

check_ucode:
	cmpl	%edi,%esi			/* past end of table? */
	jae	no_more_ucode

	cmpl	$0,UCODE_HDR_VER(%esi)		/* Check for end markers */
	je	no_more_ucode
	cmpl	$0xFFFFFFFF,UCODE_HDR_VER(%esi)
	je	no_more_ucode
	cmpl	$0xCCCCCCCC,UCODE_HDR_VER(%esi)
	je	no_more_ucode

	cmpl	$UCODE_CUR_HDR_VER,UCODE_HDR_VER(%esi)
	jne	skip_this_ucode

	cmpl	UCODE_PROCESSOR(%esi),%eax
	jne	skip_this_ucode

	cmpl	$UCODE_CUR_LDR_REV,UCODE_LDR_REV(%esi)  /* Correct loader version? */
	jne	skip_this_ucode

	lea	UCODE_DATA(%esi),%eax		/* Point at update data */
	xor	%edx,%edx
	movl	$MSR_UCODE_UPDATE,%ecx		/* Load it in! */
	wrmsr

	clc				/* all done, no errors */
	jmp	*%esp


skip_this_ucode:
	add	$UCODE_SIZE,%esi	/* next block, go back for more  */
	jmp	check_ucode

no_more_ucode:
	stc				/* Could not find block, error! */
	jmp	*%esp


/*  *********************************************************************
    *  End
    ********************************************************************* */
	
