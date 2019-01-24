/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  setjmp/longjmp routines			File: lib_setjmp.S
    *  
    *  This module contains a simple setjmp/longjmp for X86 processors
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2003
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

	.text
	.globl lib_setjmp
lib_setjmp:
	movl	4(%esp),%eax
	movl	%ebx, 0(%eax)
	movl	%esp, 4(%eax)
	movl	%ebp, 8(%eax)
	movl	%esi,12(%eax)
	movl	%edi,16(%eax)
	movl	(%esp),%edx	
	movl	%edx,20(%eax)	
	xorl	%eax,%eax	
	ret


	.globl lib_longjmp
lib_longjmp:
	movl	4(%esp),%eax
	movl	0(%eax),%ebx
	movl	4(%eax),%esp
	movl	8(%eax),%ebp
	movl	12(%eax),%esi
	movl	16(%eax),%edi
	movl	20(%eax),%edx
	movl	%edx,(%esp)	
	movl	$1,%eax		

	ret

