/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Physical memory peek/poke routines	File: lib_physio.S
    *  
    *  Little stub routines to allow access to arbitrary physical
    *  addresses and I/O ports.
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
    *  PHYSOP(inst,instmem,reg)
    *  
    *  Macro to construct code for doing the physical I/O
    *  We try to avoid messing with KX or doing 64-bit stuff
    *  unless necessary.
    *  
    *  Input parameters: 
    *  	   INST - instruction name to run
    *      INSTMEM - instruction name for memory ops
    *  	   REG - register containing arg or return value
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

/*
 * Gross, hideous hack:  X86 has a distinction between I/O 
 * and memory space, with different instructions for accessing
 * each.  There is only 64K of I/O space, so let's make any "physop"
 * that refers to address <= 0x10000 mean "I/O" and anything more
 * mean "memory."   PCI devices often have memory addressess for their
 * BARs, so we need a way to describe either one.
 */

#define PHYSOP(INST,INSTMEM,REG) \
		movl    4(%esp),%edx ; \
		cmpl	$0xFFFF,%edx ; \
		ja      1f ; \
		INST	%dx,%REG       ; \
		ret			; \
1:		INSTMEM  (%edx),%REG ; \
		ret

#define PHYSOP2(INST,INSTMEM,REG) \
		movl    4(%esp),%edx ; \
		movl    8(%esp),%eax ; \
		cmpl	$0xFFFF,%edx ; \
		ja      1f ; \
		INST	%REG,%dx       ; \
		ret		       ; \
1:		INSTMEM %REG,(%edx) ; \
		ret


/*  *********************************************************************
    *  phys_read8 - read 8-bit bytes
    ********************************************************************* */

	.global	phys_read8
phys_read8:
PHYSOP(inb,movb,al)

/*  *********************************************************************
    *  phys_read16 - read 16-bit shorts
    ********************************************************************* */

	.global phys_read16
phys_read16:
PHYSOP(inw,movw,ax)

/*  *********************************************************************
    *  phys_read32 - read 32-bit ints
    ********************************************************************* */

	.global	phys_read32
phys_read32:
PHYSOP(inl,movl,eax)

/*  *********************************************************************
    *  phys_write8 - write 8-bit bytes
    ********************************************************************* */

	.global	phys_write8
phys_write8:
PHYSOP2(outb,movb,al)

/*  *********************************************************************
    *  phys_write16 - write 16-bit shorts
    ********************************************************************* */

	.global	phys_write16
phys_write16:
PHYSOP2(outw,movw,ax)

/*  *********************************************************************
    *  phys_write32 - write 32-bit longs
    ********************************************************************* */

	.global	phys_write32
phys_write32:
PHYSOP2(outl,movl,eax)

/*  *********************************************************************
    *  The 'in' and 'out' routines live here too.
    ********************************************************************* */

        .globl inb
inb:
	movl   4(%esp),%edx
	xor    %eax,%eax
	inb    %dx,%al
	ret

        .globl inw
inw:
	movl   4(%esp),%edx
	xor    %eax,%eax
	inw    %dx,%ax
	ret

        .globl inl
inl:
	movl   4(%esp),%edx
	inl    %dx,%eax
	ret

        .globl outb
outb:
	movl   4(%esp),%edx
	movl   8(%esp),%eax
	outb   %al,%dx
	ret

        .globl outw
outw:
	movl   4(%esp),%edx
	movl   8(%esp),%eax
	outw   %ax,%dx
	ret

        .globl outl
outl:
	movl   4(%esp),%edx
	movl   8(%esp),%eax
	outl   %eax,%dx
	ret



/*  *********************************************************************
    *  End
    ********************************************************************* */


