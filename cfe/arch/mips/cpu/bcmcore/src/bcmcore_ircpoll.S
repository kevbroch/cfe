/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  IRQ support routines			File: bcmcore_ircpoll.S
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

#include "sbmips32.h"
#include "cpu_config.h"
#include "mipsmacros.h"

/*
 * Dispatch function address.  Access to this function pointer assumes
 * that gp has been initialized and any relocation has been done.
 */
	.sdata

	.globl	irq_dispatch
irq_dispatch:
	_LONG_	0

	.text	
	
/*  *********************************************************************
    * bcmcore_irq_install()
    *	
    * Initialize dispatch pointer.  See CALLINIT_RELOC in mipsmacros.h
    * for the logic used here.  This function should be called after
    * any relocation is done and gp is initialized.
    *
    * This function should be called with interrupts disabled.
    *
    * Input parameters:	
    *	nothing
    *
    * Return value:	
    *	nothing					
    ********************************************************************* */
	
	.set    push
	.set	noreorder

	.extern	bcmcore_dispatch_pending

dispatch_func:
	_LONG_	bcmcore_dispatch_pending
	
LEAF(bcmcore_irq_install)

	la	v0,dispatch_func	
	LR	v0, 0(v0)
1:	SR	v0, irq_dispatch

	jr	ra
	nop	

END(bcmcore_irq_install)

	.set	pop

	
/*  *********************************************************************
    *  CFE_IRQ_POLL
    *  
    *  Read CP0 registers to determine the set of pending and
    *  enabled registers.  Call bcmcore_dispatch_pending with the
    *  result (bits as in SR and CAUSE registers).
    *
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

	.set    push
	.set	noreorder
	.set	noat
	.set    mips32

LEAF(cfe_irq_poll)

	addiu	sp, -20		/* saved register space */
	sw	ra, 16(sp)

	mfc0	a0, C0_CAUSE
	mfc0	a1, C0_SR
	and	a0, a0, a1
	li	v0, M_CAUSE_IPMASK
	and	v0, a0, v0
	beq	v0, zero, 1f
	nop
	la	t9, bcmcore_dispatch_pending
	jalr	ra, t9
	nop
	
1:                        
	lw	ra, 16(sp)	/* restore registers */
	
	jr	ra
	addiu	sp, 20		/* saved register space */
		
END(cfe_irq_poll)
	
	.set	pop

/*  *********************************************************************
    * bcmcore_irq_arm()
    *	
    * Set up CP0 Status and Cause per selected dispatch conventions.
    *
    * This function should be called with interrupts disabled.
    *
    * Input parameters:	
    *	nothing
    *
    * Return value:	
    *	nothing					
    ********************************************************************* */
	
	.set    push
	.set	noreorder
	.set    mips32

LEAF(bcmcore_irq_arm)
	
	mfc0	t0,C0_CAUSE
	or	t0,t0,M_CAUSE_IV
	mtc0	t0,C0_CAUSE

	mfc0	t0,C0_SR
	li	t1,M_SR_IMMASK		/* Mask all interrupt levels */
	nor	t1,t1,zero
	and	t0,t0,t1
	or	t0,t0,M_SR_IE		/* but set IE */
	mtc0	t0,C0_SR
	HAZARD

	jr	ra
	nop	

END(bcmcore_irq_arm)

	.set	pop



/*  *********************************************************************
    * cfe_irq_disable()
    *	
    * Disable interrupts
    * XXX: This is not really atomic.
    *
    * Input parameters:	
    *	none
    *
    * Return value:	
    *	current SR interrupt mask
    ********************************************************************* */
	
	.set    push
	.set	noreorder
	.set    mips32

LEAF(cfe_irq_disable)
	
	mfc0	t0,C0_SR
	li	t1,M_SR_IMMASK|M_SR_IE
	li	t2,~(M_SR_IMMASK|M_SR_IE)
	and	v0,t0,t1                /* current mask bits */
	and	t0,t0,t2
	mtc0	t0,C0_SR                /* all enables cleared */
	HAZARD	

	jr	ra
	nop	

END(cfe_irq_disable)

	.set	pop

	
/*  *********************************************************************
    * cfe_irq_enable(mask)
    *	
    * Restore enabled interrupts
    * XXX: This is not really atomic.
    *
    * Input parameters:	
    *	interrupt mask (from irq_disable)
    *
    * Return value:	
    *	nothing					
    ********************************************************************* */
	
	.set    push
	.set	noreorder
	.set    mips32

LEAF(cfe_irq_enable)
	
	mfc0	t0,C0_SR
	li	t1,M_SR_IMMASK|M_SR_IE
	li	t2,~(M_SR_IMMASK|M_SR_IE)
	and	a0,a0,t1
	and	t0,t0,t2
	or	t0,t0,a0
	mtc0	t0,C0_SR
	HAZARD	

	jr	ra
	nop	

END(cfe_irq_enable)

	.set	pop


/*  *********************************************************************
    * bcmcore_update_sr(clear,set)
    *	
    * Upate Status.IM according to masks
    *
    * Caller should disable interrupts if the effect is to be atomic.
    *
    * Input parameters:	
    *	a0         SR bits to be cleared
    *   a1         SR bits to be set	
    *
    * Return value:	
    *	none
    ********************************************************************* */
	
	.set    push
	.set	noreorder
	.set    mips32

LEAF(bcmcore_update_sr)
	
	nor	a0,a0,zero
	mfc0	t0,C0_SR
	and	t0,t0,a0                /* current mask bits */
	or	t0,t0,a1
	mtc0	t0,C0_SR                /* all enables cleared */
	HAZARD
	
	jr	ra
	nop	

END(bcmcore_update_sr)

	.set	pop
