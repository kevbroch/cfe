/*  *********************************************************************
    *  BCM1280/BCM1480 Board Support Package
    *  
    *  IRQ polling				File: bcm1480_ircpoll.S
    *  
    *  This module contains code to poll the interrupt controller
    *  and invoke the servicing of pending, unmasked interrupts.
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2004,2005
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

#include "sbmips.h"
#include "cpu_config.h"
#include "mipsmacros.h"
#include "bcm1480_regs.h"
#include "bcm1480_int.h"

	 .set mips64

/*
 * External interrupt conventions:
 *   i0 (IP2) is used for for registered interrupts:
 *        CFG_INTERRUPTS=1:  unmasked, asynchronous hardware interrupts
 *        CFG_INTERRUPTS=0:  masked, synchronous software polling
 *   i1-i4 are available for specific devices.
 *   i5 (IP7) is reserved for sources within the CPU
 */
	
/* EOIs are issued when HT interrupts are programmed as levels. */
	
#define A_HT_EOI  PHYS_TO_XKPHYS_UNCACHED(0x00D8000000)
#define D_EOI_MDT (0x7 << 2)
	
	.text	
	
/*  *********************************************************************
    *  CFE_IRQ_POLL
    *  
    *  Scan the interrupt_source_status and interrupt_ldt registers.
    *  For those not masked by the interrupt_mask register, invoke
    *  <cpu>_dispatch_irq with the interrupt number as argument.
    *
    *  For HT-signaled interrupts, also clear the pending vector.
    *  
    *  For asynchronous dispatch, called with interrupts disabled.
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
	.set    mips64

	.extern	bcm1480_dispatch_irq	/* NB: accessed via gp */

	/* This version accomodates level-triggered HT interrupts by
	issuing EOI's on return from each handler.  This code depends
	on the fact that an EOI will cause a level-sensitive interrupt
	to resend the interrupt message if that interrupt is still
	pending.  It is not safe for edge-triggered interrupts.  */

	/* Note that we rely on interrupt mapping hardware to do
	merging and masking by reading interrupt status[n], with n=0
	(IP2) currently.  We still need to determine which bits are
	sourced by HT and potentially require EOIs.  We thus must read
	and save the HT interrupt vector before calling the
	handler(s). */

LEAF(cfe_irq_poll)

	daddiu	sp, -72		/* saved register space */
	sd	s0, 32(sp)	/* start the load early */
	la	v0, K1BASE + A_BCM1480_IMR_CPU0_BASE
	ld	s0, R_BCM1480_IMR_INTERRUPT_STATUS_BASE_H+0(v0)
	sd	s1, 40(sp)
	sd	s2, 48(sp)
	sd	s3, 56(sp)
	sd	ra, 64(sp)

	mfc0	a0, C0_CAUSE
	andi	a0, a0, M_CAUSE_IP2
	beqz	a0, 5f
	
	/* Do the high vector first */
high:
	ld	s2, R_BCM1480_IMR_LDT_INTERRUPT_H(v0)
	beqz	s0, low		/* No interrupts.  Return if no cascade?  */
	
	/* The bit scan loop */
	dclz	s1, s0		/* Find index of the next interrupt */
3:
	li	s3, 1
	dsubu	s1, zero, s1
	daddiu	s1, s1, 63     
	beqz	s1, low		/* this is the cascade bit */
	dsllv	s3, s3, s1
	xor	s0, s0, s3	/* clear current bit */

	la	t9, bcm1480_dispatch_irq
	jalr	ra, t9		/* NB:	 SVR4 PIC requires t9 */
	move	a0, s1		/* a0 is interrupt number */
	
	and	a0, s2, s3
	beqz	a0, 4f		/* not HT */
	dsll	s1, s1, 16

	/* Clear the latched bit and send EOI if a pending HT interrupt */
	la	v0, K1BASE + A_BCM1480_IMR_CPU0_BASE
	sd	s3, R_BCM1480_IMR_LDT_INTERRUPT_CLR_H(v0)
	dla	v0, A_HT_EOI + (M_BCM1480_HTVECT_RAISE_INTLDT_HIGH << 16)
	or	v0, v0, s1
	sync

#if 0
	/* Current CFE runs with Status.KX set, even when built in
	nominal 32 bit mode.  Code such as the following is needed
	otherwise.  32-bit systems that only save 32 bits of registers
	must be very careful with XKPHYS addresses (v0). */

	mfc0	s3, C0_SR
	ori	AT, s3, M_SR_KX
	mtc0	AT, C0_SR
	HAZARD
#endif

	sw	zero, D_EOI_MDT(v0)

#if 0
	/* See above. */
	mtc0	s3, C0_SR
	HAZARD
#endif

4:	
	bnez	s0, 3b		/* More interrupts to service?       */
	dclz	s1, s0		/* unroll for branch delay slot */

	b	5f		/* No cascade, so skip low. */
	nop	

	/* Then the low. */
low:
	la	v0, K1BASE + A_BCM1480_IMR_CPU0_BASE
        ld	s0, R_BCM1480_IMR_INTERRUPT_STATUS_BASE_L+0(v0)
	ld	s2, R_BCM1480_IMR_LDT_INTERRUPT_L(v0)
	beqz	s0, 5f		/* No more interrupts.  Return */
	
	/* The bit scan loop */
	dclz	s1, s0		/* Find index of the next interrupt */
3:
	li	s3, 1
	dsubu	s1, zero, s1
	daddiu	s1, s1, 63     
	dsllv	s3, s3, s1
	xor	s0, s0, s3	/* clear current bit */

	la	t9, bcm1480_dispatch_irq
	jalr	ra, t9		/* NB:	 SVR4 PIC requires t9 */
	daddiu	a0, s1, 64	/* a0 is interrupt number */
	
	and	a0, s2, s3
	beqz	a0, 4f		/* not HT */
	dsll	s1, s1, 16

	/* Clear the latched bit and send EOI if a pending HT interrupt */
	la	v0, K1BASE + A_BCM1480_IMR_CPU0_BASE
	sd	s3, R_BCM1480_IMR_LDT_INTERRUPT_CLR_L(v0)
	dla	v0, A_HT_EOI + (M_BCM1480_HTVECT_RAISE_INTLDT_LO << 16)
	or	v0, v0, s1
	sync

#if 0
	/* See comments above about Status.KX */
	mfc0	s3, C0_SR
	ori	AT, s3, M_SR_KX
	mtc0	AT, C0_SR
	HAZARD
#endif

	sw	zero, D_EOI_MDT(v0)

#if 0
	/* Ditto */
	mtc0	s3, C0_SR
	HAZARD
#endif

4:	
	bnez	s0, 3b		/* More interrupts to service?       */
	dclz	s1, s0		/* unroll for branch delay slot */
5:                        
	ld	ra, 64(sp)	/* restore registers */
	ld	s3, 56(sp)
	ld	s2, 48(sp)
	ld	s1, 40(sp)
	ld	s0, 32(sp)
	
	jr	ra
	daddiu	sp, 72		/* saved register space */
		
END(cfe_irq_poll)
	
	.set	pop


/*  *********************************************************************
    * bcm1480_irq_arm()
    *	
    * Set up CP0 Status and Cause per conventions above (not really -- yet)
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
	.set    mips64

LEAF(bcm1480_irq_arm)
	
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

END(bcm1480_irq_arm)

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
	.set    mips64

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
	.set    mips64

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
    * bcm1480_update_sr(clear,set)
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
	.set    mips64

LEAF(bcm1480_update_sr)
	
	nor	a0,a0,zero
	mfc0	t0,C0_SR
	and	t0,t0,a0                /* current mask bits */
	or	t0,t0,a1
	mtc0	t0,C0_SR                /* all enables cleared */
	
	HAZARD

	jr	ra
	nop	

END(bcm1480_update_sr)

	.set	pop
