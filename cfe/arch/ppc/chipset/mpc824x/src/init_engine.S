/*  *********************************************************************
    *  PPC Board Support Package
    *  
    *  initialization engine			File: init_engine.S
    *
    *  This routine parses a table of initialization primitives to
    *  set values into MSRs, PCI config space, etc.
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

#include "ppcdefs.h"
#include "ppcmacros.h"
#include "mpc824x.h"

#include "init_engine.h"

/*  *********************************************************************
    *  Layout of the table we pass into init_engine
    ********************************************************************* */


#define R_IE_CMD	_TBLIDX(0)
#define R_IE_ADDR	_TBLIDX(1)
#define R_IE_MASK	_TBLIDX(2)
#define R_IE_VAL	_TBLIDX(3)
#define R_IE_CMDSIZE	_TBLIDX(4)

/*  *********************************************************************
    *  Macros to define little code stubs called to initialize
    *  certain registers, since we don't have a "register indirect
    *  SPR write" instruction on PPC.
    ********************************************************************* */

#define SPRFUNC(spr) .long spr ; mtspr spr,r3 ; blr ; mfspr r3,spr ; blr

#define R_SPR_ID     _TBLIDX(0)
#define R_SPR_SET    _TBLIDX(1)
#define R_SPR_GET    _TBLIDX(3)
#define R_SPR_SIZE   _TBLIDX(5)

#define IBATFUNC(bat) mtibatu bat,r3 ; mtibatl bat,r4 ; isync ; blr
#define DBATFUNC(bat) mtdbatu bat,r3 ; mtdbatl bat,r4 ; isync ; blr
#define SRFUNC(s)     mtsr s,r3 ; isync ; nop ; blr
#define R_BAT_SIZE    _TBLIDX(4)


/*  *********************************************************************
    *  Global Offset Table
    ********************************************************************* */

#if CFG_RELOC
BEGIN_GOT()
GOT_ENTRY(sprtable)
GOT_ENTRY(ibattable)
GOT_ENTRY(dbattable)
GOT_ENTRY(srtable)
END_GOT()
#endif


/*  *********************************************************************
    *  init_engine
    *  
    *  Process an init table, doing the register and PCI writes
    *  the the table calls for
    *  
    *  Input parameters: 
    *  	   r6 - pointer to init table
    *  	   
    *  Return value:
    *  	   nothing.
    ********************************************************************* */

#define tblptr	r7

LEAF(init_engine)

	SAVE_LR(r15)		/* need to save/restore LR here. */
	LOAD_GOT(GOT_REG)
	RESTORE_LR(r15)

	mr	tblptr,r6

initlp:
	lwz	r6,R_IE_CMD(tblptr)
	cmplwi	r6,IE_EOT
	bne	99f
	b	initdone

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:	cmplwi	r6,IE_PCICFG32
	bne	99f

	/*
	 * Point the CONFIG_ADDR at the interesting PCI space address
	 * and load a pointer (r6) to CONFIG_DATA.
	 */

	LDCONST(r6,A_MPC_CONFIG_ADDR)
	lwz	r3,R_IE_ADDR(tblptr)
	stwbrx	r3,0,r6
	sync
	LDCONST(r6,A_MPC_CONFIG_DATA)

	/*
	 * If the mask is zero, don't read PCI register, only write.
	 * Otherwise, do a read/and/or/write sequence to clear
	 * (mask) and set (val) bits.  Then, write back to register.
	 */

	li	r3,0
	lwz	r4,R_IE_MASK(tblptr)
	cmplwi	r4,0
	beq	1f
	lwbrx	r3,0,r6
	and	r3,r3,r4
1:	lwz	r4,R_IE_VAL(tblptr)
	or	r3,r3,r4
	stwbrx	r3,0,r6
	sync

	b	initnxt

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
99:	cmplwi	r6,IE_PCICFG16
	bne	99f

	/*
	 * Point the CONFIG_ADDR at the interesting PCI space address
	 * and load a pointer (r6) to CONFIG_DATA.  For half-word accesses,
	 * also compute the "offset" (either 0 or 2) into r5.
	 */

	LDCONST(r6,A_MPC_CONFIG_ADDR)
	lwz	r3,R_IE_ADDR(tblptr)
	andi.	r5,r3,2
	li	r4,-4		/* 0xfffffffc */
	and	r3,r3,r4
	stwbrx	r3,0,r6
	sync
	LDCONST(r6,A_MPC_CONFIG_DATA)

	/*
	 * If the mask is zero, don't read PCI register, only write.
	 * Otherwise, do a read/and/or/write sequence to clear
	 * (mask) and set (val) bits.  Then, write back to register.
	 */

	li	r3,0
	lwz	r4,R_IE_MASK(tblptr)
	cmplwi	r4,0
	beq	1f
	lhbrx	r3,r5,r6
	and	r3,r3,r4
1:	lwz	r4,R_IE_VAL(tblptr)
	or	r3,r3,r4
	sthbrx	r3,r5,r6
	sync

	b	initnxt

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
99:	cmplwi	r6,IE_PCICFG8
	bne	99f

	/*
	 * Point the CONFIG_ADDR at the interesting PCI space address
	 * and load a pointer (r6) to CONFIG_DATA.  For byte accesses,
	 * also compute the "offset" (0..3) into r5.
	 */

	LDCONST(r6,A_MPC_CONFIG_ADDR)
	lwz	r3,R_IE_ADDR(tblptr)
	andi.	r5,r3,3
	li	r4,-4		/* 0xfffffffc */
	and	r3,r3,r4
	stwbrx	r3,0,r6
	sync
	LDCONST(r6,A_MPC_CONFIG_DATA)

	/*
	 * If the mask is zero, don't read PCI register, only write.
	 * Otherwise, do a read/and/or/write sequence to clear
	 * (mask) and set (val) bits.  Then, write back to register.
	 */

	li	r3,0
	lwz	r4,R_IE_MASK(tblptr)
	cmplwi	r4,0
	beq	1f
	lbzux	r3,r5,r6
	and	r3,r3,r4
1:	lwz	r4,R_IE_VAL(tblptr)
	or	r3,r3,r4
	stbux	r3,r5,r6
	sync

	b	initnxt
#endif
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:	cmplwi	r6,IE_SPR
	bne	99f

	mflr	r1				/* save LR in R1 */

	lwz	r6,R_IE_ADDR(tblptr)
	LDADDR(r8,sprtable)

sprlp:	lwz	r4,R_SPR_ID(r8)			/* r8 points at SPR routines */

	cmplwi	r4,0				/* test for EOT */
	beq	badspr

	cmplw	r4,r6				/* Test for correct SPR */
	bne	nxtspr

	addi	r5,r8,R_SPR_GET			/* call GET method */
	mtlr	r5
	blrl					/* returns SPR in r3 */

	lwz	r5,R_IE_MASK(tblptr)		/* AND in the mask */
	and	r3,r3,r5
	lwz	r5,R_IE_VAL(tblptr)		/* OR in the value */
	or	r3,r3,r5

	addi	r5,r8,R_SPR_SET			/* call SET method */
	mtlr	r5
	blrl					/* writes SPR from r3 */

nxtspr:	addi	r8,r8,R_SPR_SIZE		/* point @ next SPR */
	b	sprlp

	
badspr:	mtlr	r1
	b	initnxt

/*
 * Table of little mtspr/mfspr routines since there's no indirect way to do this.
 */

sprtable:
	SPRFUNC(SPR_XER)
	SPRFUNC(SPR_LR)
	SPRFUNC(SPR_CTR)

	SPRFUNC(SPR_ICR)

	SPRFUNC(SPR_TBL)
	SPRFUNC(SPR_TBU)

	SPRFUNC(SPR_HID0)
	SPRFUNC(SPR_HID1)
	SPRFUNC(SPR_HID2)

	SPRFUNC(SPR_MBAR)
	SPRFUNC(SPR_SVR)
	SPRFUNC(SPR_PVR)

	SPRFUNC(SPR_IBAT0U)
	SPRFUNC(SPR_IBAT0L)
	SPRFUNC(SPR_IBAT1U)
	SPRFUNC(SPR_IBAT1L)
	SPRFUNC(SPR_IBAT2U)
	SPRFUNC(SPR_IBAT2L)
	SPRFUNC(SPR_IBAT3U)
	SPRFUNC(SPR_IBAT3L)

	SPRFUNC(SPR_DBAT0U)
	SPRFUNC(SPR_DBAT0L)
	SPRFUNC(SPR_DBAT1U)
	SPRFUNC(SPR_DBAT1L)
	SPRFUNC(SPR_DBAT2U)
	SPRFUNC(SPR_DBAT2L)
	SPRFUNC(SPR_DBAT3U)
	SPRFUNC(SPR_DBAT3L)

	SPRFUNC(SPR_IBAT4U)
	SPRFUNC(SPR_IBAT4L)
	SPRFUNC(SPR_IBAT5U)
	SPRFUNC(SPR_IBAT5L)
	SPRFUNC(SPR_IBAT6U)
	SPRFUNC(SPR_IBAT6L)
	SPRFUNC(SPR_IBAT7U)
	SPRFUNC(SPR_IBAT7L)

	SPRFUNC(SPR_DBAT4U)
	SPRFUNC(SPR_DBAT4L)
	SPRFUNC(SPR_DBAT5U)
	SPRFUNC(SPR_DBAT5L)
	SPRFUNC(SPR_DBAT6U)
	SPRFUNC(SPR_DBAT6L)
	SPRFUNC(SPR_DBAT7U)
	SPRFUNC(SPR_DBAT7L)

	SPRFUNC(SPR_DMISS)
	SPRFUNC(SPR_DCMP)
	SPRFUNC(SPR_HASH1)
	SPRFUNC(SPR_HASH2)
	SPRFUNC(SPR_IMISS)
	SPRFUNC(SPR_ICMP)
	SPRFUNC(SPR_RPA)

	SPRFUNC(SPR_SDR1)

	SPRFUNC(SPR_SPRG0)
	SPRFUNC(SPR_SPRG1)
	SPRFUNC(SPR_SPRG2)
	SPRFUNC(SPR_SPRG3)
	SPRFUNC(SPR_SPRG4)
	SPRFUNC(SPR_SPRG5)
	SPRFUNC(SPR_SPRG6)
	SPRFUNC(SPR_SPRG7)

	SPRFUNC(SPR_DSISR)
	SPRFUNC(SPR_SRR0)
	SPRFUNC(SPR_SRR1)
	SPRFUNC(SPR_DAR)
	SPRFUNC(SPR_DEC)
	SPRFUNC(SPR_EAR)
	SPRFUNC(SPR_WTBL)
	SPRFUNC(SPR_WTBH)

	SPRFUNC(SPR_CSRR0)
	SPRFUNC(SPR_CSRR1)

	SPRFUNC(SPR_IABR)
	SPRFUNC(SPR_IABR2)
	SPRFUNC(SPR_DABR)
	SPRFUNC(SPR_DABR2)

	SPRFUNC(SPR_IBCR)
	SPRFUNC(SPR_DBCR)
	.long 0
	
#endif
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:	cmplwi	r6,IE_MSR
	bne	99f

	mfmsr	r3
	isync

	lwz	r4,R_IE_MASK(tblptr)
	and	r3,r3,r4
	lwz	r4,R_IE_VAL(tblptr)
	or	r3,r3,r4
	isync

	mtmsr	r3
	isync
	sync

	b	initnxt
#endif
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:	cmplwi	r6,IE_IBAT
	bne	99f

	mflr	r0

	lwz	r4,R_IE_ADDR(tblptr)
	add	r4,r4,r4		/* times 2 */
	add	r4,r4,r4		/* times 4 */
	add	r4,r4,r4		/* times 8 */
	add	r4,r4,r4		/* times 16 */
	LDADDR(r5,ibattable)
	add	r5,r5,r4		/* address of routine */
	lwz	r3,R_IE_MASK(tblptr)	/* upper BAT */
	lwz	r4,R_IE_VAL(tblptr)	/* lower BAT */
	mtlr	r5
	blrl

	mtlr	r0
	b	initnxt

ibattable:
	IBATFUNC(0)
	IBATFUNC(1)
	IBATFUNC(2)
	IBATFUNC(3)
#endif

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:	cmplwi	r6,IE_DBAT
	bne	99f

	mflr	r0

	lwz	r4,R_IE_ADDR(tblptr)
	add	r4,r4,r4		/* times 2 */
	add	r4,r4,r4		/* times 4 */
	add	r4,r4,r4		/* times 8 */
	add	r4,r4,r4		/* times 16 */
	LDADDR(r5,dbattable)
	add	r5,r5,r4		/* address of routine */
	lwz	r3,R_IE_MASK(tblptr)	/* upper BAT */
	lwz	r4,R_IE_VAL(tblptr)	/* lower BAT */
	mtlr	r5
	blrl

	mtlr	r0
	b	initnxt

dbattable:
	DBATFUNC(0)
	DBATFUNC(1)
	DBATFUNC(2)
	DBATFUNC(3)
#endif
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:	cmplwi	r6,IE_SR
	bne	99f

	mflr	r0

	lwz	r4,R_IE_ADDR(tblptr)
	add	r4,r4,r4		/* times 2 */
	add	r4,r4,r4		/* times 4 */
	LDADDR(r5,srtable)
	add	r5,r5,r4		/* address of routine */
	lwz	r3,R_IE_VAL(tblptr)	/* value */
	mtlr	r5
	blrl

	mtlr	r0
	b	initnxt

srtable:
	SRFUNC(0)
	SRFUNC(1)
	SRFUNC(2)
	SRFUNC(3)
	SRFUNC(4)
	SRFUNC(5)
	SRFUNC(6)
	SRFUNC(7)
#endif

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:	cmplwi	r6,IE_DELAY
	bne	99f

	lwz	r3,R_IE_VAL(tblptr)
	mtctr	r3
1:	bdnz	1b

#endif

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:     cmplwi  r6,IE_ENABBATS
	bne	99f

	LDCONST(r4,(M_MSR_DR|M_MSR_IR))
	mfmsr	r3
	or	r3,r3,r4
	mtmsr	r3
	isync
	sync

	b	initnxt
#endif
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:     cmplwi  r6,IE_ENABICACHE
	bne	99f

	mfspr	r4,SPR_HID0
	LDCONST(r3,(M_HID0_ICE|M_HID0_ICFI))
	andc	r4,r4,r3
	ori	r3,r4,(M_HID0_ICE|M_HID0_ICFI)
	ori	r4,r4,(M_HID0_ICE)

	isync
	mtspr	SPR_HID0,r3
	mtspr	SPR_HID0,r4
	isync

	b	initnxt
#endif
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if 1
99:     cmplwi  r6,IE_ENABDCACHE
	bne	99f

	mfspr	r4,SPR_HID0
	li	r3,(M_HID0_DCE|M_HID0_DCFI)
	andc	r4,r4,r3
	ori	r3,r4,(M_HID0_DCE|M_HID0_DCFI)
	ori	r4,r4,(M_HID0_DCE)

	sync
	isync
	mtspr	SPR_HID0,r3
	isync
	mtspr	SPR_HID0,r4
	isync

	b	initnxt
#endif
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

99:
initnxt:

	addi	tblptr,tblptr,R_IE_CMDSIZE
	b	initlp

initdone:
	blr

END(init_engine)
