/*  *********************************************************************
    *  SB1250 Board Support Package
    *  
    *  L1C initialization			File: bcmcore_l1cache.S
    *  
    *  This module contains code for various CPU cache operations
    *  
    *  Note: the init routines in this module rely on registers only,
    *        since DRAM may not be active yet.
    *
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001
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
#include "bsp_config.h"


		.text

		.set push	
		.set mips32

/*  *********************************************************************
    *  Macros
    ********************************************************************* */


#define BCMCORE_CACHELINE_SIZE  16

#define L1CACHEOP(cachename,op) ((cachename) | ((op) << 2))

#define L1C_OP_IDXINVAL     0
#define L1C_OP_IDXLOADTAG   1
#define L1C_OP_IDXWBINV	    1	/* for the DCache */
#define L1C_OP_IDXSTORETAG  2
#define L1C_OP_IMPLRSVD     3
#define L1C_OP_HITINVAL     4
#define L1C_OP_FILL         5	/* for the ICache */
#define L1C_OP_HITWBINVAL   5	/* for the DCache */
#define L1C_OP_HITWRITEBACK 6
#define L1C_OP_FETCHLOCK    7

#define L1C_I		    0
#define L1C_D		    1


/*  *********************************************************************
    *  BCMCORE_L1CACHE_INIT()
    *  
    *  Initialize the L1 Cache
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    *  
    *  Registers used:
    *  	   t0,t1,t2
    ********************************************************************* */

LEAF(bcmcore_l1cache_init)

	/*
	 * Size the icache
	 */

		mfc0	t0,C0_CONFIG,1

	/* 
	 * Sets Per Way = 64 << CFG_IS
	 */

		li	t2,M_CFG_IS
		and	t2,t0
		srl	t2,S_CFG_IS
		li	t1,64
		sll	t1,t2

	/*
	 * Associativity = 1 + CFG_IA
	 */

		li	t2,M_CFG_IA
		and	t2,t0
		srl	t2,S_CFG_IA
		addiu	t2,1

	/* 
	 * Associativity * Sets Per Way
	 */

		multu	t1,t2
		mflo	t1
	
	/* 
	 * Line Size = 2 << CFG_IL
	 */

		li	t2,M_CFG_IL
		and	t2,t0
		srl	t2,S_CFG_IL
		li	t0,2
		sll	t2,t0,t2
		
	/* 
	 * Instruction Cache Size = 
	 * Associativity * Line Size * Sets Per Way
	 */

		multu	t1,t2
		mflo	t0

	/*
	 * Zero the icache
	 */

		mtc0	zero,C0_TAGLO
		mtc0	zero,C0_TAGHI
		li	t1,K0BASE
1:		sub	t0,t0,t2
		cache	Index_Store_Tag_I,0(t1)
		add	t1,t2
		bne	t0,zero,1b

	/*
	 * Size the dcache
	 */

		mfc0	t0,C0_CONFIG,1

	/* 
	 * Sets Per Way = 64 << CFG_DS
	 */

		li	t2,M_CFG_DS
		and	t2,t0
		srl	t2,S_CFG_DS
		li	t1,64
		sll	t1,t2


	/*
	 * Associativity = 1 + CFG_DA
	 */

		li	t2,M_CFG_DA
		and	t2,t0
		srl	t2,S_CFG_DA
		addiu	t2,1

	/* 
	 * Associativity * Sets Per Way
	 */

		multu	t1,t2
		mflo	t1
	
	/* 
	 * Line Size = 2 << CFG_DL
	 */

		li	t2,M_CFG_DL
		and	t2,t0
		srl	t2,S_CFG_DL
		li	t0,2
		sll	t2,t0,t2
		
	/* 
	 * Data Cache Size = 
	 * Associativity * Line Size * Sets Per Way
	 */

		multu	t1,t2
		mflo	t0

	/*
	 * Zero the dcache
	 */

		mtc0	zero,C0_TAGLO
		mtc0	zero,C0_TAGHI
		li	t1,K0BASE
1:		sub	t0,t0,t2
		cache	Index_Store_Tag_D,0(t1)
		add	t1,t2
		bne	t0,zero,1b

		jr	ra

END(bcmcore_l1cache_init)

/*  *********************************************************************
    *  BCMCORE_L1CACHE_INVAL_I()
    *  
    *  Invalidate the entire ICache
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    *  
    *  Registers used:
    *  	   t0,t1,t2
    ********************************************************************* */

LEAF(bcmcore_l1cache_inval_i)
		
	/*
	 * Size the icache
	 */

		mfc0	t0,C0_CONFIG,1

	/* 
	 * Sets Per Way = 64 << CFG_IS
	 */

		li	t2,M_CFG_IS
		and	t2,t0
		srl	t2,S_CFG_IS
		li	t1,64
		sll	t1,t2

	/*
	 * Associativity = 1 + CFG_IA
	 */

		li	t2,M_CFG_IA
		and	t2,t0
		srl	t2,S_CFG_IA
		addiu	t2,1

	/* 
	 * Associativity * Sets Per Way
	 */

		multu	t1,t2
		mflo	t1
	
	/* 
	 * Line Size = 2 << CFG_IL
	 */

		li	t2,M_CFG_IL
		and	t2,t0
		srl	t2,S_CFG_IL
		li	t0,2
		sll	t2,t0,t2
		
	/* 
	 * Instruction Cache Size = 
	 * Associativity * Line Size * Sets Per Way
	 */

		multu	t1,t2
		mflo	t0

	/*
	 * Invalidate the icache
	 */

		mtc0	zero,C0_TAGLO
		mtc0	zero,C0_TAGHI
		li	t1,K0BASE
1:		sub	t0,t0,t2
		cache	Index_Invalidate_I,0(t1)
		add	t1,t2
		bne	t0,zero,1b

		j	ra

END(bcmcore_l1cache_inval_i)

/*  *********************************************************************
    *  BCMCORE_L1CACHE_FLUSH_D()
    *  
    *  Flush the entire DCache
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    *  
    *  Registers used:
    *  	   t0,t1,t2,t3
    ********************************************************************* */

LEAF(bcmcore_l1cache_flush_d)

	/*
	 * Size the dcache
	 */

		mfc0	t0,C0_CONFIG,1

	/* 
	 * Sets Per Way = 64 << CFG_DS
	 */

		li	t2,M_CFG_DS
		and	t2,t0
		srl	t2,S_CFG_DS
		li	t1,64
		sll	t1,t2


	/*
	 * Associativity = 1 + CFG_DA
	 */

		li	t2,M_CFG_DA
		and	t2,t0
		srl	t2,S_CFG_DA
		addiu	t2,1

	/* 
	 * Associativity * Sets Per Way
	 */

		multu	t1,t2
		mflo	t1
	
	/* 
	 * Line Size = 2 << CFG_DL
	 */

		li	t2,M_CFG_DL
		and	t2,t0
		srl	t2,S_CFG_DL
		li	t0,2
		sll	t2,t0,t2
		
	/* 
	 * Data Cache Size = 
	 * Associativity * Line Size * Sets Per Way
	 */

		multu	t1,t2
		mflo	t0

	/*
	 * Flush the dcache
	 */

		mtc0	zero,C0_TAGLO
		mtc0	zero,C0_TAGHI
		li	t1,K0BASE
1:		sub	t0,t0,t2
#ifdef BCM4710
		/* HNBU PR2795: out-of-order write workaround */
		li	t3,0xb8000f90
		lw	t3,0(t3)
#endif
		cache	Index_Writeback_Inv_D,0(t1)
		add	t1,t2
		bne	t0,zero,1b

		j	ra

END(bcmcore_l1cache_flush_d)


		.set	noreorder
		.set	noat
	
#define LINESIZE BCMCORE_CACHELINE_SIZE
	
/*  *********************************************************************
    *  BCMCORE_SYNC_RANGE(start, length)
    *  
    *  Make memory range consistent with L1 DCache
    *  
    *  Input parameters: 
    *  	   a0 - base address
    *  	   a1 - length
    *	
    *  Return value:
    *  	   nothing
    *  
    *  Registers used:
    *  	   a0, a1, t0, t1
    ********************************************************************* */

LEAF(bcmcore_sync_range)

		beq	a1,zero,2f

	/*
	 * At least one line.  Compute aligned start and limit.
	 */
		li	t0,~(LINESIZE-1)
		add	a1,a0,a1
		and	a0,a0,t0
		addi	a1,a1,LINESIZE-1
		and	a1,a1,t0
		addi	a0,a0,LINESIZE

	/*
	 * This and the lw below are workarounds for HND PR2795 apparently.
	 */
		li	t0,0xb8000f90	/* must be uncached */

	/*
	 * Flush the lines in L1 D
	 */
1:		lw	t1,0(t0)
		cache	L1CACHEOP(L1C_D,L1C_OP_HITWBINVAL),-LINESIZE(a0)
		bne	a0,a1,1b
		addi	a0,a0,LINESIZE
	
2:		j	ra
		nop

END(bcmcore_sync_range)


/*  *********************************************************************
    *  BCMCORE_INVAL_RANGE(start, length)
    *  
    *  Flush a memory range from the L1 DCache
    *  
    *  Input parameters: 
    *  	   a0 - base address
    *  	   a1 - length
    *	
    *  Return value:
    *  	   nothing
    *  
    *  Registers used:
    *  	   a0,a1,t0
    *
    ********************************************************************* */

LEAF(bcmcore_inval_range)

		beq	a1,zero,2f

	/*
	 * At least one line.  Compute aligned start and limit.
	 */
		li	t0,~(LINESIZE-1)
		add	a1,a0,a1
		and	a0,a0,t0
		addi	a1,a1,LINESIZE-1
		and	a1,a1,t0
		addi	a0,a0,LINESIZE

	/*
	 * Invalidate the lines in L1 D
	 */
1:		cache	L1CACHEOP(L1C_D,L1C_OP_HITINVAL),-LINESIZE(a0)
		bne	a0,a1,1b
		addi	a0,a0,LINESIZE
	
2:		j	ra
		nop

END(bcmcore_inval_range)


		.set pop

/*  *********************************************************************
    *  End
    ********************************************************************* */

