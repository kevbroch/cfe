/*  *********************************************************************
    *  SB1250 Board Support Package
    *  
    *  L1 Cache initialization			File: sb1250_l1cache.S
    *  
    *  This module contains code to initialize the L1 cache.
    *  
    *  Note: all the routines in this module rely on registers only,
    *        since DRAM may not be active yet.
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

#include "sbmips.h"
#include "cpu_config.h"
#include "mipsmacros.h"


		.text

		.set	mips64


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#define L1CACHE_NUMWAYS	4
#define L1CACHE_NUMIDX  256
#define L1CACHE_LINESIZE 32
#define L1CACHE_IDXHIGH (L1CACHE_LINESIZE*L1CACHE_NUMWAYS*L1CACHE_NUMIDX)

#define L1CACHEOP(cachename,op) ((cachename) | ((op) << 2))

#define L1C_OP_IDXINVAL     0
#define L1C_OP_IDXLOADTAG   1
#define L1C_OP_IDXSTORETAG  2
#define L1C_OP_IMPLRSVD     3
#define L1C_OP_HITINVAL     4
#define L1C_OP_FILL         5
#define L1C_OP_HITWRITEBACK 6
#define L1C_OP_FETCHLOCK    7

#define L1C_I		    0
#define L1C_D		    1


#ifndef HAZARD
#define HAZARD ssnop ;  ssnop ;  ssnop ;  ssnop ;  ssnop ;  ssnop ;  ssnop
#endif





/*  *********************************************************************
    *  SB1250_L1CACHE_INIT()
    *  
    *  Initialize the L1 Cache tags to be "invalid"
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

	
LEAF(sb1250_l1cache_init)

_sb1250_l1cache_init:

		dmtc0	zero,C0_TAGLO
		dmtc0	zero,C0_TAGHI

		li	t2,K1BASE
#ifdef _FASTINIT_
		li	t3,L1CACHE_LINESIZE*8			/* only 8 indicies now */
#else
		li	t3,L1CACHE_IDXHIGH
#endif

		add     t0,t2,t3
		.align	4
1:		cache   L1CACHEOP(L1C_I,L1C_OP_IDXSTORETAG),0(t2)
		addu    t2,L1CACHE_LINESIZE
		bne     t0,t2,1b

		dmtc0	zero,C0_TAGLO,2
		dmtc0	zero,C0_TAGHI,2

		li	t2,K1BASE
		add     t0,t2,t3
		.align	4
1:		cache   L1CACHEOP(L1C_D,L1C_OP_IDXSTORETAG),0(t2)
		addu    t2,L1CACHE_LINESIZE
		bne     t0,t2,1b

		j	ra

END(sb1250_l1cache_init)


#if CFG_VAPI

LEAF(sb1250_reset_defeature)

		j	ra

END(sb1250_reset_defeature)

#endif


/*  *********************************************************************
    *  SB1250_L1CACHE_INVAL_I()
    *  
    *  Invalidate the L1 ICache
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

	
LEAF(sb1250_l1cache_inval_i)


		li	t2,K1BASE
		li	t3,L1CACHE_IDXHIGH

		add     t0,t2,t3
		.align	4
1:		cache   L1CACHEOP(L1C_I,L1C_OP_IDXINVAL),0(t2)
		addu    t2,L1CACHE_LINESIZE
		bne     t0,t2,1b

		j	ra

END(sb1250_l1cache_inval_i)


/*  *********************************************************************
    *  SB1250_L1CACHE_FLUSH_D()
    *  
    *  Flush the entire L1 DCache (write dirty lines back to memory)
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

	
LEAF(sb1250_l1cache_flush_d)

		li	t2,K1BASE
		li	t3,L1CACHE_IDXHIGH

		li	t2,K1BASE
		add     t0,t2,t3
		.align	4
1:		cache   L1CACHEOP(L1C_D,L1C_OP_IDXINVAL),0(t2)
		addu    t2,L1CACHE_LINESIZE
		bne     t0,t2,1b

		sync
		sync				/* pass1 issue. */

		j	ra

END(sb1250_l1cache_flush_d)


/*  *********************************************************************
    *  sb1250_sync_range(ptr,len)
    *  
    *  Flush specified range from L1 cache
    *  
    *  Input parameters: 
    *  	   a0 - pointer to start of buffer
    *  	   a1 - length of buffer
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(sb1250_sync_range)

1:	cache	L1CACHEOP(L1C_D,L1C_OP_HITWRITEBACK),0(a0)
	addu	a0,a0,L1CACHE_LINESIZE
	sub	a1,a1,L1CACHE_LINESIZE
	bge	a1,zero,1b		

	j	ra

END(sb1250_sync_range)


/*  *********************************************************************
    *  sb1250_inval_range(ptr,len)
    *  
    *  Invalidate specified range from L1 cache
    *  
    *  Input parameters: 
    *  	   a0 - pointer to start of buffer
    *  	   a1 - length of buffer
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(sb1250_inval_range)

1:	cache	L1CACHEOP(L1C_D,L1C_OP_HITINVAL),0(a0)
	addu	a0,a0,L1CACHE_LINESIZE
	sub	a1,a1,L1CACHE_LINESIZE
	bge	a1,zero,1b		

	j	ra

END(sb1250_inval_range)

/*  *********************************************************************
    *  End
    ********************************************************************* */

