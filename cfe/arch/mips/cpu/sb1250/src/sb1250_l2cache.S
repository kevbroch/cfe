/*  *********************************************************************
    *  SB1250 Board Support Package
    *  
    *  L2 Cache initialization			File: sb1250_l2cache.S
    *  
    *  This module contains code to initialize the L2 cache.
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
#include "sb1250_regs.h"
#include "sb1250_l2c.h"
#include "sb1250_mc.h"
#include "sb1250_scd.h"


		.text
		.set mips64


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

/*#define PHYS_TO_XKPHYS(x) (0x9000000000000000|(x))*/
#define CACHE_LINE_SIZE	  32
#ifndef HAZARD
#define HAZARD ssnop ; ssnop ; ssnop ; ssnop ; ssnop ; ssnop ; ssnop
#endif



/*  *********************************************************************
    *  SB1250_L2CACHE_INIT()
    *
    *  Initialize the L2 Cache tags to be "invalid"
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


LEAF(sb1250_l2cache_init)

_sb1250_l2cache_init:

	# Do nothing (return immediately) if L2 has been disabled via JTAG.

		li	t0, PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0, 0(t0)
		and	t0, t0, M_SYS_L2C_RESET
		beq	zero, t0, 1f
		jr	ra
1:

	# Save the old status register, and set the KX bit.

		mfc0	t2,C0_SR
		or	t1,t2,M_SR_KX
		mtc0	t1,C0_SR
		HAZARD

	# Start the index at the base of the cache management
	# area, but leave the address bit for "Valid" zero.
	# Note that the management tags are at 00_D000_0000,
	# which cannot be expressed with the PHYS_TO_K1 macro,
	# so we will need to use a 64-bit address to get to it.

		dli	t0,PHYS_TO_XKSEG_UNCACHED(A_L2C_MGMT_TAG_BASE)

	# Loop through each entry and each way

#ifdef _FASTINIT_
		li	t1,16
#else
		li	t1,L2C_ENTRIES_PER_WAY*L2C_NUM_WAYS
#endif


	# Write a zero to the cache management register at each
	# address.

		.align 4
1:		sd	zero,0(t0)
		sd	zero,CACHE_LINE_SIZE(t0)
		sd	zero,2*CACHE_LINE_SIZE(t0)
		sd	zero,3*CACHE_LINE_SIZE(t0)
		daddu	t0,(4*CACHE_LINE_SIZE) # size of a cache line
		subu	t1,4
		bne	t1,0,1b

	#
	# Restore old KX bit setting
	#

		mtc0	t2,C0_SR
		HAZARD

		j	ra		# return to caller

END(sb1250_l2cache_init)


/*  *********************************************************************
    *  SB1250_L2CACHE_DISABLE()
    *
    *  Convert the entire L2 Cache into static memory, for use by
    *  the bootstrap loader.  Actually, it only removes three of the
    *  ways, since you must leave at least one way active at all
    *  times.
    *
    *  Input parameters:
    *  	   nothing
    *
    *  Return value:
    *  	   nothing
    *
    *  Registers used:
    *  	   t0,t1
    ********************************************************************* */


LEAF(sb1250_l2cache_disable)

	# Do nothing (return immediately) if L2 has been disabled via JTAG.

		li	t0, PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0, 0(t0)
		and	t0, t0, M_SYS_L2C_RESET
		beq	zero, t0, 1f
		jr	ra
1:

	# Save the old status register, and set the KX bit.
	# Configure the L2 cache as SRAM (all ways disabled except one)
	# Do a memory reference at the "way_disable" address
	# to switch it off.
	# Warning: do NOT try to configure all of the ways off - you
	# must leave at least one way active!  This code leaves
	# way #3 active and gives ways 0..2 to the program.

		li	t0,PHYS_TO_K1(A_L2_MAKEDISABLE(0x07))
		sd	zero,(t0)
		ld	t0,(t0)

	# Use the result of the load to stall the pipe here.
	# Ref sec 5.4.2
	# XXX is this necessary for global enable/disable operations?

		addu	t0,t0,t0

	# Re-write all the tags

		b	_sb1250_l2cache_init

END(sb1250_l2cache_disable)


/*  *********************************************************************
    *  SB1250_L2CACHE_ENABLE()
    *
    *  Convert the L2 Cache memory into the actual L2 cache, enabling
    *  the cache for future memory accesses.
    *
    *  Input parameters:
    *  	   nothing
    *
    *  Return value:
    *  	   nothing
    *
    *  Registers used:
    *  	   t0,t1
    ********************************************************************* */

LEAF(sb1250_l2cache_enable)

	# Do nothing (return immediately) if L2 has been disabled via JTAG.

		li	t0, PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0, 0(t0)
		and	t0, t0, M_SYS_L2C_RESET
		beq	zero, t0, 1f
		jr	ra
1:

	# Save the old status register, and set the KX bit.
	# Configure the L2 cache as Cache (all ways enabled)
	# Do a memory reference at the "way_disable" address
	# to switch it on.

		li	t0,PHYS_TO_K1(A_L2_MAKEDISABLE(0x0))
		ld	t0,(t0)

	# Use the result of the load to stall the pipe here.
	# Ref sec 5.4.2
	# XXX is this necessary for global enable/disable operations?

		addu	t0,t0,t0

	# Re-write all the tags

		b	_sb1250_l2cache_init

END(sb1250_l2cache_enable)


/*  *********************************************************************
    *  SB1250_L2CACHE_FLUSH()
    *
    *  Flush the entire L2 cache.  All dirty lines are written back
    *  out to memory.
    *
    *  Input parameters:
    *  	   nothing
    *
    *  Return value:
    *  	   nothing
    *
    *  Registers used:
    *      t0, t1, t2, t3: scratch
    *      t5: saved SR
    *      t6, t7: saved MC0,1 config values.
    ********************************************************************* */

LEAF(sb1250_l2cache_flush)

	# Do nothing (return immediately) if L2 has been disabled via JTAG.

		li	t0, PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0, 0(t0)
		and	t0, t0, M_SYS_L2C_RESET
		beq	zero, t0, 1f
		jr	ra
1:

	# Save the old status register, and set the KX bit.

		mfc0	t5,C0_SR
		or	t0,t5,M_SR_KX
		mtc0	t0,C0_SR
		HAZARD

	#
	# Set the BERR bits in both memory controllers.  We're
	# going to do cacheable reads where there is no memory.
	#
	#
	# Note that on an 1125, we can still do this on MC 0 even
	# though there is only one memory controller.  The register
	# is there, it just ignores the bits we're trying to write.
	#

		li	t0,PHYS_TO_K1(A_MC_REGISTER(0,R_MC_CONFIG))
		ld	t6,0(t0)
		dli	t1,(M_MC_BERR_DISABLE | M_MC_ECC_DISABLE)
		or	t1,t1,t6
		sd	t1,0(t0)

		li	t0,PHYS_TO_K1(A_MC_REGISTER(1,R_MC_CONFIG))
		ld	t7,0(t0)
		dli	t1,(M_MC_BERR_DISABLE | M_MC_ECC_DISABLE)
		or	t1,t1,t7
		sd	t1,0(t0)


	# Flush all of the lines in the cache
	#
	# We use the following algorithm, for each way and index of
	# the cache:
	#
	# * do a management mode access to select a victim way.
	#
	# * do a cacheable read of an address not in the cache.
	#
	# The index used in the second read, in the selected victim way,
	# will be replaced with the data from the cacheable read.
	#
	# We use PAs starting at 30_0000_0000 (in the middle
	# of the memory expansion area) for the cacheable reads.
	# They'll return garbage data, but we're just going to
	# invalidate afterward.
	#
	# Note that if the cacheable read is done to an address that
	# is present in the cache, the victim way will *not* be ejected
	# (since there's no need to victimize it).

		dli	t0, PHYS_TO_XKPHYS(K_CALG_UNCACHED,
					   (A_L2C_MGMT_TAG_BASE
					    | V_L2C_MGMT_ECC_DIAG(1)))
		dli	t1, PHYS_TO_XKPHYS(K_CALG_COH_SHAREABLE, 0x3000000000)
		li	t2, (L2C_ENTRIES_PER_WAY * L2C_NUM_WAYS \
			     * CACHE_LINE_SIZE)

1:
		daddiu	t2, -CACHE_LINE_SIZE

		# Select the line to be victimized, and wait for the
		# read data to return.
		or	t3, t0, t2
		ld	t3, 0(t3)
		daddu	t3, t3, zero

		# Read the high-memory flush address for this line, and
		# wait for the read data to return.
		or	t3, t1, t2
		ld	t3, 0(t3)
		daddu	t3, t3, zero

		bnez	t2, 1b


	#
	# Now, invalidate the entire cache.  Of course, we could just
	# reinit the lines we flushed, but this routine is mucking
	# the entire cache anyway, so it doesn't matter.
	#

		dli	t0,PHYS_TO_XKSEG_UNCACHED(A_L2C_MGMT_TAG_BASE)
		li	t1,L2C_ENTRIES_PER_WAY*L2C_NUM_WAYS

	# Write a zero to the cache management register at each
	# address.

1:		sd	zero,0(t0)
		sd	zero,CACHE_LINE_SIZE(t0)
		sd	zero,2*CACHE_LINE_SIZE(t0)
		sd	zero,3*CACHE_LINE_SIZE(t0)
		daddu	t0,(4*CACHE_LINE_SIZE) # size of a cache line
		subu	t1,4
		bne	t1,0,1b

	#
	# Restore the old MC register values
	#


		li	t0,PHYS_TO_K1(A_MC_REGISTER(0,R_MC_CONFIG))
		sd	t6,0(t0)

		li	t0,PHYS_TO_K1(A_MC_REGISTER(1,R_MC_CONFIG))
		sd	t7,0(t0)

	#
	# Restore old KX bit setting
	#

		mtc0	t5,C0_SR
		HAZARD

		j	ra		# return to caller

END(sb1250_l2cache_flush)




/*  *********************************************************************
    *  End
    ********************************************************************* */
