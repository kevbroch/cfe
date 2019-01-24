/*  *********************************************************************
    *  BM1280/BCM1480 Board Support Package
    *  
    *  L2 Cache initialization			File: bcm1480_l2cache.S
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
    *  Copyright 2000,2001,2002,2003,2004
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
#include "bcm1480_l2c.h"
#include "bcm1480_mc.h"
#include "bcm1480_scd.h"

		.text
		.set mips64


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#define CACHE_LINE_SIZE	  32

/*  *********************************************************************
    *  BCM1480_L2CACHE_INIT()
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


LEAF(bcm1480_l2cache_init)
_bcm1480_l2cache_init:

#ifdef _BIGSUR_
       /*
        * This is a special hack just for the BCM1480 bringup board
	* (customers, don't use this!).  For testing we can hold the
	* L2 cache in reset, but the BCM1480 does not provide a way to
	* test for that at runtime.  We will use the least significant
	* config bit for this purpose.
	*/
		li	t0, PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0, 0(t0)
		and	t0, t0, (1<<S_BCM1480_SYS_CONFIG)
		beq	zero, t0, 1f
		jr	ra
1:
#endif

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

		dli	t0,PHYS_TO_XKSEG_UNCACHED(A_BCM1480_L2C_MGMT_TAG_BASE)

	# Loop through each entry and each way

#ifdef _FASTINIT_
		li	t1,16
#else
		li	t1,BCM1480_L2C_ENTRIES_PER_WAY*BCM1480_L2C_NUM_WAYS
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

#ifdef _BCM1480_PASS1_WORKAROUNDS_
	# S0 Erratum SOC-9: Doing a mgmt access to way 7 stops random
	# replacement from working correctly.  To work around this
	# we simply zero a line in another way again.  (This won't
	# negatively impact operation on other revs, so we don't
	# conditionalize it at run-time.)
	
		dli	t0,PHYS_TO_XKSEG_UNCACHED(A_BCM1480_L2C_MGMT_TAG_BASE)
		sd	zero,0(t0)
#endif

	#
	# Restore old KX bit setting
	#

		mtc0	t2,C0_SR
		HAZARD

		j	ra		# return to caller

END(bcm1480_l2cache_init)


/*  *********************************************************************
    *  BCM1480_L2CACHE_DISABLE()
    *
    *  Convert the entire L2 Cache into static memory, for use by
    *  the bootstrap loader.  Actually, it only removes seven of the
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


LEAF(bcm1480_l2cache_disable)

#ifdef _BIGSUR_
	# Do nothing (return immediately) if L2 has been disabled via JTAG.
	# See comments in bcm1480_l2cache_init.

		li	t0, PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0, 0(t0)
		and	t0, t0, (1<<S_BCM1480_SYS_CONFIG)
		beq	zero, t0, 1f
		jr	ra
1:
#endif

	# Save the old status register, and set the KX bit.
	# Configure the L2 cache as SRAM (all ways disabled except one)
	# Do a memory reference at the "way_disable" address
	# to switch it off.
	# Warning: do NOT try to configure all of the ways off - you
	# must leave at least one way active!  This code leaves
	# way #7 active and gives ways 0..6 to the program.

		li	t0,PHYS_TO_K1(A_BCM1480_L2_MAKE_WAY_ENABLE_LO(0x0))
		sd	zero,(t0)
		li	t0,PHYS_TO_K1(A_BCM1480_L2_MAKE_WAY_ENABLE_HI(0x8))
		sd	zero,(t0)

	# Use the result of the load to stall the pipe here.
	# Ref sec 5.4.2
	# XXX is this necessary for global enable/disable operations?

		ld	t0,(t0)
		addu	t0,t0,t0

	# Re-write all the tags

		b	_bcm1480_l2cache_init

END(bcm1480_l2cache_disable)


/*  *********************************************************************
    *  BCM1480_L2CACHE_ENABLE()
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

LEAF(bcm1480_l2cache_enable)

#ifdef _BIGSUR_
	# Do nothing (return immediately) if L2 has been disabled via JTAG.
	# See comments in bcm1480_l2cache_init.

		li	t0, PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0, 0(t0)
		and	t0, t0, (1<<S_BCM1480_SYS_CONFIG)
		beq	zero, t0, 1f
		jr	ra
1:
#endif

	# Save the old status register, and set the KX bit.
	# Configure the L2 cache as Cache (all ways enabled)
	# Do a memory reference at the "way_disable" address
	# to switch it on.

		li	t0,PHYS_TO_K1(A_BCM1480_L2_MAKE_WAY_ENABLE_LO(0xF))
		sd	zero,(t0)
		li	t0,PHYS_TO_K1(A_BCM1480_L2_MAKE_WAY_ENABLE_HI(0xF))
		sd	zero,(t0)

	# Use the result of the load to stall the pipe here.
	# Ref sec 5.4.2
	# XXX is this necessary for global enable/disable operations?

		ld	t0,(t0)
		addu	t0,t0,t0

	# Re-write all the tags

		b	_bcm1480_l2cache_init

END(bcm1480_l2cache_enable)


/*  *********************************************************************
    *  BCM1480_L2CACHE_FLUSH()
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
    *      t0,t1,t2,t3,t4: scratch
    *      t5: saved SR
    *      t6: MC initial state information
    ********************************************************************* */


LEAF(bcm1480_l2cache_flush)

#ifdef _BIGSUR_
	# Do nothing (return immediately) if L2 has been disabled via JTAG.

		li	t0, PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0, 0(t0)
		and	t0, t0, (1<<S_BCM1480_SYS_CONFIG)
		beq	zero, t0, 1f
		jr	ra
1:
#endif

	# Save the old status register, and set the KX bit.

		mfc0	t5,C0_SR
		or	t0,t5,M_SR_KX
		mtc0	t0,C0_SR
		HAZARD

	#
	# Set the BERR_DISABLE bits in the memory controller.  We're
	# going to do cacheable reads where there is no memory.
	# Also, turn off ECC.  We may be reading garbage, so we don't
	# want ECC errors.
	#

		move	t6, zero

		# disable buserrs in global config, and put a bit
		# into t6 to indicate whether we actually changed the
		# value.
		la	t0, PHYS_TO_K1(A_BCM1480_MC_GLB_CONFIG)
		dli	t1, M_BCM1480_MC_BERR_DISABLE
		ld	t2, 0(t0)
		or	t3, t1, t2
		beq	t2, t3, 1f
		ori	t6, t6, 1
		sd	t3, 0(t0)
1:

		# disable ECC errors in each channel, and put bits
		# into t6 to indicate whether we actually changed the
		# channels.
		la	t0, PHYS_TO_K1(A_BCM1480_MC_BASE(0))
		li	t4, 4
2:
		dli	t1, M_BCM1480_MC_ECC_DISABLE
		ld	t2, (R_BCM1480_MC_DRAMMODE)(t0)
		or	t3, t1, t2
		sll	t6, t6, 1
		beq	t2, t3, 3f
		ori	t6, t6, 1
		sd	t3, (R_BCM1480_MC_DRAMMODE)(t0)
3:
		daddiu	t0, t0, BCM1480_MC_REGISTER_SPACING
		daddiu	t4, t4, -1
		bnez	t4, 2b

		sync

		# t6 now contains:
		# bit 4: buserr needs clr
		# bit 3: chan 0 ecc disable needs clear
		# bit 2: chan 1 ecc disable needs clear
		# ...

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
	# We use PAs starting at 0F_0000_0000 (in the middle
	# of the memory expansion area) for the cacheable reads.
	# They'll return garbage data, but we're just going to
	# invalidate afterward.
	#
	# Note that if the cacheable read is done to an address that
	# is present in the cache, the victim way will *not* be ejected
	# (since there's no need to victimize it).

		dli	t0, PHYS_TO_XKPHYS(K_CALG_UNCACHED,
					   (A_BCM1480_L2C_MGMT_TAG_BASE
					    | V_BCM1480_L2C_MGMT_ECC_DIAG(1)))
		dli	t1, PHYS_TO_XKPHYS(K_CALG_COH_SHAREABLE, 0x0f00000000)
		li	t2, (BCM1480_L2C_ENTRIES_PER_WAY \
			     * BCM1480_L2C_NUM_WAYS * CACHE_LINE_SIZE)

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


		dli	t0,PHYS_TO_XKSEG_UNCACHED(A_BCM1480_L2C_MGMT_TAG_BASE)
		li	t1,BCM1480_L2C_ENTRIES_PER_WAY*BCM1480_L2C_NUM_WAYS

	# Write a zero to the cache management register at each
	# address.

1:		sd	zero,0(t0)
		sd	zero,CACHE_LINE_SIZE(t0)
		sd	zero,2*CACHE_LINE_SIZE(t0)
		sd	zero,3*CACHE_LINE_SIZE(t0)
		daddu	t0,(4*CACHE_LINE_SIZE) # size of a cache line
		subu	t1,4
		bne	t1,0,1b

#ifdef _BCM1480_PASS1_WORKAROUNDS_
	# S0 Erratum SOC-9: Doing a mgmt access to way 7 stops random
	# replacement from working correctly.  To work around this
	# we simply zero a line in another way again.  (This won't
	# negatively impact operation on other revs, so we don't
	# conditionalize it at run-time.)
		dli	t0,PHYS_TO_XKSEG_UNCACHED(A_BCM1480_L2C_MGMT_TAG_BASE)
		sd	zero,0(t0)
#endif

	#
	# Restore the old MC register values and turn the bus errors back on.
	#

		# t6 contains:
		# bit 4: buserr needs clr
		# bit 3: chan 0 ecc disable needs clear
		# bit 2: chan 1 ecc disable needs clear
		# ...
		# so we work backwards to restore state.

		# re-enable ECC errors in each channel as appropriate
		# based on the values in t6.
		la	t0, PHYS_TO_K1(A_BCM1480_MC_BASE(3))
		li	t4, 4
1:		andi	t1, t6, 1
		beqz	t1, 2f
		dli	t1, M_BCM1480_MC_ECC_DISABLE
		ld	t2, (R_BCM1480_MC_DRAMMODE)(t0)
		xor	t3, t1, t2
		sd	t3, (R_BCM1480_MC_DRAMMODE)(t0)
2:		srl	t6, t6, 1
		daddiu	t0, t0, -BCM1480_MC_REGISTER_SPACING
		daddiu	t4, t4, -1
		bnez	t4, 1b

		# re-enable ECC errors in global config as appropriate
		# based on the value in t6.
		andi	t1, t6, 1
		beqz	t1, 3f
		la	t0, PHYS_TO_K1(A_BCM1480_MC_GLB_CONFIG)
		dli	t1, M_BCM1480_MC_BERR_DISABLE
		ld	t2, 0(t0)
		xor	t3, t1, t2
		sd	t3, 0(t0)
3:

		sync


	#
	# Restore old KX bit setting
	#

		mtc0	t5,C0_SR
		HAZARD

		j	ra		# return to caller

END(bcm1480_l2cache_flush)




/*  *********************************************************************
    *  End
    ********************************************************************* */
