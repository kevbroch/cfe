/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU init module				File: sb1250_altcpu.S
    *
    *  Secondary core startup routines for CFE
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
#include "exception.h"

#include "bsp_config.h"

#ifdef _CFE_
#include "cfe_devfuncs.h"
#else
#define CFE_EPTSEAL 0x43464531
#endif

#include "sb1250_defs.h"
#include "sb1250_regs.h"
#include "sb1250_scd.h"

#include "cpu_config.h"

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#include "mipsmacros.h"

#define SETLEDS1(a,b,c,d)                     \
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
       JAL_KSEG1(board_setleds)
#define SETLEDS(a,b,c,d)                     \
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
       JAL(board_setleds)


/*  *********************************************************************
    *  Initialized Data
    ********************************************************************* */

                .sdata

/*
 * Initial start addresses for secondary CPUs
 */

		.globl cpu_startvectors
cpu_startvectors:
		.dword	0			# cpu #0 (not used)
		.dword	0			# cpu #1

/*
 * Initial values for SP, GP, and A1 (user argument) 
 */

cpu_start_spvals:
		.dword	0			# cpu #0 (not used)
		.dword	0			# cpu #1

cpu_start_gpvals:
		.dword	0			# cpu #0 (not used)
		.dword	0			# cpu #1

cpu_start_args:
		.dword	0			# cpu #0 (not used)
		.dword	0			# cpu #1


		.extern mem_datareloc


/*  *********************************************************************
    *  Linkage Tables
    * 
    *  This table contains pointers to routines in other modules.
    *  we do things this way so we can stay position-independent and
    *  also avoid problems with the limitations of relative branching.
    ********************************************************************* */

		.text

		.set mips64

/*  *********************************************************************
    *  ALTCPU_KSEG1_SWITCH
    *  
    *  Hack the return address so we will come back in KSEG1 (uncached)
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

altcpu_kseg1_switch:

		and	ra,(K0SIZE-1)
		or	ra,K1BASE
		jr	ra


/*  *********************************************************************
    *  ALTCPU_KSEG0_SWITCH
    *  
    *  Hack the return address so we will come back in KSEG0
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

altcpu_kseg0_switch:

		and	ra,(K0SIZE-1)
		or	ra,K0BASE
		jr	ra


/*  *********************************************************************
    *  SB1250_ALTCPU_START1
    *  
    *  Start secondary processor(s).  These processors will start
    *  running the code at ALTCPU_RESET (see below).  We wait here
    *  for the secondary processor(s) to finish their cache
    *  initialization and then  return.
    *
    *  This routine is normally run from KSEG1
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(sb1250_altcpu_start1)

	/*
	 * Don't do this if we have only one CPU.  This way we can
	 * support running the multiprocessor version of CFE
	 * with only one core.
	 */

sb1250_altcpu_start1a:

		la	t0,PHYS_TO_K1(A_SCD_SYSTEM_REVISION)
		ld	t0,(t0)			# Get system revision
		dsrl	t0,S_SYS_PART		# Shift part # to low bits
		dsrl	t0,8			# isolate CPU part of number
		and	t0,0x0F			# T0 = number of CPUs
		bgt	t0,1,1f			# Keep going if more than one CPU
		j	ra			# Go back home, nothing to do
1:

	/*
	 * Clear out our mailbox registers (both CPUs)
	 */

		la	a0,PHYS_TO_K1(A_IMR_REGISTER(0,R_IMR_MAILBOX_CLR_CPU))
		dli	t0,-1			# clear all 64 bits
		sd	t0,(a0)
		la	a0,PHYS_TO_K1(A_IMR_REGISTER(1,R_IMR_MAILBOX_CLR_CPU))
		sd	t0,(a0)

	/*
	 * Let the secondary CPU(s) out of reset
	 * 
	 * XXX This is very SB1250-specific at the moment.
	 */

		la	a0,PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0,0(a0)
		dli	t1,M_SYS_CPU_RESET_1	# Reset mask
		not	t1			# clear this bit
		and	t0,t1			# New value to write
		sd	t0,0(a0)		# CPU1 is now running

	/*
	 * Wait for the other CPU to ring our doorbell
	 */


1:		la	a0,PHYS_TO_K1(A_IMR_REGISTER(0,R_IMR_MAILBOX_CPU));
		ld	t0,(a0)			# Read mailbox
		beq	t0,zero,1b		# Loop till the bit is set

	/*
	 * Clear the mailbox to dismiss the pending interrupts
	 */

		la	a0,PHYS_TO_K1(A_IMR_REGISTER(0,R_IMR_MAILBOX_CLR_CPU))
		dli	t0,-1			# clear all 64 bits
		sd	t0,(a0)

	/*
	 * Okay, it's safe to return
	 */

		j	ra


END(sb1250_altcpu_start1)

/*  *********************************************************************
    *  SB1250_ALTCPU_START2
    *  
    *  Finish startup of secondary processor(s) - we pass the relocation
    *  offset to the other CPUs here, and the CPUs relocate their
    *  data segments and go to the idle loop.
    *
    *  This routine is normally run from KSEG0
    *  
    *  Input parameters: 
    *  	   a0 - data relocation offset (0=none)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(sb1250_altcpu_start2)

	/*
	 * Don't do this if we have only one CPU. 
	 */

		la	t0,PHYS_TO_K1(A_SCD_SYSTEM_REVISION)
		ld	t0,(t0)			# Get system revision
		dsrl	t0,S_SYS_PART		# Shift part # to low bits
		dsrl	t0,8			# isolate CPU part of number
		and	t0,0x0F			# T0 = number of CPUs
		bgt	t0,1,1f			# Keep going if more than one CPU
		j	ra			# Go back home, nothing to do
1:

	/*
	 * Let secondary CPU(s) run their idle loops.  Set the 
	 * mailbox register to our relocation factor so we can read
	 * it out of the mailbox register and relocate GP properly.
	 */

		la	t1,PHYS_TO_K1(A_IMR_REGISTER(1,R_IMR_MAILBOX_SET_CPU))
		or	t0,a0,1		# hack - make sure reloc is nonzero
		sd	t0,0(t1)	# Write to mailbox register

		j	ra	

END(sb1250_altcpu_start2)

/*  *********************************************************************
    *  SB1250_ALTCPU_KILL
    *  
    *  Kill a secondary CPU, causing it to return to the idle
    *  loop.  We do this by switching to uncached mode, 
    *  asserting RESET on the other CPU, and then re-run
    *  ALTCPU_START again.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(sb1250_altcpu_kill)

	/*
	 * Don't do this if we have only one CPU. 
	 */

		la	t0,PHYS_TO_K1(A_SCD_SYSTEM_REVISION)
		ld	t0,(t0)			# Get system revision
		dsrl	t0,S_SYS_PART		# Shift part # to low bits
		dsrl	t0,8			# isolate CPU part of number
		and	t0,0x0F			# T0 = number of CPUs
		bgt	t0,1,1f			# Keep going if more than one CPU
		j	ra			# Go back home, nothing to do
1:

	/*
	 * More than one CPU, go ahead...
	 */

		move	t7,ra			# save RA, we'll make calls

#ifdef _SB1250_PASS1_WORKAROUNDS_
       #
       # Not sure what we need to do here wrt cacheability of
       # the genbus space, if anything.  Some portion of CPU1's
       # istream will come from L1, but the data should all be from
       # DRAM.  These references will be cacheable noncoherent,
       # should we worry if cpu0 is coherent shared at this time?
       # probably.
       #
#endif

		la	t1,cpu_startvectors
		sd	zero,8(t1)		# Reset address of CPU1 (2nd entry in table)

	#
	# Flush the D cache to ensure that the write above made it 
	# out of our L1.
	#

		JAL(sb1250_l1cache_flush_d)	# uses t0, t2, t3

	#
	# Switch to KSEG1 to quiesce our cache activity.
	#

		bal	altcpu_kseg1_switch	# switch to uncached mode

	#
	# Force CPU1 into reset
	#

		li	a0,PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0,0(a0)
		dli	t1,M_SYS_CPU_RESET_1	# Reset mask
		or	t0,t1			# New value to write
		sd	t0,0(a0)		# CPU1 is now in reset

	#
	# Not sure how long we're supposed to wait.
	#
		ssnop
		ssnop
		ssnop
		ssnop

	#
	# Now restart CPU1
	#

		bal	sb1250_altcpu_start1a

	#
	# It's safe to be cached again.
	#

		bal	altcpu_kseg0_switch

	#
	# At this point, CPU1 is waiting for us to indicate that it's
	# okay to use memory again.  Ring its doorbell.
	#

		la	a0,PHYS_TO_K1(A_IMR_REGISTER(1,R_IMR_MAILBOX_SET_CPU))
		LR	t0,mem_datareloc
		or	t0,1
		sd	t0,0(a0)

	#
	# CPU1 is back in our control.
	#

		move	ra,t7
		move	v0,zero

		j	ra


END(sb1250_altcpu_kill)


/*  *********************************************************************
    *  ALTCPU_RESET
    *  
    *  Start address for secondary CPU(s) - do the initialization of
    *  the local CPU and then notify CPU0 that we're done.
    *
    *  This routine is called in KSEG1.
    *  
    *  Input parameters: 
    *  	   t0 - CPU identifier
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */


LEAF(sb1250_altcpu_reset)


		mfc0	t0,C0_PRID		# get CPU PRID register
		and	t0,t0,0xe000000		# determine cpu number
		beq	t0,zero,iscpu0		# go if  on CPU0


#if CFG_RELOC
       /*
        * SVR4 PIC mode: get a copy of GP for use in the boot ROM.
	*/
		lui	gp,%hi(_gp)
		addiu	gp,%lo(_gp)
		or	gp,gp,K1BASE
#endif

	/*
	 * Note: we should never get to the CPU1 code if we're
	 * with only one CPU.  Theoretically, nobody got past the
	 * check in altcpu_start.  But, just in case, if we
	 * get here and we're on CPU1, and we supposedly only
	 * have one CPU, reset CPU1.
	 */

		la	t0,PHYS_TO_K1(A_SCD_SYSTEM_REVISION)
		ld	t0,(t0)			# Get system revision
		dsrl	t0,S_SYS_PART		# Shift part # to low bits
		dsrl	t0,8			# isolate CPU part of number
		and	t0,0x0F			# T0 = number of CPUs
		beq	t0,1,iscpu0		# If only one CPU, kill off CPU1


	/*
	 * Initialize CPU registers.
	 */

		JAL_KSEG1(sb1_cpu_init)

#ifdef _SB1250_PASS1_WORKAROUNDS_
	/*
	 * See the above note about this workaround.  It's important
	 * to remain noncoherent until we're done with the flash.
	 */

		SETCCAMODE(v0,K_CFG_K0COH_CACHEABLE) /* cacheable NONCOHERENT */
#endif

		SETLEDS1('C','P','U','1')

	/*
	 * Initialize the L1 cache
	 */

#if CFG_INIT_L1
		JAL_KSEG1(sb1250_l1cache_init)
#endif


	/*
	 * Notify the SCD that we're done initializing.  Do this by 
	 * ringing CPU0's doorbell.
	 */

		la	a0,PHYS_TO_K1(A_IMR_REGISTER(0,R_IMR_MAILBOX_SET_CPU));

		mfc0	t0,C0_PRID		# get processor number
		srl	t0,t0,25		# shift CPU bits into low
		and	t0,t0,7			# keep only low 3 bits
		li	t1,1			# make a bit mask depending on CPU
		sll	t1,t1,t0		# calculate t1 = 1 shl cpu number
		sd	t1,0(a0)		# set corresponding bit in mailbox


	 /*
	  * Go to the idle loop
	  */

		b	altcpu_idle		# go to idle loop		


   	 /*
	  * We get here if we were running on CPU0.  Make things
	  * pretty for the reset of CPU initialization.
	  */


iscpu0:		

	/*
	 * If we are on CPU0, then force CPU1 into reset.  This is needed
	 * for the case where the firmware has crashed and we need to get
	 * control of the system again.
	 */

		li	a0,PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0,0(a0)
		dli	t1,M_SYS_CPU_RESET_1	# Reset mask
		or	t0,t1			# New value to write
		sd	t0,0(a0)		# CPU1 is now in reset

		j	ra			# return (we were on CPU 0)

END(sb1250_altcpu_reset)


/*  *********************************************************************
    *  ALTCPU_CMD_START(cpu,addr)
    *  
    *  Start an alternate CPU.
    *  
    *  Input parameters: 
    *  	   a0 - cpu number (must be 1 for the SB1250)
    *  	   a1 - pointer to start parameters (four 64-bit values)
    *             array[0] = start address (PC)
    *             array[1] = start stack pointer (SP)
    *             array[2] = start global pointer (GP)
    *             array[3] = start user argument (A1)
    *  	   
    *  Return value:
    *  	   v0 - 0 if ok
    *  	   else -1 if request could not be handled
    ********************************************************************* */

#define R_CPUSTART_PCVAL  0
#define R_CPUSTART_SPVAL  8
#define R_CPUSTART_GPVAL  16
#define R_CPUSTART_A1VAL  24

LEAF(altcpu_cmd_start)

		li	v0,-1		/* assume failure */
		bne	a0,1,1f		/* go if not CPU 1 */

	/*
	 * Return an error if running in uniprocessor mode.
	 */

		la	t0,PHYS_TO_K1(A_SCD_SYSTEM_REVISION)
		ld	t0,(t0)			# Get system revision
		dsrl	t0,S_SYS_PART		# Shift part # to low bits
		dsrl	t0,8			# isolate CPU part of number
		and	t0,0x0F			# T0 = number of CPUs
		beq	t0,1,1f			# If only one CPU, error.

	/*
	 * Multiprocessor mode, start the other CPU
	 */

		move	t0,a0		/* get CPU number */
		sll	t0,3		/* multiply by 8 for table index */

		la	t1,cpu_start_gpvals
		add	t1,t0		/* copy the GP value */
		ld	t2,R_CPUSTART_GPVAL(a1)
		sd	t2,0(t1)

		la	t1,cpu_start_spvals
		add	t1,t0		/* copy the SP value */
		ld	t2,R_CPUSTART_SPVAL(a1)
		sd	t2,0(t1)

		la	t1,cpu_start_args
		add	t1,t0		/* copy the A1 value */
		ld	t2,R_CPUSTART_A1VAL(a1)
		sd	t2,0(t1)

		la	t1,cpu_startvectors
		add	t1,t0		/* copy the PC value */
		ld	t2,R_CPUSTART_PCVAL(a1)
		sd	t2,0(t1)

		move	v0,zero		/* success */

1:		j	ra

END(altcpu_cmd_start)

/*  *********************************************************************
    *  ALTCPU_CMD_STOP(cpu)
    *  
    *  Stop the specified CPU.
    *  
    *  We don't really support this at the moment.
    *  
    *  Input parameters: 
    *  	   a0 - cpu number
    *  	   
    *  Return value:
    *  	   v0 - 0 if ok, else error code
    ********************************************************************* */

LEAF(altcpu_cmd_stop)

		li	v0,-1		/* assume failure */
		bne	a0,1,1f		/* go if not CPU 1 */

	/*
	 * Return an error if running in uniprocessor mode.
	 */

		la	t0,PHYS_TO_K1(A_SCD_SYSTEM_REVISION)
		ld	t0,(t0)			# Get system revision
		dsrl	t0,S_SYS_PART		# Shift part # to low bits
		dsrl	t0,8			# isolate CPU part of number
		and	t0,0x0F			# T0 = number of CPUs
		beq	t0,1,1f			# If only one CPU, error.

	/*
	 * Multiprocessor mode, stop the other CPU
	 */

		JMP(sb1250_altcpu_kill)	/* kill the CPU */

1:		j	ra

END(altcpu_cmd_stop)

/*  *********************************************************************
    *  ALTCPU_IDLE
    *  
    *  Loop forever waiting for someone to tell us where to go.
    *  
    *  Input parameters: 
    *  	   nothing.
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

altcpu_idle:	

	/*
	 * Switch to KSEG0 (cached)
	 */

		bal	altcpu_kseg0_switch

#if CFG_RELOC
		lui	gp,%hi(_gp)
		addiu	gp,%lo(_gp)
#endif

		SETLEDS('c','p','u','1')


	/*
	 * Now wait for CPU0 to ring *our* doorbell.  This is our signal that
	 * it's safe to go to the idle loop.  Until CPU0 rings our
	 * doorbell, we can't use memory (but we can use the cache).
	 * XXX Very SB1250 specific here.
	 */

1:		la	a0,PHYS_TO_K1(A_IMR_REGISTER(1,R_IMR_MAILBOX_CPU))
		ld	t0,(a0)			# Read mailbox
		beq	t0,zero,1b		# Loop till the bit is set

	/*
	 * Clear all the bits in the mailbox register to dismiss the 
	 * pending interrupt 
	 */

		la	a0,PHYS_TO_K1(A_IMR_REGISTER(1,R_IMR_MAILBOX_CLR_CPU))
		li	t1,-1
		sd	t1,0(a0)


	/*
	 * We may need GP, especially in relocated version
	 *
	 * Yucky hack: The relocation factor was passed to us in
	 * the mailbox register, which is conveniently in t0 right now.
	 * (except the lower bit is set just in case the reloc was
	 * zero, so clear that first).
	 */

		li	t1,1			# 1
		not	t1			# FFFFFFFFE
		and	t0,t1			# clear lower bit.


#if (CFG_RELOC)

	/*
	 * SVR4 PIC: GP is already set to GOT, relocate
	 * it and jump to relocated code
	 */
__AltCpuGoRel:

		la	t1,1f			# Get address of where to go
		ADDU	gp,t0			# Relocate GP
		ADDU	t1,t0			# Relocate address
		jr	t1			# Go there.

1:						# we will go "here" in the reloc world
#else
	 /*
	  * non-PIC: Standard GP
	  */
		la	gp,_gp
		ADD	gp,t0			# relocate GP.
#endif


	/*
	 * Get our processor number
	 */

		mfc0	t0,C0_PRID		# Get PRID (for processor id)

		srl	t0,t0,25		# shift CPU bits into low
		and	t0,t0,7			# keep only low 3 bits
		sll	t0,t0,3			# multiply by 8 for table index

	/*
	 * Set up registers like we were launching a program.
	 */

		la	a2,cpu_apientry # A2 = firmware entry vector
		move	a0,gp		# A0 = handle
		li	a3,CFE_EPTSEAL  # A3 = entrypoint signature


#ifdef _SB1250_PASS1_WORKAROUNDS_
	/*
	 * Okay, it's safe now to be coherent.  
	 * Flush the D cache to invalidate all the lines we have,
	 * then change the config register back.
	 *
	 * Danger! It's imperative that *no stores to memory* be done
	 * prior to this point, otherwise flushing the cache
	 * will race with core 0, which will also be flushing
	 * lines at this time.
	 */
		move	k0,t0
		JAL(sb1250_l1cache_flush_d)
		SETCCAMODE(v0,K_CFG_K0COH_COHERENT) /* cacheable coherent */
		move    t0,k0
#endif

	/*
	 * Read the start address from the CPU restart table
	 * and jump to it.  For an idle CPU, the address in the
	 * table below will be the zero, causing
	 * the CPU to loop forever.  To start a secondary CPU,
	 * just write an address in cpu_startvectors[cpu_id]
	 *
	 * Warning: This kind of assumes that this code will 
	 * live in cacheable space.  If it doesn't, it will
	 * probably cause lots of unwanted bus traffic.
	 */
		li	s4,0

loop_forever:	ld	t1,cpu_startvectors(t0) # Load address of routine
		beq	t1,zero,loop_forever
		ld	a1,cpu_start_args(t0)	# Load user argument (A1)
		ld	sp,cpu_start_spvals(t0)	# Load stack pointer
		ld	t2,cpu_start_gpvals(t0) # Load global pointer
		move	gp,t2			# and put in real register
		j	t1			# jump to start address


/*  *********************************************************************
    *  End
    ********************************************************************* */
