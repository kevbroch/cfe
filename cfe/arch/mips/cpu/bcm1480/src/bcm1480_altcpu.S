/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU init module				File: bcm1480_altcpu.S
    *
    *  Secondary core startup routines for CFE
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
#include "exception.h"

#include "bsp_config.h"

#ifdef _CFE_
#include "cfe_devfuncs.h"
#else
#define CFE_EPTSEAL 0x43464531
#endif

#include "bcm1480_regs.h"
#include "bcm1480_scd.h"

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

#define SETLEDS1_ADD(a,b,c,d,add)			\
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;	\
       addu   a0, a0, add ;				\
       JAL_KSEG1(board_setleds)
#define SETLEDS_ADD(a,b,c,d,add)			\
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;	\
       addu   a0, a0, add ;				\
       JAL(board_setleds)

/*
 * Read system revision register to get # of CPUs on this part
 *   Do this by reading CPU disables from system_cfg,
 *   and using that to index a bit array to get the number of
 *   cpus enabled (minus 1).
 */ 
#define GET_NUM_CPUS(reg, tmp) 					\
		la	reg,PHYS_TO_K1(A_SCD_SYSTEM_CFG)	; \
		ld	reg,(reg)				; \
		dsrl	reg,(S_BCM1480_SYS_DISABLECPU0)		; \
		and	reg,0x0f				; \
		li	tmp, 0x0116166b				; \
		srl	tmp, tmp, reg				; \
		srl	tmp, tmp, reg   /* really, twice */	; \
		andi	tmp, tmp, 0x3				; \
		addiu	reg, tmp, 1

#define GET_CUR_CPU(reg) \
		mfc0	reg,C0_PRID ; \
		srl	reg,reg,25 ; \
		and	reg,reg,7 

  


/*  *********************************************************************
    *  Initialized Data
    ********************************************************************* */

                .sdata

#define R_CPU_STARTVECT	(0*8)
#define R_CPU_SPVAL	(1*8)
#define R_CPU_GPVAL	(2*8)
#define R_CPU_ARG	(3*8)
#define R_CPU_RECSIZE	32		/* One cache line per record */

/*
 * Initial start addresses for secondary CPUs.  Each record below
 * is on its own cache line.  That cache line will be held exclusive
 * by the owning CPU and only the snoop traffic should show up
 * on the bus.
 */

		.align 5
		.globl	cpu_idledata
cpu_idledata:
		.dword 0,0,0,0			# cpu #0 (not used)
		.dword 0,0,0,0			# cpu #1
		.dword 0,0,0,0			# cpu #2
		.dword 0,0,0,0			# cpu #3

		.extern mem_datareloc

		.text

		.set mips64


/*  *********************************************************************
    *  BCM1480_NUM_CPUS
    *  
    *  Return the number of CPUs which are enabled on this
    *  BCM1x80/BCM1x55 SOC.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   v0 = number of CPUs enabled
    ********************************************************************* */

LEAF(bcm1480_num_cpus)

		GET_NUM_CPUS(v0, t0)

		jr	ra

END(bcm1480_num_cpus)


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

LEAF(altcpu_kseg1_switch)
_altcpu_kseg1_switch:

		and	ra,(K0SIZE-1)
		or	ra,K1BASE
		jr	ra

END(altcpu_kseg1_switch)

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

LEAF(altcpu_kseg0_switch)
_altcpu_kseg0_switch:

		and	ra,(K0SIZE-1)
		or	ra,K0BASE
		jr	ra

END(altcpu_kseg0_switch)

/*  *********************************************************************
    *  BCM1480_ALTCPU_START1
    *  
    *  Start secondary processor(s).  These processors will start
    *  running the code at ALTCPU_RESET (see below).  We wait here
    *  for the secondary processor(s) to finish their cache
    *  initialization and then  return.
    *
    *  For simplicity, we serialize starting the secondary processors
    *  and waiting until done for now.
    *
    *  This routine is normally run from KSEG1
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(bcm1480_altcpu_start1)

	/*
	 * Do this only for CPUs with non-zero processor IDs, possibly
	 * none if we have only one CPU.  This way we can
	 * support running the multiprocessor version of CFE
	 * with only one core.
	 */

		GET_NUM_CPUS(v0, t0)
		b	3f			# loop test

	/*
	 * Main loop, CPU number is in v0.
	 */
1:

	/*
	 * Clear out our mailbox registers (both CPUs)
	 * XXX - need to do all CPUs and both mailboxes
	 */

		la	a0,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CLR_CPU))
		dli	t0,-1			# clear all 64 bits
		dli	t1,BCM1480_IMR_REGISTER_SPACING
		sd	t0,(a0)
		multu	v0,t1
		mflo	t1
		daddu	a0,a0,t1
		sd	t0,(a0)

	/*
	 * Let the secondary CPU(s) out of reset
	 * 
	 * XXX This is very BCM1480-specific at the moment.
	 */

		la	a0,PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0,0(a0)
		dli	t1,M_BCM1480_SYS_CPU_RESET_0	# Base reset mask
		dsll	t1,t1,v0
		not	t1			# clear this bit
		and	t0,t1			# New value to write
		BCM1480_WRITE_SYSTEM_CFG(a0, t0)
						# CPU[v0] is now running

	/*
	 * Wait for the other CPU to ring our doorbell
	 */


2:		la	a0,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CPU));
		ld	t0,(a0)			# Read mailbox
		beq	t0,zero,2b		# Loop till the bit is set

	/*
	 * Clear the mailbox to dismiss the pending interrupts
	 */

		la	a0,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CLR_CPU))
		dli	t0,-1			# clear all 64 bits
		sd	t0,(a0)

3:
		daddiu	v0,v0,-1
		bne	v0,zero,1b		# Keep going if more secondary CPUs
	
	/*
	 * Okay, it's safe to return
	 */

		j	ra

END(bcm1480_altcpu_start1)

/*  *********************************************************************
    *  BCM1480_ALTCPU_START2
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

LEAF(bcm1480_altcpu_start2)

	/*
	 * Do this for all secondary CPUs (if any)
	 */

		GET_NUM_CPUS(v0, t0)
		b	2f

	/*
	 * Main loop, CPU number is in v0.
	 */
1:

	/*
	 * Let secondary CPU(s) run their idle loops.  Set the 
	 * mailbox register to our relocation factor so we can read
	 * it out of the mailbox register and relocate GP properly.
	 */

		la	t1,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_SET_CPU))
		dli	t2,BCM1480_IMR_REGISTER_SPACING
		or	t0,a0,1		# hack - make sure reloc is nonzero
		multu	v0,t2
		mflo	t2
		daddu	t1,t1,t2
		sd	t0,0(t1)	# Write to mailbox register

2:
		daddiu	v0,v0,-1
		bne	v0,zero,1b		# Keep going if more secondary CPUs
		j	ra	

END(bcm1480_altcpu_start2)

/*  *********************************************************************
    *  BCM1480_ALTCPU_KILL
    *  
    *  Kill a secondary CPU, causing it to return to the idle
    *  loop.  We do this by switching to uncached mode, 
    *  asserting RESET on the other CPU, and then re-run
    *  ALTCPU_START again.
    *  
    *  Input parameters: 
    *  	   a0 - cpu number to stop
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(bcm1480_altcpu_kill)
_bcm1480_altcpu_kill:

	/*
	 * Don't do this if we have only one CPU. 
	 */

		GET_NUM_CPUS(t0, t1)
		bgt	t0,1,1f			# Keep going if more than one CPU
		j	ra			# Go back home, nothing to do
1:

	/*
	 * More than one CPU, go ahead...
	 */

		move	t7,ra			# save RA, we'll make calls

	#
	# Stop CPU[a0], where a0 is the cpu number.
	#

		move	t0,a0			# t0 = cpu #
		sll	t0,t0,5			# index by cache lines
		la	t1,cpu_idledata		# 
		add	t1,t0			# point at cpu[a0]'s data

		sd	zero,R_CPU_STARTVECT(t1)  # Reset address of CPU1

	#
	# Flush the D cache to ensure that the write above made it 
	# out of our L1.
	#

		JAL(bcm1480_l1cache_flush_d)	# uses t0, t2, t3

	#
	# Switch to KSEG1 to quiesce our cache activity.
	#

		bal	_altcpu_kseg1_switch	# switch to uncached mode

	#
	# Force CPU into reset (cpu number should still be in a0)
	#

		li	t2,PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0,0(t2)
		dli	t1,M_BCM1480_SYS_CPU_RESET_0	# Base Reset mask
		dsll	t1,t1,a0		# Shift by processor number
		or	t0,t1			# New value to write
		BCM1480_WRITE_SYSTEM_CFG(t2, t0)
						# CPUn is now in reset

	#
	# Not sure how long we're supposed to wait.
	#
		ssnop
		ssnop
		ssnop
		ssnop

	#
	# Now restart CPUn.  We can't use "altcpu_start1" here because
	# we're probably restarting only one of 'n' cpus.
	#


	/*
	 * Clear out our mailbox registers (both CPUs we're interested in)
	 */

		la	t2,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CLR_CPU))
		dli	t0,-1			# clear all 64 bits
		dli	t1,BCM1480_IMR_REGISTER_SPACING
		sd	t0,(t2)			# do CPU0 
		multu	a0,t1
		mflo	t1
		daddu	t2,t2,t1
		sd	t0,(t2)			# do CPU[a0]

	/*
	 * Let the secondary CPU(s) out of reset
	 * 
	 * XXX This is very BCM1480-specific at the moment.
	 */

		la	t2,PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0,0(t2)
		dli	t1,M_BCM1480_SYS_CPU_RESET_0	# Base reset mask
		dsll	t1,t1,a0
		not	t1			# clear this bit
		and	t0,t1			# New value to write
		BCM1480_WRITE_SYSTEM_CFG(t2, t0)
						# CPU[v0] is now running

	/*
	 * Wait for the other CPU to ring our doorbell
	 */


2:		la	t2,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CPU));
		ld	t0,(t2)			# Read mailbox
		beq	t0,zero,2b		# Loop till the bit is set

	/*
	 * Clear the mailbox to dismiss the pending interrupts
	 */

		la	t2,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CLR_CPU))
		dli	t0,-1			# clear all 64 bits
		sd	t0,(t2)


	#
	# It's safe to be cached again.
	#

		bal	_altcpu_kseg0_switch

	#
	# At this point, CPU1 is waiting for us to indicate that it's
	# okay to use memory again.  Ring its doorbell.
	#

		la	t2,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_SET_CPU))
		dli	t1,BCM1480_IMR_REGISTER_SPACING
		multu	a0,t1
		mflo	t1
		daddu	t2,t2,t1
		LR	t0,mem_datareloc
		or	t0,1
		sd	t0,(t2)

	#
	# CPU[a0] is back in our control.
	#

		move	ra,t7
		move	v0,zero

		j	ra


END(bcm1480_altcpu_kill)


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


LEAF(bcm1480_altcpu_reset)


		GET_CUR_CPU(t0)			# read current cpu # to t0
		beq	t0,zero,iscpu0		# go if on CPU0

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

		GET_NUM_CPUS(t0, t1)
		beq	t0,1,iscpu0		# If only one CPU, kill off CPU1


	/*
	 * Initialize CPU registers.
	 */
		JAL_KSEG1(sb1_cpu_init)

		GET_CUR_CPU(t1)
		SETLEDS1_ADD('C','P','U','0', t1)

	/*
	 * Initialize the L1 cache
	 */

#if CFG_INIT_L1
		JAL_KSEG1(bcm1480_l1cache_init)
#endif


	/*
	 * Notify the SCD that we're done initializing.  Do this by 
	 * ringing CPU0's doorbell.
	 */

		la	a0,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_SET_CPU));

		GET_CUR_CPU(t0)			# get our processor # to t0
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
	 * If we are on CPU0, then force secondary CPUs1 into reset.
	 *  This is needed for the case where the firmware has crashed
	 * and we need to get control of the system again.
	 */

		li	a0,PHYS_TO_K1(A_SCD_SYSTEM_CFG)
		ld	t0,0(a0)
		dli	t1,(M_BCM1480_SYS_CPU_RESET_1|M_BCM1480_SYS_CPU_RESET_2|M_BCM1480_SYS_CPU_RESET_3)
		or	t0,t1			# New value to write
		BCM1480_WRITE_SYSTEM_CFG(a0, t0)
						# 2ndary CPUs are now in reset

		j	ra			# return (we were on CPU 0)

END(bcm1480_altcpu_reset)


/*  *********************************************************************
    *  ALTCPU_CMD_START(cpu,addr)
    *  
    *  Start an alternate CPU.
    *  
    *  Input parameters: 
    *  	   a0 - cpu number (must be 1-3 for the BCM1480)
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
		beq	a0,0,2f		/* can't start CPU0 with this routine */

	/*
	 * Return an error if running in uniprocessor mode.
	 */

		GET_NUM_CPUS(t0, t1)
		beq	t0,1,2f			# If only one CPU, error.

	/*
	 * Multiprocessor mode, start the other CPU
	 */

		move	t0,a0		/* get CPU number */
		sll	t0,5		/* shift left 5 bits for cache offset */
		la	t1,cpu_idledata
		add	t1,t0		/* t1 = desired CPU's idle data */

		ld	t2,R_CPUSTART_GPVAL(a1)
		sd	t2,R_CPU_GPVAL(t1)

		ld	t2,R_CPUSTART_SPVAL(a1)
		sd	t2,R_CPU_SPVAL(t1)

		ld	t2,R_CPUSTART_A1VAL(a1)
		sd	t2,R_CPU_ARG(t1)

		ld	t2,R_CPUSTART_PCVAL(a1)	/* this one actually starts the CPU */
		sd	t2,R_CPU_STARTVECT(t1)

		move	v0,zero		/* success */

		j	ra

	/*
	 * Error return - invalid cpu number
	 */

2:
		li	v0,-1
		j	ra

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

		beq	a0,zero,1f		# can't stop CPU0

	/*
	 * Return an error if running in uniprocessor mode.
	 */

		GET_NUM_CPUS(t0, t1)		# t0 = number of CPUs
		beq	t0,1,1f			# If only one CPU, error.
		bge	a0,t0,1f		# Range check CPU number

	/*
	 * Multiprocessor mode, stop the other CPU (a0 = cpu number)
	 */

		b	_bcm1480_altcpu_kill	/* kill the CPU */

1:		li	v0,-1
		j	ra

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

#if CFG_RELOC
		lui	gp,%hi(_gp)
		addiu	gp,%lo(_gp)

	/* On versions of the 1480 prior to B0, lines in the data cache
	 * can be written back to flash even when clean.  This is bad,
	 * especially for Intel-style flash.  Keep lines with GOT 
	 * entries out of the cache until running from RAM.
	 */
		or	gp,gp,K1BASE
#endif

		GET_CUR_CPU(t1)
		SETLEDS1_ADD('c','p','u','0', t1)


	/*
	 * Now wait for CPU0 to ring *our* doorbell.  This is our signal that
	 * it's safe to go to the idle loop.  Until CPU0 rings our
	 * doorbell, we can't use memory or the cache (since other CPUs
	 * caches may not be initted yet).
	 * XXX Very BCM1480 specific here.
	 */

1:		la	a0,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CPU))

		GET_CUR_CPU(t0)

		li	t1,BCM1480_IMR_REGISTER_SPACING
		multu	t0,t1
		mflo	t1
		daddu	a0,a0,t1
		ld	t0,(a0)			# Read mailbox
		beq	t0,zero,1b		# Loop till the bit is set

	/*
	 * Switch to KSEG0 (cached)
	 */

		bal	_altcpu_kseg0_switch

	/*
	 * Clear all the bits in the mailbox register to dismiss the 
	 * pending interrupt 
	 */

		daddiu	a0,a0,(R_BCM1480_IMR_MAILBOX_0_CLR_CPU-R_BCM1480_IMR_MAILBOX_0_CPU)
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
	 * Now that we can talk to memory again, get the "text relocation"
	 * and move the loop into DRAM.
	 */

__AltCpuGoRel:

		la	t1,1f			# Get address of where to go
	/*
	 * Switch to a cacheable version of the GOT (see above).  This
	 * needs to be done after the la above but before relocation 
	 * of gp.
	 */
		lui	gp,%hi(_gp)
		addiu	gp,%lo(_gp)
		ADDU	gp,t0			# Relocate GP

		ADDU	t1,t0			# Relocate PC
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
	 * Get our processor number, and calculate address of cpu's idle data
	 */

		GET_CUR_CPU(t0)			# processor number to t0

		sll	t0,t0,5			# Multiply by 32 for cache offset
		la	t1,cpu_idledata		# address of base of table
		addu	t0,t1			# t0 = our cpu data

	/*
	 * Set up registers like we were launching a program.
	 */

		la	a2,cpu_apientry # A2 = firmware entry vector
		move	a0,gp		# A0 = handle
		li	a3,CFE_EPTSEAL  # A3 = entrypoint signature


	/*
	 * Read the start address from the CPU restart table
	 * and jump to it.  For an idle CPU, the address in the
	 * table below will be the zero, causing
	 * the CPU to loop forever.  To start a secondary CPU,
	 * just write an address in cpu_idledata[cpu_id].start_vector
	 *
	 * Warning: This kind of assumes that this code will 
	 * live in cacheable space.  If it doesn't, it will
	 * probably cause lots of unwanted bus traffic.
	 */
		li	s4,0

loop_forever:	ld	t1,R_CPU_STARTVECT(t0)  # Load address of routine
		beq	t1,zero,loop_forever
		ld	a1,R_CPU_ARG(t0)	# Load user argument (A1)
		ld	sp,R_CPU_SPVAL(t0)	# Load stack pointer
		ld	t2,R_CPU_GPVAL(t0)	# Load global pointer
		move	gp,t2			# and put in real register
		j	t1			# jump to start address


/*  *********************************************************************
    *  End
    ********************************************************************* */
