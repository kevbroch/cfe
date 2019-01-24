/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU Configuration file			File: cpu_config.h
    *  
    *  This file contains the names of the routines to be used
    *  in the dispatch table in init_mips.S
    *
    *  It lives here in the CPU directory so we can direct
    *  the init calls to routines named in this directory.
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

/*
 */

#define CPUCFG_CPUINIT		bcm1480_cpuinit
#define CPUCFG_ALTCPU_START1	bcm1480_altcpu_start1
#define CPUCFG_ALTCPU_START2	bcm1480_altcpu_start2
#define CPUCFG_ALTCPU_RESET	bcm1480_altcpu_reset
#define CPUCFG_CPURESTART	bcm1480_cpurestart
#define CPUCFG_DRAMINIT		bcm1480_dram_init
#define CPUCFG_CACHEOPS		bcm1480_cacheops
#define CPUCFG_ARENAINIT	sb1250_arena_init
#define CPUCFG_PAGETBLINIT	sb1250_pagetable_init
#define CPUCFG_TLBHANDLER	bcm1480_tlbhandler
#define CPUCFG_CERRHANDLER	bcm1480_cerrhandler
#define CPUCFG_CPUSPEED		bcm1480_cpu_speed
#define CPUCFG_SYNCRANGE	bcm1480_sync_range
#define CPUCFG_INVALRANGE	bcm1480_inval_range

#ifdef _FUNCSIM_
#define CPUCFG_DIAG_TEST1	diag_null
#else
#define CPUCFG_DIAG_TEST1	diag_null
#endif
#define CPUCFG_DIAG_TEST2	0

/*
 * Hazard macro
 */

#define HAZARD .set push ; .set mips64 ; ssnop ; ssnop ; ssnop ; ssnop ; ssnop ; ssnop ; ssnop ; .set pop
#define ERET eret

/*
 * Let others know we can do coherent DMA
 */

#define CPUCFG_COHERENT_DMA	1

#define CPUCFG_REGS32	0	/* GPRs are 64 bits */
#define CPUCFG_REGS64	1

#define CPUCFG_ARCHNAME		"MIPS"
#define CPUCFG_ELFTYPE		EM_MIPS


/*
 * Workaround for BCM1480 S0 erratum SOC-111:
 *
 * When writing to the system config register, do two dummy writes to
 * another SCD CSR first (RO CSRs are OK, we choose the system revision
 * register in this case since it's easy).
 *
 * This makes the assumption that when this code is run, other processors
 * will **NOT** be writing any SCD registers.  (If other processors do,
 * this workaround may not be successful.)
 *
 * This workaround doesn't need to be conditional on the chip rev
 * actually in use; it will cause on harm on revisions in which this
 * erratum is fixed.
 */

#if _BCM1480_PASS1_WORKAROUNDS_
#define BCM1480_WRITE_SYSTEM_CFG(areg, dreg)				 \
	sd	dreg, (A_SCD_SYSTEM_REVISION - A_SCD_SYSTEM_CFG)(areg) ; \
	sd	dreg, (A_SCD_SYSTEM_REVISION - A_SCD_SYSTEM_CFG)(areg) ; \
	sd	dreg, 0(areg)
#else
#define BCM1480_WRITE_SYSTEM_CFG(areg, dreg)				 \
	sd	dreg, 0(areg)
#endif


#ifndef __LANGUAGE_ASSEMBLY

/*
 * Exported functions which are BCM1480-family specific.
 */

int	bcm1480_num_cpus(void);

#endif /* __LANGUAGE_ASSEMBLY */
