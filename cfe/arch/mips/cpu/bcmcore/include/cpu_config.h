/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU Configuration file			File: cpu_config.h
    *  
    *  This file contains the names of the routines to be used in the
    *  dispatch table in init_mips.S It lives here in the CPU
    *  directory so we can direct the init calls to routines named in
    *  this directory.
    *
    *  It also contains implementation-specific definitions for the
    *  MIPS32 and BMIPS-3300 cores used in certain Broadcom 47xx
    *  (SiliconBackplane) products.
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

#ifndef _CPU_CONFIG_H
#define _CPU_CONFIG_H

#define CPUCFG_ARCHNAME		"MIPS"
#define CPUCFG_ELFTYPE		EM_MIPS

/*  *********************************************************************
    *  Map the function names
    ********************************************************************* */

#define CPUCFG_CPUINIT		bcmcore_cpuinit
#define CPUCFG_ALTCPU_START1	bcmcore_null
#define CPUCFG_ALTCPU_START2	bcmcore_null
#define CPUCFG_ALTCPU_RESET	bcmcore_null
#define CPUCFG_CPURESTART	bcmcore_cpurestart
#define CPUCFG_DRAMINIT		board_draminit		/* no dram on CPU */
#define CPUCFG_CACHEOPS		bcmcore_cacheops
#define CPUCFG_ARENAINIT	bcmcore_arena_init
#define CPUCFG_PAGETBLINIT	bcmcore_pagetable_init
#define CPUCFG_TLBHANDLER	bcmcore_tlbhandler
#define CPUCFG_DIAG_TEST1	bcmcore_null
#define CPUCFG_DIAG_TEST2	bcmcore_null
#define CPUCFG_CPUSPEED		sb_cpu_clock
#define CPUCFG_SYNCRANGE	bcmcore_sync_range
#define CPUCFG_INVALRANGE	bcmcore_inval_range


/*  *********************************************************************
    *  Implementation properties of BCMCORE
    ********************************************************************* */

/* The BCMCORE ticks CP0 every other cycle. */

#define CPUCFG_CYCLESPERCPUTICK	2

/* Hazard macros.  The BCMCORE is single issue. */

#define HAZARD nop ; nop ; nop ; nop ; nop ; nop ; nop

#define ERET \
		.set push ; \
		.set mips4 ; \
		eret ; \
		.set pop 



#define CPUCFG_COHERENT_DMA	0	/* The BCMCORE does not support coherent DMA. */

#define CPUCFG_REGS32	1	/* GPRs are 32 bits */
#define CPUCFG_REGS64	0	/* and are not 64 bits*/


/*  *********************************************************************
    *  BCMCORE CP0 Registers 
    ********************************************************************* */

#if defined(__ASSEMBLER__)
#define C0_BRCMCFG      $22             /* CP0: Broadcom Config */
#else
#define C0_BRCMCFG      22              /* CP0: Broadcom Config */
#endif
#ifndef C0_DIAGNOSTIC
#define C0_DIAGNOSTIC   C0_BRCMCFG      /* alias */
#endif


/* Status and Cause registers support an additional interrupt level. */
#if 0    /* but ignore that for now, pending refactoring of sbmip*. */

#undef  M_SR_IMMASK
#define M_SR_IMMASK	_MM_MAKEMASK(9,S_SR_IMMASK)

#define M_SR_IM8	_MM_MAKEMASK1(16)	/* Broadcom extension */

#undef  M_CAUSE_IPMASK
#define M_CAUSE_IPMASK	_MM_MAKEMASK(9,S_CAUSE_IPMASK)

#define M_CAUSE_IP8	_MM_MAKEMASK1(16)	/* Broadcom extension */
#endif /* 0 */


/* Diagnostic (BRCM Configuration) Register (select 0) */

#define M_BCM0_SI       _MM_MAKEMASK1(24)      /* Stall Enable, I Fetch */
#define M_BCM0_SB       _MM_MAKEMASK1(25)      /* Stall Enable, BIU Access */
#define M_BCM0_WN       _MM_MAKEMASK1(26)      /* Write NoAllocate Enable */
#define M_BCM0_SL       _MM_MAKEMASK1(27)      /* Stall Enable, Load */
#define M_BCM0_SS       _MM_MAKEMASK1(28)      /* Stall Enable, Store */
#define M_BCM0_C1       _MM_MAKEMASK1(29)      /* Coprocessor 1 Enable */
#define M_BCM0_DE       _MM_MAKEMASK1(30)      /* Dcache Enable */
#define M_BCM0_IE       _MM_MAKEMASK1(31)      /* Icache Enable */

#endif /* _CPU_CONFIG_H */
