/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU Configuration file			File: cpu_config.h
    *  
    *  This file contains the names of the routines to be used
    *  in the dispatch table in init_ppc.S
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

#define CPUCFG_CPUINIT		mpc8245_cpuinit
#define CPUCFG_CPURESTART	mpc8245_cpurestart
#define CPUCFG_DRAMINIT		board_dram_init
#define CPUCFG_CACHEOPS		mpc8245_cacheops
#define CPUCFG_ARENAINIT	ppc_arena_init
#define CPUCFG_PAGETBLINIT	mpc8245_pagetable_init
#define CPUCFG_CPUSPEED		mpc8245_cpu_speed

/*
 * TBL increments once per 4 sys_logic_clk ticks, which is only
 * partially related to the core clock.  Typical core-to-syslogic
 * values are 2.0, so for now wire this at 4*2=8.
 *
 * We can get fancy someday if CFE really requires it, but
 * only a ballpark is necessary for most stuff.
 */
#define CPUCFG_CYCLESPERCPUTICK	8

#define CPUCFG_CACHELINESIZE	32


/*
 * Hazard macro
 */

#define HAZARD isync

/*
 * Let others know we can do coherent DMA
 */

#define CPUCFG_COHERENT_DMA	1

#define CPUCFG_REGS32		1
#define CPUCFG_REGS64		0

#define CPUCFG_ARCHNAME		"PPC"
#define CPUCFG_ELFTYPE		EM_PPC

#define CPUCFG_MPC8245		1
