/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Exception Handler			File: exchandler.c       
    *  
    *  This is the "C" part of the exception handler and the
    *  associated setup routines.  We call these routines from
    *  the assembly-language exception handler.
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


#include "cfe.h"
#include "sbmips.h"
#include "lib_hssubr.h"
#include "exception.h"
#include "lib_try.h"

/*  *********************************************************************
    *  Constants
    ********************************************************************* */

/* 
 * Temporary until all our CPU packages support a cache error handler
 */

#ifndef CPUCFG_CERRHANDLER
#define CPUCFG_CERRHANDLER 0xBFC00000
#else
extern void CPUCFG_CERRHANDLER(void);
#endif

/*  *********************************************************************
    *  Globals 
    ********************************************************************* */

extern void _exc_entry(void);
extern void _exc_setup_locore(long);
extern void CPUCFG_TLBHANDLER(void);
extern uint32_t _getstatus(void);
extern void _setstatus(uint32_t);

static const char *regnames = "0 ATv0v1a0a1a2a3t0t1t2t3t4t5t6t7"
                              "s0s1s2s3s4s5s6s7t8t9k0k1gpspfpra";
static const char *excnames = 
    "Interrupt"	/* 0 */
    "TLBMod   "	/* 1 */
    "TLBMissRd"	/* 2 */
    "TLBMissWr"	/* 3 */
    "AddrErrRd"	/* 4 */
    "AddrErrWr"	/* 5 */
    "BusErrI  "	/* 6 */
    "BusErrD  "	/* 7 */
    "Syscall  "	/* 8 */
    "Breakpt  "	/* 9 */
    "InvOpcode"	/* 10 */
    "CoProcUnu"	/* 11 */
    "ArithOvfl"	/* 12 */
    "TrapExc  "	/* 13 */
    "VCEI     "	/* 14 */
    "FPUExc   "	/* 15 */
    "CP2Exc   "	/* 16 */
    "Exc17    " /* 17 */
    "Exc18    " /* 18 */
    "Exc19    " /* 19 */
    "Exc20    " /* 20 */
    "Exc21    " /* 21 */
    "Exc22    " /* 22 */
    "Watchpt  " /* 23 */
    "Exc24    " /* 24 */
    "Exc25    " /* 25 */
    "Exc26    " /* 26 */
    "Exc27    " /* 27 */
    "Exc28    " /* 28 */
    "Exc29    " /* 29 */
    "Exc30    " /* 30 */
    "VCED     "; /* 31 */



/*  *********************************************************************
    *  cfe_exception(code,info)
    *  
    *  Exception handler.  This routine is called when any CPU 
    *  exception that is handled by the assembly-language
    *  vectors is reached.  The usual thing to do here is just to
    *  reboot.
    *  
    *  Input parameters: 
    *  	   code - exception type
    *  	   info - exception stack frame
    *  	   
    *  Return value:
    *  	   usually reboots
    ********************************************************************* */

void cfe_exception(int code,mips_reg_t *info)
{
    int idx;

    SETLEDS("EXC!");
    
    if(exc_handler.catch_exc == 1) {
	/*Deal with exception without restarting CFE.*/
	
	/*Clear relevant SR bits*/
	_exc_clear_sr_exl();
	_exc_clear_sr_erl();
	
	/*Reset flag*/
	exc_handler.catch_exc = 0;

	exc_longjmp_handler();       
      }
    

#if CPUCFG_REGS32
    xprintf("**Exception %d: EPC=%08X, Cause=%08X (%9s)\n",
	    code,(uint32_t)info[XCP0_EPC],
	    (uint32_t)info[XCP0_CAUSE],
	    excnames + G_CAUSE_EXC((uint32_t)info[XCP0_CAUSE])*9);
    xprintf("                RA=%08X, VAddr=%08X\n",
	    (uint32_t)info[XGR_RA],(uint32_t)info[XCP0_VADDR]);
    xprintf("\n");
    for (idx = 0;idx < 32; idx+= 2) {
	xprintf("        %2s ($%2d) = %08X     %2s ($%2d) = %08X\n",
		regnames+(idx*2),
		idx,(uint32_t)info[XGR_ZERO+idx],
		regnames+((idx+1)*2),
		idx+1,(uint32_t)info[XGR_ZERO+idx+1]);
	}
#else
    xprintf("**Exception %d: EPC=%016llX, Cause=%08X (%9s)"
#if CFG_MULTI_CPUS
	    " (CPU%d)"
#endif
	    "\n",
	    code,info[XCP0_EPC],info[XCP0_CAUSE],
	    excnames + G_CAUSE_EXC((uint32_t)info[XCP0_CAUSE])*9
#if CFG_MULTI_CPUS
	    ,((info[XCP0_PRID] >> 25) & 7)
#endif
	);
    xprintf("                RA=%016llX, VAddr=%016llX, SR=%08X\n",
	    info[XGR_RA],info[XCP0_VADDR],info[XCP0_SR]);
    xprintf("\n");
    for (idx = 0;idx < 32; idx+= 2) {
	xprintf("        %2s ($%2d) = %016llX     %2s ($%2d) = %016llX\n",
		regnames+(idx*2),
		idx,info[XGR_ZERO+idx],
		regnames+((idx+1)*2),
		idx+1,info[XGR_ZERO+idx+1]);
	}
#endif

    xprintf("\n");
    _exc_restart();
}

#if ((!CFG_BOOTRAM) && (!CFG_L2_RAM) && (CFG_INIT_DRAM) && (CFG_RUNFROMKSEG0))
/*  *********************************************************************
    *  exc_setup_hw_vector(vecoffset,target,k0code)
    *  
    *  Install a patch of code at the specified offset in low
    *  KSEG0 memory that will jump to 'target' and load k0
    *  with the specified code value.  This is used when we
    *  run with RAM vectors.
    *  
    *  Input parameters: 
    *  	   vecoffset - offset into KSEG0
    *  	   target - location where we should branch when vector is called
    *  	   k0code - value to load into K0 before branching
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void exc_setup_hw_vector(uint32_t vecoffset,
				  void *target,
				  uint32_t k0code)
{
    uint32_t *vec;
    uint32_t new;
    uint32_t lower,upper;

    new = (uint32_t) (intptr_t) target;	/* warning: assumes compatibility addresses! */

    lower = new & 0xffff;
    upper = (new >> 16) & 0xffff;
    if ((lower & 0x8000) != 0) {
	upper++;
	}

    /*
     * Get a KSEG0 version of the vector offset.
     */
    vec = (uint32_t *) PHYS_TO_K0(vecoffset);

    /*
     * Patch in the vector.  Note that we have to flush
     * the L1 Dcache and invalidate the L1 Icache before
     * we can use this.  
     */

    vec[0] = 0x3c1b0000 | upper;   /* lui   k1, HIGH(new)     */
    vec[1] = 0x277b0000 | lower;   /* addiu k1, k1, LOW(new)  */
    vec[2] = 0x03600008;           /* jr    k1                */
    vec[3] = 0x241a0000 | k0code;  /*  li   k0, code          */

}


/*  *********************************************************************
    *  exc_install_ram_vectors()
    *  
    *  Install all of the hardware vectors into low memory,
    *  flush the cache, and clear the BEV bit so we can start
    *  using them.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void exc_install_ram_vectors(void)
{
    uint32_t *ptr;
    int idx;

    /* Debug: blow away the vector area so we can see what we did */
    ptr = (uint32_t *) PHYS_TO_K0(0);
    for (idx = 0; idx < MIPS_RAM_VEC_END/sizeof(uint32_t); idx++) *ptr++ = 0;

    /*
     * Set up the vectors.  The cache error handler is set up
     * specially.
     */

    exc_setup_hw_vector(MIPS_RAM_VEC_TLBFILL,  CPUCFG_TLBHANDLER,XTYPE_TLBFILL);
    exc_setup_hw_vector(MIPS_RAM_VEC_XTLBFILL, _exc_entry,XTYPE_XTLBFILL);
    exc_setup_hw_vector(MIPS_RAM_VEC_CACHEERR, _exc_entry,XTYPE_CACHEERR);
    exc_setup_hw_vector(MIPS_RAM_VEC_EXCEPTION,_exc_entry,XTYPE_EXCEPTION);
    exc_setup_hw_vector(MIPS_RAM_VEC_INTERRUPT,_exc_entry,XTYPE_INTERRUPT);

    /*
     * Flush the D-cache and invalidate the I-cache so we can start
     * using these vectors.
     */

    cfe_flushcache(CFE_CACHE_FLUSH_D | CFE_CACHE_INVAL_I);

    /*
     * Write the handle into our low memory space.  If we need to save
     * other stuff down there, this is a good place to do it.
     * This call uses uncached writes - we have not touched the
     * memory in the handlers just yet, so they should not be 
     * in our caches.
     */

    _exc_setup_locore((intptr_t) CPUCFG_CERRHANDLER);

    /*
     * Finally, clear BEV so we'll use the vectors in RAM.
     */

    _setstatus(_getstatus() & ~M_SR_BEV);

    /*
     * XXX There's a hazard here, but we're not going to worry about
     * XXX it.  It is unlikely we'll use the vectors any time soon.
     */
}
#endif

/*  *********************************************************************
    *  cfe_setup_exceptions()
    *  
    *  Set up the exception handlers.  
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
void cfe_setup_exceptions(void)
{
    _exc_setvector(XTYPE_TLBFILL,  (void *) cfe_exception);
    _exc_setvector(XTYPE_XTLBFILL, (void *) cfe_exception);
    _exc_setvector(XTYPE_CACHEERR, (void *) _exc_cache_crash_sim);
    _exc_setvector(XTYPE_EXCEPTION,(void *) cfe_exception);
    _exc_setvector(XTYPE_INTERRUPT,(void *) cfe_exception);
    _exc_setvector(XTYPE_EJTAG,    (void *) cfe_exception);


    exc_init();

#if ((!CFG_BOOTRAM) && (!CFG_L2_RAM) && (CFG_INIT_DRAM) && (CFG_RUNFROMKSEG0))
    /*
     * Install RAM vectors, and clear the BEV bit in the status
     * register.  Don't do this if we're running from PromICE RAM
     */
    exc_install_ram_vectors();
#endif
}











