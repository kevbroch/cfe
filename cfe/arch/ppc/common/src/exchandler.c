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
#include "lib_try.h"
#include "exception.h"
#include "ppcdefs.h"

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#define INSTALL_VECTOR(vec,hand) install_handler(vec,(uint32_t) hand);

/*  *********************************************************************
    *  Constants
    ********************************************************************* */

/*  *********************************************************************
    *  Globals 
    ********************************************************************* */

extern uint32_t _readmsr(void);
extern void _writemsr(uint32_t);
exc_handler_t exc_handler;
extern void _exc_entry(void);
extern uint32_t cfe_doxreq(void *);

static const char *regnames = "r0  r1  r2  r3  r4  r5  r6  r7  "
                              "r8  r9  r10 r11 r12 r13 r14 r15 "
                              "r16 r17 r18 r19 r20 r21 r22 r23 "
                              "r24 r25 r26 r27 r28 r29 r30 r31 ";
static const char *excnames = 
    "Rsvd0   " /* 0 */
    "Reset   "
    "MCheck  "
    "DSI     "
    "ISI     "
    "INTR    "
    "Align   "
    "Program "
    "FPUnavl "
    "DECR    "
    "CINT    "
    "RsvdB   "
    "Syscall "
    "Trace   "
    "RsvdE   "
    "RsvdF   "
    "ITLBMiss"
    "DTLBMisL"
    "DTLBMisS"
    "IBreak  "
    "SMI     ";


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

void cfe_exception(int code,uint32_t *info)
{
    int idx;

    if (exc_handler.catch_exc == 1) {
	/* XXX do something to MSR bits here? */

	/* Unwind exception, return to CFE. */
	exc_handler.catch_exc = 0;
	exc_longjmp_handler();
	}

    xprintf("** Exception 0x%04X: SRR0=%08X SRR1=%08X [%8s]\n",
	    code,info[XSPR_SRR0],info[XSPR_SRR1],
	    excnames + ((code >> 8) * 8));
    xprintf("        LR=%08X CTR=%08X XER=%08X DSISR=%08X\n",
	    info[XGR_LR],info[XGR_CTR],info[XSPR_XER],info[XSPR_DSISR]);
    xprintf("\n");
    for (idx = 0;idx < 32; idx+= 2) {
	xprintf("        %3s = %08X     %3s = %08X\n",
		regnames+(idx*4),
		info[XGR_R0+idx],
		regnames+((idx+1)*4),
		info[XGR_R0+idx+1]);
	}

    xprintf("\n");
    _exc_restart();
}


#if CFG_RELOC
/*  *********************************************************************
    *  install_handler(vec,entry)
    *  
    *  Create and install a vector in low memory that jumps
    *  to wherever CFE is currently located
    *  
    *  Input parameters:
    *      vec - vector offset
    *      entry - location of handler
    *  
    *  Return value:
    *      nothing
    ********************************************************************* */

static void install_handler(uint32_t vec,uint32_t entry)
{
    /*
     * Vector code is as follows:
     *
     *        7c 11 43 a6     mtsprg  1,r0
     *        7c 08 02 a6     mflr    r0
     *        7c 10 43 a6     mtsprg  0,r0
     *        3c 00 EH EH     lis     r0,EHEH
     *        60 00 EL EL     ori     r0,r0,ELEL
     *        7c 08 03 a6     mtlr    r0
     *        38 00 VV VV     li      r0,VVVV
     *        4e 80 00 20     blr
     */

    volatile uint32_t *v = (volatile uint32_t *) vec;

    /*
     * Construct exception vector code in memory
     */

    *v++ = 0x7c1143a6;
    *v++ = 0x7c0802a6;
    *v++ = 0x7c1043a6;
    *v++ = 0x3c000000 | (entry >> 16);
    *v++ = 0x60000000 | (entry & 0xFFFF);
    *v++ = 0x7c0803a6;
    *v++ = 0x38000000 | (vec);
    *v++ = 0x4e800020;

    /*
     * Flush from cache into memory
     */

    __asm __volatile ("dcbf 0,%0" :: "r"(vec));
    __asm __volatile ("icbi 0,%0" :: "r"(vec));
    __asm __volatile ("dcbf 0,%0" :: "r"(vec+CPUCFG_CACHELINESIZE));
    __asm __volatile ("icbi 0,%0" :: "r"(vec+CPUCFG_CACHELINESIZE));

}

static void install_apientry(uint32_t vec,uint32_t entry)
{
    /*
     * API dispatch looks like this:
     *
     *        38 21 ff fc     addi    r1,r1,-4
     *        7c 08 02 a6     mflr    r0
     *        90 01 00 00     stw     r0,0(r1)
     *        3c 00 EH EH     lis     r0,EHEH
     *        60 00 EL EL     ori     r0,r0,ELEL
     *        7c 08 03 a6     mtlr    r0
     *        4e 80 00 21     blrl
     *        80 01 00 00     lwz     r0,0(r1)
     *        38 21 00 04     addi    r1,r1,4
     *        7c 08 03 a6     mtlr    r0
     *        4e 80 00 20     blr     
     */

    volatile uint32_t *vp = (volatile uint32_t *) vec;

    *vp++ = 0x3821fffc;
    *vp++ = 0x7c0802a6;
    *vp++ = 0x90010000;
    *vp++ = 0x3c000000 | (entry >> 16);
    *vp++ = 0x60000000 | (entry & 0xFFFF);
    *vp++ = 0x7c0803a6;
    *vp++ = 0x4e800021;
    *vp++ = 0x80010000;
    *vp++ = 0x38210004;
    *vp++ = 0x7c0803a6;
    *vp++ = 0x4e800020;

    __asm __volatile ("dcbf 0,%0" :: "r"(vec));
    __asm __volatile ("icbi 0,%0" :: "r"(vec));
    __asm __volatile ("dcbf 0,%0" :: "r"(vec+CPUCFG_CACHELINESIZE));
    __asm __volatile ("icbi 0,%0" :: "r"(vec+CPUCFG_CACHELINESIZE));

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
#if CFG_RELOC
    uint32_t msr;
#endif

    /*
     * Set up the exception stack (queue of jmpbufs that we follow on exception)
     */

    exc_handler.catch_exc = 0;
    q_init( &(exc_handler.jmpbuf_stack));

#if CFG_RELOC
    /*
     * Install the low-level vectors
     */

    INSTALL_VECTOR(EXC_OFF_MCHECK,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_DSI,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_ISI,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_INT,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_ALIGN,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_PROGRAM,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_FPUNVL,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_DECR,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_CINT,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_SYSCALL,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_TRACE,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_RSVD,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_ITLBMISS,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_DTLBMISS_LD,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_DTLBMISS_ST,_exc_entry);
    INSTALL_VECTOR(EXC_OFF_SMI,_exc_entry);

    install_apientry(0x1500,(uint32_t) cfe_doxreq);

    /*
     * Switch the CPU to using the RAM vectors.
     */

    msr = _readmsr();
    msr &= ~M_MSR_IP;	/* turn off instruction prefix */
    _writemsr(msr);
#endif
}












