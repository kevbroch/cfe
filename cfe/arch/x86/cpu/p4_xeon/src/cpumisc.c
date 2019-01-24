 /*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Miscellaneous CPU routines		File: cpumisc.c
    *  
    *  Miscellaneous X86 CPU routines, for reading MSRs, dealing
    *  with CPUID, etc.
    *  
    *  Author:  Mitch Lichtenberg 
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2003
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

#include "lib_types.h"
#include "cpumisc.h"


void rdmsr(uint32_t in_ecx,
		  uint32_t *out_eax,
		  uint32_t *out_edx)
{
    uint32_t a,d;

    __asm __volatile ("movl %2,%%ecx ; "
		      "rdmsr ; "
		      "movl %%eax,%0 ; "
		      "movl %%edx,%1 ; "
		      : "=r"(a),"=r"(d)
		      : "r"(in_ecx));

    *out_eax = a;
    *out_edx = d;
}


#define MHZ 1000000
uint32_t cpu_speed(uint32_t busclk)
{
    uint64_t cpu_hz;
    uint32_t a,d;

    if (busclk == 0) busclk = 100*MHZ;

    rdmsr(0x2C,&a,&d);

    cpu_hz = (uint64_t) (a >> 24) * (uint64_t) busclk;

    /* Hack for now - return 1GHz if rdmsr doesn't return a nice value */
    if (cpu_hz == 0) cpu_hz = 1000000000;

    return cpu_hz / 1000000;		/* return speed in MHz */
}


uint32_t p4_xeon_cpu_speed(void);
uint32_t p4_xeon_cpu_speed(void)
{
    return cpu_speed(100*MHZ) * 1000000U; /* XXX get busclk from bsp_config? */
}
