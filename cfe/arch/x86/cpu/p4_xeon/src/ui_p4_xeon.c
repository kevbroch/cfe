 /*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Miscellaneous CPU routines		File: ui_p4_xeon.c
    *  
    *  UI command specific to Pentium IV's and Xeons
    *  
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

#include "cfe.h"
#include "env_subr.h"
#include "ui_command.h"

#include "lib_physio.h"

#include "cpumisc.h"

extern int altcpu_startup(void);
extern long cpu_ids[];

static int ui_cmd_altcpu_start(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int ncpus;
    int idx;

    ncpus = altcpu_startup();

    printf("%d cpus started.  APIC IDS:\n",ncpus);
    for (idx = 0; idx < ncpus; idx++) {
	printf("%08X\n",cpu_ids[idx]);
	}
    return 0;
}

static int ui_cmd_cpuid(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint32_t outregs[4];
    uint32_t idx,topidx;
    uint32_t a,d;

    cpuid(0,outregs);

    printf("[EAX=%08X]: %08X %08X %08X %08X\n",
	   0,
	   outregs[CPUID_EAX],outregs[CPUID_EBX],
	   outregs[CPUID_ECX],outregs[CPUID_EDX]);

    topidx = outregs[0];
    for (idx = 1; idx <= topidx; idx++) {
	cpuid(idx,outregs);
	printf("[EAX=%08X]: %08X %08X %08X %08X\n",
	       idx,
	       outregs[CPUID_EAX],outregs[CPUID_EBX],
	       outregs[CPUID_ECX],outregs[CPUID_EDX]);
	}

    rdmsr(0x2C,&a,&d);

    printf("FreqMSR: %08X %08X\n",a,d);

    return 0;
}


int ui_init_cpucmds(void);
int ui_init_cpucmds(void)
{

    cmd_addcmd("cpuid",
	       ui_cmd_cpuid,
	       NULL,
	       "Read processor information",
	       "cpuid\n",
	       "");


    cmd_addcmd("altcpu start",
	       ui_cmd_altcpu_start,
	       NULL,
	       "Start up auxillary processors",
	       "altcpu start\n",
	       "");

    return 0;
}


