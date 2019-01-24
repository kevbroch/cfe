/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Switch Trace commands			File: ui_swtrace.c
    *  
    *  Commands and test stuff to check out the switch trace
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
#include "ui_command.h"

#include "lib_hssubr.h"
#include "lib_try.h"
#include "lib_memfuncs.h"
#include "lib_physio.h"
#include "env_subr.h"
#include "sb1250_defs.h"

#include "bcm1480_regs.h"
#include "bcm1480_pm.h"
#include "bcm1480_hr.h"
#include "bcm1480_ht.h"
#include "bcm1480_hsp.h"
#include "bcm1480_scd.h"

#include "ui_bitfields.h"

typedef volatile uint32_t sbeth_port_t;
typedef uint32_t sbeth_physaddr_t;
#define SBETH_PORT(x) PHYS_TO_K1(x)

#if CFG_L2_RAM
#define SBETH_VTOP(x) ((sbeth_physaddr_t)0xD0000000 + (sbeth_physaddr_t)(x))
#else
#define SBETH_VTOP(x) (K1_TO_PHYS((sbeth_physaddr_t)(x)))
#endif

//#define WRITECSR(x,y) printf("WRITE_REG %016llx %016llx\n",(uint64_t)(x),(uint64_t)(y));phys_write64(x,y)
#define WRITECSR(x,y) phys_write64(x,y)
#define READCSR(x) phys_read64(x)

#define NTRACE 1024


/*  *********************************************************************
    *  Configuration
    ********************************************************************* */


/*  *********************************************************************
    *  prototypes
    ********************************************************************* */

int ui_init_swtrccmds(void);
extern int dumpmem(hsaddr_t addr,hsaddr_t dispaddr,int length,int wlen);

/*  *********************************************************************
    *  Data
    ********************************************************************* */

static bitfield_t swtrc_status[] = {
    {M_BCM1480_SWTRC_CFG_RESET,"reset"},
    {M_BCM1480_SWTRC_CFG_START,"start"},
    {M_BCM1480_SWTRC_CFG_STOP,"stop"},
    {M_BCM1480_SWTRC_CFG_FREEZE,"freeze"},
    {M_BCM1480_SWTRC_CFG_FREEZEFULL,"freeze_full"},
    {M_BCM1480_SWTRC_CFG_DEBUGFULL,"debug_full"},
    {M_BCM1480_SWTRC_CFG_TRCFULL,"trc_full"},
    {M_BCM1480_SWTRC_CFG_FORCECNT,"forcecnt"},
    {M_BCM1480_SWTRC_CFG_TRCADDR,"addr"},
    {0,NULL}};





static int ui_cmd_sw_counters(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    int c0,c1,c2,c3;
    uint64_t reg;

    c0 = 0; c1 = 0; c2 = 0; c3 = 0;

    if (cmd_sw_isset(cmd,"-init")) {
	if ((x = cmd_getarg(cmd,0))) c0 = atoi(x);
	else ui_showusage(cmd);
	if ((x = cmd_getarg(cmd,1))) c1 = atoi(x);
	else ui_showusage(cmd);
	if ((x = cmd_getarg(cmd,2))) c2 = atoi(x);
	else ui_showusage(cmd);
	if ((x = cmd_getarg(cmd,3))) c3 = atoi(x);
	else ui_showusage(cmd);

	reg = V_BCM1480_SWPERF_CFG_C0SRC(c0)|
	    V_BCM1480_SWPERF_CFG_C1SRC(c1)|
	    V_BCM1480_SWPERF_CFG_C2SRC(c2)|
	    V_BCM1480_SWPERF_CFG_C3SRC(c3) |
	    M_BCM1480_SWPERF_CFG_CLEAR |
	    M_BCM1480_SWPERF_CFG_ENABLE;

	phys_write64(A_BCM1480_SWPERF_CFG,reg);
	return 0;		     
	}
    else {
	printf("Counter 0: %llu\n",phys_read64(A_BCM1480_SWPERF_CNT0));
	printf("Counter 1: %llu\n",phys_read64(A_BCM1480_SWPERF_CNT1));
	printf("Counter 2: %llu\n",phys_read64(A_BCM1480_SWPERF_CNT2));
	printf("Counter 3: %llu\n",phys_read64(A_BCM1480_SWPERF_CNT3));
	}

    return 0;
}



static void tracedump(uint64_t d0,uint64_t d1,uint64_t d2)
{
    static char *agents[] = {"HSP0","HSP1","HSP2","Unk3","NC  ","HT  ","PM  ","Unk7"};

    if (!(d0 & M_BCM1480_SWENT_FMT)) {
	printf("%s ",(d0 & M_BCM1480_SWENT_VALID) ? "Val " : "    ");
	printf("%s ",(d0 & M_BCM1480_SWENT_MULTI) ? "Mul " : "    ");
	printf("%s ",(d0 & M_BCM1480_SWENT_BCAST) ? "Bcs " : "    ");
	printf("%s ",(d0 & M_BCM1480_SWENT_SOP) ? "SOP "   : "    ");
	printf("%s ",(d0 & M_BCM1480_SWENT_EOP) ? "EOP "   : "    ");
	printf("%s ",(d0 & M_BCM1480_SWENT_DRELOAD) ? "Drl "   : "    ");
	printf("%s ",(d0 & M_BCM1480_SWENT_SRELOAD) ? "Srl "   : "    ");
	printf("%s ",(d0 & M_BCM1480_SWENT_TYPE) ? "HT  "   : "PKT ");

	printf("%s->",agents[G_BCM1480_SWENT_SRC(d0)]);
	printf("%s",agents[G_BCM1480_SWENT_DEST(d0)]);
	printf("[%2d] ",G_BCM1480_SWENT_VC(d0));

	int cnt = G_BCM1480_SWENT_CNT(d0);
	if (cnt == 0) cnt = 16;
	printf("%2dB ",cnt);

	// printf("Tag:%08x ",G_BCM1480_SWENT_TAG(d0));	// not useful (?)
	printf("Cyc:%5d ",G_BCM1480_SWENT_CYCDIFF(d0));

	printf("%016llX %016llX\n",d1,d2);
	}
    else {
	printf("Control format, not supported\n");
	}
}

static int ui_cmd_sw_trace(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint64_t reg;
    uint64_t value;
    int freeze;

    if (cmd_sw_isset(cmd,"-no_freeze")) {		// default freeze
    	printf("Switch trace buffer will record most recent 1024 entries\n");
    	freeze = 0;
	}
    else {
    	printf("Switch trace buffer will record initial 1024 entries, then freeze\n");
	freeze = 1;
	}

    if (cmd_sw_isset(cmd,"-init")) {
	phys_write64(A_BCM1480_SWTRC_MATCH_CONTROL(0),0xFFFFFFFFFFFFFFFFLL);	/* trace everything */
	phys_write64(A_BCM1480_SWTRC_EVENT(0),0x10001); /* enable event 0, enable control match */
	phys_write64(A_BCM1480_SWTRC_SEQUENCE(0),0x40fff);	/* Monitor all activity */
	phys_write64(A_BCM1480_SWTRC_CFG,
		     (M_BCM1480_SWTRC_CFG_RESET));

	value = M_BCM1480_SWTRC_CFG_START;
	if (freeze) value |= M_BCM1480_SWTRC_CFG_FREEZEFULL;
	phys_write64(A_BCM1480_SWTRC_CFG, value);
	}
   
    else if (cmd_sw_isset(cmd,"-dump")) {
    	printf("Displaying trace buffer in order (oldest->youngest):\n");
	uint64_t data0[NTRACE];
	uint64_t data1[NTRACE];
	uint64_t data2[NTRACE];
	int i = 0;
	int num;

	phys_write64(A_BCM1480_SWTRC_CFG,
		     (M_BCM1480_SWTRC_CFG_STARTREAD |
		      M_BCM1480_SWTRC_CFG_FREEZE));

	reg = phys_read64(A_BCM1480_SWTRC_CFG);

	for (;;) {
	    if (console_status()) break;

	    data0[i] = phys_read64(A_BCM1480_SWTRC_READ);
	    data1[i] = phys_read64(A_BCM1480_SWTRC_READ);
	    data2[i] = phys_read64(A_BCM1480_SWTRC_READ);

	    if (!(data0[i] & (M_BCM1480_SWENT_VALID))) break;	/* no more data */
	    i++;
	    }
	// flip it around for user convenience so it reads top-down
	num = i;
	for (i=num-1; i>=0; i--) {
	    tracedump(data0[i],data1[i],data2[i]);
	    }

	phys_write64(A_BCM1480_SWTRC_CFG,(M_BCM1480_SWTRC_CFG_RESET));

	}

    else {
	reg = phys_read64(A_BCM1480_SWTRC_CFG);
	printf("Status: %s\n",showfields(swtrc_status,reg));
	}

    return 0;
}



/*  *********************************************************************
    *  ui_init_swtrccmds()
    *  
    *  Add BIGSUR-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */


int ui_init_swtrccmds(void)
{
    cmd_addcmd("sw counters",
	       ui_cmd_sw_counters,
	       NULL,
	       "Display switch counters",
	       "sw counters [-init]",
	       "-init;Initialize counters");

    cmd_addcmd("sw trace",
	       ui_cmd_sw_trace,
	       NULL,
	       "Display/init switch trace",
	       "sw trace [-init]",
	       "-init;Initialize trace|"
	       "-no_freeze;Run continuously, recording most recent 1024 entries|"
	       "-dump;Dump trace buffer");

    return 0;
}




