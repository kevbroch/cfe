/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Test commands				File: ui_bcm95836cpci.c
    *  
    *  A temporary sandbox for misc test routines and commands.
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
#include "env_subr.h"

#include "ui_command.h"

#include "sbmips32.h"
#include "sb_bp.h"

#include "sb_utils.h"

int ui_init_nvramcmds(void);

int ui_init_bcm95836cpcicmds(void);
static int ui_cmd_timertest(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_envdev(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_clock(ui_cmdline_t* cmd, int argc, char *argv[]);
static int ui_cmd_showconfig(ui_cmdline_t* cmd, int argc, char *argv[]);

int ui_init_bcm95836cpcicmds(void)
{

    ui_init_nvramcmds();

    cmd_addcmd("test timer",
	       ui_cmd_timertest,
	       NULL,
	       "Test the timer",
	       "test timer",
	       "");

    cmd_addcmd("envdev",
	       ui_cmd_envdev,
	       NULL,
	       "set environment device",
	       "envdev sourcedev",
	       "");

    cmd_addcmd("clock",
	       ui_cmd_clock,
	       NULL,
	       "Change CPU/PCI clock (clock <cpu> <pci>)",
	       "clock",
	       "");

    cmd_addcmd("show config",
	       ui_cmd_showconfig,
	       NULL,
	       "Dump CP0 configuration registers",
	       "show config",
	       "");

    return 0;
}


static int ui_cmd_envdev(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;

    x = cmd_getarg(cmd,0);
    if (x) {
	cfe_set_envdevice(x);
	printf("environment device set to %s\n",x);
	}

    return 0;
}

static int ui_cmd_clock(ui_cmdline_t* cmd, int argc, char *argv[])
{
    char *x = NULL, *y = NULL;
    unsigned long req_sb, req_cpu, req_pci;

    req_sb  = 100000000;
    req_cpu = 200000000;
    req_pci =  33000000;     /* Defaults at reset */

    x = cmd_getarg(cmd,0);
    y = cmd_getarg(cmd,1);

    if (x) req_cpu = atoi(x);
    if (y) req_pci = atoi(y);

    /* Multiply for user */
    if (req_pci < 1000000) req_pci *= 1000000;
    if (req_cpu < 1000000) req_cpu *= 1000000;

    if (x || y) { 
	sb_setclock(req_cpu, req_pci);

	/* If either frequency changed, sb_setclock will reboot the system
	   and there will be no return to the following code.
	   XXX Furthermore, the reboot will reset the PCI frequency to
	   the default for the board.
	*/
	}

    cfe_cpu_speed = sb_cpu_clock();
    printf("CPU clock at %dMhz\n", (cfe_cpu_speed+500000)/1000000);

    return 0;
}   


static int ui_cmd_timertest(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int64_t t;

    t = cfe_ticks;

    while (!console_status()) {
	cfe_sleep(CFE_HZ);
	if (t != cfe_ticks) {
	    xprintf("Time is 0x%llX\n",cfe_ticks);
	    t = cfe_ticks;	    
	    }
	}

    return 0;
}


extern uint32_t read_config0(void);
extern uint32_t read_config1(void);

extern uint32_t read_bcm0(void);
extern uint32_t read_bcm1(void);
extern uint32_t read_bcm2(void);
extern uint32_t read_bcm3(void);
extern uint32_t read_bcm4(void);
extern uint32_t read_bcm5(void);
extern uint32_t read_bcm6(void);
extern uint32_t read_bcm7(void);

static int ui_cmd_showconfig(ui_cmdline_t *cmd,int argc,char *argv[])
{
    xprintf("config 0: 0x%08x,  1: 0x%08x\n", read_config0(), read_config1());

    xprintf("brcm   0: 0x%08x,", read_bcm0());
    xprintf("  1: 0x%08x,", read_bcm1());
    xprintf("  2: 0x%08x,", read_bcm2());
    xprintf("  3: 0x%08x\n", read_bcm3());

    xprintf("       4: 0x%08x,", read_bcm4());
    xprintf("  5: 0x%08x,", read_bcm5());
    xprintf("  6: 0x%08x,", read_bcm6());
    xprintf("  7: 0x%08x\n", read_bcm7());

    return 0;
}
