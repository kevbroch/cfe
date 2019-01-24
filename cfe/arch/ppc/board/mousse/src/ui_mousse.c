/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  MOUSSE-specific commands			File: ui_mousse.c
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
#include "ui_command.h"

#include "ppcdefs.h"
#include "mpc824x.h"

#include "mousse.h"

int ui_init_moussecmds(void);
static int ui_cmd_timertest(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_crash(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_read_hid0(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_write_hid0(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_read_hid1(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_read_msr(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_write_msr(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_read_bats(ui_cmdline_t *cmd,int argc,char *argv[]);



/*  *********************************************************************
    *  ui_init_moussecmds()
    *  
    *  Add MOUSSE-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */


int ui_init_moussecmds(void)
{
    cmd_addcmd("test timer",
	       ui_cmd_timertest,
	       NULL,
	       "Test the timer",
	       "test timer",
	       "");

    cmd_addcmd("crash",
	       ui_cmd_crash,
	       NULL,
	       "Cause an exception",
	       "crash",
	       "-divby0;Cause a division by zero exception|"
	       "-align;Cause an alignment exception|"
               "-addr;Cause an address fault|"
	       "-syscall;Do a syscall");


    cmd_addcmd("read hid0",
	       ui_cmd_read_hid0,
	       NULL,
	       "Read HID0 register",
	       "read hid0",
	       "");

    cmd_addcmd("write hid0",
	       ui_cmd_write_hid0,
	       NULL,
	       "Write HID0 register.",
	       "write hid0 [value]",
	       "");

    cmd_addcmd("read hid1",
	       ui_cmd_read_hid1,
	       NULL,
	       "Read HID1 register",
	       "read hid1",
	       "");

    cmd_addcmd("read msr",
	       ui_cmd_read_msr,
	       NULL,
	       "Read MSR register.",
	       "read msr",
	       "");

    cmd_addcmd("write msr",
	       ui_cmd_write_msr,
	       NULL,
	       "Write MSR register.",
	       "write msr [value]",
	       "");


    cmd_addcmd("read bats",
	       ui_cmd_read_bats,
	       NULL,
	       "Reat BAT registers.",
	       "read bats",
	       "");


    return 0;
}


static int ui_cmd_crash(ui_cmdline_t *cmd,int argc,char *argv[])
{
    if (cmd_sw_isset(cmd,"-divby0")) {
	int a = 1;
	int b = 0;
	printf("whoopie! %d\n",a/b);
	}
    else if (cmd_sw_isset(cmd,"-align")) {
	uint32_t a = 17;
	__asm __volatile ("stmw 14,0(%0)" : : "r"(a));
	}
    else if (cmd_sw_isset(cmd,"-addr")) {
	}
    else if (cmd_sw_isset(cmd,"-syscall")) {
	__asm __volatile ("li 0,0x0999 ; li 4,0x0444 ; li 10,0x0aaa ; li 30,0x0eee ;  sc");
	}

    return 0;
}


static int ui_cmd_timertest(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int64_t t;

    t = cfe_ticks;

    while (!console_status()) {
	cfe_sleep(CFE_HZ);
	if (t != cfe_ticks) {
	    xprintf("Time is %lld\n",cfe_ticks);
	    t = cfe_ticks;	    
	    }
	}

    return 0;
}

extern uint32_t read_hid0(void);
extern void write_hid0(uint32_t);

static int ui_cmd_read_hid0(ui_cmdline_t *cmd,int argc,char *argv[])
{
    printf("HID0 = 0x%08X\n",read_hid0());
    return 0;
}

static int ui_cmd_write_hid0(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    uint32_t val;

    if ((x = cmd_getarg(cmd,0)) == NULL) return ui_showusage(cmd);

    val = xtoi(x);
    write_hid0(val);

    return 0;
}


extern uint32_t read_hid1(void);

static int ui_cmd_read_hid1(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint32_t hid1 = read_hid1();

    printf("HID1 = 0x%08X, pllconfig=0x%02X\n",hid1,G_HID1_PLLCFG(hid1));
    return 0;
}

extern uint32_t read_msr(void);
extern void write_msr(uint32_t);

static int ui_cmd_read_msr(ui_cmdline_t *cmd,int argc,char *argv[])
{
    printf("MSR = 0x%08X\n",read_msr());
    return 0;
}

static int ui_cmd_write_msr(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    uint32_t val;

    if ((x = cmd_getarg(cmd,0)) == NULL) return ui_showusage(cmd);

    val = xtoi(x);
    write_msr(val);

    return 0;
}

extern void read_bats(uint32_t *);
static int ui_cmd_read_bats(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint32_t bats[16];
    int idx;

    read_bats(bats);

    for (idx = 0; idx < 8; idx += 2) {
	printf("IBAT%d  U:%08X  L:%08X\n",idx/2,bats[idx],bats[idx+1]);
	}
    for (idx = 8; idx < 16; idx += 2) {
	printf("DBAT%d  U:%08X  L:%08X\n",(idx-8)/2,bats[idx],bats[idx+1]);
	}
    return 0;
}
