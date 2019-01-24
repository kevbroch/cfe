/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  BCM91280E-specific commands		File: ui_bcm91280e.c
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

#if CFG_PCI
#include "pcivar.h"
#endif

/*  *********************************************************************
    *  prototypes
    ********************************************************************* */

int ui_init_bcm91280ecmds(void);


/*  *********************************************************************
    *  commands
    ********************************************************************* */

static int ui_cmd_home(ui_cmdline_t *cmd,int argc,char *argv[])
{
    printf("\033[H");
    return 0;
}

static int ui_cmd_cls(ui_cmdline_t *cmd,int argc,char *argv[])
{
    printf("\033[H\033[2J");
    return 0;
}



static int ui_cmd_uacwrite(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint32_t addr = 0;
    hsaddr_t haddr;
    uint64_t data0 = 0;
    uint64_t data1 = 0;
    uint64_t data2 = 0;
    uint64_t data3 = 0;
    int loops = 1;
    int idx;
    char *x;

    if (((x = cmd_getarg(cmd,0)))) addr = lib_xtoi(x);
    else ui_showusage(cmd);

    haddr = (0xb800000000000000LL + (hsaddr_t)addr);

    if (((x = cmd_getarg(cmd,1)))) data0 = lib_xtoq(x);
    else ui_showusage(cmd);
    if (((x = cmd_getarg(cmd,2)))) data1 = lib_xtoq(x);
    else ui_showusage(cmd);
    if (((x = cmd_getarg(cmd,3)))) data2 = lib_xtoq(x);
    else ui_showusage(cmd);
    if (((x = cmd_getarg(cmd,4)))) data3 = lib_xtoq(x);
    else ui_showusage(cmd);

    if (cmd_sw_value(cmd,"-loops",&x)) loops = atoi(x);

    for (idx = 0; idx < loops; idx++) {
	__asm __volatile ( " sd %0,0(%4) ; "
			   " sd %1,8(%4) ; "
			   " sd %2,16(%4) ; "
			   " sd %3,24(%4) ; "
			   " sync ; "
			   " sync "
			   : : "r"(data0),"r"(data1),"r"(data2),"r"(data3),"r"(haddr));
	}

    return 0;

}

static int ui_cmd_memmargin(ui_cmdline_t *cmd,int argc,char *argv[])
{
    hsaddr_t uacbase = 0xb800000000000500ULL;
    hsaddr_t ucbase = 0x9000000000000500ULL;
    int nblks=1024*1024;
    uint64_t offset = 0;
    uint64_t d0,d1,d2,d3;
    uint64_t pattern;
    int passnum = 0;
    int idx;
    int errors = 0;
    int wo,ro;

    ro = cmd_sw_isset(cmd,"-ro");
    wo = cmd_sw_isset(cmd,"-wo");

    if (ro && wo) return -1;

    for (;;) {
	errors = 0;
	if (!ro) {
	    offset = 0;
	    pattern = 0x5555555555555555ULL;
	    if (passnum&1) pattern = ~pattern;
	    for (idx = 0; idx < nblks; idx++,offset += 32) {
		__asm __volatile ( " sd %0,0(%1) ; "
				   " sd %0,8(%1) ; "
				   " sd %0,16(%1) ; "
				   " sd %0,24(%1) ; "
				   " sync ; "
				   " sync "
				   : : "r"(pattern),"r"(uacbase+offset));
		pattern = ~pattern;
		}
	    }

	if (!wo) {
	    errors = 0;
	    pattern = 0x5555555555555555ULL;
	    offset = 0;
	    if (passnum&1) pattern = ~pattern;
	    for (idx = 0; idx < nblks; idx++,offset += 32) {
		__asm __volatile ( " ld %1,0(%0) ; "
				   " ld %2,8(%0) ; "
				   " ld %3,16(%0) ; "
				   " ld %4,24(%0) ; "
				   " sync ; "
				   " sync "
				   : "=r"(d0),"=r"(d1),"=r"(d2),"=r"(d3) :"r"(ucbase+offset));
		if ((d0 != pattern) || (d1 != pattern) || (d2 !=pattern) || (d3 != pattern)) errors++;
		pattern = ~pattern;
		}
	    }
	if (console_status()) break;
	printf("%5d Errors: %d\n",passnum,errors);
	errors = 0;
	passnum++;
	}

    return 0;

}


#if CFG_PCI
static uint64_t parse_hex(const char *num)
{
    uint64_t x = 0;
    unsigned int digit;

    if ((*num == '0') && (*(num+1) == 'x')) num += 2;

    while (*num) {
        if ((*num >= '0') && (*num <= '9')) {
            digit = *num - '0';
            }
        else if ((*num >= 'A') && (*num <= 'F')) {
            digit = 10 + *num - 'A';
            }
        else if ((*num >= 'a') && (*num <= 'f')) {
            digit = 10 + *num - 'a';
            }
        else {
            break;
            }
        x *= 16;
        x += digit;
        num++;
        }

    return x;
}

static int ui_cmd_map_pci(ui_cmdline_t *cmd,int argc,char *argv[])
{
    unsigned long offset, size;
    uint64_t paddr;
    int l2ca, endian;
    int enable;
    int result;

    enable = !cmd_sw_isset(cmd, "-off");
    if (enable) {
	offset = parse_hex(cmd_getarg(cmd, 0));
	size = parse_hex(cmd_getarg(cmd, 1));
	paddr = parse_hex(cmd_getarg(cmd, 2));
	l2ca = cmd_sw_isset(cmd,"-l2ca");
	endian = cmd_sw_isset(cmd, "-matchbits");
	result = pci_map_window(paddr, offset, size, l2ca, endian);
	}
    else {
	offset = parse_hex(cmd_getarg(cmd, 0));
	size = parse_hex(cmd_getarg(cmd, 1));
	result = pci_unmap_window(offset, size);
	}

    return result;
}
#endif


/*  *********************************************************************
    *  ui_init_bcm91280ecmds()
    *  
    *  Add BCM91280E-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */


int ui_init_bcm91280ecmds(void)
{
    cmd_addcmd("home",
	       ui_cmd_home,
	       NULL,
	       "Cursor to home position",
	       "home",
	       "");
    cmd_addcmd("cls",
	       ui_cmd_cls,
	       NULL,
	       "Clear screen",
	       "home",
	       "");
    cmd_addcmd("uacwrite",
	       ui_cmd_uacwrite,
	       NULL,
	       "Do an uncached write",
	       "uacwrite addr data0 data1 data2 data3",
	       "-loops=*;Number of iterations");

    cmd_addcmd("memmargin",
	       ui_cmd_memmargin,
	       NULL,
	       "Read/write memory continuously, count errors",
	       "memmargin",
	       "-ro;only do reads|"
	       "-wo;only do writes");

#if CFG_PCI
    cmd_addcmd("map pci",
	       ui_cmd_map_pci,
	       NULL,
	       "Define a BAR0 window available to PCI devices",
	       "map pci offset size paddr [-off] [-l2ca] [-matchbits]\n\n"
	       "Map the region of size bytes starting at paddr to appear\n"
	       "at offset relative to BAR0\n",
	       "-off;Remove the region|"
	       "-l2ca;Make L2 cachable|"
	       "-matchbits;Use match bits policy");
#endif

    return 0;
}




