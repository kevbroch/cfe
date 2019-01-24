/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  BCM91480B-specific commands		File: ui_bcm91480b.c
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

#include "bcm91480b.h"
#include "lib_hssubr.h"
#include "lib_try.h"
#include "lib_memfuncs.h"
#include "sb1250_defs.h"
#include "bcm1480_regs.h"
#include "bcm1480_scd.h"

/*  *********************************************************************
    *  Configuration
    ********************************************************************* */


/*  *********************************************************************
    *  prototypes
    ********************************************************************* */

int ui_init_bcm91480bcmds(void);


/*  *********************************************************************
    *  Data
    ********************************************************************* */

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

    haddr = PHYS_TO_XKPHYS(K_CALG_UNCACHED_ACCEL, (hsaddr_t)addr);

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
    hsaddr_t uacbase = PHYS_TO_XKPHYS(K_CALG_UNCACHED_ACCEL, (hsaddr_t)0x500);
    hsaddr_t ucbase = PHYS_TO_XKPHYS(K_CALG_UNCACHED, (hsaddr_t)0x500);
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


#if CFG_LDT
/*  *********************************************************************
    *  ui_cmd_node_enable(cmd,argc,argv)
    *  
    *  Set up node-based routing (board specific)
    *  
    *  Return value:
    *  	   -1 if error occurred.
    ********************************************************************* */

extern int _pci_enumerated;
extern void ht_node_route(int port, int source_node, int dest_node);

static int ui_cmd_node_enable(ui_cmdline_t *cmd,int argc,char *argv[])
{
    unsigned int board_config;
    unsigned int node_id;
    unsigned int port;
    char *x;

    /* In a multinode system, the slave side of each link must 
       complete PCI/HT initialization before the master side is 
       configured.  Currently, PCI/HT is delayed on both ends of 
       the link and must be initiated manually in the proper sequence. 
       If it hasn't been done already, force it now.  */
    if (!_pci_enumerated)
	ui_docommands("show pci -init");

    board_config = board_get_config();
    node_id = (board_config & BOARD_CFG_NODE_ID ? 5 : 4);

    port = 0;   /* default */
    if (cmd_sw_value(cmd, "-port", &x)) port = atoi(x);
    if (port != 0 && port != 1) {
	xprintf("port %d not available for internode links\n", port);
	return CFE_ERR;
	}

    /* XXX Routing currently the nodes on each end have node ids that
       differ only in the least significant bit.  */
    ht_node_route(port, node_id, node_id ^ 1);

    return 0;
}
#endif


/*  *********************************************************************
    *  ui_init_bcm91480bcmds()
    *  
    *  Add BCM91480B-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */


int ui_init_bcm91480bcmds(void)
{
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

#if CFG_LDT
    cmd_addcmd("node enable",
	       ui_cmd_node_enable,
	       NULL,
	       "Enable HT node routing",
	       "node enable",
	       "-port=*;Port to use, 0 (default) or 1");
#endif

    return 0;
}




