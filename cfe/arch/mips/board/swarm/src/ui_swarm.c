/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  SWARM-specific commands			File: ui_swarm.c
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

#include "sbmips.h"
#include "sb1250_regs.h"
#include "sb1250_scd.h"

#include "swarm.h"

/*  *********************************************************************
    *  Configuration
    ********************************************************************* */

/*  *********************************************************************
    *  prototypes
    ********************************************************************* */

int ui_init_swarmcmds(void);

#if CFG_PCI
static int ui_cmd_map_pci(ui_cmdline_t *cmd,int argc,char *argv[]);
#endif

#if CFG_VGACONSOLE
static int ui_cmd_vgadump(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_vgainit(ui_cmdline_t *cmd,int argc,char *argv[]);
extern int vga_biosinit(void);
extern void vgaraw_dump(char *tail);
#endif


/*  *********************************************************************
    *  Data
    ********************************************************************* */


/*  *********************************************************************
    *  ui_init_swarmcmds()
    *  
    *  Add SWARM-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */


int ui_init_swarmcmds(void)
{
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
#if CFG_VGACONSOLE
    cmd_addcmd("vga init",
	       ui_cmd_vgainit,
	       NULL,
	       "Initialize the VGA adapter.",
	       "vgainit",
	       "");

    cmd_addcmd("vga dumpbios",
	       ui_cmd_vgadump,
	       NULL,
	       "Dump the VGA BIOS to the console",
	       "vga dumpbios",
	       "");
#endif

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


#if CFG_VGACONSOLE
static int ui_cmd_vgainit(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int res;

    res = vga_biosinit();

    xprintf("vgaraw_init returns %d\n",res);
    
    return 0;
}

static int ui_cmd_vgadump(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;

    x = cmd_getarg(cmd,0);
    if (!x) x = "";

    vgaraw_dump(x);
    
    return 0;
}
#endif


