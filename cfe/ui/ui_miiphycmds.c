/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PHY hacking commands			File: ui_phycmds.c
    *  
    *  These commands let you directly muck with the PHYs
    *  attached to the Ethernet controllers.
    *  
    *********************************************************************  
    *
    *  Copyright 2004
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

#include "cfe_mii.h"


int ui_init_phycmds(void);

static int ui_cmd_phydump(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_physet(ui_cmdline_t *cmd,int argc,char *argv[]);


int ui_init_phycmds(void)
{
    cmd_addcmd("phy dump",
	       ui_cmd_phydump,
	       NULL,
	       "Dump the registers on the PHY",
	       "phy dump macid [reg]\n\n"
	       "This command displays the contents of the registers on the PHY\n"
	       "attached to the specified Ethernet controller.  macid is the\n"
	       "Ethernet controller ID (0..2 for the BCM1250) and reg\n"
	       "is an optional register number (0..31).  By default, all registers\n"
	       "are displayed.",
	       "-phy=*;Specify PHY address (default=1)");

    cmd_addcmd("phy set",
	       ui_cmd_physet,
	       NULL,
	       "Set the value of a PHY register",
	       "phy set macid reg value\n\n"
	       "Sets the value of a register on a PHY.  macid is the Ethernet\n"
	       "controller number (0..2 for the BCM1250), reg is the register\n"
	       "number (0..31), and value is the 16-bit value to write to the\n"
	       "register.\n",
	       "-phy=*;Specify PHY address (default=1)");

    return 0;
}

static int ui_cmd_phydump(ui_cmdline_t *cmd,int argc,char *argv[])
{
    cfe_mii_channel_t *mii_channel;
    int phynum;
    int idx;
    int mac;
    char *x;
    unsigned int reg;
    int allreg = 1;

    x = cmd_getarg(cmd,0);
    if (!x) return ui_showusage(cmd);

    mac = atoi(x);
    if ((mac < 0) || (mac >= MII_CHANNELS_MAX) || (MII_CHANNEL(mac) == NULL)) {
	return ui_showerror(CFE_ERR_INV_PARAM,"Invalid MAC number");
	}
    mii_channel = MII_CHANNEL(mac);

    if (cmd_sw_value(cmd,"-phy",&x)) {
	phynum = atoi(x);
	}
    else phynum = MII_DEFAULT_ADDR(mii_channel);
    
    x = cmd_getarg(cmd,1);
    reg = 0;
    if (x) {
	reg = atoi(x);
	if ((reg < 0) || (reg > 31)) {
	    return ui_showerror(CFE_ERR_INV_PARAM,"Invalid phy register number");
	    }
	allreg = 0;
	}

    if (allreg) {
	printf("** PHY registers on MAC %d PHY %d **\n",mac,phynum);
	for (idx = 0; idx < 31; idx+=2) {
	    printf("Reg 0x%02X  =  0x%04X   |  ",idx,MII_READ(mii_channel,phynum,idx));
	    printf("Reg 0x%02X  =  0x%04X",idx+1,MII_READ(mii_channel,phynum,idx+1));
	    printf("\n");
	    }
	}
    else {
	printf("Reg %02X = %04X\n",reg,MII_READ(mii_channel,phynum,reg));
	}

    return 0;

}

static int ui_cmd_physet(ui_cmdline_t *cmd,int argc,char *argv[])
{
    cfe_mii_channel_t *mii_channel;
    int phynum;
    int mac;
    char *x;
    unsigned int value;
    unsigned int reg;

    x = cmd_getarg(cmd,0);
    if (!x) return ui_showusage(cmd);

    mac = atoi(x);
    if ((mac < 0) || (mac >= MII_CHANNELS_MAX) || (MII_CHANNEL(mac) == NULL)) {
	return ui_showerror(CFE_ERR_INV_PARAM,"Invalid MAC number");
	}
    mii_channel = MII_CHANNEL(mac);

    if (cmd_sw_value(cmd,"-phy",&x)) {
	phynum = atoi(x);
	}
    else phynum = MII_DEFAULT_ADDR(mii_channel);
    
    x = cmd_getarg(cmd,1);
    if (!x) return ui_showusage(cmd);
    reg = atoi(x);
    if ((reg < 0) || (reg > 31)) {
	return ui_showerror(CFE_ERR_INV_PARAM,"Invalid phy register number");
	}

    x = cmd_getarg(cmd,2);
    if (!x) return ui_showusage(cmd);
    value = atoi(x) & 0xFFFF;

    MII_WRITE(mii_channel,phynum,reg,value);

    printf("Wrote 0x%04X to phy %d register 0x%02X on mac %d\n",
	   value,phynum,reg,mac);

    return 0;
}
