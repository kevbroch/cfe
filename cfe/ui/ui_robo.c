/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  RoboSwitch SPI hacking commands		File: ui_robospi.c
    *  
    *  These commands let you read and write RoboSwitch registers.
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


int ui_init_robocmds(void);

static int ui_cmd_roboget(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_roboset(ui_cmdline_t *cmd,int argc,char *argv[]);

int ui_init_robocmds(void)
{
    cmd_addcmd("robo get",
	       ui_cmd_roboget,
	       NULL,
	       "Get register value on Robo switch",
	       "robo get page reg [regsize]\n\n"
	       "This command displays the contents of the register on a Robo switch.\n"
	       "By default regsize is 1 (8-bit register)\n",
	       "-dev=*;Specify Robo device number (default=0)");

    cmd_addcmd("robo set",
	       ui_cmd_roboset,
	       NULL,
	       "Set the value of a Robo switch register",
	       "robo set page reg byte0 [byte1 byte2 ...]\n\n"
	       "Sets the value of a register on a Robo switch.  All bytes must be\n"
	       "specified to ensure correct behavior, e.g. for a 16-bit register\n"
	       "byte0 and byte1 must be specified\n",
	       "-dev=*;Specify Robo device number (default=0)");

    return 0;
}

static int ui_cmd_roboget(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    char fname[8];
    int page, reg, len, i, offset, fh, bytes, devno;
    unsigned char buf[8];

    if (cmd_sw_value(cmd,"-dev",&x)) {
	devno = atoi(x);
	}
    else devno = 0;
    xsprintf(fname,"robo%d",devno);

    x = cmd_getarg(cmd,0);
    if (!x) return ui_showusage(cmd);
    page = atoi(x);

    x = cmd_getarg(cmd,1);
    if (!x) return ui_showusage(cmd);
    reg = atoi(x);

    x = cmd_getarg(cmd,2);
    len = x ? atoi(x) : 1;

    fh = cfe_open(fname);
    if (fh < 0) {
        return ui_showerror(fh,"Could not open RoboSwitch device");
        }

    offset = ((page & 0xFF) << 8) | (reg & 0xFF);
    bytes = cfe_readblk(fh,offset,PTR2HSADDR(buf),len);

    cfe_close(fh);

    if (bytes == 0) return -2;

    printf("RoboSwitch register %02X on page %02X =",reg,page);
    for (i = 0; i < len; i++) {
        printf(" %02X", buf[i]);
        }
    printf("\n");

    return 0;
}

static int ui_cmd_roboset(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    char fname[8];
    int page, reg, len, fh, offset, bytes, devno;
    unsigned char buf[8];

    if (cmd_sw_value(cmd,"-dev",&x)) {
	devno = atoi(x);
	}
    else devno = 0;
    xsprintf(fname,"robo%d",devno);

    x = cmd_getarg(cmd,0);
    if (!x) return ui_showusage(cmd);
    page = atoi(x);

    x = cmd_getarg(cmd,1);
    if (!x) return ui_showusage(cmd);
    reg = atoi(x);

    len = 0;
    do {
        x = cmd_getarg(cmd,2+len);
        if (x) {
            buf[len++] = atoi(x);
        }
    } while (x && len < sizeof(buf));

    if (len == 0) {
        return ui_showusage(cmd);
        }

    fh = cfe_open(fname);
    if (fh < 0) {
        return ui_showerror(fh,"Could not open RoboSwitch device");
        }

    offset = ((page & 0xFF) << 8) | (reg & 0xFF);
    bytes = cfe_writeblk(fh,offset,PTR2HSADDR(buf),len);

    cfe_close(fh);

    if (bytes == 0) return -2;

    printf("Wrote %d byte(s) to RoboSwitch page 0x%02X register 0x%02X\n",
           len,page,reg);

    return 0;
}
