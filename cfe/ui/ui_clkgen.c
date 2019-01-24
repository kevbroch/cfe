/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Chunnel clock generator commands		File: ui_clkgen.c
    *  
    *  Author: Ed Satterthwaite
    *  
    *********************************************************************  
    *
    *  Copyright 2002,2003
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
    *     and retain this copyright notice and list of conditions as 
    *     they appear in the source file.
    *  
    *  2) No right is granted to use any trade name, trademark, or 
    *     logo of Broadcom Corporation. Neither the "Broadcom 
    *     Corporation" name nor any trademark or logo of Broadcom 
    *     Corporation may be used to endorse or promote products 
    *     derived from this software without the prior written 
    *     permission of Broadcom Corporation.
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
#include "cfe_smbus.h"


/* The following are for devices with single-byte internal addresses
   (offsets) */

/*  *********************************************************************
    *  smbus_readbyte(base,slaveaddr,offset)
    *  
    *  Read a byte from the chip.
    *  
    *  Input parameters: 
    *  	   base - SMBus channel base (k1seg)
    *  	   dev -  SMBus slave address
    *  	   offset - address of byte within device
    *  	   
    *  Return value:
    *  	   byte if ok else error (< 0)
    ********************************************************************* */

static int
smbus_readbyte(cfe_smbus_channel_t *chan, uint8_t dev, uint8_t offset)
{
    uint8_t buf[1];
    int err;

    err = SMBUS_XACT(chan, dev, offset, buf, 1);
    if (err < 0) return err;

    return buf[0];
}

/*  *********************************************************************
    *  smbus_writebyte(base,dev,offset,b)
    *  
    *  write a byte to the chip
    *  
    *  Input parameters: 
    *  	   base - SMBus channel base (k1seg)
    *  	   dev -  SMBus slave address
    *  	   offset - byte with in the device to read
    *      b - byte to write
    *  	   
    *  Return value:
    *  	   0 if ok else error (< 0)
    ********************************************************************* */

static int
smbus_writebyte(cfe_smbus_channel_t *chan, uint8_t dev,
		uint8_t offset, uint8_t b)
{
    uint8_t buf[2];
    int err;

    buf[0] = offset;
    buf[1] = b;

    err = SMBUS_WRITE(chan, dev, buf, 2);
    return err;
}

#ifndef CY22393_SMBUS_CHAN
#define CY22393_SMBUS_CHAN  1
#endif

#ifndef CY22393_SMBUS_DEV
#define CY22393_SMBUS_DEV   0x59
#endif

#define SWITCH_DIVSEL 0

static int      smerr = -1;

int clkgen_present(void);
int clkgen_init(void);
int clkgen_update(void);


/* ClkA is driven through a divider from PLL1.  If SWITCH_DIVSEL,
   switching is done by changing the DivSel bit in the appropriate
   PLL1 entry.  Currently, the host board pulls S0 and S1 (SDAT and
   SCLK) to 0 at Chunnel power up and S1 = 1, so the appropriate PLL1
   entry spans indices 0x4C-0x4E.  Otherwise, the ClkA(0) divider is
   changed directly.  */

int
clkgen_init(void)
{
    cfe_smbus_channel_t *chan = SMBUS_CHANNEL(CY22393_SMBUS_CHAN);

    smerr = smbus_readbyte(chan, CY22393_SMBUS_DEV, 0x08);
    if (smerr < 0)
	return -1;

    /* Set CLKA divider = 6. 400/6 = 66MHZ */
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x08, 0x06);
    
    /* Set CLKA DivSel divider = 3. 300/3 = 100MHZ */
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x09, 0x03);    

    /* 0x01 = use PLL1 on CLKA */
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x0E, 0x01);

    /* PLL1: S2,1,0=000 DivSel=0 400MHZ */
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x40, 0x02);
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x41, 0x05);
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x42, 0x40);

    /* PLL1: S2,1,0=100 DivSel=1 300MHZ */
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x4C, 0x04);
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x4D, 0x06);
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x4E, 0xC0);    

#if SWITCH_DIVSEL
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x09, 0x08);
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x4e, 0x40);
#endif
    cfe_sleep(CFE_HZ/8);
    return 0;
}

int
clkgen_present(void)
{
    clkgen_init();

    return smerr >= 0;
}

int
clkgen_update(void)
{
    cfe_smbus_channel_t *chan = SMBUS_CHANNEL(CY22393_SMBUS_CHAN);

    if (smbus_readbyte(chan, CY22393_SMBUS_DEV, 0x08) != 0x06)
	return -1;

#if SWITCH_DIVSEL
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x4E, 0xC0);
#else
    smbus_writebyte(chan, CY22393_SMBUS_DEV, 0x08, 0x08);
#endif
    cfe_sleep(CFE_HZ/8);
    return 0;
}


#include "ui_command.h"

int ui_init_clkgencmds(void);

static int ui_cmd_rdclk(ui_cmdline_t *cmd, int argc, char *argv[]);
static int ui_cmd_wrclk(ui_cmdline_t *cmd, int argc, char *argv[]);
static int ui_cmd_dumpclk(ui_cmdline_t *cmd, int argc, char *argv[]);

int
ui_init_clkgencmds(void)
{
    cmd_addcmd("clkgen read",
	       ui_cmd_rdclk,
	       NULL,
	       "Read a CY22393 register",
	       "clkgen read addr",
	       "");
    cmd_addcmd("clkgen write",
	       ui_cmd_wrclk,
	       NULL,
	       "Read a CY22393 register",
	       "clkgen write addr data",
	       "");
    cmd_addcmd("clkgen dump",
	       ui_cmd_dumpclk,
	       NULL,
	       "Dump all CY22393 registers",
	       "clkgen dump",
	       "");
    return 0;
}

static int
ui_cmd_rdclk(ui_cmdline_t *cmd, int argc, char *argv[])
{
    uint8_t offset;
    uint8_t b;
    cfe_smbus_channel_t *chan = SMBUS_CHANNEL(CY22393_SMBUS_CHAN);


    offset = 0;
    if (argc == 1) {
        char *x = cmd_getarg(cmd, 0);
	if (x) offset = (uint8_t) xtoi(x);
        }
    else
	return -1;

    b = smbus_readbyte(chan, CY22393_SMBUS_DEV, offset);
    xprintf("%02X: %02X\n", offset, b);
    return 0;
}

static int
ui_cmd_wrclk(ui_cmdline_t *cmd, int argc, char *argv[])
{
    uint8_t offset;
    uint8_t b;
    cfe_smbus_channel_t *chan = SMBUS_CHANNEL(CY22393_SMBUS_CHAN);
    int err;

    offset = 0;
    b = 0xFF;

    if (argc == 2) {
        char *x;
	x = cmd_getarg(cmd, 0);
	if (x) offset = (uint8_t) xtoi(x);
	x = cmd_getarg(cmd, 1);
	if (x) b = (uint8_t) xtoi(x);
        }
    else
	return -1;

    err = smbus_writebyte(chan, CY22393_SMBUS_DEV, offset, b);
    return 0;
}

static int
ui_cmd_dumpclk(ui_cmdline_t *cmd, int argc, char *argv[])
{
    uint8_t offset;
    uint8_t b;
    cfe_smbus_channel_t *chan = SMBUS_CHANNEL(CY22393_SMBUS_CHAN);

    for (offset = 0x08; offset < 0x18; offset++) {
	if (offset % 8 == 0) 
	    xprintf("%02X:", offset);
	b = smbus_readbyte(chan, CY22393_SMBUS_DEV, offset);
	xprintf(" %02X", b);
	if (offset % 8 == 7)
	    xprintf("\n");
    }

    for (offset = 0x40; offset < 0x58; offset++) {
	if (offset % 8 == 0) 
	    xprintf("%02X:", offset);
	b = smbus_readbyte(chan, CY22393_SMBUS_DEV, offset);
	xprintf(" %02X", b);
	if (offset % 8 == 7)
	    xprintf("\n");
    }

    return 0;
}
