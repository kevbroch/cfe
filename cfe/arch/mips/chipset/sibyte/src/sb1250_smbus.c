/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  BCM1250 SMBus controller driver		File: sb1250_smbus.c
    *  
    *  Basic operations for BCM1250 SMBus controller.
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

#include "cfe_smbus.h"

#include "sb1250_defs.h"
#include "sb1250_regs.h"
#include "sb1250_smbus.h"

/*  *********************************************************************
    *  Types
    ********************************************************************* */


typedef struct sb1250_smbus_softc_s {
    long base;				/* physical address of channel */
} sb1250_smbus_softc_t;

/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */


static int sb1250_smbus_qcmd(cfe_smbus_channel_t *chan,uint8_t slave,int rw);
static int sb1250_smbus_xact(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t cmd,uint8_t *buf,int len);
static int sb1250_smbus_read(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t *buf,int len);
static int sb1250_smbus_write(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t *buf,int len);
static int sb1250_smbus_init(cfe_smbus_channel_t *chan);
static cfe_smbus_channel_t *sb1250_smbus_attach(cfe_smbus_t *ops,uint64_t probe_a,uint64_t probe_b);

/*
 * SMBus Extended mode read write routines
 */

static int sb1250_smbus_xread(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t cmd,uint8_t numcmd,uint8_t numdat,int *buf);
static int sb1250_smbus_xwrite(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t cmd,uint8_t numcmd,uint8_t numdat,int *buf);

/*  *********************************************************************
    *  SMBus operations
    ********************************************************************* */

cfe_smbus_t sb1250_smbus = {
    sb1250_smbus_attach,
    sb1250_smbus_init,
    sb1250_smbus_write,
    sb1250_smbus_read,
    sb1250_smbus_xact,
    sb1250_smbus_qcmd,
    sb1250_smbus_xread,
    sb1250_smbus_xwrite
};


static int sb1250_smbus_waitready(sb1250_smbus_softc_t *softc)
{
    uint64_t status;
    int cnt  = 10000000;	/* about 1 second at 1Ghz */

    while (cnt > 0) {
	status = SBREADCSR(softc->base+R_SMB_STATUS);
	if (status & M_SMB_BUSY) {
	    cnt--;
	    continue;
	    }
	break;
	}

    if (cnt == 0) return CFE_ERR_TIMEOUT;

    if (status & M_SMB_ERROR) {
	SBWRITECSR(softc->base+R_SMB_STATUS,(status & M_SMB_ERROR));
	return CFE_ERR_NOTREADY;
	}
    return 0;
}

static cfe_smbus_channel_t *sb1250_smbus_attach(cfe_smbus_t *ops,uint64_t probe_a,uint64_t probe_b)
{
    cfe_smbus_channel_t *chan;	
    sb1250_smbus_softc_t *softc;

    chan = KMALLOC(sizeof(cfe_smbus_channel_t)+sizeof(sb1250_smbus_softc_t),0);

    if (!chan) return NULL;

    chan->ops = ops;
    chan->softc = (void *) (chan+1);

    softc = chan->softc;
    softc->base = probe_a;

    return chan;
}

static int sb1250_smbus_init(cfe_smbus_channel_t *chan)
{
    sb1250_smbus_softc_t *softc = chan->softc;

    /*
     * Assume 100KHz for all devices.  We don't need to go fast
     * ever.
     *
     * Also turn off direct mode and disable interrupts.
     */

    SBWRITECSR(softc->base+R_SMB_FREQ,K_SMB_FREQ_100KHZ);
    SBWRITECSR(softc->base+R_SMB_CONTROL,0);

    /*
     * XXX Could switch to bit-bang mode here and send a bunch
     * of null clocks to reset devices.
     */

    return 0;
}



static int sb1250_smbus_write(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t *buf,int len)
{
    int err;
    sb1250_smbus_softc_t *softc = chan->softc;

    /*
     * Make sure the bus is idle (ignore error here)
     */

    sb1250_smbus_waitready(softc);

    /*
     * Depending on how many bytes we're writing, fill in the various
     * SMB registers and execute the command.
     */

    switch (len) {
	case 1:		/* "command" byte alone */
	    SBWRITECSR(softc->base+R_SMB_CMD,buf[0]);
	    SBWRITECSR(softc->base+R_SMB_START,V_SMB_TT(K_SMB_TT_WR1BYTE) | ((uint64_t)slave));
	    break;
	case 2:		/* "command" byte plus a data byte */
	    SBWRITECSR(softc->base+R_SMB_CMD,buf[0]);
	    SBWRITECSR(softc->base+R_SMB_DATA,buf[1]);
	    SBWRITECSR(softc->base+R_SMB_START,V_SMB_TT(K_SMB_TT_WR2BYTE) | ((uint64_t)slave));
	    break;
	case 3:		/* "command" byte plus 2 data bytes */
	    SBWRITECSR(softc->base+R_SMB_CMD,buf[0]);
	    SBWRITECSR(softc->base+R_SMB_DATA,((uint64_t)(buf[1])) | (((uint64_t)buf[2]) << 8));
	    SBWRITECSR(softc->base+R_SMB_START,V_SMB_TT(K_SMB_TT_WR3BYTE) | ((uint64_t)slave));
	    break;
	default:
	    return CFE_ERR_INV_PARAM;
	    break;
	}

    /*
     * Wait for command to complete.
     */

    err = sb1250_smbus_waitready(softc);
    if (err < 0) return err;

    return 0;
}

static int sb1250_smbus_read(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t *buf,int len)
{
    int err;
    sb1250_smbus_softc_t *softc = chan->softc;

    while (len > 0) {
	err = sb1250_smbus_waitready(softc);
	if (err < 0) return err;

	SBWRITECSR(softc->base+R_SMB_START,V_SMB_TT(K_SMB_TT_RD1BYTE) | ((uint64_t)slave));

	err = sb1250_smbus_waitready(softc);
	if (err < 0) return err;

	*buf++ = (uint8_t) SBREADCSR(softc->base+R_SMB_DATA);
	len--;
	}

    return 0;
}

static int sb1250_smbus_xwrite(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t cmd,uint8_t numcmd,uint8_t numdat,int *buf)
{
    sb1250_smbus_softc_t *softc = chan->softc;
    int t;
    int err ;

    /*
     * Make sure the bus is idle (ignore error here)
     */

    sb1250_smbus_waitready(softc);

    /*
     * Do extended mode write access
     */	

    SBWRITECSR(softc->base+R_SMB_CMD,cmd);
    t = buf[0] + (buf[1]<<8) ;
    SBWRITECSR(softc->base+R_SMB_DATA,t);
    // if numdat>2 SBWRITECSR(softc->base+R_SMB_?,ext);
    
    SBWRITECSR(softc->base+ R_SMB_START, M_SMB_EXTEND |  V_SMB_AFMT(numcmd) | V_SMB_DFMT(numdat) | ((uint64_t)slave));

    err = sb1250_smbus_waitready(softc);
    if (err < 0) return err;
	
   return 0;
}

static int sb1250_smbus_xread(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t cmd,uint8_t numcmd,uint8_t numdat,int *buf)
{
    sb1250_smbus_softc_t *softc = chan->softc;

    /*
     * Make sure the bus is idle (ignore error here)
     */

    sb1250_smbus_waitready(softc);

    /*
     * Do extended mode read access
     */	

    SBWRITECSR(softc->base+R_SMB_CMD,cmd);
    SBWRITECSR(softc->base+ R_SMB_START, M_SMB_EXTEND | M_SMB_DIR | V_SMB_AFMT(numcmd) | V_SMB_DFMT(numdat) | ((uint64_t)slave));

    sb1250_smbus_waitready(softc);

    *buf = (int) SBREADCSR(softc->base+R_SMB_DATA);
	
    return 0;
}

static int sb1250_smbus_xact(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t cmd,uint8_t *buf,int len)
{
    sb1250_smbus_softc_t *softc = chan->softc;
    uint64_t data;
    int err;

    /*
     * Make sure the bus is idle (ignore error here)
     */

    sb1250_smbus_waitready(softc);

    /*
     * The "command" byte goes out...
     */

    SBWRITECSR(softc->base+R_SMB_CMD,cmd);

    /*
     * And a variable number of bytes come back.
     */

    switch (len) {
	case 1:
	    SBWRITECSR(softc->base+R_SMB_START,V_SMB_TT(K_SMB_TT_CMD_RD1BYTE) | ((uint64_t)slave));
	    break;
	case 2:
	    SBWRITECSR(softc->base+R_SMB_START,V_SMB_TT(K_SMB_TT_CMD_RD2BYTE) | ((uint64_t)slave));
	    break;
	default:
	    return CFE_ERR_INV_PARAM;
	}

    err = sb1250_smbus_waitready(softc);
    if (err < 0) return err;

    /*
     * Get data from device and unpack it for application.
     */

    data = SBREADCSR(softc->base+R_SMB_DATA);

    switch (len) {
	case 1:	
	    *buf++ = (data & 0xFF);
	    break;
	case 2:
	    *buf++ = (data & 0xFF);
	    data >>= 8;
	    *buf++ = (data & 0xFF);
	    break;
	}
    
    return 0;

}

static int sb1250_smbus_qcmd(cfe_smbus_channel_t *chan,uint8_t slave,int rw)
{
    int err;
    sb1250_smbus_softc_t *softc = chan->softc;

    /*
     * Make sure the bus is idle (ignore error here)
     */

    sb1250_smbus_waitready(softc);

    SBWRITECSR(softc->base+R_SMB_START,V_SMB_TT(K_SMB_TT_QUICKCMD) | ((uint64_t)slave) |
	       (rw ? M_SMB_QDATA : 0));

    err = sb1250_smbus_waitready(softc);

    return err;
}
