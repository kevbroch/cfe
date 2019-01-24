/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PCA9543A SMBus Switch  driver		File: dev_smbus_switch.c
    *  
    *  This module contains a CFE driver for a PCA9543A SMBus Switch.
    *  
    *  Author: Binh Vo
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
#include "cfe_smbus.h"


/*  *********************************************************************
    *  Constants
    ********************************************************************* */

/*
 * Register bits
 */

#define SWITCH_CHAN0		0x01		/* Channel 0 */
#define SWITCH_CHAN1		0x02		/* Channel 1 */

/*
 * Register numbers
 */


/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */

static void switch_probe(cfe_driver_t *drv,
			      unsigned long probe_a, unsigned long probe_b, 
			      void *probe_ptr);

static int switch_open(cfe_devctx_t *ctx);
static int switch_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int switch_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int switch_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int switch_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int switch_close(cfe_devctx_t *ctx);


/*  *********************************************************************
    *  Device dispatch
    ********************************************************************* */

const static cfe_devdisp_t switch_dispatch = {
  switch_open,
  switch_read,
  switch_inpstat,
  switch_write,
  switch_ioctl,
  switch_close,
  NULL,
  NULL
};

const cfe_driver_t smbus_switch = {
  "SMBUS SWITCH",
  "switch",
  CFE_DEV_OTHER,
  &switch_dispatch,
  switch_probe
};
  

/*  *********************************************************************
    *  Structures
    ********************************************************************* */
typedef struct switch_s {
  cfe_smbus_channel_t *smbus_channel;
} switch_t;
  
/*  *********************************************************************
    *  smbus_readbyte(chan,slaveaddr,devaddr)
    *  
    *  Read a byte from the chip.
    *
    *  Input parameters: 
    *  	   chan - SMBus channel
    *  	   slaveaddr -  SMBus slave address
    *  	   devaddr - byte within the at24c02 device to read
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else -1
    ********************************************************************* */

static int smbus_readbyte(cfe_smbus_channel_t *chan,int slaveaddr,int devaddr)
{
    uint8_t buf[1];
    int err;

    /*
     * Start the command
     */

    buf[0] = devaddr;
    err = SMBUS_WRITE(chan,slaveaddr,buf,1);
    if (err < 0) return err;

    /*
     * Read the data byte
     */

    err = SMBUS_READ(chan,slaveaddr,buf,1);
    if (err < 0) return err;

    return buf[0];
}

/*  *********************************************************************
    *  smbus_writebyte(chan,slaveaddr,devaddr,b)
    *  
    *  write a byte from the chip.
    *  
    *  Input parameters: 
    *  	   chan - SMBus channel
    *  	   slaveaddr -  SMBus slave address
    *  	   devaddr - byte within the at24c02 device to read
    *      b - byte to write
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else -1
    ********************************************************************* */

static int smbus_writebyte(cfe_smbus_channel_t *chan,int slaveaddr,int devaddr,int b)
{
    uint8_t buf[2];
    int err;
    int64_t timer;

    /*
     * Write the data to the controller
     */

    buf[0] = devaddr;
    buf[1] = b;

    err = SMBUS_WRITE(chan,slaveaddr,buf,2);

    /*
     * Pound on the device with a current address read
     * to poll for the write complete
     */

    TIMER_SET(timer,50);
    err = -1; 

    while (!TIMER_EXPIRED(timer)) {
	POLL();

	err = SMBUS_READ(chan,slaveaddr,buf,1);
	if (err == 0) break;
	}

    return err;
}

/*  *********************************************************************
    *  switch_probe(drv,a,b,ptr)
    *  
    *  Probe routine for this driver.  This routine creates the 
    *  local device context and attaches it to the driver list
    *  within CFE.
    *  
    *  Input parameters: 
    *  	   drv - driver handle
    *  	   a,b - probe hints (longs)
    *  	   ptr - probe hint (pointer)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void switch_probe(cfe_driver_t *drv,
				     unsigned long probe_a, unsigned long probe_b, 
				     void *probe_ptr)
{
    switch_t *softc;
    char descr[80];

    softc = (switch_t *) KMALLOC(sizeof(switch_t),0);

    /*
     * Probe_a is the SMBus channel number
     * Probe_b is the SMBus device address
     * Probe_ptr is unused.
     */

    softc->smbus_channel = SMBUS_CHANNEL((int)probe_a);

    xsprintf(descr,"%s on SMBus channel %d dev 0x%02X",
	     drv->drv_description,probe_a,probe_b);
    cfe_attach(drv,softc,NULL,descr);
   
}

/*  *********************************************************************
    *  switch_open(ctx)
    *  
    *  Open this device.  For the M41T81, we do a quick test 
    *  read to be sure the device is out there.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else error code
    ********************************************************************* */

static int switch_open(cfe_devctx_t *ctx)
{
    switch_t *softc = ctx->dev_softc;
    int b = -1;

    b = smbus_readbyte(softc->smbus_channel,
		       0,
		       0);
    b = smbus_writebyte(softc->smbus_channel,
			0,
			0,
			0x01);

    return (b < 0) ? -1 : 0;
}

/*  *********************************************************************
    *  switch_read(ctx,buffer)
    *  
    *  Read from switch.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   buffer - buffer descriptor (target buffer, length, offset)
    *  	   
    *  Return value:
    *  	   number of bytes read
    *  	   -1 if an error occured
    ********************************************************************* */

static int switch_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    return 0;
}

/*  *********************************************************************
    *  switch_write(ctx,buffer)
    *  
    *  Write to switch.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   buffer - buffer descriptor (target buffer, length, offset)
    *  	   
    *  Return value:
    *  	   number of bytes written
    *  	   -1 if an error occured
    ********************************************************************* */

static int switch_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    return 0;
}

/*  *********************************************************************
    *  switch_inpstat(ctx,inpstat)
    *  
    *  Test input (read) status for the device
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   inpstat - input status descriptor to receive value
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   -1 if an error occured
    ********************************************************************* */

static int switch_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat)
{
    inpstat->inp_status = 1;

    return 0;
}

/*  *********************************************************************
    *  switch_ioctl(ctx,buffer)
    *  
    *  Perform miscellaneous I/O control operations on the device.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   buffer - buffer descriptor (target buffer, length, offset)
    *  	   
    *  Return value:
    *  	   number of bytes read
    *  	   -1 if an error occured
    ********************************************************************* */

static int switch_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer) 
{
  return 0;
}

/*  *********************************************************************
    *  switch_close(ctx,buffer)
    *  
    *  Close the device.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   -1 if an error occured
    ********************************************************************* */

static int switch_close(cfe_devctx_t *ctx)
{
    return 0;
}








