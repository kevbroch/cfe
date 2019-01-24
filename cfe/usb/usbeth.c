/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  USB Ethernet				File: usbeth.c
    *  
    *  Generic top-level interface for USB Ethernet devices
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2005
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

/*  *********************************************************************
    *  USB-Ethernet driver - CFE Network Layer Interfaces
    ********************************************************************* */

#include "cfe.h"

#include "usbd.h"
#include "usbeth.h"


typedef struct usbeth_unit_s {
    int unit;
    usbeth_disp_t *dispatch;
    void *softc;
} usbeth_unit_t;


/*  *********************************************************************
    *  CFE Device registration stuff
    *  
    *  Like the USB serial device, board packages instantiate
    *  a "usbeth" device and as the actual usb devices come and
    *  go they attach to an array of pointers to devices, below.
    *  
    *  The USB device attach process eventually calls usbeth_register
    *  to attach the dynamic device to the static CFE device,
    *  and usbeth_unregister to remove it.  the CFE device
    *  stays around.
    ********************************************************************* */

#define USBETH_MAXUNITS 4
static usbeth_unit_t *usbeth_units[USBETH_MAXUNITS] = {0};

int usbeth_register(usbeth_disp_t *disp,void *softc)
{
    int idx;
    usbeth_unit_t *unit;

    for (idx = 0; idx < USBETH_MAXUNITS; idx++) {
	unit = usbeth_units[idx];
	if (!unit) continue;
	if (!unit->dispatch) {
	    unit->dispatch = disp;
	    unit->softc = softc;
	    return idx;
	    }
	}

    return -1;
}


void usbeth_unregister(void *softc)
{
    int idx;
    usbeth_unit_t *unit;

    /*
     * We had better find the softc.  If we fail to find it,
     * something is very wrong! 
     */
      
    for (idx = 0; idx < USBETH_MAXUNITS; idx++) {
	unit = usbeth_units[idx];
	if (!unit) continue;
	if (unit->softc == softc) {
	    unit->dispatch = NULL;
	    unit->softc = NULL;
	    }
	}
}

/*  *********************************************************************
    * CFE-Ethernet device interfaces
    ********************************************************************* */


static int usb_ether_open(cfe_devctx_t *ctx)
{
    return 0;
}

static int usb_ether_read( cfe_devctx_t *ctx, iocb_buffer_t *buffer )
{
    usbeth_unit_t *unit = (usbeth_unit_t *) ctx->dev_softc;

    /* If no device is hooked up, pretend like the cable is disconnected */
    if (unit->softc == NULL) {
	return CFE_ERR_NOTREADY;
	}

    buffer->buf_retlen = (*(unit->dispatch->read))(unit->softc,buffer->buf_ptr);

    return 0;
}


static int usb_ether_inpstat( cfe_devctx_t *ctx, iocb_inpstat_t *inpstat )
{
    usbeth_unit_t *unit = (usbeth_unit_t *) ctx->dev_softc;

    /* If no device is hooked up, pretend like the cable is disconnected */
    if (unit->softc == NULL) {
	inpstat->inp_status = 0;
	return 0;
	}

    inpstat->inp_status = (*(unit->dispatch->inpstat))(unit->softc);

    return 0;
}


static int usb_ether_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    usbeth_unit_t *unit = (usbeth_unit_t *) ctx->dev_softc;

    /* If no device is hooked up, pretend like the cable is disconnected */
    if (unit->softc == NULL) {
	return CFE_ERR_NOTREADY;
	}

    /* Block until hw notifies you data is sent. */
    (*(unit->dispatch->write))(unit->softc,buffer->buf_ptr, buffer->buf_length );

    return 0;
}


static int usb_ether_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    usbeth_unit_t *unit = (usbeth_unit_t *) ctx->dev_softc;
    int retval = 0;

    /* If no device is hooked up, pretend like the cable is disconnected */
    if (unit->softc == NULL) {
	return CFE_ERR_UNSUPPORTED;
	}

    switch( (int)buffer->buf_ioctlcmd ) {
	case IOCTL_ETHER_GETHWADDR:
	    (*(unit->dispatch->getaddr))(unit->softc,buffer->buf_ptr );
	    break;
	default:
	    retval = CFE_ERR_UNSUPPORTED;
	}

    return retval;
}


static int usb_ether_close(cfe_devctx_t *ctx)
{
    return 0;
}

static void usb_ether_probe(cfe_driver_t *drv,
			   unsigned long probe_a, unsigned long probe_b, 
			   void *probe_ptr)
{
    usbeth_unit_t *unit;
    char descr[80];

    unit = (usbeth_unit_t *) KMALLOC(sizeof(usbeth_unit_t),0);
    memset(unit,0,sizeof(usbeth_unit_t));
    unit->unit = (int)probe_a;

    usbeth_units[unit->unit] = unit;

    xsprintf(descr,"USB Ethernet unit %d",(int)probe_a);
    cfe_attach(drv,unit,NULL,descr);
}


/* CFE ethernet device interface structures */

const static cfe_devdisp_t usb_ether_dispatch =
{
    usb_ether_open,
    usb_ether_read,
    usb_ether_inpstat,
    usb_ether_write,
    usb_ether_ioctl,
    usb_ether_close,
    NULL,
    NULL
};

const cfe_driver_t usb_ether =
{
    "USB-Ethernet Device",
    "eth",
    CFE_DEV_NETWORK,
    &usb_ether_dispatch,
    usb_ether_probe,
};

