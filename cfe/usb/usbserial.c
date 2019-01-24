/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  USB Serial Port Driver			File: usbserial.c	
    *  
    *  This device can talk to a few of those usb->serial converters
    *  out there.
    *  
    *  Author:  Mitch Lichtenberg
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


#ifndef _CFE_
#include <stdio.h>
#include <time.h>
#include <memory.h>
#include <stdint.h>
#include "usbhack.h"
#include "lib_malloc.h"
#include "lib_queue.h"
#else
#include "cfe.h"
#endif

#include "usbchap9.h"
#include "usbd.h"

/* This driver supports USB serial devices based on the Prolific
   PL-2303 and the MCT USB-232 chips.  Such devices have been marketed
   by a large number of vendors, and a specific device will not be
   recognized until the vendor and model number have been entered in
   tables of known devices here and in usbdevs.c.  */

/* For systems with non-coherent DMA, allocate all buffers to be
   cache-aligned and multiples of a cache line in size, so that they
   can be safely flushed or invalidated. */

#define CACHE_ALIGN    32       /* XXX place holder, big enough to now. */
#define BUFF_ALIGN     16
#define ALIGN(n,align) (((n)+((align)-1)) & ~((align)-1))

#define usb_dma_alloc(n) (KMALLOC(ALIGN((n),CACHE_ALIGN),BUFF_ALIGN))
#define usb_dma_free(p)  (KFREE(p))

/*  *********************************************************************
    *  Constants
    ********************************************************************* */

#define USER_FIFOSIZE	256

/*  *********************************************************************
    *  Structures
    ********************************************************************* */

/* For Prolific-style devices. The value of LineDataBaud is the
   desired bit ("baud") rate. */

typedef struct usbser_linedata_s {
    uint8_t dLineDataBaud0,dLineDataBaud1,dLineDataBaud2,dLineDataBaud3;
    uint8_t bLineDataStopBits;	/* 0=1, 1=1.5, 2=2 */
    uint8_t bLineDataParity;	/* 0=none, 1=odd, 2=even, 3=mark, 4=space */
    uint8_t bLineDataBits;	/* 5,6,7,8 */
} usbser_linedata_t;

#define PL_SET_REQ              0x21
#define PL_GET_REQ              0xA1

#define PL_REQ_SET_LINE_CODING  0x20
#define PL_REQ_GET_LINE_CODING  0x21


/* For MCT-style devices.  The value of BaudData is either an index
   into a list of supported baud rates or (115200/baud) depending on
   the particular device.  */

typedef struct usbser_bauddata_s {
    uint8_t dBaudData0,dBaudData1,dBaudData2,dBaudData3;
} usbser_bauddata_t;

#define MCT_SET_REQ             0x40
#define MCT_GET_REQ             0xC0

#define MCT_REQ_SET_BAUD        0x05


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#define GETDWFIELD(s,f) ((uint32_t)((s)->f##0) | ((uint32_t)((s)->f##1) << 8) | \
                          ((uint32_t)((s)->f##2) << 16) | ((uint32_t)((s)->f##3) << 24))
#define PUTDWFIELD(s,f,v) (s)->f##0 = (v & 0xFF); \
                           (s)->f##1 = ((v)>>8 & 0xFF); \
                           (s)->f##2 = ((v)>>16 & 0xFF); \
                           (s)->f##3 = ((v)>>24 & 0xFF);



/*  *********************************************************************
    *  Forward Definitions
    ********************************************************************* */

static int usbserial_attach(usbdev_t *dev,usb_driver_t *drv);
static int usbserial_detach(usbdev_t *dev);

#ifdef _CFE_
static void usb_uart_probe(cfe_driver_t *drv,
			   unsigned long probe_a, unsigned long probe_b, 
			   void *probe_ptr);

static int usb_uart_open(cfe_devctx_t *ctx);
static int usb_uart_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int usb_uart_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int usb_uart_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int usb_uart_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int usb_uart_close(cfe_devctx_t *ctx);

const static cfe_devdisp_t usb_uart_dispatch = {
    usb_uart_open,
    usb_uart_read,
    usb_uart_inpstat,
    usb_uart_write,
    usb_uart_ioctl,
    usb_uart_close,	
    NULL,
    NULL
};

const cfe_driver_t usb_uart = {
    "USB UART",
    "uart",
    CFE_DEV_SERIAL,
    &usb_uart_dispatch,
    usb_uart_probe
};

typedef struct usb_uart_s {
    int uart_unit;
    int uart_speed;
    int uart_flowcontrol;
} usb_uart_t;

#define USBUART_MAXUNITS	4
static usbdev_t *usbuart_units[USBUART_MAXUNITS];
#endif


/*  *********************************************************************
    *  Structures
    ********************************************************************* */

typedef struct usbserial_softc_s {
    int user_inpipe;
    int user_outpipe;
    int user_outmps;
    int user_intpipe;
    int user_intmps;
    uint8_t user_inbuf[USER_FIFOSIZE];
    int user_inbuf_in;
    int user_inbuf_out;
    uint8_t *user_devinbuf;
    int user_devinbufsize;
    int user_unit;
    uint8_t *user_intbuf;
    usbser_linedata_t user_linedata;
    uint16_t dev_id;
    uint16_t vendor_id;
    enum { DEV_PL, DEV_PL_HX, DEV_MCT } dev_type; 
} usbserial_softc_t;

usb_driver_t usbserial_driver = {
    "USB Serial Port",
    usbserial_attach,
    usbserial_detach
};


#if 0
/*  *********************************************************************
    *  usbserial_get_linedata(dev,linedata)
    *  
    *  Request line data from the device (Prolific-style devices only).
    *  
    *  Input parameters: 
    *  	   dev - USB device
    *  	   linedata - pointer to structure
    *  	   
    *  Return value:
    *  	   # of bytes returned 
    *  	   <0 if error
    ********************************************************************* */

static int usbserial_get_linedata(usbdev_t *dev,usbser_linedata_t *ldata)
{
    uint8_t *respbuf;
    int res;

    respbuf =  usb_dma_alloc(32);
    res = usb_std_request(dev, PL_GET_REQ, PL_REQ_GET_LINE_CODING, 0, 0,
			  respbuf, sizeof(usbser_linedata_t));
    if ((res >= 0) && ldata) memcpy(ldata,respbuf,sizeof(usbser_linedata_t));
    usb_dma_free(respbuf);

    return res;
}
#endif

/*  *********************************************************************
    *  usbserial_set_linedata(dev,linedata)
    *  
    *  Set line data to the device (Prolific-style devices only).
    *  
    *  Input parameters: 
    *  	   dev - USB device
    *  	   linedata - pointer to structure
    *  	   
    *  Return value:
    *  	   # of bytes returned 
    *  	   <0 if error
    ********************************************************************* */

static int usbserial_set_linedata(usbdev_t *dev,usbser_linedata_t *ldata)
{
    /* Send request to device. */

    return usb_std_request(dev, PL_SET_REQ, PL_REQ_SET_LINE_CODING, 0, 0,
			   (uint8_t *)ldata, sizeof(usbser_linedata_t));
}

/*  *********************************************************************
    *  usbserial_tx_data(dev,buffer,len)
    *  
    *  Synchronously transmit data via the USB.
    *  
    *  Input parameters: 
    *  	   dev - device pointer
    *  	   buffer,len - data we want to send
    *  	   
    *  Return value:
    *  	   number of bytes sent.
    ********************************************************************* */

static int usbserial_tx_data(usbdev_t *dev,hsaddr_t buffer,int len)
{
    uint8_t *bptr;
    usbreq_t *ur;
    usbserial_softc_t *softc = dev->ud_private;
    int res;

    bptr = usb_dma_alloc(len);
    hs_memcpy_from_hs(bptr,buffer,len);

    ur = usb_make_request(dev,softc->user_outpipe,bptr,len,UR_FLAG_OUT);
    res = usb_sync_request(ur);
    
//  printf("Data sent, status=%d, xferred=%d\n",res,ur->ur_xferred);

    res = ur->ur_xferred;
    usb_free_request(ur);
    usb_dma_free(bptr);

    return res;
}

/*  *********************************************************************
    *  usbserial_int_callback(ur)
    *  
    *  Callback routine for the interrupt request, for devices
    *  that have an interrupt pipe.  We ignore this.
    *  
    *  Input parameters: 
    *  	   ur - usb request
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static int usbserial_int_callback(usbreq_t *ur)
{
    /*
     * Check to see if the request was cancelled by someone
     * deleting our endpoint.  We also check for 
     * "device not responding", which typically happens
     * when the device is removed.
     */

    if ((ur->ur_status == UR_ERR_CANCELLED) ||
	(ur->ur_status == UR_ERR_DEVICENOTRESPONDING)) {
	usb_free_request(ur);
	return 0;
	}

    /* Just requeue the request */
    usb_queue_request(ur);

    return 0;
}


/*  *********************************************************************
    *  usbserial_rx_callback(ur)
    *  
    *  Callback routine for the regular data pipe.
    *  
    *  Input parameters: 
    *  	   ur - usb request
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static int usbserial_rx_callback(usbreq_t *ur)
{
    int idx;
    int iptr;
    usbserial_softc_t *user = (ur->ur_dev->ud_private);

    /*
     * Check to see if the request was cancelled by someone
     * deleting our endpoint.  We also check for "device not
     * responding", which happens when a device is removed
     * while the request was pending.
     */

    if ((ur->ur_status == UR_ERR_CANCELLED) ||
	(ur->ur_status == UR_ERR_DEVICENOTRESPONDING)) {
	usb_free_request(ur);
	return 0;
	}

    /* Add characters to the receive fifo */
    for (idx = 0; idx < ur->ur_xferred; idx++) {
	iptr = (user->user_inbuf_in + 1) & (USER_FIFOSIZE-1);
	if (iptr == user->user_inbuf_out) break;	/* overflow */
	user->user_inbuf[user->user_inbuf_in] = ur->ur_buffer[idx];
	user->user_inbuf_in = iptr;
	}

    /* Requeue the request */
    usb_queue_request(ur);

    return 0;
}


/*  *********************************************************************
    *  usbserial_attach(dev,drv)
    *  
    *  This routine is called when the bus scan stuff finds a serial
    *  device.  We finish up the initialization by configuring the
    *  device and allocating our softc here.
    *  
    *  Input parameters: 
    *  	   dev - usb device, in the "addressed" state.
    *  	   drv - the driver table entry that matched
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */

static int usbserial_attach(usbdev_t *dev,usb_driver_t *drv)
{
    usb_device_descr_t *devdscr = &dev->ud_devdescr;
    usb_config_descr_t *cfgdscr = dev->ud_cfgdescr;
    usb_endpoint_descr_t *epdscr;
    usb_endpoint_descr_t *indscr = NULL;
    usb_endpoint_descr_t *outdscr = NULL;
    usb_endpoint_descr_t *intdscr = NULL;
    usb_interface_descr_t *ifdscr;
    usbserial_softc_t *softc;
    int idx;
    usbreq_t *ur;

    dev->ud_drv = drv;

    softc = KMALLOC(sizeof(usbserial_softc_t),0);
    memset(softc,0,sizeof(usbserial_softc_t));
    dev->ud_private = softc;

    softc->vendor_id = (devdscr->idVendorHigh << 8) + devdscr->idVendorLow;
    softc->dev_id = (devdscr->idProductHigh << 8) + devdscr->idProductLow;

    /* Make this table driven eventually. */
    if (softc->vendor_id == 0x050D && softc->dev_id == 0x0109) {
	/* This is the Belkin "USB PDA Adapter" (MCT-style). */
	softc->dev_type = DEV_MCT;
	}
    else {
	/* Other currently supported devices are based on variations of
           the Prolific PL-2303 chip, which implements a subset of the
           USB Communications class. */
	if (devdscr->bMaxPacketSize0 == 0x40) {
	    /* Linux thinks this distinguishes the HX variant. */
	    softc->dev_type = DEV_PL_HX;
	    }
	else
	    softc->dev_type = DEV_PL;
	}

    ifdscr = usb_find_cfg_descr(dev,USB_INTERFACE_DESCRIPTOR_TYPE,0);
    if (ifdscr == NULL) {
	printf("Could not get interface descriptor\n");
	return -1;
	}

    if (softc->dev_type == DEV_MCT) {
	/* In MCT-based devices, the bulk input endpoint descriptor is
           incorrectly tagged as an interrupt endpoint descriptor.
           For now, we assume known indices; an alternative approach
           would be to choose based on wMaxPacketSize. */
	intdscr = usb_find_cfg_descr(dev,USB_ENDPOINT_DESCRIPTOR_TYPE,0);
	indscr = usb_find_cfg_descr(dev,USB_ENDPOINT_DESCRIPTOR_TYPE,1);
	outdscr = usb_find_cfg_descr(dev,USB_ENDPOINT_DESCRIPTOR_TYPE,2);
	}
    else {
	for (idx = 0; idx < ifdscr->bNumEndpoints; idx++) {
	    epdscr = usb_find_cfg_descr(dev,USB_ENDPOINT_DESCRIPTOR_TYPE,idx);

	    if ((epdscr->bmAttributes & USB_ENDPOINT_TYPE_MASK) == 
		USB_ENDPOINT_TYPE_INTERRUPT) {
		intdscr = epdscr;
		}
	    else if (USB_ENDPOINT_DIR_OUT(epdscr->bEndpointAddress)) {
		outdscr = epdscr;
		}
	    else {
		indscr = epdscr;
		}
	    }
	}

    if (!indscr || !outdscr) {
	printf("IN or OUT endpoint descriptors are missing\n");
	/*
	 * Could not get descriptors, something is very wrong.
	 * Leave device addressed but not configured.
	 */
	return 0;
	}

    /* Choose the standard configuration. */
    usb_set_configuration(dev,cfgdscr->bConfigurationValue);

    /* Open the pipes. */

    softc->user_inpipe     = usb_open_pipe(dev,indscr);
    softc->user_devinbufsize = GETUSBFIELD(indscr,wMaxPacketSize);
    softc->user_outpipe    = usb_open_pipe(dev,outdscr);
    softc->user_outmps = GETUSBFIELD(outdscr,wMaxPacketSize);
    if (intdscr) {
	softc->user_intpipe     = usb_open_pipe(dev,intdscr);
	softc->user_intmps = GETUSBFIELD(intdscr,wMaxPacketSize);
	}
    else {
	softc->user_intpipe = -1;
	softc->user_intmps = 0;
	}

#ifdef _CFE_
    softc->user_unit = -1;
    for (idx = 0; idx < USBUART_MAXUNITS; idx++) {
	if (usbuart_units[idx] == NULL) {
	    softc->user_unit = idx;
	    usbuart_units[idx] = dev;
	    break;
	    }
	}

    console_log("USBSERIAL: Unit %d connected",softc->user_unit);
    console_log("  vendor %04x, product %04x", softc->vendor_id, softc->dev_id);
#endif

    /* UART status and control operations are not standardized. */
    if (softc->dev_type == DEV_MCT) {
        usbser_bauddata_t *bdata;

	bdata = usb_dma_alloc(sizeof(usbser_bauddata_t));
	PUTDWFIELD(bdata,dBaudData,0xC);    /* encoding of 115200 */
	usb_std_request(dev, MCT_SET_REQ, MCT_REQ_SET_BAUD,
			0, ifdscr->bInterfaceNumber,
			(uint8_t *)bdata, sizeof(usbser_bauddata_t));
	usb_dma_free(bdata);
	}
    else {
        usbser_linedata_t *ldata;
	
	ldata = usb_dma_alloc(sizeof(usbser_linedata_t));
	PUTDWFIELD(ldata,dLineDataBaud,115200);
	ldata->bLineDataStopBits = 0;
	ldata->bLineDataParity = 0;      /* none */
	ldata->bLineDataBits = 8;

	softc->user_linedata = *ldata;   /* XXX needed? */

	usbserial_set_linedata(dev,ldata);
	usb_dma_free(ldata);
//	usbserial_get_linedata(dev,NULL);

	/* The following code follows NetBSD in providing some magic
           "Undocumented (vendor unresponsive)" setup for the HX
           variant of the Prolific part. */
	if (softc->dev_type == DEV_PL_HX) {
	    usb_simple_request(dev, 0x40, 0x1, 2, 0x44);
	    usb_simple_request(dev, 0x40, 0x1, 8, 0);
	    usb_simple_request(dev, 0x40, 0x1, 9, 0);
	    }
	}


    softc->user_devinbuf = usb_dma_alloc(softc->user_devinbufsize);
    ur = usb_make_request(dev,softc->user_inpipe,softc->user_devinbuf,
			  softc->user_devinbufsize,
			  UR_FLAG_IN | UR_FLAG_SHORTOK);
    ur->ur_callback = usbserial_rx_callback;
    usb_queue_request(ur);

    if (softc->user_intpipe != -1) {
	softc->user_intbuf = usb_dma_alloc(softc->user_intmps);
	ur = usb_make_request(dev,softc->user_intpipe,softc->user_intbuf,
			      softc->user_intmps,
			      UR_FLAG_IN | UR_FLAG_SHORTOK);
	ur->ur_callback = usbserial_int_callback;	
	usb_queue_request(ur);
	}

    return 0;
}

/*  *********************************************************************
    *  usbserial_detach(dev)
    *  
    *  This routine is called when the bus scanner notices that
    *  this device has been removed from the system.  We should
    *  do any cleanup that is required.  The pending requests
    *  will be cancelled automagically.
    *  
    *  Input parameters: 
    *  	   dev - usb device
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */

static int usbserial_detach(usbdev_t *dev)
{
    usbserial_softc_t *softc;

    softc = dev->ud_private;

#ifdef _CFE_
    console_log("USBSERIAL: USB unit %d disconnected",softc->user_unit);
    if (softc->user_unit >= 0) usbuart_units[softc->user_unit] = NULL;
#endif

    if (softc) {
	if (softc->user_devinbuf) usb_dma_free(softc->user_devinbuf);
	if (softc->user_intbuf) usb_dma_free(softc->user_intbuf);
	dev->ud_private = NULL;
	KFREE(softc);
	}

    return 0;
}


#ifdef _CFE_

static void usb_uart_probe(cfe_driver_t *drv,
			   unsigned long probe_a, unsigned long probe_b, 
			   void *probe_ptr)
{
    usb_uart_t *softc;
    char descr[80];

    softc = (usb_uart_t *) KMALLOC(sizeof(usb_uart_t),0);
    memset(softc,0,sizeof(usb_uart_t));
    softc->uart_unit = (int)probe_a;

    xsprintf(descr,"USB UART unit %d",(int)probe_a);
    cfe_attach(drv,softc,NULL,descr);
}


static int usb_uart_open(cfe_devctx_t *ctx)
{
    return 0;
}

static int usb_uart_read(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    usb_uart_t *softc = ctx->dev_softc;
    usbdev_t *dev = usbuart_units[softc->uart_unit];
    usbserial_softc_t *user;
    hsaddr_t bptr;
    int blen;

    if (!dev) {
	buffer->buf_retlen = 0;
	return 0;
	}
    user = dev->ud_private;

    bptr = buffer->buf_ptr;
    blen = buffer->buf_length;

    while ((blen > 0) && (user->user_inbuf_out != user->user_inbuf_in)) {
	hs_write8(bptr,user->user_inbuf[user->user_inbuf_out]);
	bptr++;
	user->user_inbuf_out = (user->user_inbuf_out + 1) & (USER_FIFOSIZE-1);
	blen--;
	}

    buffer->buf_retlen = buffer->buf_length - blen;
    return 0;
}

static int usb_uart_inpstat(cfe_devctx_t *ctx, iocb_inpstat_t *inpstat)
{
    usb_uart_t *softc = ctx->dev_softc;
    usbdev_t *dev = usbuart_units[softc->uart_unit];
    usbserial_softc_t *user;

    inpstat->inp_status = 0;

    if (!dev) return 0;
    user = dev->ud_private;

    inpstat->inp_status = (user->user_inbuf_in != user->user_inbuf_out);

    return 0;
}

static int usb_uart_write(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    usb_uart_t *softc = ctx->dev_softc;
    hsaddr_t bptr;
    int blen;
    usbdev_t *dev = usbuart_units[softc->uart_unit];
    usbserial_softc_t *user;

    bptr = buffer->buf_ptr;
    blen = buffer->buf_length;

    if (!dev) {
	buffer->buf_retlen = blen;
	return 0;
	}
    user = dev->ud_private;

    if (blen > user->user_outmps) blen = user->user_outmps;

    usbserial_tx_data(dev,bptr,blen);

    buffer->buf_retlen = blen;
    return 0;
}

static int usb_uart_ioctl(cfe_devctx_t *ctx, iocb_buffer_t *buffer) 
{
    usb_uart_t *softc = ctx->dev_softc;
    usbdev_t *dev = usbuart_units[softc->uart_unit];
    unsigned int info;

    if (!dev) return -1;

    switch ((int)buffer->buf_ioctlcmd) {
	case IOCTL_SERIAL_GETSPEED:
	    info = softc->uart_speed;
	    hs_memcpy_to_hs(buffer->buf_ptr,&info,sizeof(info));
	    break;
	case IOCTL_SERIAL_SETSPEED:
	    hs_memcpy_from_hs(&info,buffer->buf_ptr,sizeof(info));
	    softc->uart_speed = info;
	    /* NYI */
	    break;
	case IOCTL_SERIAL_GETFLOW:
	    info = softc->uart_flowcontrol;
	    hs_memcpy_to_hs(buffer->buf_ptr,&info,sizeof(info));
	    break;
	case IOCTL_SERIAL_SETFLOW:
	    hs_memcpy_from_hs(&info,buffer->buf_ptr,sizeof(info));
	    softc->uart_flowcontrol = info;
	    /* NYI */
	    break;
	default:
	    return -1;
	}

    return 0;
}

static int usb_uart_close(cfe_devctx_t *ctx)
{
    return 0;
}
#endif

