/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  USB Ethernet				File: dev_usb_klsi.c
    *  
    *  Driver for USB Ethernet devices using Kawasaki KL5KUSB101B chip.
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
    * Kawasaki KL5KUSB101B USB-Ethernet driver - CFE Network Layer Interfaces
    ********************************************************************* */

#include "cfe.h"

#include "usbd.h"
#include "usbeth.h"
#include "klsi_fw.h"

#if 0
#define USBETH_TRACE( x, y ... ) xprintf( x, ##y )
#else
#define USBETH_TRACE( x, y ... ) ((void)0)
#endif

#define FAIL				-1

#define CACHE_ALIGN    32       /* XXX place holder, big enough to now. */
#define ALIGN(n,align) (((n)+((align)-1)) & ~((align)-1))

#define usb_dma_alloc(n) (KMALLOC(ALIGN((n),CACHE_ALIGN),CACHE_ALIGN))
#define usb_dma_free(p)  (KFREE(p))

/******************************************************************************
  Debug functions
******************************************************************************/

#ifndef USBETH_DEBUG
#define USBETH_DEBUG 0
#endif

#if USBETH_DEBUG
static void hexdump( unsigned char *src, int srclen, int rowlen, int rows )
{
    unsigned char *rowptr;
    unsigned char *srcstp;
    unsigned char *byteptr;

    srcstp = src + srclen;

    for( rowptr = src; rowptr < src + rowlen * rows; rowptr += rowlen ) {
	for( byteptr = rowptr; byteptr < rowptr + rowlen && byteptr < srcstp; byteptr++ ) {
	    xprintf( "%2X ", *byteptr );
	    }
	xprintf( "\n" );
	}
    xprintf( "\n" );
}
#else
#define hexdump(src,srclen,rowlen,rows) ((void)0)
#endif


/*  *********************************************************************
    * Interface functions for USB-Ethernet adapters
    ********************************************************************* */

enum { VEN_NONE, ADS  };
static const char *VENDOR_NAMES[] = {
    "?", "ADS", "Yikes!"
};

static const int ID_TBL[] = {
    0x06E1, 0x0008, ADS,
    -1
};

typedef struct klsi_softc_s {
    usbdev_t *dev;
    int bulk_inpipe;
    int bulk_outpipe;
    int ven_code;
    uint8_t mac_addr[6];
    usbreq_t *rx_ur;
    uint8_t rxbuf[1600];    /* arbitrary but enough for ethernet packet */
} klsi_softc_t;


/* **************************************
   * KAWASAKI LSI KL5KUSB101B I/F Functions.
   ************************************** */

static int klsi_send_eth_frame( void *ctx, hsaddr_t buf, int len );
static int klsi_get_eth_frame( void *ctx, hsaddr_t buf );
static int klsi_data_rx( void *ctx );
static int klsi_get_dev_addr( void *ctx, hsaddr_t mac_addr );

static usbeth_disp_t usbeth_klsi = {
    klsi_get_eth_frame,
    klsi_data_rx,
    klsi_send_eth_frame,
    klsi_get_dev_addr
};


static int klsi_load( klsi_softc_t *softc )
{
    usbdev_t *dev = softc->dev;

    usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
		     KLSI_SEND_SCAN, 0, 0,
		     (uint8_t *)kue_code_seg, sizeof(kue_code_seg) );
    usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
		     KLSI_SEND_SCAN, 0, 0,
		     (uint8_t *)kue_fix_seg, sizeof(kue_fix_seg) );
    usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
		     KLSI_SEND_SCAN, 0, 0,
		     (uint8_t *)kue_trig_seg, sizeof(kue_trig_seg) );

    cfe_sleep(CFE_HZ/10);

    return 0;
}

static int klsi_init_device( klsi_softc_t *softc )
{
    usb_device_descr_t dev_desc;
    uint16_t vendor_id, device_id;
    usbdev_t *dev = softc->dev;
    uint16_t bcd_dev;
    klsi_ether_desc_t *desc;
    const int *ptr=ID_TBL;

    /* find out which device is connected */
    usb_get_device_descriptor( softc->dev, &dev_desc, 0 );
    vendor_id = (dev_desc.idVendorHigh  << 8) + dev_desc.idVendorLow;
    device_id = (dev_desc.idProductHigh << 8) + dev_desc.idProductLow;

    while( *ptr != -1 )	{
	if( (vendor_id == ptr[0]) && (device_id == ptr[1]) ) {
	    softc->ven_code = ptr[2];
	    break;
	    }
	ptr += 3;
	}
    if( *ptr == -1 ) {
	xprintf( "Unknown Kawasaki USB-Ethernet device\n" );
	return -1;
	}

    /* load the on-chip firmware, if necessary */
    bcd_dev = ((dev->ud_devdescr.bcdDeviceHigh << 8) +
	       dev->ud_devdescr.bcdDeviceLow);
    if ((bcd_dev & 0x0200) == 0) {
	klsi_load(softc);
	/* Should reload device descriptor here. */
	usb_set_configuration(dev, dev->ud_cfgdescr->bConfigurationValue);
	cfe_sleep(CFE_HZ/10);
	}
    desc = usb_dma_alloc(sizeof(klsi_ether_desc_t));
    usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_IN),
		     KLSI_GET_ETH_DESC, 0, 0,
		     (uint8_t *)desc, sizeof(klsi_ether_desc_t));

    if (USBETH_DEBUG) {
	int i;
	printf("klsi [dev %04x] ether descriptor\n", bcd_dev);
	for (i = 0; i < sizeof(klsi_ether_desc_t); i++) {
	    printf(" %02x", ((uint8_t *)desc)[i]);
	    }
	printf("\n");
	}

    memcpy( softc->mac_addr, desc->klsi_macaddr, 6);
    usb_dma_free(desc);

    usb_simple_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
			KLSI_SET_URB_SIZE, 64, 0);

    /* display adapter info */
    xprintf( "%s USB-Ethernet Adapter (%a)\n",
	     VENDOR_NAMES[softc->ven_code], softc->mac_addr);

    return 0;
}

static int klsi_get_dev_addr( void *ctx, hsaddr_t mac_addr )
{
    klsi_softc_t *softc = (klsi_softc_t *) ctx;
    hs_memcpy_to_hs( mac_addr, softc->mac_addr, 6 );
    return 0;
}

static void klsi_queue_rx( klsi_softc_t *softc )
{
    softc->rx_ur = usb_make_request(softc->dev, softc->bulk_inpipe,
				    softc->rxbuf, sizeof(softc->rxbuf),
				    (UR_FLAG_IN | UR_FLAG_SHORTOK));
    usb_queue_request(softc->rx_ur);
}

static int klsi_data_rx( void *ctx )
{
    klsi_softc_t *softc = (klsi_softc_t *) ctx;
    usb_poll(softc->dev->ud_bus);
    return( !softc->rx_ur->ur_inprogress );
}

static int klsi_get_eth_frame( void *ctx, hsaddr_t buf )
{
    int len = 0;
    klsi_softc_t *softc = (klsi_softc_t *) ctx;
    uint8_t *rxbuf;

    if( !softc->rx_ur->ur_inprogress ) {
	rxbuf = softc->rxbuf;
	len = softc->rx_ur->ur_xferred;

	/* The klsi chip evidently completes the request with a
	   single zero byte when there is no packet available. */
	if (len >= 2)
	    len = rxbuf[0] + (rxbuf[1] << 8);
	else
	    len = 0;
	rxbuf += 2;

	if (len > 0) {
#if USBETH_DEBUG
	    xprintf( "Incoming packet :\n" );
	    hexdump( rxbuf, len, 16, len / 16 + 1 );
#endif
	    hs_memcpy_to_hs( buf, rxbuf, len );
	    }
	usb_free_request(softc->rx_ur);
	klsi_queue_rx( softc );
	}
    else
	xprintf( "Bulk data is not available yet!\n" );

    return( len );
}

static int klsi_send_eth_frame( void *ctx, hsaddr_t buf, int len )
{
    klsi_softc_t *softc = (klsi_softc_t *) ctx;
    usbreq_t *ur;
    int txlen = len;
    unsigned char *txbuf;

    txbuf = usb_dma_alloc(len+2);
    txbuf[0] = txlen & 0xFF;
    txbuf[1] = (txlen >> 8) & 0xFF;   /* 1st two bytes...little endian */
    hs_memcpy_from_hs( &txbuf[2], buf, txlen );
    txlen += 2;

    /* The klsi chip evidently needs at least one extra byte and a
       length that is a multiple of 64. */
    txlen += 64 - (txlen % 64);

#if USBETH_DEBUG
    xprintf( "Outgoing packet :\n" );
    hexdump( txbuf, txlen, 16, txlen / 16 + 1 );
#endif
    ur = usb_make_request(softc->dev, softc->bulk_outpipe,
	                      txbuf, txlen, UR_FLAG_OUT);
    usb_sync_request(ur);
    usb_free_request(ur);
    usb_dma_free(txbuf);

    return( len );
}

static void klsi_open_device( klsi_softc_t *softc )
{
    /* Accept broadcast and own packets */
    usb_simple_request( softc->dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
			KLSI_SET_PKT_FILTER,
			KLSI_RX_UNICAST | KLSI_RX_BROADCAST, 0);

    /* kick start the receive */
    klsi_queue_rx( softc );
}

static void klsi_close_device( klsi_softc_t *softc )
{
    usbdev_t *dev = softc->dev;

    usb_simple_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
			KLSI_SET_PKT_FILTER, 0, 0);
}


/*  *********************************************************************
    * CFE-USB interfaces
    ********************************************************************* */

/*  *********************************************************************
    *  klsi_attach(dev,drv)
    *
    *  This routine is called when the bus scan stuff finds a usb-ethernet
    *  device.  We finish up the initialization by configuring the
    *  device and allocating our softc here.
    *
    *  Input parameters:
    *      dev - usb device, in the "addressed" state.
    *      drv - the driver table entry that matched
    *
    *  Return value:
    *      0
    ********************************************************************* */

const cfe_driver_t usbklsidrv;		/* forward declaration */

static int klsi_attach(usbdev_t *dev, usb_driver_t *drv)
{
    usb_config_descr_t *cfgdscr = dev->ud_cfgdescr;
    usb_endpoint_descr_t *epdscr;
    usb_endpoint_descr_t *indscr = NULL;
    usb_endpoint_descr_t *outdscr = NULL;
    usb_interface_descr_t *ifdscr;
    klsi_softc_t *softc;
    int idx;

    dev->ud_drv = drv;

    softc = (klsi_softc_t *) KMALLOC( sizeof(klsi_softc_t), 0 );
    if( softc == NULL )	{
	xprintf( "Failed to allocate softc memory.\n" );
	return -1;
	}
    memset( softc, 0, sizeof(klsi_softc_t) );
    dev->ud_private = softc;
    softc->dev = dev;

    ifdscr = usb_find_cfg_descr(dev,USB_INTERFACE_DESCRIPTOR_TYPE,0);
    if (ifdscr == NULL) {
	xprintf("USBETH: ERROR...no interace descriptor\n");
	return -1;
	}

    for (idx = 0; idx < 2; idx++) {
	epdscr = usb_find_cfg_descr(dev,USB_ENDPOINT_DESCRIPTOR_TYPE,idx);
	if (USB_ENDPOINT_DIR_OUT(epdscr->bEndpointAddress))
	    outdscr = epdscr;
	else
	    indscr = epdscr;
	}

    if (!indscr || !outdscr) {
	/*
	 * Could not get descriptors, something is very wrong.
	 * Leave device addressed but not configured.
	 */
	xprintf("USBETH: ERROR...no endpoint descriptors\n");
	return -1;
	}

    /* Choose the standard configuration. */
    usb_set_configuration(dev,cfgdscr->bConfigurationValue);

    /* Quit if not able to initialize the device */
    if (klsi_init_device(softc) < 0)
	return -1;

    /* Open the pipes. */
    softc->bulk_inpipe     = usb_open_pipe(dev,indscr);
    softc->bulk_outpipe    = usb_open_pipe(dev,outdscr);

    /* Register the device */
    usbeth_register(&usbeth_klsi,softc);

    /* Open the device */
    klsi_open_device( softc );

    return 0;
}

/*  *********************************************************************
    *  klsi_detach(dev)
    *
    *  This routine is called when the bus scanner notices that
    *  this device has been removed from the system.  We should
    *  do any cleanup that is required.  The pending requests
    *  will be cancelled automagically.
    *
    *  Input parameters:
    *      dev - usb device
    *
    *  Return value:
    *      0
    ********************************************************************* */

static int klsi_detach(usbdev_t *dev)
{
    klsi_softc_t *softc = (klsi_softc_t *) dev->ud_private;

    if (softc != NULL) {
	usbeth_unregister( softc );
	klsi_close_device ( softc );
	dev->ud_private = NULL;
	softc->dev = NULL;
	KFREE(softc);
	}

    return 0;
}

/*  CFE USB device interface structure */
usb_driver_t usbklsi_driver =
{
    "Ethernet Device",
    klsi_attach,
    klsi_detach
};



