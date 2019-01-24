/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  USB Ethernet				File: dev_usb_rtek.c
    *  
    *  Driver for USB Ethernet devices using Realtek RTL8150 chip.
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

enum { VEN_NONE, LINKSYS_100M };
static const char *VENDOR_NAMES[] = {
    "?", "Linksys-100M", "Yikes!"
};

static const int ID_TBL[] = {
    0x0BDA, 0x8150, LINKSYS_100M,
    -1
};

typedef struct rtek_softc_s {
    usbdev_t *dev;
    int bulk_inpipe;
    int bulk_outpipe;
    int ven_code;
    uint8_t mac_addr[6];
    usbreq_t *rx_ur;
    uint8_t rxbuf[1600];    /* arbitrary but enough for ethernet packet */
} rtek_softc_t;


/* **************************************
   * REALTEK RTL8150 I/F Functions.
   ************************************** */

static int rtek_send_eth_frame( void *ctx, hsaddr_t buf, int len );
static int rtek_get_eth_frame( void *ctx, hsaddr_t buf );
static int rtek_data_rx( void *ctx );
static int rtek_get_dev_addr( void *ctx, hsaddr_t mac_addr );

static usbeth_disp_t usbeth_rtek = {
    rtek_get_eth_frame,
    rtek_data_rx,
    rtek_send_eth_frame,
    rtek_get_dev_addr
};


static int rtek_get_reg( usbdev_t *dev, int16_t reg, uint8_t *val, int16_t len )
{
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_IN),
			    RTEK_REG_ACCESS, reg, 0, val, len );
}

static int rtek_set_reg( usbdev_t *dev, int16_t reg, int8_t val )
{
    uint8_t data[1];

    data[0] = val;
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
			    RTEK_REG_ACCESS, reg, 0, data, 1 );
}

static int rtek_get_mac_addr( usbdev_t *dev, uint8_t *mac_addr )
{
    return rtek_get_reg( dev, R_RTEK_MAC, mac_addr, 6 );
}

static int rtek_init_device( rtek_softc_t *softc )
{
    usb_device_descr_t dev_desc;
    uint16_t vendor_id, product_id;
    const int *ptr=ID_TBL;
    int i;
    usbdev_t *dev = softc->dev;
    uint8_t val;

    /* find out which device is connected */
    usb_get_device_descriptor( softc->dev, &dev_desc, 0 );
    vendor_id = (dev_desc.idVendorHigh  << 8) + dev_desc.idVendorLow;
    product_id = (dev_desc.idProductHigh << 8) + dev_desc.idProductLow;

    while( *ptr != -1 )	{
	if( (vendor_id == ptr[0]) && (product_id == ptr[1]) ) {
	    softc->ven_code = ptr[2];
	    break;
	    }
	ptr += 3;
	}
    if( *ptr == -1 ) {
	xprintf( "Unrecognized Realtek USB-Ethernet device\n" );
	return -1;
	}

    /* Reset the adapter */
    rtek_set_reg( dev, R_RTEK_CMD, RTEK_RESET );
    for( i = 0; i < 10; ++i ) {
	rtek_get_reg( dev, R_RTEK_CMD, &val, 1 );
	if( !(val & RTEK_RESET) )
	    break;
	usb_delay_ms( NULL, 1 );
	}

    /* Autoload the internal registers */
    rtek_set_reg( dev, R_RTEK_CMD, RTEK_AUTOLOAD );
    for( i = 0; i < 50; ++i ) {
	rtek_get_reg( dev, R_RTEK_CMD, &val, 1 );
	if( !(val & RTEK_AUTOLOAD) )
	    break;
	usb_delay_ms( NULL, 1 );
	}

    /* Read the adapter's MAC addr */
    rtek_get_mac_addr( dev, softc->mac_addr );

    /* display adapter info */
    xprintf( "%s USB-Ethernet Adapter (%a)\n",
	     VENDOR_NAMES[softc->ven_code], softc->mac_addr);

    return 0;
}

static int rtek_get_dev_addr( void *ctx, hsaddr_t mac_addr )
{
    rtek_softc_t *softc = (rtek_softc_t *) ctx;
    hs_memcpy_to_hs( mac_addr, softc->mac_addr, 6 );
    return 0;
}

static void rtek_queue_rx( rtek_softc_t *softc )
{
    softc->rx_ur = usb_make_request(softc->dev, softc->bulk_inpipe,
				    softc->rxbuf, sizeof(softc->rxbuf),
				    (UR_FLAG_IN | UR_FLAG_SHORTOK));
    usb_queue_request(softc->rx_ur);
}

static int rtek_data_rx( void *ctx )
{
    rtek_softc_t *softc = (rtek_softc_t *) ctx;
    usb_poll(softc->dev->ud_bus);
    return( !softc->rx_ur->ur_inprogress );
}

static int rtek_get_eth_frame( void *ctx, hsaddr_t buf )
{
    int len = 0;
    rtek_softc_t *softc = (rtek_softc_t *) ctx;
    uint8_t *rxbuf;

    if( !softc->rx_ur->ur_inprogress ) {
	rxbuf = softc->rxbuf;
	len = softc->rx_ur->ur_xferred;
	if (len > 0) {
#if USBETH_DEBUG
	    xprintf( "Incoming packet :\n" );
	    hexdump( rxbuf, len, 16, len / 16 + 1 );
#endif
	    hs_memcpy_to_hs( buf, rxbuf, len );
	    }
	usb_free_request(softc->rx_ur);
	rtek_queue_rx( softc );
	}
    else
	xprintf( "Bulk data is not available yet!\n" );

    return( len );
}

static int rtek_send_eth_frame( void *ctx, hsaddr_t buf, int len )
{
    rtek_softc_t *softc = (rtek_softc_t *) ctx;
    usbreq_t *ur;
    int txlen = len;
    unsigned char *txbuf;

    /* First some Realtek chip workarounds */
    if( txlen < 60 )		/* some strange limitation */
	txlen = 60;
    else if( !(txlen % 64) )	/* to handle module 64 packets */
	++txlen;

    txbuf = usb_dma_alloc(txlen);
    hs_memcpy_from_hs( txbuf, buf, txlen );

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

static void rtek_open_device( rtek_softc_t *softc )
{
    /* Accept broadcast & own packets */
    rtek_set_reg( softc->dev, R_RTEK_RXCFG, RTEK_MACADDR | RTEK_BCASTADDR );

    /* Enable adapter to receive and transmit packets */
    rtek_set_reg( softc->dev, R_RTEK_CMD, RTEK_RXENABLE | RTEK_TXENABLE );

    /* kick start the receive */
    rtek_queue_rx( softc );
}


static void rtek_close_device( rtek_softc_t *softc )
{
    usbdev_t *dev = softc->dev;

    /* Disable adapter from receiving or transmitting packets */
    rtek_set_reg( dev, R_RTEK_CMD, 0 );
}


/*  *********************************************************************
    * CFE-USB interfaces
    ********************************************************************* */

/*  *********************************************************************
    *  rtek_attach(dev,drv)
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

const cfe_driver_t usbrtekdrv;		/* forward declaration */

static int rtek_attach(usbdev_t *dev, usb_driver_t *drv)
{
    usb_config_descr_t *cfgdscr = dev->ud_cfgdescr;
    usb_endpoint_descr_t *epdscr;
    usb_endpoint_descr_t *indscr = NULL;
    usb_endpoint_descr_t *outdscr = NULL;
    usb_interface_descr_t *ifdscr;
    rtek_softc_t *softc;
    int idx;

    dev->ud_drv = drv;

    softc = (rtek_softc_t *) KMALLOC( sizeof(rtek_softc_t), 0 );
    if( softc == NULL )	{
	xprintf( "Failed to allocate softc memory.\n" );
	return -1;
	}
    memset( softc, 0, sizeof(rtek_softc_t) );
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
    if (rtek_init_device(softc) < 0)
	return -1;

    /* Open the pipes. */
    softc->bulk_inpipe     = usb_open_pipe(dev,indscr);
    softc->bulk_outpipe    = usb_open_pipe(dev,outdscr);

    /* Register the device */
    usbeth_register(&usbeth_rtek,softc);

    /* Open the device */
    rtek_open_device( softc );

    return 0;
}

/*  *********************************************************************
    *  rtek_detach(dev)
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

static int rtek_detach(usbdev_t *dev)
{
    rtek_softc_t *softc = (rtek_softc_t *) dev->ud_private;

    if (softc != NULL) {
	usbeth_unregister( softc );
	rtek_close_device( softc );
	dev->ud_private = NULL;
	softc->dev = NULL;
	KFREE(softc);
	}

    return 0;
}

/*  CFE USB device interface structure */
usb_driver_t usbrtek_driver =
{
    "Ethernet Device",
    rtek_attach,
    rtek_detach
};




