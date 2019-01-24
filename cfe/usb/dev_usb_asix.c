/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  USB Ethernet				File: dev_usb_asix.c
    *
    *  Driver for USB Ethernet devices using the ASIX AX8817 chip.
    *
    *********************************************************************
    *
    *  Copyright 2000,2001,2005
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

/*  *********************************************************************
    *  USB-Ethernet driver - CFE Network Layer Interfaces
    ********************************************************************* */

#include "cfe.h"

#include "usbd.h"
#include "usbeth.h"

/* XXX Move to usbeth.h */
/* **************************************
   *  ASIX AX8817 adapter
   ************************************** */

#define ASIX_MII_SWOP_CMD	0x06
#define ASIX_MII_READ_CMD	0x07
#define ASIX_MII_WRITE_CMD	0x08
#define ASIX_MII_HWOP_CMD	0x0a
#define ASIX_RXCTL_CMD		0x10
#define ASIX_IPG1_CMD		0x12
#define ASIX_IPG2_CMD		0x13
#define ASIX_IPG3_CMD		0x14
#define ASIX_MAC_ADDR_CMD	0x17
#define ASIX_PHYID_CMD		0x19
#define ASIX_MED_WRITE_CMD	0x1b
#define ASIX_GPIO_WRITE_CMD	0x1f


#if 0
#define USBETH_TRACE( x, y ... ) xprintf( x, ##y )
#else
#define USBETH_TRACE( x, y ... )
#endif

#define FAIL				-1

#define USB_MALLOC_VALUE 32


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

enum { VEN_NONE, HAWKING, NETGEAR };
static char *VENDOR_NAMES[] = {
    "?", "Hawking", "Netgear", "Yikes!"
};

static const int ID_TBL[] = {
    0x0846, 0x1040, NETGEAR,        /* Netgear FA120 */
    0x07b8, 0x420a, HAWKING,        /* Hawking UF200 */
    -1
};

typedef struct asix_softc_s
{
    usbdev_t *dev;
    int bulk_inpipe;
    int bulk_outpipe;
    int dev_id;
    int ven_code;
    uint8_t mac_addr[6];
    usbreq_t *rx_ur;
    uint8_t rxbuf[1600];	/* artbitrary but enough for ethernet packet */
} asix_softc_t;


/* **************************************
   *  ASIX AX8817x I/F Functions
   ************************************** */

static int asix_send_eth_frame( void *ctx, hsaddr_t buf, int len );
static int asix_get_eth_frame( void *ctx, hsaddr_t buf );
static int asix_data_rx( void *ctx );
static int asix_get_dev_addr( void *ctx, hsaddr_t mac_addr );

static usbeth_disp_t usbeth_asix = {
    asix_get_eth_frame,
    asix_data_rx,
    asix_send_eth_frame,
    asix_get_dev_addr
};


static int asix_get_reg( usbdev_t *dev, uint8_t cmd, int16_t val, int16_t index, uint8_t *buf, int16_t len )
{
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_IN),
			    cmd, val, index, buf, len );
}

static int asix_set_reg( usbdev_t *dev, uint8_t cmd, int16_t val, int16_t index, uint8_t *buf, int16_t len )
{
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
			    cmd, val, index, buf, len );
}

static int asix_get_mac_addr( usbdev_t *dev, uint8_t *mac_addr )
{
    return asix_get_reg( dev, ASIX_MAC_ADDR_CMD, 0, 0, mac_addr, 6 );
}

static int asix_init_device( asix_softc_t *softc )
{
    usb_device_descr_t dev_desc;
    uint16_t vendor_id, device_id;
    const int *ptr=ID_TBL;

    /* find out which device is connected */
    usb_get_device_descriptor( softc->dev, &dev_desc, 0 );
    vendor_id = (dev_desc.idVendorHigh  << 8) + dev_desc.idVendorLow;
    device_id = (dev_desc.idProductHigh << 8) + dev_desc.idProductLow;
    xprintf( "USB device:  vendor id %04x, device id %04x\n",
	     vendor_id, device_id );

    while( *ptr != -1 ) {
	if( (vendor_id == ptr[0]) && (device_id == ptr[1]) ) {
	    softc->dev_id = ptr[2];
	    softc->ven_code = ptr[3];
	    break;
	    }
	ptr += 4;
	}
    if( *ptr == -1 ) {
	xprintf( "Unrecognized USB-Ethernet device\n" );
	return -1;
	}

    /* Read the adapter's MAC addr */
    asix_get_mac_addr( softc->dev, softc->mac_addr );

    /* display adapter info */
    xprintf( "%s USB-Ethernet Adapter (%a)\n",
	     VENDOR_NAMES[softc->ven_code], softc->mac_addr);

    return 0;
}

static int asix_get_dev_addr( void *ctx, hsaddr_t mac_addr )
{
    asix_softc_t *softc = (asix_softc_t *) ctx;

    hs_memcpy_to_hs( mac_addr, softc->mac_addr, 6 );
    return 0;
}

static void asix_queue_rx( asix_softc_t *softc )
{
    softc->rx_ur = usb_make_request(softc->dev, softc->bulk_inpipe,
				    softc->rxbuf, sizeof(softc->rxbuf),
				    (UR_FLAG_IN | UR_FLAG_SHORTOK));
    usb_queue_request(softc->rx_ur);
}

static int asix_data_rx( void *ctx )
{
    asix_softc_t *softc = (asix_softc_t *) ctx;

    usb_poll(softc->dev->ud_bus);
    return( !softc->rx_ur->ur_inprogress );
}

static int asix_get_eth_frame( void *ctx, hsaddr_t buf )
{
    asix_softc_t *softc = (asix_softc_t *) ctx;
    int len = 0;

    if( !softc->rx_ur->ur_inprogress ) {
	len = softc->rx_ur->ur_xferred;
	hs_memcpy_to_hs( buf, softc->rxbuf, len );
	usb_free_request(softc->rx_ur);
	asix_queue_rx( softc );
	}
    else
	xprintf( "Bulk data is not available yet!\n" );

    return( len );
}

static int asix_send_eth_frame( void *ctx, hsaddr_t buf, int len )
{
    asix_softc_t *softc = (asix_softc_t *) ctx;
    usbreq_t *ur;
    int txlen = len;
    unsigned char *txbuf;

    txbuf = KMALLOC(txlen, USB_MALLOC_VALUE);
    hs_memcpy_from_hs( txbuf, buf, txlen );
    ur = usb_make_request(softc->dev, softc->bulk_outpipe,
			  txbuf, txlen, UR_FLAG_OUT);
    usb_sync_request(ur);
    usb_free_request(ur);
    KFREE(txbuf);

    return( len );
}

static void asix_open_device( asix_softc_t *softc )
{
    usbdev_t *dev = softc->dev;
    uint8_t data[2];
    int16_t phyid;

    asix_set_reg( dev, ASIX_MII_SWOP_CMD, 0, 0, NULL, 0 );
    asix_get_reg( dev, ASIX_PHYID_CMD, 0, 0, data, 2 );

    /* UF200 seems to need a GPIO settings */
    asix_set_reg( dev, ASIX_GPIO_WRITE_CMD, 0x11, 0, NULL, 0 );
    asix_set_reg( dev, ASIX_GPIO_WRITE_CMD, 0x13, 0, NULL, 0 );
    asix_set_reg( dev, ASIX_GPIO_WRITE_CMD, 0x0c, 0, NULL, 0 );

    phyid = data[1];
    data[0] = 0; data[1] = 0;
    asix_set_reg( dev, ASIX_MII_WRITE_CMD, phyid, 0, data, 2 );
    data[1] = 0x80;
    asix_set_reg( dev, ASIX_MII_WRITE_CMD, phyid, 0, data, 2 );
    asix_set_reg( dev, ASIX_MED_WRITE_CMD, 6, 0, NULL, 0 );
    asix_set_reg( dev, ASIX_IPG1_CMD, 0x15, 0, NULL, 0 );
    asix_set_reg( dev, ASIX_IPG2_CMD, 0x0c, 0, NULL, 0 );
    asix_set_reg( dev, ASIX_IPG3_CMD, 0x12, 0, NULL, 0 );
    data[0] = 0x01; data[1] = 0x05;
    asix_set_reg( dev, ASIX_MII_WRITE_CMD, phyid, 4, data, 2 );
    data[0] = 0; data[1] = 0x12;
    asix_set_reg( dev, ASIX_MII_WRITE_CMD, phyid, 0, data, 2 );
    asix_set_reg( dev, ASIX_MII_HWOP_CMD, 0, 0, NULL, 0 );
    asix_set_reg( dev, ASIX_RXCTL_CMD, 0x81, 0, NULL, 0 );

    /* kick start the receive */
    asix_queue_rx( softc );
}

static void asix_close_device( asix_softc_t *softc )
{
    /* disable adapter from receiving packets */
    asix_set_reg( softc->dev, ASIX_RXCTL_CMD, 0, 0, NULL, 0 );
}


/*  *********************************************************************
    * CFE-USB interfaces
    ********************************************************************* */

/*  *********************************************************************
    *  asix_attach(dev,drv)
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

const cfe_driver_t usbasix_driver;		/* forward declaration */

static int asix_attach(usbdev_t *dev,usb_driver_t *drv)
{
    usb_config_descr_t *cfgdscr = dev->ud_cfgdescr;
    usb_endpoint_descr_t *epdscr;
    usb_endpoint_descr_t *indscr = NULL;
    usb_endpoint_descr_t *outdscr = NULL;
    usb_interface_descr_t *ifdscr;
    asix_softc_t *softc;
    int idx;

    dev->ud_drv = drv;

    softc = (asix_softc_t *) KMALLOC( sizeof(asix_softc_t), USB_MALLOC_VALUE );
    if( softc == NULL ) {
	xprintf( "Failed to allocate softc memory.\n" );
	return -1;
	}
    memset( softc, 0, sizeof(asix_softc_t) );
    dev->ud_private = softc;
    softc->dev = dev;

    ifdscr = usb_find_cfg_descr(dev,USB_INTERFACE_DESCRIPTOR_TYPE,0);
    if (ifdscr == NULL) {
	xprintf("USBETH: ERROR...no interace descriptor\n");
	return -1;
	}

    for (idx = 0; idx < 3; idx++) {
	epdscr = usb_find_cfg_descr(dev,USB_ENDPOINT_DESCRIPTOR_TYPE,idx);
	if((epdscr->bmAttributes & USB_ENDPOINT_TYPE_MASK) == USB_ENDPOINT_TYPE_BULK) {
	    if (USB_ENDPOINT_DIR_OUT(epdscr->bEndpointAddress))
		outdscr = epdscr;
	    else
		indscr = epdscr;
	    }
	}

    if (!indscr || !outdscr) {
	/*
	 * Could not get descriptors, something is very wrong.
	 * Leave device addressed but not configured.
	 */
	xprintf("USBETH: ERROR...bulk endpoint descriptor(s) missing\n");
	return -1;
	}

    /* Choose the standard configuration. */
    usb_set_configuration(dev,cfgdscr->bConfigurationValue);

    /* Quit if not able to initialize the device */
    if (asix_init_device(softc) < 0)
	return -1;

    /* Open the pipes. */
    softc->bulk_inpipe     = usb_open_pipe(dev,indscr);
    softc->bulk_outpipe    = usb_open_pipe(dev,outdscr);

    /* Register the device */
    usbeth_register(&usbeth_asix, softc);

    /* Open the device */
    asix_open_device( softc );

    return 0;
}

/*  *********************************************************************
    *  asix_detach(dev)
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

static int asix_detach(usbdev_t *dev)
{
    asix_softc_t *softc = (asix_softc_t *) dev->ud_private;

    if (softc != NULL) {
	usbeth_unregister( softc );
	asix_close_device( softc );
	dev->ud_private = NULL;
	softc->dev = NULL;
	KFREE(softc);
	}

    return 0;
}

/* CFE USB device interface structure */
usb_driver_t usbeth_driver =
{
    "Ethernet Device",
    asix_attach,
    asix_detach
};
