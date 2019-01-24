/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  USB Ethernet				File: dev_usb_pegasus.c
    *  
    *  Driver for USB Ethernet devices using ADMTEK Pegasus chip.
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
static void hexdump( unsigned char * src, int srclen, int rowlen, int rows )
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

enum { PEGASUS, PEGASUS_II};
enum { VEN_NONE, _3_COM, LINKSYS, LINKSYS_10, LINKSYS_100, LINKSYS_100M };
static const char *VENDOR_NAMES[] = {
    "?", "3-COM", "LinkSys", "LinkSys-10TX", "LinkSys-100TX", "Yikes!"
};

static const int ID_TBL[] = {
    0x0506, 0x4601, PEGASUS_II, _3_COM,		/* 3-Com */
    0x066b, 0x2202, PEGASUS_II, LINKSYS_10,	/* LinkSys */
    0x066b, 0x2203, PEGASUS,    LINKSYS_100,
    0x066b, 0x2204, PEGASUS,    LINKSYS_100,
    0x066b, 0x2206, PEGASUS,    LINKSYS,
    0x066b, 0x400b, PEGASUS_II, LINKSYS_10,
    0x066b, 0x200c, PEGASUS_II, LINKSYS_10,
    -1
};

typedef struct peg_softc_s {
    usbdev_t *dev;
    int bulk_inpipe;
    int bulk_outpipe;
    int dev_id;
    int ven_code;
    uint8_t mac_addr[6];
    usbreq_t *rx_ur;
    uint8_t rxbuf[1600];    /* arbitrary but enough for ethernet packet */
} peg_softc_t;


/* **************************************
   *  ADMTEK PEGASUS I/F Functions
   ************************************** */

static int peg_send_eth_frame( void *ctx, hsaddr_t buf, int len );
static int peg_get_eth_frame( void *ctx, hsaddr_t buf );
static int peg_data_rx( void *ctx );
static int peg_get_dev_addr( void *ctx, hsaddr_t mac_addr );

static usbeth_disp_t usbeth_peg = {
    peg_get_eth_frame,
    peg_data_rx,
    peg_send_eth_frame,
    peg_get_dev_addr
};


static int peg_get_reg( usbdev_t *dev, int16_t reg, uint8_t *val, int16_t len )
{
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_IN),
			    PEG_GET_REG, 0, reg, val, len );
}

static int peg_set_reg( usbdev_t *dev, int16_t reg, int16_t val )
{
    unsigned char data = (uint8_t) val & 0xff;

    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
			    PEG_SET_REG, val, reg, &data, 1 );
}

static int peg_set_regs( usbdev_t *dev, int16_t reg, int8_t *vals, int16_t len )
{
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
			    PEG_SET_REG, 0, reg, (uint8_t *)vals, len );
}

static int peg_get_eep_word( usbdev_t *dev, int16_t ofs, uint8_t *val )
{
    int status=0, tries=20;
    uint8_t data[2];

    if( peg_set_reg( dev, R_PEG_EEPROM_CTL, 0 ) == FAIL )
	return FAIL;
    if( peg_set_reg( dev, R_PEG_EEPROM_OFS, ofs ) == FAIL )
	return FAIL;
    if( peg_set_reg( dev, R_PEG_EEPROM_CTL, 0x02 ) == FAIL )	/* read */
	return FAIL;
    while( --tries ) {
	if( peg_get_reg( dev, R_PEG_EEPROM_CTL, data, 1 ) == FAIL )
	    return FAIL;
	if( data[0] & 0x04 )
	    break;	/* eeprom data ready */
	}
    if( !tries ) {
	xprintf( "Pegasus Eeprom read failed!\n" );
	return FAIL;
	}
    if( peg_get_reg( dev, R_PEG_EEPROM_DATA, data, 2 ) == FAIL )
	return FAIL;
    val[0] = data[0];
    val[1] = data[1];

    return( status );
}

static int peg_get_mac_addr( usbdev_t *dev, uint8_t *mac_addr )
{
    int i, status;

    for( i = 0; i < 3; ++i ) {
	status = peg_get_eep_word( dev, i, &mac_addr[i*2] );
	}
    return( status );
}

static void peg_init_phy( usbdev_t *dev )
{
    /* needed for earlier versions (before Rev B) of the USB-100TX adapters */
    static uint8_t phy_magic_wr[] = { 0, 4, 0, 0x1b };
    static uint8_t read_status[] = { 0, 0, 0, 1 };
    uint8_t data[4];

    /* reset the MAC and set up GPIOs */
    peg_set_reg( dev, R_PEG_ETH_CTL1, 0x08 );
    peg_get_reg( dev, R_PEG_ETH_CTL1, data, 1 );

    /* do following steps to enable link activitiy LED */
    peg_set_reg( dev, R_PEG_GPIO1, 0x26 );
    peg_set_reg( dev, R_PEG_GPIO0, 0x24 );
    peg_set_reg( dev, R_PEG_GPIO0, 0x26 );

    /* do following set of steps to enable LINK LED */
    memcpy( data, phy_magic_wr, 4 );
    peg_set_regs( dev, R_PEG_PHY_ADDR, (int8_t *)data, 4);   /* magic word */
    peg_set_reg( dev, R_PEG_PHY_CTRL, (0x1b | PEG_PHY_WRITE) );
    peg_get_reg( dev, R_PEG_PHY_CTRL, data, 1 );   /* status of write */
    memcpy( data, read_status, 4 );
    peg_set_regs( dev, R_PEG_PHY_ADDR, (int8_t *)data, 4);   /* phy status reg */
    peg_set_reg( dev, R_PEG_PHY_CTRL, (1 | PEG_PHY_READ) );
    peg_get_reg( dev, R_PEG_PHY_CTRL, data, 1 );   /* status of read */
    peg_get_reg( dev, R_PEG_PHY_DATA, data, 2 );   /* read status regs */
}

static int peg_init_device( peg_softc_t * softc )
{
    usb_device_descr_t dev_desc;
    uint16_t vendor_id, product_id;
    const int *ptr=ID_TBL;
    usbdev_t *dev = softc->dev;

    /* find out which device is connected */
    usb_get_device_descriptor( softc->dev, &dev_desc, 0 );
    vendor_id = (dev_desc.idVendorHigh  << 8) + dev_desc.idVendorLow;
    product_id = (dev_desc.idProductHigh << 8) + dev_desc.idProductLow;

    while( *ptr != -1 )	{
	if( (vendor_id == ptr[0]) && (product_id == ptr[1]) ) {
	    softc->dev_id = ptr[2];
	    softc->ven_code = ptr[3];
	    break;
	    }
	ptr += 4;
	}
    if( *ptr == -1 ) {
	xprintf( "Unrecognized ADMTEK USB-Ethernet device\n" );
	return -1;
	}

    /* init the adapter */
    if( softc->dev_id == PEGASUS_II )
	peg_set_reg( dev, R_PEG_INT_PHY, 0x02 );    /* enable internal PHY */
    else
	peg_init_phy( dev );

    /* Read the adapter's MAC addr */
    peg_get_mac_addr( dev, softc->mac_addr );

    /* display adapter info */
    xprintf( "%s USB-Ethernet Adapter (%a)\n",
	     VENDOR_NAMES[softc->ven_code], softc->mac_addr);

    return 0;
}

static int peg_get_dev_addr( void *ctx, hsaddr_t mac_addr )
{
    peg_softc_t *softc = (peg_softc_t *) ctx;
    hs_memcpy_to_hs( mac_addr, softc->mac_addr, 6 );
    return 0;
}

static void peg_queue_rx( peg_softc_t *softc )
{
    softc->rx_ur = usb_make_request(softc->dev, softc->bulk_inpipe,
				    softc->rxbuf, sizeof(softc->rxbuf),
				    (UR_FLAG_IN | UR_FLAG_SHORTOK));
    usb_queue_request(softc->rx_ur);
}

static int peg_data_rx( void *ctx )
{
    peg_softc_t *softc = (peg_softc_t *) ctx;
    usb_poll(softc->dev->ud_bus);
    return( !softc->rx_ur->ur_inprogress );
}

static int peg_get_eth_frame( void *ctx, hsaddr_t buf )
{
    int len = 0;
    peg_softc_t *softc = (peg_softc_t *) ctx;
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
	peg_queue_rx( softc );
	}
    else
	xprintf( "Bulk data is not available yet!\n" );

    return( len );
}

static int peg_send_eth_frame( void *ctx, hsaddr_t buf, int len )
{
    peg_softc_t *softc = (peg_softc_t *) ctx;
    usbreq_t *ur;
    int txlen = len;
    unsigned char *txbuf;

    txbuf = usb_dma_alloc(len+2);
    txbuf[0] = txlen & 0xff;
    txbuf[1] = (txlen >> 8) & 0xff;   /* 1st two bytes...little endian */
    hs_memcpy_from_hs( &txbuf[2], buf, txlen );
    txlen += 2;
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


static void peg_open_device( peg_softc_t *softc )
{
    usbdev_t *dev = softc->dev;

    /* setup adapter's receiver with MAC address */
    peg_set_regs( dev, R_PEG_MAC_ADDR_0, (int8_t *)softc->mac_addr, 6 );

    /* enable adapter to receive and transmit packets */
    peg_set_reg( dev, R_PEG_ETH_CTL0, 0xc1 );
    peg_set_reg( dev, R_PEG_ETH_CTL1, 0x30 );

    /* kick start the receive */
    peg_queue_rx( softc );
}

static void peg_close_device( peg_softc_t *softc )
{
    usbdev_t *dev = softc->dev;

    /* Disable adapter from receiving or transmitting packets */
    peg_set_reg( dev, R_PEG_ETH_CTL1, 0 );
}


/*  *********************************************************************
    * CFE-USB interfaces
    ********************************************************************* */

/*  *********************************************************************
    *  peg_attach(dev,drv)
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

const cfe_driver_t usbpegdrv;		/* forward declaration */

static int peg_attach(usbdev_t *dev, usb_driver_t *drv)
{
    usb_config_descr_t *cfgdscr = dev->ud_cfgdescr;
    usb_endpoint_descr_t *epdscr;
    usb_endpoint_descr_t *indscr = NULL;
    usb_endpoint_descr_t *outdscr = NULL;
    usb_interface_descr_t *ifdscr;
    peg_softc_t *softc;
    int idx;

    dev->ud_drv = drv;

    softc = (peg_softc_t *) KMALLOC( sizeof(peg_softc_t), 0 );
    if( softc == NULL )	{
	xprintf( "Failed to allocate softc memory.\n" );
	return -1;
	}
    memset( softc, 0, sizeof(peg_softc_t) );
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
    if (peg_init_device(softc) < 0)
	return -1;

    /* Open the pipes. */
    softc->bulk_inpipe     = usb_open_pipe(dev,indscr);
    softc->bulk_outpipe    = usb_open_pipe(dev,outdscr);

    /* Register the device */
    usbeth_register(&usbeth_peg,softc);

    /* Open the device */
    peg_open_device( softc );

    return 0;
}

/*  *********************************************************************
    *  peg_detach(dev)
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

static int peg_detach(usbdev_t *dev)
{
    peg_softc_t *softc = (peg_softc_t *) dev->ud_private;

    if (softc != NULL) {
	usbeth_unregister( softc );
	peg_close_device ( softc );
	dev->ud_private = NULL;
	softc->dev = NULL;
	KFREE(softc);
	}

    return 0;
}

/*  CFE USB device interface structure */
usb_driver_t usbpeg_driver =
{
    "Ethernet Device",
    peg_attach,
    peg_detach
};


