/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  USB Ethernet				File: dev_usb_catc.c
    *  
    *  Driver for USB Ethernet devices using the CATC Netmate chip.
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

enum { VEN_NONE, CATC_NM, BELKIN_CATC };
static const char *VENDOR_NAMES[] = {
    "?", "CATC-Netmate", "Belkin/CATC", "Yikes!"
};

static const int ID_TBL[] = {
    0x0423, 0x000a, CATC_NM,		/* CATC (Netmate I) */
    0x0423, 0x000c, BELKIN_CATC,	/* Belkin/CATC (Netmate II) */
    -1
};

typedef struct catc_softc_s {
    usbdev_t *dev;
    int bulk_inpipe;
    int bulk_outpipe;
    int ven_code;
    uint8_t mac_addr[6];
    usbreq_t *rx_ur;
    uint8_t rxbuf[1600];    /* arbitrary but enough for ethernet packet */
} catc_softc_t;


/* **************************************
   *  CATC I/F Functions
   ************************************** */

static int catc_send_eth_frame( void *ctx, hsaddr_t buf, int len );
static int catc_get_eth_frame( void *ctx, hsaddr_t buf );
static int catc_data_rx( void *ctx );
static int catc_get_dev_addr( void *ctx, hsaddr_t mac_addr );

static usbeth_disp_t usbeth_catc = {
    catc_get_eth_frame,
    catc_data_rx,
    catc_send_eth_frame,
    catc_get_dev_addr
};

#if 0
static int catc_get_reg( usbdev_t *dev, int16_t reg, uint8_t *val )
{
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_IN),
			    CATC_GET_REG, 0, reg, val, 1 );
}
#endif

static int catc_set_reg( usbdev_t *dev, int16_t reg, int16_t val )
{
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
			    CATC_SET_REG, val, reg, NULL, 0 );
}

static int catc_set_mem( usbdev_t *dev, int16_t addr,
			 uint8_t *data, int16_t len )
{
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_OUT),
			    CATC_SET_MEM, 0, addr, data, len );
}

static int catc_get_mac_addr( usbdev_t *dev, uint8_t *mac_addr )
{
    return usb_std_request( dev, (USBREQ_TYPE_VENDOR | USBREQ_DIR_IN),
			      CATC_GET_MAC_ADDR, 0, 0, mac_addr, 6 );
}

static int catc_init_device( catc_softc_t *softc )
{
    usb_device_descr_t dev_desc;
    uint16_t vendor_id, product_id;
    const int *ptr=ID_TBL;
    usbdev_t *dev = softc->dev;
    unsigned char *mcast_tbl;


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
	xprintf( "Unrecognized CATC USB-Ethernet device\n" );
	return -1;
	}

    usb_std_request( dev, (USBREQ_TYPE_STD | USBREQ_REC_INTERFACE),
		     USB_REQUEST_SET_INTERFACE,
		     1,         /* alt setting 1 */
		     0, NULL, 0 );

    catc_set_reg(dev, CATC_TX_BUF_CNT_REG, 0x04 );
    catc_set_reg(dev, CATC_RX_BUF_CNT_REG, 0x10 );
    catc_set_reg(dev, CATC_ADV_OP_MODES_REG, 0x01 );
    catc_set_reg(dev, CATC_LED_CTRL_REG, 0x08 );

    /* Enable broadcast rx via bit in multicast table */
    mcast_tbl = KMALLOC(64, CACHE_ALIGN);
    memset( mcast_tbl, 0, 64 );
    mcast_tbl[31] = 0x80;     /* broadcast bit */
    catc_set_mem( dev, CATC_MCAST_TBL_ADDR, mcast_tbl, 64 );
    KFREE(mcast_tbl);

    /* Read the adapter's MAC addr */
    catc_get_mac_addr( dev, softc->mac_addr );

    /* display adapter info */
    xprintf( "%s USB-Ethernet Adapter (%a)\n",
	     VENDOR_NAMES[softc->ven_code], softc->mac_addr);

    return 0;
}

static int catc_get_dev_addr( void *ctx, hsaddr_t mac_addr )
{
    catc_softc_t *softc = (catc_softc_t *) ctx;
    hs_memcpy_to_hs( mac_addr, softc->mac_addr, 6 );
    return 0;
}

static void catc_queue_rx( catc_softc_t *softc )
{
    softc->rx_ur = usb_make_request(softc->dev, softc->bulk_inpipe,
				    softc->rxbuf, sizeof(softc->rxbuf),
				    (UR_FLAG_IN | UR_FLAG_SHORTOK));
    usb_queue_request(softc->rx_ur);
}

static int catc_data_rx( void *ctx )
{
    catc_softc_t *softc = (catc_softc_t *) ctx;
    usb_poll(softc->dev->ud_bus);
    return( !softc->rx_ur->ur_inprogress );
}

static int catc_get_eth_frame( void  *ctx, hsaddr_t buf )
{
    catc_softc_t *softc = (catc_softc_t *) ctx;
    int len = 0;
    uint8_t *rxbuf;

    if( !softc->rx_ur->ur_inprogress ) {
	rxbuf = softc->rxbuf;
	len = softc->rx_ur->ur_xferred;
	if (len > 0) {
#if CATC_DEBUG
	    xprintf( "Incoming packet :\n" );
	    hexdump( rxbuf, len, 16, len / 16 + 1 );
#endif
	    hs_memcpy_to_hs( buf, rxbuf, len );
	    }
	usb_free_request(softc->rx_ur);
	catc_queue_rx( softc );
	}
    else
	xprintf( "Bulk data is not available yet!\n" );

    return( len );
}

static int catc_send_eth_frame( void *ctx, hsaddr_t buf, int len )
{
    catc_softc_t *softc = (catc_softc_t *) ctx;
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

static void catc_open_device( catc_softc_t *softc )
{
    int i;

    for(i = 0; i < 6; ++i)
	catc_set_reg( softc->dev, (CATC_ETH_ADDR_0_REG - i), softc->mac_addr[i] );

    /* Enable adapter to receive packets */
    catc_set_reg( softc->dev, CATC_ETH_CTRL_REG, 0x09 );

    /* kick start the receive */
    catc_queue_rx( softc );
}

static void catc_close_device( catc_softc_t *softc )
{
    usbdev_t *dev = softc->dev;

    /* Disable adapter from receiving packets */
    catc_set_reg( dev, CATC_ETH_CTRL_REG, 0 );
}


/*  *********************************************************************
    * CFE-USB interfaces
    ********************************************************************* */

/*  *********************************************************************
    *  catc_attach(dev,drv)
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

const cfe_driver_t usbcatcdrv;		/* forward declaration */

static int catc_attach(usbdev_t *dev, usb_driver_t *drv)
{
    usb_config_descr_t *cfgdscr = dev->ud_cfgdescr;
    usb_endpoint_descr_t *epdscr;
    usb_endpoint_descr_t *indscr = NULL;
    usb_endpoint_descr_t *outdscr = NULL;
    usb_interface_descr_t *ifdscr;
    catc_softc_t *softc;
    int idx;

    dev->ud_drv = drv;

    softc = (catc_softc_t *) KMALLOC( sizeof(catc_softc_t), 0 );
    if( softc == NULL )	{
	xprintf( "Failed to allocate softc memory.\n" );
	return -1;
	}
    memset( softc, 0, sizeof(catc_softc_t) );
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
    if (catc_init_device(softc) < 0)
	return -1;

    /* Open the pipes. */
    softc->bulk_inpipe     = usb_open_pipe(dev,indscr);
    softc->bulk_outpipe    = usb_open_pipe(dev,outdscr);

    /* Register the device */
    usbeth_register(&usbeth_catc,softc);

    /* Open the device */
    catc_open_device( softc );

    return 0;
}

/*  *********************************************************************
    *  catc_detach(dev)
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

static int catc_detach(usbdev_t *dev)
{
    catc_softc_t *softc = (catc_softc_t *) dev->ud_private;

    if (softc != NULL) {
	usbeth_unregister( softc );
	catc_close_device ( softc );
	dev->ud_private = NULL;
	softc->dev = NULL;
	KFREE(softc);
	}

    return 0;
}

/*  CFE USB device interface structure */
usb_driver_t usbcatc_driver =
{
    "Ethernet Device",
    catc_attach,
    catc_detach
};


/*  *********************************************************************
    * CFE-Ethernet device interfaces
    ********************************************************************* */


static int catc_ether_open(cfe_devctx_t *ctx)
{
    catc_softc_t *softc = (catc_softc_t *) ctx->dev_softc;

    if (softc->dev == NULL)
	return CFE_ERR_NOTREADY;

    USBETH_TRACE( "%s called.\n", __FUNCTION__ );
    catc_open_device( softc );

    return 0;
}

static int catc_ether_read( cfe_devctx_t *ctx, iocb_buffer_t *buffer )
{
    catc_softc_t *softc = (catc_softc_t *) ctx->dev_softc;

    if (softc->dev == NULL)
	return CFE_ERR_NOTREADY;

    buffer->buf_retlen = catc_get_eth_frame( softc, buffer->buf_ptr );

    return 0;
}


static int catc_ether_inpstat( cfe_devctx_t *ctx, iocb_inpstat_t *inpstat )
{
    catc_softc_t *softc = (catc_softc_t *) ctx->dev_softc;

    if (softc->dev == NULL)
	return CFE_ERR_NOTREADY;

    inpstat->inp_status = catc_data_rx( softc );
    return 0;
}


static int catc_ether_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    catc_softc_t *softc = (catc_softc_t *) ctx->dev_softc;

    if (softc->dev == NULL)
	return CFE_ERR_NOTREADY;

    /* Block until hw notifies you data is sent. */
    catc_send_eth_frame( softc, buffer->buf_ptr, buffer->buf_length );

    return 0;
}


static int catc_ether_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    catc_softc_t *softc = (catc_softc_t *) ctx->dev_softc;
    int retval = 0;

    if (softc->dev == NULL)
	return CFE_ERR_NOTREADY;

    switch( (int)buffer->buf_ioctlcmd ) {
	case IOCTL_ETHER_GETHWADDR:
	    USBETH_TRACE( "IOCTL_ETHER_GETHWADDR called.\n" );
	    catc_get_dev_addr( softc, buffer->buf_ptr );
	    break;
	case IOCTL_ETHER_SETHWADDR:
	    xprintf( "IOCTL_ETHER_SETHWADDR not implemented.\n" );
	    break;
#if 0
	case IOCTL_ETHER_GETSPEED:
	    xprintf( "GETSPEED not implemented.\n" );
	    retval = -1;
	    break;
	case IOCTL_ETHER_SETSPEED:
	    xprintf( "SETSPEED not implemented.\n" );
	    retval = -1;
	    break;
	case IOCTL_ETHER_GETLINK:
	    xprintf( "GETLINK not implemented.\n" );
	    retval = -1;
	    break;
	case IOCTL_ETHER_GETLOOPBACK:
	    xprintf( "GETLOOPBACK not implemented.\n" );
	    retval = -1;
	    break;
	case IOCTL_ETHER_SETLOOPBACK:
	    xprintf( "SETLOOPBACK not implemented.\n" );
	    retval = -1;
	    break;
#endif
	default:
	    xprintf( "Invalid IOCTL to catc_ether_ioctl.\n" );
	    retval = -1;
	}

    return retval;
}


static int catc_ether_close(cfe_devctx_t *ctx)
{
    catc_softc_t *softc = (catc_softc_t *) ctx->dev_softc;

    if (softc->dev == NULL)
	return CFE_ERR_NOTREADY;

    USBETH_TRACE( "%s called.\n", __FUNCTION__ );
    catc_close_device( softc );
    return 0;
}


/* CFE ethernet device interface structures */

const static cfe_devdisp_t catc_ether_dispatch =
{
    catc_ether_open,
    catc_ether_read,
    catc_ether_inpstat,
    catc_ether_write,
    catc_ether_ioctl,
    catc_ether_close,
    NULL,
    NULL
};

const cfe_driver_t usbcatcdrv =
{
    "USB-Ethernet Device",
    "eth",
    CFE_DEV_NETWORK,
    &catc_ether_dispatch,
    NULL,			/* probe...not needed */
};

