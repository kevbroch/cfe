/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  RoboSwitch SPI driver			File: dev_robo_spi.c
    *  
    *  This is a simple driver for accessing RoboSwitch registers
    *  using the Motorola SPI serial protocol.  
    *  
    *********************************************************************  
    *
    *  Copyright 2004
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

#include "cfe_spi.h"

#undef _ROBOSPI_DEBUG_

#ifndef CFG_ROBO_FAST_SPI
#define CFG_ROBO_FAST_SPI 1
#endif

/*  *********************************************************************
    *  Robo definitions
    ********************************************************************* */

#define _DD_MAKEMASK1(n) (1 << (n))
#define _DD_MAKEMASK(v,n) ((((1)<<(v))-1) << (n))
#define _DD_MAKEVALUE(v,n) ((v) << (n))
#define _DD_GETVALUE(v,n,m) (((v) & (m)) >> (n))

/* Global Robo registers */
#define R_ROBOSPI_SPI_DATA      0xf0
#define R_ROBOSPI_SPI_STATUS    0xfe
#define R_ROBOSPI_PAGE          0xff

/* Robo SPI Status registers */
#define M_SPISTAT_TXRDY         _DD_MAKEMASK1(0)
#define M_SPISTAT_RXRDY         _DD_MAKEMASK1(1)
#define M_SPISTAT_MDIO_START    _DD_MAKEMASK1(2)
#define M_SPISTAT_RACK          _DD_MAKEMASK1(5)
#define M_SPISTAT_WCOL          _DD_MAKEMASK1(6)
#define M_SPISTAT_SPIF          _DD_MAKEMASK1(7)

/* Robo Fast SPI read response */
#define M_SPISTAT_FAST_RACK     _DD_MAKEMASK1(0)

#define POLL_DELAY()            cfe_usleep(10)

/* Convenience macro for writing a single byte */
#define SPI_WRITE8(c,b) do { uint8_t _d8 = b; SPI_WRITE(c,&_d8,1); } while (0)

/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */

static void robo_probe(cfe_driver_t *drv,
                       unsigned long probe_a,unsigned long probe_b, 
                       void *probe_ptr);

static int robo_open(cfe_devctx_t *ctx);
static int robo_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int robo_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int robo_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int robo_close(cfe_devctx_t *ctx);


/*  *********************************************************************
    *  Device Dispatch
    ********************************************************************* */

static cfe_devdisp_t robo_dispatch = {
    robo_open,
    robo_read,
    NULL,
    robo_write,
    robo_ioctl,
    robo_close,
    NULL,
    NULL
};

const cfe_driver_t spi_robo = {
    "Robo Management",
    "robo",
    CFE_DEV_OTHER,
    &robo_dispatch,
    robo_probe
};

typedef struct robo_s {
    cfe_spi_channel_t *spi_channel;
    int cur_page;
    int fast_spi;
} robo_spi_t;

static void _revert_buffer(uint8_t *buf,int len)
{
    uint8_t tbuf[8];
    int i;

    if (len < 2 || len > 8) return;

    memcpy(tbuf,buf,len);
    for (i = 0; i < len; i++) {
        buf[i] = tbuf[len-i-1];
    }
}

/*  *********************************************************************
    *  Robo SPI Protocol
    *  
    ********************************************************************* */

static int robo_read_reg(robo_spi_t *softc,int reg,uint8_t *buf,int len)
{
    cfe_spi_channel_t *chan = softc->spi_channel;

    SPI_ENABLE(chan,0);

    SPI_WRITE8(chan,0x60);
    SPI_WRITE8(chan,reg);
    SPI_READ(chan,buf,len,0);

    SPI_DISABLE(chan,0);

    return 0;
}

static int robo_write_reg(robo_spi_t *softc,int reg,uint8_t *buf,int len)
{
    cfe_spi_channel_t *chan = softc->spi_channel;

    SPI_ENABLE(chan,0);

    SPI_WRITE8(chan,0x61);
    SPI_WRITE8(chan,reg);
    SPI_WRITE(chan,buf,len);

    SPI_DISABLE(chan,0);

    return 0;
}

#if CFG_ROBO_FAST_SPI
static int robo_fast_rack_poll(robo_spi_t *softc)
{
    cfe_spi_channel_t *chan = softc->spi_channel;
    uint8_t status;
    int timeout = 10;

    while (timeout-- > 0) {

        SPI_READ(chan,&status,1,0);

        if (status & M_SPISTAT_FAST_RACK) {
            return 0;
        }

        POLL_DELAY();
    }
#ifdef _ROBOSPI_DEBUG_
    printf("SPI: fast RACK poll FAILED status=0x%02X\n",status);
#endif

    return -1;
}
#endif

static int robo_status_poll(robo_spi_t *softc,uint8_t mask,uint8_t val)
{
    uint8_t status;
    int timeout = 100;

    while (timeout-- > 0) {

        robo_read_reg(softc,R_ROBOSPI_SPI_STATUS,&status,1);

        if ((status & mask) == val) {
            return 0;
        }

        POLL_DELAY();
    }
#ifdef _ROBOSPI_DEBUG_
    printf("SPI: poll status FAILED status=0x%02X\n",status);
#endif

    return -1;
}

static int robo_select_page(robo_spi_t *softc,int page)
{
    uint8_t data;

    if (softc->cur_page == page) {
        return 0;
    }
    softc->cur_page = page;

    data = page;
    robo_write_reg(softc,R_ROBOSPI_PAGE,&data,1);

    return 0;
}

static int robo_reset(robo_spi_t *softc)
{
    /* Force page change */
    softc->cur_page = -1;
    robo_select_page(softc,0);
    softc->cur_page = -1;

    return 0;
}

/*  *********************************************************************
    *  robo_probe(drv,probe_a,probe_b,probe_ptr)
    *  
    *  Our probe routine.  Attach a SPI device to the firmware.
    *  
    *  Input parameters: 
    *  	   drv - driver structure
    *  	   probe_a - SPI channel
    *  	   probe_b - slave id (currently not used)
    *  	   probe_ptr - not used
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void robo_probe(cfe_driver_t *drv,
                       unsigned long probe_a,unsigned long probe_b, 
                       void *probe_ptr)
{
    robo_spi_t *softc;
    char descr[80];

    softc = (robo_spi_t *) KMALLOC(sizeof(robo_spi_t),0);

    if (!softc) return;

    /*
     * Probe_a is the SPI channel number
     * Probe_b is unused
     * Probe_ptr is unused.
     */

    softc->spi_channel = SPI_CHANNEL((int)probe_a);
    softc->cur_page = -1;

    xsprintf(descr,"%s on SPI channel %d",
	     drv->drv_description,(int)probe_a);
    cfe_attach(drv,softc,NULL,descr);
}

/*  *********************************************************************
    *  robo_open(ctx)
    *  
    *  Open this device. 
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else error code
    ********************************************************************* */

static int robo_open(cfe_devctx_t *ctx)
{
    robo_spi_t *softc = ctx->dev_softc;

    return softc->spi_channel ? 0 : -1;
}

/*  *********************************************************************
    *  robo_read(ctx,buffer)
    *  
    *  Read bytes from the device.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   buffer - buffer descriptor (target buffer, length, offset)
    *  	   
    *  Return value:
    *  	   number of bytes read
    *  	   -1 if an error occured
    ********************************************************************* */

static int robo_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    robo_spi_t *softc = ctx->dev_softc;
#if CFG_ROBO_FAST_SPI
    cfe_spi_channel_t *chan = softc->spi_channel;
#endif
    unsigned char *bptr;
    int blen;
    int page;
    int reg;

    bptr = HSADDR2PTR(buffer->buf_ptr);		/* XXX not 64-bit compliant */
    blen = buffer->buf_length;

    page = (buffer->buf_offset >> 8) & 0xFF;
    reg = buffer->buf_offset & 0xFF;

#ifdef _ROBOSPI_DEBUG_
    printf("robo_read: page=0x%02X reg=0x%02X\n",page,reg);
#endif

    if (blen > 8) return -1;

    if (robo_status_poll(softc,M_SPISTAT_MDIO_START,0) < 0) {
        /* Timeout */
        robo_reset(softc);
        return -1;
    }

    robo_select_page(softc,page);

#if CFG_ROBO_FAST_SPI

    SPI_ENABLE(chan,0);

    SPI_WRITE8(chan,0x10);
    SPI_WRITE8(chan,reg);

    if (robo_fast_rack_poll(softc) < 0) {
        /* Timeout */
        SPI_DISABLE(chan,0);
        robo_reset(softc);
        return -3;
    }

    /* Read registers */
    SPI_READ(chan,bptr,blen,0);

    SPI_DISABLE(chan,0);

#else

    /* Discard first read */
    robo_read_reg(softc,reg,bptr,1);

    if (robo_status_poll(softc,M_SPISTAT_RACK,M_SPISTAT_RACK) < 0) {
        /* Timeout */
        robo_reset(softc);
        return -2;
    }

    /* Read registers from dataport */
    robo_read_reg(softc,R_ROBOSPI_SPI_DATA,bptr,blen);

#endif

    /* Multi-byte Robo registers have LSB first */
    _revert_buffer(bptr,blen);

    return 0;
}

/*  *********************************************************************
    *  robo_write(ctx,buffer)
    *  
    *  Write bytes from the device.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   buffer - buffer descriptor (target buffer, length, offset)
    *  	   
    *  Return value:
    *  	   number of bytes read
    *  	   -1 if an error occured
    ********************************************************************* */

static int robo_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    robo_spi_t *softc = ctx->dev_softc;
    unsigned char *bptr;
    int blen;
    int page;
    int reg;

    bptr = HSADDR2PTR(buffer->buf_ptr);		/* XXX not 64-bit compliant */
    blen = buffer->buf_length;

    page = (buffer->buf_offset >> 8) & 0xFF;
    reg = buffer->buf_offset & 0xFF;

#ifdef _ROBOSPI_DEBUG_
    printf("robo_write: page=0x%02X reg=0x%02X\n",page,reg);
#endif

    if (blen > 8) return -1;

    /* Multi-byte Robo registers must have LSB first */
    _revert_buffer(bptr,blen);

    if (robo_status_poll(softc,M_SPISTAT_MDIO_START,0) < 0) {
        /* Timeout */
        robo_reset(softc);
        return -1;
    }

    robo_select_page(softc,page);

    robo_write_reg(softc,reg,bptr,blen);

    return 0;
}

/*  *********************************************************************
    *  robo_ioctl(ctx,buffer)
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

static int robo_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer) 
{
    return -1;
}

/*  *********************************************************************
    *  robo_close(ctx,buffer)
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

static int robo_close(cfe_devctx_t *ctx)
{
    return 0;
}


