/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  DS1743W RTC  driver		File: dev_ds1743wclock.c
    *  
    *  This module contains a CFE driver for a DS1743W generic bus
    *  real-time-clock.
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
#include "lib_physio.h"


/*  *********************************************************************
    *  Constants
    ********************************************************************* */

#define M_DS1743_BF	0x80
#define M_DS1743_FT	0x40
#define M_DS1743_OSC	0x80
#define M_DS1743_W	0x80
#define M_DS1743_R	0x40

#define M_DS1743_MONTH	0x1F
#define M_DS1743_DATE	0x3F
#define M_DS1743_DAY	0x03
#define M_DS1743_MINUTE 0x7F
#define M_DS1743_SECOND	0x7F
#define M_DS1743_CENTURY 0x3F


#define DS1743_NVRAM_SIZE 0x1FF8

#define DS1743_CONTROL	0x1FF8
#define DS1743_CENTURY	0x1FF8
#define DS1743_SECOND	0x1FF9
#define DS1743_MINUTE	0x1FFA
#define DS1743_HOUR	0x1FFB
#define DS1743_DAY	0x1FFC
#define DS1743_DATE	0x1FFD
#define DS1743_MONTH	0x1FFE
#define DS1743_YEAR	0x1FFF

#define BCD(x) (((x) % 10) + (((x) / 10) << 4))
#define SET_TIME	0x00
#define SET_DATE	0x01

#if ENDIAN_BIG
#define WRITECSR(p,v) phys_write8((p)^3,(v))
#define READCSR(p) phys_read8((p)^3)
#elif ENDIAN_LITTLE
#define WRITECSR(p,v) phys_write8((p),(v))
#define READCSR(p) phys_read8((p))
#else
#error "dev_ds1743: system endian not set"
#endif

/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */

static void ds1743_clock_probe(cfe_driver_t *drv,
			      unsigned long probe_a, unsigned long probe_b, 
			      void *probe_ptr);

static int ds1743_clock_open(cfe_devctx_t *ctx);
static int ds1743_clock_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int ds1743_clock_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int ds1743_clock_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int ds1743_clock_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int ds1743_clock_close(cfe_devctx_t *ctx);


/*  *********************************************************************
    *  Device dispatch
    ********************************************************************* */

const static cfe_devdisp_t ds1743_clock_dispatch = {
  ds1743_clock_open,
  ds1743_clock_read,
  ds1743_clock_inpstat,
  ds1743_clock_write,
  ds1743_clock_ioctl,
  ds1743_clock_close,
  NULL,
  NULL
};

const cfe_driver_t ds1743_clock = {
  "Dallas DS1743 RTC",
  "clock",
  CFE_DEV_CLOCK,
  &ds1743_clock_dispatch,
  ds1743_clock_probe
};
  

/*  *********************************************************************
    *  Structures
    ********************************************************************* */
typedef struct ds1743_clock_s {
    physaddr_t clock_base;
} ds1743_clock_t;
  
/*  *********************************************************************
    *  ds1743_clock_probe(drv,a,b,ptr)
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

static void ds1743_clock_probe(cfe_driver_t *drv,
				     unsigned long probe_a, unsigned long probe_b, 
				     void *probe_ptr)
{
    ds1743_clock_t *softc;
    char descr[80];

    softc = (ds1743_clock_t *) KMALLOC(sizeof(ds1743_clock_t),0);

    /*
     * Probe_a is the clock base address
     * Probe_b is unused.
     * Probe_ptr is unused.
     */

    softc->clock_base = probe_a;

    xsprintf(descr,"%s at 0x%X",
	     drv->drv_description,(uint32_t)probe_a);
    cfe_attach(drv,softc,NULL,descr);
   
}

/*  *********************************************************************
    *  ds1743_clock_open(ctx)
    *  
    *  Open this device.  For the DS1743, we do a quick test 
    *  read to be sure the device is out there.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else error code
    ********************************************************************* */

static int ds1743_clock_open(cfe_devctx_t *ctx)
{
    ds1743_clock_t *softc = ctx->dev_softc;
    physaddr_t clockbase;

    clockbase =  softc->clock_base;
    
    /* Make sure battery is still good and RTC valid */
    if (!(READCSR(clockbase+DS1743_DAY) & M_DS1743_BF)) {
	printf("Warning: Battery has failed.  Clock setting is not accurate.\n");
	}

  
    return 0;
}

/*  *********************************************************************
    *  ds1743_clock_read(ctx,buffer)
    *  
    *  Read time/date from the RTC. Read a total of 8 bytes in this format:
    *  hour-minute-second-month-day-year1-year2
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   buffer - buffer descriptor (target buffer, length, offset)
    *  	   
    *  Return value:
    *  	   number of bytes read
    *  	   -1 if an error occured
    ********************************************************************* */

static int ds1743_clock_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{

    ds1743_clock_t *softc = ctx->dev_softc;
    hsaddr_t bptr;
    uint8_t b;
    physaddr_t clockbase;
    uint8_t byte;

    clockbase = softc->clock_base;

    bptr = buffer->buf_ptr;

    byte = (uint8_t) (READCSR(clockbase+DS1743_CONTROL) & 0xFF);
    WRITECSR(clockbase+DS1743_CONTROL,M_DS1743_R | byte);

    b =  READCSR(clockbase+DS1743_HOUR);  hs_write8(bptr,b); bptr++;
    b =  READCSR(clockbase+DS1743_MINUTE); hs_write8(bptr,b); bptr++;
    b =  READCSR(clockbase+DS1743_SECOND); hs_write8(bptr,b); bptr++;
    b =  READCSR(clockbase+DS1743_MONTH) & M_DS1743_MONTH; hs_write8(bptr,b); bptr++;
    b =  READCSR(clockbase+DS1743_DATE) & M_DS1743_DATE; hs_write8(bptr,b); bptr++;
    b =  READCSR(clockbase+DS1743_YEAR); hs_write8(bptr,b); bptr++;
    b =  READCSR(clockbase+DS1743_CENTURY) & M_DS1743_CENTURY; hs_write8(bptr,b); bptr++;

    byte = (uint8_t) (READCSR(clockbase+DS1743_CONTROL) & 0xFF);
    WRITECSR(clockbase+DS1743_CONTROL,~M_DS1743_R & byte);
    
    buffer->buf_retlen = 8;
    return 0;
}

/*  *********************************************************************
    *  ds1743_clock_write(ctx,buffer)
    *  
    *  Write time/date to the RTC. Write in this format:
    *  hour-minute-second-month-day-year1-year2-(time/date flag)
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   buffer - buffer descriptor (target buffer, length, offset)
    *  	   
    *  Return value:
    *  	   number of bytes written
    *  	   -1 if an error occured
    ********************************************************************* */

static int ds1743_clock_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    ds1743_clock_t *softc = ctx->dev_softc;
    uint8_t byte;
    hsaddr_t bptr;
    uint8_t hr,min,sec;
    uint8_t mo,day,yr,y2k;
    uint8_t timeDateFlag;
    physaddr_t clockbase;

    clockbase = softc->clock_base;

    bptr = buffer->buf_ptr;

    /* Set SET bit */
    byte = (uint8_t) (READCSR(clockbase+DS1743_CONTROL) & 0xFF);
    WRITECSR(clockbase+DS1743_CONTROL,M_DS1743_W | byte);

    timeDateFlag = hs_read8(bptr + 7);

    /* write time or date */
    if(timeDateFlag == SET_TIME) {

	hr = (uint8_t) hs_read8(bptr);
	WRITECSR(clockbase+DS1743_HOUR,BCD(hr));
	   
	min = (uint8_t) hs_read8(bptr+1);
	WRITECSR(clockbase+DS1743_MINUTE,BCD(min));
	
	sec = (uint8_t) hs_read8(bptr+2);
	WRITECSR(clockbase+DS1743_SECOND,BCD(sec));

	buffer->buf_retlen = 3;
	} 
    else if(timeDateFlag == SET_DATE) {
	uint8_t byte;

	mo = (uint8_t) hs_read8(bptr+3);
	WRITECSR(clockbase+DS1743_MONTH,BCD(mo));

	day = (uint8_t) hs_read8(bptr+4);
	WRITECSR(clockbase+DS1743_DATE,BCD(day));

	yr = (uint8_t) hs_read8(bptr+5);
	WRITECSR(clockbase+DS1743_YEAR,BCD(yr));

	y2k = (uint8_t) hs_read8(bptr+6);
	byte = READCSR(clockbase+DS1743_CENTURY);
	byte &= ~M_DS1743_CENTURY;
	byte |= (y2k & M_DS1743_CENTURY);
 	WRITECSR(clockbase+DS1743_CENTURY, byte);
   
	buffer->buf_retlen = 4;
	}
    else {
	return -1;
	}

    /* clear SET bit */
    byte = (uint8_t) (READCSR(clockbase+DS1743_CONTROL) & 0xFF);
    WRITECSR(clockbase+DS1743_CONTROL,~M_DS1743_W & byte);

  return 0;
}

/*  *********************************************************************
    *  ds1743_clock_inpstat(ctx,inpstat)
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

static int ds1743_clock_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat)
{
    inpstat->inp_status = 1;

    return 0;
}

/*  *********************************************************************
    *  ds1743_clock_ioctl(ctx,buffer)
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

static int ds1743_clock_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer) 
{
  return 0;
}

/*  *********************************************************************
    *  ds1743_clock_close(ctx,buffer)
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

static int ds1743_clock_close(cfe_devctx_t *ctx)
{
    return 0;
}



/*  *********************************************************************
    *  Forward Declarations
    ********************************************************************* */

static void ds1743_nvram_probe(cfe_driver_t *drv,
			       unsigned long probe_a, unsigned long probe_b, 
			       void *probe_ptr);


static int ds1743_nvram_open(cfe_devctx_t *ctx);
static int ds1743_nvram_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int ds1743_nvram_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int ds1743_nvram_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int ds1743_nvram_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int ds1743_nvram_close(cfe_devctx_t *ctx);

/*  *********************************************************************
    *  Dispatch tables
    ********************************************************************* */

const static cfe_devdisp_t ds1743_nvram_dispatch = {
    ds1743_nvram_open,
    ds1743_nvram_read,
    ds1743_nvram_inpstat,
    ds1743_nvram_write,
    ds1743_nvram_ioctl,
    ds1743_nvram_close,
    NULL,
    NULL
};

const cfe_driver_t ds1743_nvram = {
    "Dallas DS1743 NVRAM",
    "nvram",
    CFE_DEV_NVRAM,
    &ds1743_nvram_dispatch,
    ds1743_nvram_probe
};

typedef struct ds1743_nvram_s {
    int base_addr;
    int dev_size;
    int env_offset;
    int env_size;
    volatile unsigned char* data; /* NV region */
} ds1743_nvram_t;



/*  *********************************************************************
    *  ds1743_nvram_probe(drv,a,b,ptr)
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
static void ds1743_nvram_probe(cfe_driver_t *drv,
			       unsigned long probe_a, unsigned long probe_b, 
			       void *probe_ptr)
{
    ds1743_nvram_t *softc;
    char descr[80];

    softc = (ds1743_nvram_t *) KMALLOC(sizeof(ds1743_nvram_t),0);

    /*
     * Probe_a is the NVRAM base address.
     * Probe_b is unused
     * Probe_ptr is unused.
     */

    softc->base_addr = (int)probe_a;
    softc->dev_size = DS1743_NVRAM_SIZE;
    softc->env_offset  = 0;
    softc->env_size = softc->dev_size;
    softc->data = (volatile unsigned char*) UNCADDR(softc->base_addr);
    /* PHYS_TO_XKSEG_UNCACHED(softc->base_addr); */

    xsprintf(descr,"%s at %x size %dKB",
	     drv->drv_description,
	     softc->base_addr, 
	     softc->dev_size / 1024 );

    cfe_attach(drv,softc,NULL,descr);
}



/*  *********************************************************************
    *  ds1743_nvram_open(ctx)
    *  
    *  Open this device.  For the X1240, we do a quick test 
    *  read to be sure the device is out there.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else error code
    ********************************************************************* */

static int ds1743_nvram_open(cfe_devctx_t *ctx)
{
    return 0;
}

/*  *********************************************************************
    *  ds1743_nvram_read(ctx,buffer)
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

static int ds1743_nvram_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    ds1743_nvram_t *softc = ctx->dev_softc;
    hsaddr_t bptr;
    int blen;
    int idx;
    int b = 0;

    bptr = buffer->buf_ptr;
    blen = buffer->buf_length;

    if ((buffer->buf_offset + blen) > softc->dev_size) return -1;

    idx = (int) buffer->buf_offset;

    while (blen > 0) {
	b = softc->data[idx];
	hs_write8(bptr,(unsigned char) b);
	bptr++;
	blen--;
	idx++;
    }

    buffer->buf_retlen = bptr - buffer->buf_ptr;
    return (b < 0) ? -1 : 0;
}

/*  *********************************************************************
    *  ds1743_nvram_inpstat(ctx,inpstat)
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

static int ds1743_nvram_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat)
{
    inpstat->inp_status = 1;

    return 0;
}

/*  *********************************************************************
    *  ds1743_nvram_write(ctx,buffer)
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

static int ds1743_nvram_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    ds1743_nvram_t *softc = ctx->dev_softc;
    hsaddr_t bptr;
    int blen;
    int idx;
    int b = 0;

    bptr = buffer->buf_ptr;
    blen = buffer->buf_length;

    if ((buffer->buf_offset + blen) > softc->dev_size) return -1;

    idx = (int) buffer->buf_offset;

    while (blen > 0) {
	b = hs_read8(bptr);
	bptr++;
	softc->data[idx] = b;
	blen--;
	idx++;
    }

    buffer->buf_retlen = bptr - buffer->buf_ptr;
    return (b < 0) ? -1 : 0;
}

/*  *********************************************************************
    *  ds1743_nvram_ioctl(ctx,buffer)
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

static int ds1743_nvram_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer) 
{
    ds1743_nvram_t *softc = ctx->dev_softc;
    nvram_info_t info;

    switch ((int)buffer->buf_ioctlcmd) {
	case IOCTL_NVRAM_GETINFO:
	    if (buffer->buf_length != sizeof(nvram_info_t)) return -1;
	    info.nvram_offset = softc->env_offset;
	    info.nvram_size =   softc->env_size;
	    info.nvram_eraseflg = FALSE;
	    hs_memcpy_to_hs(buffer->buf_ptr,&info,sizeof(info));
	    buffer->buf_retlen = sizeof(nvram_info_t);
	    return 0;
	default:
	    return -1;
	}
}

/*  *********************************************************************
    *  ds1743_nvram_close(ctx,buffer)
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

static int ds1743_nvram_close(cfe_devctx_t *ctx)
{
    return 0;
}


