/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Generic NVRAM Driver for STM48Txx chips and the equivalent.
    *  
    *  This module contains a CFE driver for a battery backed SRAM
    *  (Non-volatile) ram region at a configurable base address and
    *  provided size. 
    *  
    *  Author:  James Dougherty
    *           Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002
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

#define M48TXX_NVRAM_SIZE 0x1FF0

#define M48TXX_FLAGS	0x1FF0
#define M48TXX_CONTROL	0x1FF8
#define M48TXX_SECOND	0x1FF9
#define M48TXX_MINUTE	0x1FFA
#define M48TXX_HOUR	0x1FFB
#define M48TXX_DAY	0x1FFC
#define M48TXX_DATE	0x1FFD
#define M48TXX_MONTH	0x1FFE
#define M48TXX_YEAR	0x1FFF

#define M_M48TXX_MONTH	0x1F
#define M_M48TXX_DATE	0x3F
#define M_M48TXX_DAY	0x07
#define M_M48TXX_MINUTE	0x7F
#define M_M48TXX_SECOND	0x7F

/* FLAGS */
#define M_M48TXX_BL	0x10

/* CONTROL */
#define M_M48TXX_W	0x80
#define M_M48TXX_R	0x40
#define M_M48TXX_S	0x20

/* DAY */
#define M_M48TXX_FT	0x40
#define M_M48TXX_CEB	0x20
#define M_M48TXX_CB	0x10

#define BCD(x) (((x) % 10) + (((x) / 10) << 4))
#define SET_TIME	0x00
#define SET_DATE	0x01

#if ENDIAN_BIG
#define WRITECSR(p,v) phys_write8((p)^0x3,(v))
#define READCSR(p) phys_read8((p)^0x3)
#else
#define WRITECSR(p,v) phys_write8((p),(v))
#define READCSR(p) phys_read8((p))
#endif

/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */

static void m48txx_clock_probe(cfe_driver_t *drv,
			      unsigned long probe_a, unsigned long probe_b, 
			      void *probe_ptr);

static int m48txx_clock_open(cfe_devctx_t *ctx);
static int m48txx_clock_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int m48txx_clock_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int m48txx_clock_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int m48txx_clock_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int m48txx_clock_close(cfe_devctx_t *ctx);


/*  *********************************************************************
    *  Device dispatch
    ********************************************************************* */

const static cfe_devdisp_t m48txx_clock_dispatch = {
  m48txx_clock_open,
  m48txx_clock_read,
  m48txx_clock_inpstat,
  m48txx_clock_write,
  m48txx_clock_ioctl,
  m48txx_clock_close,
  NULL,
  NULL
};

const cfe_driver_t m48txx_clock = {
  "ST M48Txx RTC",
  "clock",
  CFE_DEV_CLOCK,
  &m48txx_clock_dispatch,
  m48txx_clock_probe
};
  

/*  *********************************************************************
    *  Structures
    ********************************************************************* */
typedef struct m48txx_clock_s {
    physaddr_t clock_base;
} m48txx_clock_t;
  
/*  *********************************************************************
    *  m48txx_clock_probe(drv,a,b,ptr)
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

static void m48txx_clock_probe(cfe_driver_t *drv,
				     unsigned long probe_a, unsigned long probe_b, 
				     void *probe_ptr)
{
    m48txx_clock_t *softc;
    char descr[80];

    softc = (m48txx_clock_t *) KMALLOC(sizeof(m48txx_clock_t),0);

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
    *  m48txx_clock_open(ctx)
    *  
    *  Open this device.  For the M48TXX, we do a quick test 
    *  read to be sure the device is out there.
    *  
    *  Input parameters: 
    *  	   ctx - device context (can obtain our softc here)
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else error code
    ********************************************************************* */

static int m48txx_clock_open(cfe_devctx_t *ctx)
{
    m48txx_clock_t *softc = ctx->dev_softc;
    physaddr_t clockbase;

    clockbase =  softc->clock_base;
    
    /* Make sure battery is still good and RTC valid. */
    if ((READCSR(clockbase+M48TXX_FLAGS) & M_M48TXX_BL) != 0) {
	printf("Warning: Battery has failed.  Clock setting is not accurate.\n");
	}
  
    return 0;
}

/*  *********************************************************************
    *  m48txx_clock_read(ctx,buffer)
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

static int m48txx_clock_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{

    m48txx_clock_t *softc = ctx->dev_softc;
    hsaddr_t bptr;
    physaddr_t clockbase;
    uint8_t byte;

    clockbase = softc->clock_base;

    bptr = buffer->buf_ptr;

    byte = (uint8_t) (READCSR(clockbase+M48TXX_CONTROL) & 0xFF);
    WRITECSR(clockbase+M48TXX_CONTROL,M_M48TXX_R | byte);

    
    byte =  READCSR(clockbase+M48TXX_HOUR);
    hs_write8(bptr,byte); bptr++;
    byte =  READCSR(clockbase+M48TXX_MINUTE);
    hs_write8(bptr,byte); bptr++;
    byte =  READCSR(clockbase+M48TXX_SECOND);
    hs_write8(bptr,byte); bptr++;
    byte =  READCSR(clockbase+M48TXX_MONTH) & M_M48TXX_MONTH;
    hs_write8(bptr,byte); bptr++;
    byte =  READCSR(clockbase+M48TXX_DATE) & M_M48TXX_DATE;
    hs_write8(bptr,byte); bptr++;
    byte =  READCSR(clockbase+M48TXX_YEAR);
    hs_write8(bptr,byte); bptr++;

    /* Assume that a century bit is present and is set for 20xx. */
    byte = READCSR(clockbase+M48TXX_DAY);
    byte = (byte & M_M48TXX_CB) ? 0x20 : 0x19;
    hs_write8(bptr,byte);

    byte = (uint8_t) (READCSR(clockbase+M48TXX_CONTROL) & 0xFF);
    WRITECSR(clockbase+M48TXX_CONTROL,~M_M48TXX_R & byte);
    
    buffer->buf_retlen = 8;
    return 0;
}

/*  *********************************************************************
    *  m48txx_clock_write(ctx,buffer)
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

static int m48txx_clock_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    m48txx_clock_t *softc = ctx->dev_softc;
    uint8_t byte;
    hsaddr_t bptr;
    uint8_t hr,min,sec;
    uint8_t mo,day,yr,y2k;
    uint8_t timeDateFlag;
    physaddr_t clockbase;

    clockbase = softc->clock_base;

    bptr = buffer->buf_ptr;

    /* Set SET bit */
    byte = (uint8_t) (READCSR(clockbase+M48TXX_CONTROL) & 0xFF);
    WRITECSR(clockbase+M48TXX_CONTROL,M_M48TXX_W | byte);

    timeDateFlag = hs_read8(bptr + 7);

    /* write time or date */
    if(timeDateFlag == SET_TIME) {

	hr = hs_read8(bptr);
	WRITECSR(clockbase+M48TXX_HOUR,BCD(hr));
	   
	min = hs_read8(bptr+1);
	WRITECSR(clockbase+M48TXX_MINUTE,BCD(min));
	
	sec = hs_read8(bptr+2);
	WRITECSR(clockbase+M48TXX_SECOND,BCD(sec));

	buffer->buf_retlen = 3;
	} 
    else if(timeDateFlag == SET_DATE) {

	mo = hs_read8(bptr+3);
	WRITECSR(clockbase+M48TXX_MONTH,BCD(mo));

	day = hs_read8(bptr+4);
	WRITECSR(clockbase+M48TXX_DATE,BCD(day));

	yr = hs_read8(bptr+5);
	WRITECSR(clockbase+M48TXX_YEAR,BCD(yr));

	y2k = hs_read8(bptr+6);
	byte = READCSR(clockbase+M48TXX_DAY);
	if (((y2k - 0x19) & 0x1) == 0)
	    byte &= ~M_M48TXX_CB;
	else
	    byte |= M_M48TXX_CB;      
	byte |= M_M48TXX_CEB;
	WRITECSR(clockbase+M48TXX_DAY,byte);

	buffer->buf_retlen = 4;
	}
    else {
	return -1;
	}

    /* clear SET bit */
    byte = (uint8_t) (READCSR(clockbase+M48TXX_CONTROL) & 0xFF);
    WRITECSR(clockbase+M48TXX_CONTROL,~M_M48TXX_W & byte);

  return 0;
}

/*  *********************************************************************
    *  m48txx_clock_inpstat(ctx,inpstat)
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

static int m48txx_clock_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat)
{
    inpstat->inp_status = 1;

    return 0;
}

/*  *********************************************************************
    *  m48txx_clock_ioctl(ctx,buffer)
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

static int m48txx_clock_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer) 
{
  return 0;
}

/*  *********************************************************************
    *  m48txx_clock_close(ctx,buffer)
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

static int m48txx_clock_close(cfe_devctx_t *ctx)
{
    return 0;
}


/*  *********************************************************************
    *  Forward Declarations
    ********************************************************************* */

static void m48txx_nvram_probe(cfe_driver_t *drv,
			       unsigned long probe_a, unsigned long probe_b, 
			       void *probe_ptr);


static int m48txx_nvram_open(cfe_devctx_t *ctx);
static int m48txx_nvram_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int m48txx_nvram_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int m48txx_nvram_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int m48txx_nvram_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int m48txx_nvram_close(cfe_devctx_t *ctx);

/*  *********************************************************************
    *  Dispatch tables
    ********************************************************************* */

const static cfe_devdisp_t m48txx_nvram_dispatch = {
    m48txx_nvram_open,
    m48txx_nvram_read,
    m48txx_nvram_inpstat,
    m48txx_nvram_write,
    m48txx_nvram_ioctl,
    m48txx_nvram_close,
    NULL,
    NULL
};

const cfe_driver_t m48txx_nvram = {
    "ST M48Txx NVRAM",
    "nvram",
    CFE_DEV_NVRAM,
    &m48txx_nvram_dispatch,
    m48txx_nvram_probe
};

typedef struct m48txx_nvram_s {
    int base_addr;
    int dev_size;
    int env_offset;
    int env_size;
    volatile unsigned char* data; /* NV region */
} m48txx_nvram_t;



/*  *********************************************************************
    *  m48txx_nvram_probe(drv,a,b,ptr)
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
static void m48txx_nvram_probe(cfe_driver_t *drv,
			       unsigned long probe_a, unsigned long probe_b, 
			       void *probe_ptr)
{
    m48txx_nvram_t *softc;
    char descr[80];

    softc = (m48txx_nvram_t *) KMALLOC(sizeof(m48txx_nvram_t),0);

    /*
     * Probe_a is the NVRAM base address.
     * Probe_b is the NVRAM region size.
     * Probe_ptr is unused.
     */

    softc->base_addr = (int)probe_a;
    softc->dev_size = (int)probe_b;
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
    *  m48txx_nvram_open(ctx)
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

static int m48txx_nvram_open(cfe_devctx_t *ctx)
{
    return 0;
}

/*  *********************************************************************
    *  m48txx_nvram_read(ctx,buffer)
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

static int m48txx_nvram_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    m48txx_nvram_t *softc = ctx->dev_softc;
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
    *  m48txx_nvram_inpstat(ctx,inpstat)
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

static int m48txx_nvram_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat)
{
    inpstat->inp_status = 1;

    return 0;
}

/*  *********************************************************************
    *  m48txx_nvram_write(ctx,buffer)
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

static int m48txx_nvram_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    m48txx_nvram_t *softc = ctx->dev_softc;
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
    *  m48txx_nvram_ioctl(ctx,buffer)
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

static int m48txx_nvram_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer) 
{
    m48txx_nvram_t *softc = ctx->dev_softc;
    nvram_info_t info;

    switch ((int)buffer->buf_ioctlcmd) {
	case IOCTL_NVRAM_GETINFO:
	    if (buffer->buf_length != sizeof(nvram_info_t)) return -1;
	    info.nvram_offset = softc->env_offset;
	    info.nvram_size =   softc->env_size;
	    info.nvram_eraseflg = FALSE;
	    buffer->buf_retlen = sizeof(nvram_info_t);
	    hs_memcpy_to_hs(buffer->buf_ptr,&info,sizeof(info));
	    return 0;
	default:
	    return -1;
	}
}

/*  *********************************************************************
    *  m48txx_nvram_close(ctx,buffer)
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

static int m48txx_nvram_close(cfe_devctx_t *ctx)
{
    return 0;
}


