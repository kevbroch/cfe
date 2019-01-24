/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  NS16550 UART driver			File: dev_ns16550.c
    *  
    *  This is a console device driver for an NS16550 UART, either
    *  on-board or as a PCI-device.  In the case of a PCI device,
    *  our probe routine is called from the PCI probe code
    *  over in dev_ns16550_pci.c
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

#include "ns16550.h"

#if defined(BCM47XX) && ENDIAN_BIG
#define READREG(sc,r) phys_read8((sc)->uart_base+((r)^0x3))
#define WRITEREG(sc,r,v) phys_write8((sc)->uart_base+((r)^0x3),(v))
#else
#define READREG(sc,r) phys_read8((sc)->uart_base+(r))
#define WRITEREG(sc,r,v) phys_write8((sc)->uart_base+(r),(v))
#endif

#define READCSR  READREG
#define WRITECSR WRITEREG

/* Workarounds */
#if defined(BCM4310)
/* PR9562: Reads from MIPS to UART are unreliable */
/* PR15083 WAR: Use dummy variables */
#undef  READCSR
#define READCSR(sc,r) (READREG((sc),R_UART_SCR), READREG((sc),(r)))
#endif
#if defined(BCM4704) || defined(BCM5365) || defined(BCM5836)
/* PR13509 WAR: Read SB location after UART write */
#undef  WRITECSR
#define WRITECSR(sc,r,v) \
     do { WRITEREG((sc),(r),(v)); phys_read8(0x18000000); } while (0)
#endif

static int ns16550_uart_open(cfe_devctx_t *ctx);
static int ns16550_uart_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int ns16550_uart_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int ns16550_uart_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int ns16550_uart_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int ns16550_uart_close(cfe_devctx_t *ctx);

void ns16550_uart_probe(cfe_driver_t *drv,
			unsigned long probe_a, unsigned long probe_b, 
			void *probe_ptr);


const cfe_devdisp_t ns16550_uart_dispatch = {
    ns16550_uart_open,
    ns16550_uart_read,
    ns16550_uart_inpstat,
    ns16550_uart_write,
    ns16550_uart_ioctl,
    ns16550_uart_close,	
    NULL,
    NULL
};

const cfe_driver_t ns16550_uart = {
    "NS16550 UART",
    "uart",
    CFE_DEV_SERIAL,
    &ns16550_uart_dispatch,
    ns16550_uart_probe
};

typedef struct ns16550_uart_s {
    physaddr_t uart_base;
    int uart_flowcontrol;
    int baud_base;
    int uart_speed;
} ns16550_uart_t;


/* 
 * NS16550-compatible UART.
 * probe_a: physical address of UART
 */

void ns16550_uart_probe(cfe_driver_t *drv,
			unsigned long probe_a, unsigned long probe_b, 
			void *probe_ptr)
{
    ns16550_uart_t *softc;
    char descr[80];

    softc = (ns16550_uart_t *) KMALLOC(sizeof(ns16550_uart_t),0);
    if (softc) {
	softc->uart_base = probe_a;
	softc->baud_base = probe_b ? probe_b : NS16550_HZ;
	softc->uart_speed = CFG_SERIAL_BAUD_RATE;
	softc->uart_flowcontrol = SERIAL_FLOW_NONE;
	xsprintf(descr, "%s at 0x%X", drv->drv_description, (uint32_t)probe_a);

	cfe_attach(drv, softc, NULL, descr);
	}
}

#if !defined(MIPS33xx)

#define DELAY(n) delay(n)
extern int32_t _getticks(void);
static void delay(int ticks)
{
    int32_t t;

    t = _getticks() + ticks;
    while (_getticks() < t)
	; /* NULL LOOP */
}
#endif /* !MIPS33xx */

static void ns16550_uart_setflow(ns16550_uart_t *softc)
{
    /* noop for now */
}


static int ns16550_uart_open(cfe_devctx_t *ctx)
{
    ns16550_uart_t *softc = ctx->dev_softc;
    unsigned int brtc;

    brtc = BRTC(softc->baud_base, softc->uart_speed);

    WRITECSR(softc,R_UART_CFCR,CFCR_DLAB);
    WRITECSR(softc,R_UART_DATA,brtc & 0xFF);
    WRITECSR(softc,R_UART_IER,brtc>>8);
    WRITECSR(softc,R_UART_CFCR,CFCR_8BITS);

#if !defined(NS16550_NO_FLOW)

#if !defined(_BCM94702_CPCI_)
    WRITECSR(softc,R_UART_MCR,MCR_DTR | MCR_RTS | MCR_IENABLE);
#endif
    WRITECSR(softc,R_UART_IER,0);

    WRITECSR(softc,R_UART_FIFO,FIFO_ENABLE);
    DELAY(100);
    WRITECSR(softc,R_UART_FIFO,
	     FIFO_ENABLE | FIFO_RCV_RST | FIFO_XMT_RST | FIFO_TRIGGER_1);
    DELAY(100);

    if ((READCSR(softc,R_UART_IIR) & IIR_FIFO_MASK) !=
	IIR_FIFO_MASK) {
	WRITECSR(softc,R_UART_FIFO,0);
    }
#endif /* !NS16550_NO_FLOW */
    
    ns16550_uart_setflow(softc);

    return 0;
}

static int ns16550_uart_read(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    ns16550_uart_t *softc = ctx->dev_softc;
    hsaddr_t bptr;
    int blen;
    uint8_t b;

    bptr = buffer->buf_ptr;
    blen = buffer->buf_length;

    while ((blen > 0) && (READCSR(softc,R_UART_LSR) & LSR_RXRDY)) {
	b = (READCSR(softc,R_UART_DATA) & 0xFF);
	hs_write8(bptr,b);
	bptr++;
	blen--;
	}

    buffer->buf_retlen = buffer->buf_length - blen;
    return 0;
}

static int ns16550_uart_inpstat(cfe_devctx_t *ctx, iocb_inpstat_t *inpstat)
{
    ns16550_uart_t *softc = ctx->dev_softc;

    inpstat->inp_status = (READCSR(softc,R_UART_LSR) & LSR_RXRDY) ? 1 : 0;

    return 0;
}

static int ns16550_uart_write(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    ns16550_uart_t *softc = ctx->dev_softc;
    hsaddr_t bptr;
    uint8_t b;
    int blen;

    bptr = buffer->buf_ptr;
    blen = buffer->buf_length;
    while ((blen > 0) && (READCSR(softc,R_UART_LSR) & LSR_TXRDY)) {
	b = hs_read8(bptr);
	bptr++;
	WRITECSR(softc,R_UART_DATA, b);
	blen--;
	}

    buffer->buf_retlen = buffer->buf_length - blen;
    return 0;
}

static int ns16550_uart_ioctl(cfe_devctx_t *ctx, iocb_buffer_t *buffer) 
{
    ns16550_uart_t *softc = ctx->dev_softc;
    unsigned int info;

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
	    ns16550_uart_setflow(softc);
	    break;
	default:
	    return -1;
	}

    return 0;
}

static int ns16550_uart_close(cfe_devctx_t *ctx)
{
    ns16550_uart_t *softc = ctx->dev_softc;

    WRITECSR(softc,R_UART_MCR,0);

    return 0;
}


