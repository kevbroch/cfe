/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PCMCIA IDE disk driver		      File: dev_sb1250_pcmcia.c
    *  
    *  This is the stub driver for the SB1250's PCMCIA interface.
    *  Note that we only support ATA flash cards.
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
#include "sbmips.h"

#include "dev_ide_common.h"

#include "dev_ide.h"

#include "sb1250_regs.h"
#include "bcm1480_regs.h"
#include "sb1250_genbus.h"

/*  *********************************************************************
    *  Constants
    ********************************************************************* */

#define PCMCIA_CARDPRESENT	1

/*  *********************************************************************
    *  Data
    ********************************************************************* */

typedef struct pcmciaslot_s {
    int slot;
    idecommon_t *idecommon;
    uint32_t cfgport;
    uint32_t statusport;
} pcmciaslot_t;

/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */

static void pcmcia_ata_probe(cfe_driver_t *drv,
			      unsigned long probe_a, unsigned long probe_b, 
			      void *probe_ptr);


static int pcmcia_ata_open(cfe_devctx_t *ctx);

/*  *********************************************************************
    *  Device Dispatch
    ********************************************************************* */

static cfe_devdisp_t pcmcia_ata_dispatch = {
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

const cfe_driver_t pcmciadrv = {
    "PCMCIA ATA disk",
    "pcmcia",
    CFE_DEV_DISK,
    &pcmcia_ata_dispatch,
    pcmcia_ata_probe
};


/*  *********************************************************************
    *  Port I/O routines
    *  
    *  These routines are called back from the common code to do
    *  I/O cycles to the IDE disk.  We provide routines for
    *  reading and writing bytes, words, and strings of words.
    ********************************************************************* */


static uint8_t pcmcia_inb(idecommon_dispatch_t *disp,uint32_t reg)
{
    return *((volatile uint8_t *) PHYS_TO_K1(reg+disp->baseaddr));
}

static uint16_t pcmcia_inw(idecommon_dispatch_t *disp,uint32_t reg)
{
    return *((volatile uint16_t *) PHYS_TO_K1((reg+disp->baseaddr)));
}

static void pcmcia_ins(idecommon_dispatch_t *disp,uint32_t reg,hsaddr_t buf,int len)
{
    uint16_t data;

    while (len > 0) {
	data = *((volatile uint16_t *) PHYS_TO_K1(reg+disp->baseaddr));
	hs_write16(buf,data);
	buf += sizeof(uint16_t);
	len--;
	len--;
	}

}

static void pcmcia_outb(idecommon_dispatch_t *disp,uint32_t reg,uint8_t val)
{
    *((volatile uint8_t *) PHYS_TO_K1(reg+disp->baseaddr)) = val;
}

static void pcmcia_outw(idecommon_dispatch_t *disp,uint32_t reg,uint16_t val)
{
    *((volatile uint16_t *) PHYS_TO_K1(reg+disp->baseaddr)) = val;
}

static void pcmcia_outs(idecommon_dispatch_t *disp,uint32_t reg,hsaddr_t buf,int len)
{
    uint16_t data;

    while (len > 0) {
	data = hs_read16(buf);
	buf += sizeof(uint16_t);
	*((volatile uint16_t *) PHYS_TO_K1(reg+disp->baseaddr)) = data;
	len--;
	len--;
	}
}


/*  *********************************************************************
    *  cardpresent()
    *  
    *  Returns TRUE if a PCMCIA card (any card) is present in the
    *  slot.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   true if card is present.
    ********************************************************************* */

static int cardpresent(pcmciaslot_t *slot)
{
    uint64_t status;

    status = SBREADCSR(slot->statusport);
    status &= M_PCMCIA_STATUS_CD1 | M_PCMCIA_STATUS_CD2;

    return (status == 0);
}

/*  *********************************************************************
    *  poweron()
    *  
    *  Turn on the power to the PCMCIA slot.  This routine sets the
    *  power into "automatic" mode.  We'll let the pins do the work.
    *  
    *  Input parameters: 
    *  	   slot - points at slot info
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void poweron(pcmciaslot_t *slot)
{
    uint64_t config, status;

    status = SBREADCSR(slot->statusport);	/* clear pending interrupt */
    config = SBREADCSR(slot->cfgport);
    config &= ~(M_PCMCIA_CFG_RESET | M_PCMCIA_CFG_APWRONEN);
    config &= ~(M_PCMCIA_CFG_3VEN | M_PCMCIA_CFG_5VEN);
    config |= M_PCMCIA_CFG_APWRONEN;
    SBWRITECSR(slot->cfgport,config);

}




/*  *********************************************************************
    *  pcmcia_ata_poll(x)
    *  
    *  Called periodically so we can check for card activity
    *  (report insertions, removals, etc.)
    *  
    *  Input parameters: 
    *  	   x - void param, it's our idecommon structure
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void pcmcia_ata_poll(void *x)
{
    pcmciaslot_t *slot = (pcmciaslot_t *) x;
    idecommon_t *softc = slot->idecommon;
    int present;
    int oldpresent;

    present = cardpresent(slot);
    oldpresent = (softc->idecommon_flags & PCMCIA_CARDPRESENT) ? 1 : 0;

    if (present == oldpresent) return;

    if (!TIMER_RUNNING(softc->timer)) {
	TIMER_SET(softc->timer,10);
	return;
	}
    else {
	if (!TIMER_EXPIRED(softc->timer)) return;
	    
	present = cardpresent(slot);

	if (present == oldpresent) return;

	if (present) {
	    console_log("PCMCIA%d: Card inserted",slot->slot);
	    softc->idecommon_flags |= PCMCIA_CARDPRESENT;
	    /* This just turns off the PCMCIA RESET */
	    poweron(slot);
	    }
	else {
	    console_log("PCMCIA%d: Card removed",slot->slot);
	    softc->idecommon_flags &= ~PCMCIA_CARDPRESENT;
	    }

	TIMER_CLEAR(softc->timer);	
	}
}

/*  *********************************************************************
    *  pcmcia_ata_probe(drv,probe_a,probe_b,probe_ptr)
    *  
    *  Our probe routine.  Attach a PCMCIA ATA device to the firmware.
    *  
    *  Input parameters: 
    *  	   drv - driver structure
    *  	   probe_a - physical address of IDE registers
    *  	   probe_b - unit number
    *  	   probe_ptr - not used
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void pcmcia_ata_probe(cfe_driver_t *drv,
			      unsigned long probe_a, unsigned long probe_b, 
			      void *probe_ptr)
{
    idecommon_t *softc;
    idecommon_dispatch_t *disp;
    pcmciaslot_t *slot;
    char descr[80];

    /* 
     * probe_a is the IDE base address
     * probe_b is the unit number and other flags
     * probe_ptr is unused.
     */

    softc = (idecommon_t *) KMALLOC(sizeof(idecommon_t),0);
    slot = (pcmciaslot_t *) KMALLOC(sizeof(pcmciaslot_t),0);
    disp = (idecommon_dispatch_t *) KMALLOC(sizeof(idecommon_dispatch_t),0);
    if (softc && disp && slot) {
	softc->idecommon_addr = probe_a;
	softc->idecommon_unit = 0;	/* PCMCIA slots always master */

	slot->slot = probe_b;

	/* The probe_b value says which slot we're looking at */
	slot->cfgport = probe_b ? A_BCM1480_IO_PCMCIA_CFG_B : A_IO_PCMCIA_CFG;
	slot->statusport = probe_b ? A_BCM1480_IO_PCMCIA_STATUS_B : A_IO_PCMCIA_STATUS;
	slot->idecommon = softc;

	disp->ref = softc;
	disp->baseaddr = softc->idecommon_addr;
	softc->idecommon_dispatch = disp;
	/* always defer probing till later */
	softc->idecommon_deferprobe = 1;

	disp->outb = pcmcia_outb;
	disp->outw = pcmcia_outw;
	disp->outs = pcmcia_outs;

	disp->inb = pcmcia_inb;
	disp->inw = pcmcia_inw;
	disp->ins = pcmcia_ins;

	disp->ref = slot;

	xsprintf(descr,"%s unit %d at %08X",drv->drv_description,probe_b,probe_a);
	idecommon_attach(&pcmcia_ata_dispatch);
	pcmcia_ata_dispatch.dev_open = pcmcia_ata_open;
	cfe_attach(drv,softc,NULL,descr);
	softc->idecommon_flags = cardpresent(slot) ? PCMCIA_CARDPRESENT : 0;
	cfe_bg_add(pcmcia_ata_poll,slot);
	poweron(slot);	/* turn on automatic power-on */
	}
}


/*  *********************************************************************
    *  pcmcia_ata_open(ctx)
    *  
    *  Open routine for the PCMCIA IDE device.  It just hooks in
    *  front of the standard IDE disk open processing so we can turn
    *  on the PCIC slot and test for the card's presence.
    *  
    *  Input parameters: 
    *  	   ctx - device context
    *  	   
    *  Return value:
    *  	   0 if ok, else error code
    ********************************************************************* */

static int pcmcia_ata_open(cfe_devctx_t *ctx)
{
    int res;
    idecommon_t *softc = ctx->dev_softc;
    pcmciaslot_t *slot = softc->idecommon_dispatch->ref;

    if (!cardpresent(slot)) return CFE_ERR_NOTREADY;

    res = idecommon_devprobe(softc,0);
    if (res < 0) {
	return res;
	}

    return idecommon_open(ctx);
}
