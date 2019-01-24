/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  LDT Fabric Initialization			File: ldtinit.c
    *  
    *********************************************************************  
    *
    *  Copyright 2001,2002,2003
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

#if CFG_LDT
/*
 * ldtinit.c: generic LDT fabric initialization and capability
 *            management.
 */

#include "lib_types.h"
#include "lib_printf.h"
#include "cfe_timer.h"

#include "pcivar.h"
#include "pci_internal.h"
#include "pcireg.h"
#include "ldtreg.h"

/* Write-to-clear bit masks */

#if CFG_LDT_REV_017    /* XXX not really the right test */
#define LDT_LINKCTRL_WC (LDT_LINKCTRL_CRCERROR_MASK)
#else
#define LDT_LINKCTRL_WC (LDT_LINKCTRL_LINKFAIL | LDT_LINKCTRL_CRCERROR_MASK)
#endif


/* LDT global variables. */

/* It is sometimes necessary to restrict links to less than their
   advertised capabilities.  For link frequencies, this is done by
   adjusting capability registers at the root of the chain.  For link
   widths, adjustments of the corresponding capabilities are not
   currently provided. */

static unsigned int max_width = LDT_WIDTH_32;   /* capability-encoded */

void
ldt_set_max_width (unsigned int width_cap)
{
    max_width = width_cap;
}


/* LDT capability lookup. */

unsigned
pci_find_ldt_cap (pcitag_t tag, int secondary)
{
    pcireg_t cpr;
    pcireg_t cr;
    int offset, prev;
    int type;

    cpr = pci_conf_read(tag, PCI_CAPLISTPTR_REG);
    offset = PCI_CAPLIST_PTR(cpr) &~ 0x3;
    prev = 0;
  
    while (offset != 0 && offset != prev) {
	cr = pci_conf_read(tag, offset);
	if (PCI_CAPLIST_CAP(cr) == PCI_CAP_LDT) {
	    type = LDT_COMMAND_TYPE(cr);
	    if (secondary && type == LDT_COMMAND_TYPE_HOST)
		return offset;
	    if (!secondary && type == LDT_COMMAND_TYPE_SLAVE)
		return offset;
	}
	prev = offset;
	offset = PCI_CAPLIST_NEXT(cr) &~ 0x3;
    }
    return 0;
}


/* LDT utility functions, mostly for capabilities. */

static pcireg_t
ldt_get_link(pcitag_t tag, int offset, int index)
{
    return pci_conf_read(tag, offset + LDT_LINKn_OFF(index));
}

static void
ldt_set_link(pcitag_t tag, int offset, int index, pcireg_t lr)
{
    pci_conf_write(tag, offset + LDT_LINKn_OFF(index), lr);
}

#if (LDT_DEBUG != 0)
static void
ldt_show_cap(pcitag_t tag, int offset, int secondary)
{
    printf(" Cmd %08x", pci_conf_read(tag, offset));
    offset += 4;
    printf(" Lnk0 %08x", pci_conf_read(tag, offset));
    offset += 4;
    if (!secondary) {
        printf(" Lnk1 %08x", pci_conf_read(tag, offset));
	offset += 4;
    }
    printf(" Freq0 %08x", pci_conf_read(tag, offset));
    offset += 4;
    if (!secondary) {
	printf(" Freq1 %08x", pci_conf_read(tag, offset));
	offset += 4;
    }
    printf("\n");
}
#else
static void
ldt_show_cap(pcitag_t tag, int offset, int secondary)
{
}
#endif


/* LDT bus initialization and sizing. */

/* We expect the entire chain to be ready at approximately the same
   time, but we add some delay here for possible node-to-node
   differences.

   Empirically, neither InitDone nor LinkFail is reported for an
   unconnected link.  Thus we do not expect the outgoing link of a
   terminating tunnel node to become ready.

   Also, CRC errors are observed to occur with InitDone, so link
   errors do not necessarily force LinkFail.
*/

static int
ldt_wait_ready (pcitag_t tag, int offset, int index)
{
    volatile int count;
    volatile pcireg_t lr;
    int linkerr;

    linkerr = 0;
    count = 0x10000;   /* empirical */
    do {
	if (--count == 0)
	    return 1;
        lr = ldt_get_link(tag, offset, index);
	if ((lr & (LDT_LINKCTRL_LINKFAIL | LDT_LINKCTRL_CRCERROR_MASK)) != 0)
	    linkerr = 1;
    } while ((lr & (LDT_LINKCTRL_INITDONE | LDT_LINKCTRL_LINKFAIL)) == 0);

    return linkerr;
}

static void
ldt_end_chain (pcitag_t tag, int offset, int index)
{
    pcireg_t lr, t;

    lr = ldt_get_link(tag, offset, index);
    lr |= LDT_LINKCTRL_EOC;
    ldt_set_link(tag, offset, index, lr);
    lr |= LDT_LINKCTRL_TXOFF;
    ldt_set_link(tag, offset, index, lr);
    t = ldt_get_link(tag, offset, index);  /* push */
}


static uint16_t
ldt_freq_cap (pcitag_t tag, int offset, int index)
{
    pcireg_t cmd, cr;
    uint16_t freq_cap;

    cmd = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
    if (LDT_COMMAND_TYPE(cmd) == LDT_COMMAND_TYPE_HOST) {
	cr = pci_conf_read(tag, offset + LDT_FREQ_OFF);
	if (LDT_REVISION_ID(cr) == LDT_REV_017) {
	    /* REV 0.17 has restricted support for setting
	       frequencies.  We assume that this is the host bridge in
	       pseudo-1.0x mode, in which case the desired maximum
	       frequency was left in the LDT_FREQ register and all
	       lower frequencies are supported. */
	    freq_cap = (1 << (LDT_LINKFREQ(cr) + 1)) - 1;
	} else {
	    freq_cap = LDT_LINKFREQ_CAP(cr);
	}
    } else {
	cr = pci_conf_read(tag, offset + LDT_FREQ0_OFF);
	if (LDT_REVISION_ID(cr) == LDT_REV_017) {
	    /* REV 0.17 has restricted support for setting frequencies.
	       This is not the host bridge.  What to do?  XXX */
	    freq_cap = (1 << LDT_FREQ_200);
	} else {
	    cr = pci_conf_read(tag, offset + LDT_FREQn_OFF(index));
	    freq_cap = LDT_LINKFREQ_CAP(cr);
	}
    }
    return freq_cap;
}

static uint8_t
ldt_max_freq (uint16_t freq_cap)
{
    unsigned ldt_freq;

    /* Consider only standardized frequencies. */
    freq_cap &= 0x3F;   /* XXX make symbolic */

    /* 200 MHz (encoded as 1 << 0) is required for all devices */
    freq_cap |= (1 << LDT_FREQ_200);

    ldt_freq = 0;

    while (freq_cap != 1) {
      ldt_freq++;
      freq_cap >>= 1;
    }

    return (ldt_freq >= LDT_FREQ_200 && ldt_freq <= LDT_FREQ_1000) ?
      ldt_freq : LDT_FREQ_200;
}

static void
ldt_set_freq (pcitag_t tag, int offset, int index, uint8_t freq)
{
    pcireg_t cmd, cr;

    cmd = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
    if (LDT_COMMAND_TYPE(cmd) == LDT_COMMAND_TYPE_HOST) {
	cr = pci_conf_read(tag, offset + LDT_FREQ_OFF);
	cr &=~ LDT_LINKFREQ_MASK;
	cr |= (freq << LDT_LINKFREQ_SHIFT);
	pci_conf_write(tag, offset + LDT_FREQ_OFF, cr);
    } else {
        cr = pci_conf_read(tag, offset + LDT_FREQn_OFF(index));
	cr &=~ LDT_LINKFREQ_MASK;
	cr |= (freq << LDT_LINKFREQ_SHIFT);
	pci_conf_write(tag, offset + LDT_FREQn_OFF(index), cr);
    }
#if (LDT_DEBUG != 0)
    pci_tagprintf(tag, "set link %d freq = %02x\n", index, freq);
#endif
}


static uint16_t
ldt_set_link_freq (pcitag_t tag, int offset, int link,
		   pcitag_t prev_tag, int prev_offset, int prev_link,
		   uint16_t prev_cap)
{
    uint16_t link_cap, freq_cap_in, freq_cap_out;
    uint8_t ldt_freq;

    /* Find common frequency for upstream link. */
    freq_cap_out = ldt_freq_cap(prev_tag, prev_offset, prev_link);
    freq_cap_in = ldt_freq_cap(tag, offset, link);
    link_cap = freq_cap_in & freq_cap_out & prev_cap;
    ldt_freq = ldt_max_freq(link_cap);

#if (LDT_DEBUG > 1)
    pci_tagprintf(tag, "set freq %02x\n", ldt_freq);
#endif
    /* Set up frequency registers, next warm reset installs. */
    ldt_set_freq(prev_tag, prev_offset, prev_link, ldt_freq);
    ldt_set_freq(tag, offset, link, ldt_freq);

    return link_cap;
}


/* A link width capability is represented as a pair of hex digits,
   (width_out, width_in), with widths encoded per LDT_WIDTH_*.  This
   matches the WIDTH and MAX_WIDTH fields used in LINKCFG fields. */

#define G_LINK_WIDTH_IN(cap)    (((cap) >> 0) & 0x7)
#define V_LINK_WIDTH_IN(v)      (((v) & 0x7) << 0)
#define G_LINK_WIDTH_OUT(cap)   (((cap) >> 4) & 0x7)
#define V_LINK_WIDTH_OUT(v)     (((v) & 0x7) << 4)

static unsigned int
ldt_link_width(uint8_t width_cap)
{
    static const uint8_t bit_width[8] = {8, 16, 0, 32, 2, 4, 0, 0};

    return bit_width[width_cap & 0x7];
}

static uint8_t
ldt_max_width(uint8_t width1, uint8_t width2)
{
    /* Find the maximum width supported by both ends of the link. */
    uint8_t width;

    if (width1 == width2)
	width = width1;
    else {
	unsigned int bits1, bits2;
	
	bits1 = ldt_link_width(width1);
	bits2 = ldt_link_width(width2);
	switch ((bits1 < bits2) ? bits1 : bits2) {
	    case  8:  width = LDT_WIDTH_8;    break;
	    case 16:  width = LDT_WIDTH_16;   break;
	    case 32:  width = LDT_WIDTH_32;   break;
	    case  2:  width = LDT_WIDTH_2;    break;
	    case  4:  width = LDT_WIDTH_4;    break;
	    default:  width = LDT_WIDTH_DISC; break;
	    }
	}
    return width;
}

static unsigned int
ldt_set_link_width (pcitag_t tag, int offset, int link,
		    pcitag_t prev_tag, int prev_offset, int prev_link)
{
    pcireg_t lcl_attr, rem_attr;
    uint8_t width_up, width_down;

    lcl_attr = ldt_get_link(tag, offset, link);
    rem_attr = ldt_get_link(prev_tag, prev_offset, prev_link);

    width_down = ldt_max_width(LDT_LINKCFG_MAX_WIDTH_OUT(rem_attr),
			       LDT_LINKCFG_MAX_WIDTH_IN(lcl_attr));
    width_up = ldt_max_width(LDT_LINKCFG_MAX_WIDTH_OUT(lcl_attr),
			     LDT_LINKCFG_MAX_WIDTH_IN(rem_attr));

    if (ldt_link_width(width_down) > ldt_link_width(max_width))
	width_down = max_width;
    if (ldt_link_width(width_up) > ldt_link_width(max_width))
	width_up = max_width;

    rem_attr &= ~LDT_LINKCFG_WIDTH_MASK;
    rem_attr |= (((V_LINK_WIDTH_IN(width_up) | V_LINK_WIDTH_OUT(width_down)))
		 << LDT_LINKCFG_WIDTH_SHIFT);
    ldt_set_link(prev_tag, prev_offset, prev_link, rem_attr);


    lcl_attr &= ~LDT_LINKCFG_WIDTH_MASK;
    lcl_attr |= (((V_LINK_WIDTH_IN(width_down) | V_LINK_WIDTH_OUT(width_up)))
		 << LDT_LINKCFG_WIDTH_SHIFT);
    ldt_set_link(tag, offset, link, lcl_attr);

    return 0;
}


/* LDT fabric initialization.  See LDT Spec, Section 13.3. */
static int
ldt_fabric_init (pcitag_t br_tag, int br_offset,
		 int port, int bus, pci_flags_t flags)
{
    int next_free_id;
    pcitag_t  prev_tag, tag;
    int offset, prev_offset;
    int link, prev_link;
    uint16_t prev_cap;
    pcireg_t  cmd, lr, id, t;
    int double_ended;
    int linkerr;

    prev_tag = br_tag;  prev_offset = br_offset;  prev_link = 0;
    /* Since there is no direct peer-to-peer traffic, there is no
       point in configuring a downstream link with more capability
       than the current one. */
    prev_cap = ldt_freq_cap(br_tag, br_offset, 0);

    /* For various reasons, including some chips that advertise link
       frequencies they can't support, the system can cap the maximum
       link frequency.  Currently, the desired maximum frequency is
       left in the LDT_FREQ register of the the bridge at the root of
       the chain. */
    cmd = pci_conf_read(br_tag, br_offset + LDT_COMMAND_CAP_OFF);
#ifndef _SB14XX_
    if (LDT_COMMAND_TYPE(cmd) == LDT_COMMAND_TYPE_HOST) {
	pcireg_t cr = pci_conf_read(br_tag, br_offset + LDT_FREQ_OFF);
	prev_cap &= (1 << (LDT_LINKFREQ(cr) + 1)) - 1;
    }
#else
    /* XXX 1.05 parts need a different mechanism for this. */
#endif

    next_free_id = 1;
    double_ended = 0;

#if (LDT_DEBUG != 0)
    printf("Link sizing for bus %d, bridge freq cap %04x\n",
	   bus, ldt_freq_cap(br_tag, br_offset, 0));
#endif
    for (;;) {

	tag = pci_make_tag(port, bus, 0, 0);

	id = pci_conf_read(tag, PCI_ID_REG);
#if (LDT_DEBUG > 1)
	pci_tagprintf(tag, "ldt_fabric_init: id register %08x\n", id);
#endif
	if (PCI_VENDOR(id) == 0xFFFF) {
	    /* The incoming link had InitDone set, but we got an NXA
               trying to read the vendor id.  Either the reverse link
               is broken or we have found an LDT-Lite node.  For now,
               assume the latter.  Since an LDT-Lite device cannot be
               a tunnel, assume the chain terminates here. */
	    pci_tagprintf(tag, "assumed LDT-LITE device (virtual unit %d)\n",
			  next_free_id);
	    break;
	}

	offset = pci_find_ldt_cap(tag, LDT_PRIMARY);
	if (offset == 0) {
	    /* There is no primary interface; we must have found a host. */
	    offset = pci_find_ldt_cap(tag, LDT_SECONDARY);
	    if (offset != 0) {
		lr = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
		lr |= LDT_COMMAND_DOUBLE_ENDED;
		pci_conf_write(tag, offset + LDT_COMMAND_CAP_OFF, lr);
		double_ended = 1;
		cmd = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
		link = LDT_COMMAND_MASTER_HOST(cmd);  /* Upstream link index */

		prev_cap = ldt_set_link_freq(tag, offset, link,
					     prev_tag, prev_offset, prev_link,
					     prev_cap);
		(void)ldt_set_link_width(tag, offset, link,
					 prev_tag, prev_offset, prev_link);
	    }
	    break;
	}

	/* Otherwise, we have the primary interface. */

	/* Rewrite the old value to set master host bit.  */
	cmd = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
	pci_conf_write(tag, offset + LDT_COMMAND_CAP_OFF, cmd);
	cmd = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);  /* push */

	id = pci_conf_read(tag, PCI_ID_REG);

	/* Update the unit id, gingerly. */
	cmd &= ~LDT_COMMAND_UNIT_ID_MASK;
	cmd |= (next_free_id << LDT_COMMAND_UNIT_ID_SHIFT);
#if (LDT_DEBUG != 0)
	pci_tagprintf(tag, "ldt_fabric_init: set unit id %d\n", next_free_id);
#endif
	if (!pci_conf_write_acked(tag, offset + LDT_COMMAND_CAP_OFF, cmd)) {
	    pci_tagprintf(tag, "no ack of id update to %d\n", next_free_id);
	}

	/* The unit id just changed.  Update the tag */
	tag = pci_make_tag(port, bus, next_free_id, 0);
	t = pci_conf_read(tag, PCI_ID_REG);
	if (t != id) {
	    pci_tagprintf(tag, "id mismatch: old %08x, new %08x\n", id, t);
	}

	next_free_id += LDT_COMMAND_UNIT_COUNT(cmd);

	link = LDT_COMMAND_MASTER_HOST(cmd);  /* Upstream link index */

	/* LDT Rev 0.17 does not support frequency updates. */
	if ((flags & PCI_FLG_LDT_REV_017) == 0) {
	    prev_cap = ldt_set_link_freq(tag, offset, link,
					 prev_tag, prev_offset, prev_link,
					 prev_cap);
	    (void)ldt_set_link_width(tag, offset, link,
				     prev_tag, prev_offset, prev_link);
	}

	link ^= 1;                            /* Downstream */
	linkerr = ldt_wait_ready(tag, offset, link);
	lr = ldt_get_link(tag, offset, link);
	ldt_set_link(tag, offset, link, lr | LDT_LINKCTRL_WC);

#if (LDT_DEBUG != 0)
	pci_tagprintf(tag, "node: up %d down %d:\n", link ^ 1, link);
#endif
	ldt_show_cap(tag, offset, LDT_PRIMARY);

	if (linkerr || next_free_id > 0x1F) {
	    /* No downstream link or too many devices, set end of chain */
	    ldt_end_chain(tag, offset, link);
	    break;
	}
	
	prev_tag = tag;  prev_offset = offset;  prev_link = link;
    }

    return double_ended;
}


static int
ldt_fabric_reinit (int port, int bus)
{
    int next_free_id;
    pcitag_t  tag;
    int offset;
    int link;
    pcireg_t  cmd, lr, id, t;
    int linkerr;

    next_free_id = 1;

#if (LDT_DEBUG != 0)
    printf("Link resizing for bus %d\n", bus);
#endif
    for (;;) {

	tag = pci_make_tag(port, bus, 0, 0);

	id = pci_conf_read(tag, PCI_ID_REG);
	if (PCI_VENDOR(id) == 0xFFFF) {
	    /* Per the init pass, assume this indicates a link to an
               LDT-Lite node, and the chain terminates here. */
	    break;
	}

	offset = pci_find_ldt_cap(tag, LDT_PRIMARY);
	if (offset == 0) {
	    /* There is no primary interface; we must have found a host. */
	    offset = pci_find_ldt_cap(tag, LDT_SECONDARY);
	    if (offset != 0) {
		lr = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
		lr |= LDT_COMMAND_DOUBLE_ENDED;
		pci_conf_write(tag, offset + LDT_COMMAND_CAP_OFF, lr);
	    }
	    break;
	}

	/* Otherwise, we have the primary interface. */

	/* Rewrite the old value to set master host bit.  */
	cmd = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
	pci_conf_write(tag, offset + LDT_COMMAND_CAP_OFF, cmd);
	cmd = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);

	id = pci_conf_read(tag, PCI_ID_REG);

	/* (Re)update the unit id.  See above. */
	cmd &= ~LDT_COMMAND_UNIT_ID_MASK;
	cmd |= (next_free_id << LDT_COMMAND_UNIT_ID_SHIFT);
	if (!pci_conf_write_acked(tag, offset + LDT_COMMAND_CAP_OFF, cmd)) {
	    pci_tagprintf(tag, "no ack of id update to %d\n", next_free_id);
	}

	/* The unit id just changed.  Update the tag */
	tag = pci_make_tag(port, bus, next_free_id, 0);
	t = pci_conf_read(tag, PCI_ID_REG);   /* for good measure */
	if (t != id) {
	    pci_tagprintf(tag, "id mismatch: old %08x, new %08x\n", id, t);
	}

	next_free_id += LDT_COMMAND_UNIT_COUNT(cmd);

	link = LDT_COMMAND_MASTER_HOST(cmd);  /* Upstream link index */
	link ^= 1;                            /* Downstream */

	linkerr = ldt_wait_ready(tag, offset, link);

	lr = ldt_get_link(tag, offset, link);
	ldt_set_link(tag, offset, link, lr | LDT_LINKCTRL_WC);
#if (LDT_DEBUG > 1)
	pci_tagprintf(tag, "node: up %d down %d:\n", link ^ 1, link);
	ldt_show_cap(tag, offset, LDT_PRIMARY);
#endif
	if (linkerr || next_free_id > 0x1F) {
	    /* No downstream link or too many devices, set end of chain */
	    ldt_end_chain(tag, offset, link);
	    break;
	}
    }
    return next_free_id - 1;
}


/* LDT link reset (warm or cold as set up by caller) */

void
ldt_link_reset (pcitag_t tag, int delay)
{
    pcireg_t brctl;

    /* This code is necessary for doing a warm reset as part of the HT
       1.0x link sizing.  Cold resets may be useful for LDT buses
       behind bridges (none of which yet exist) but seem to be a bad
       idea for the SB-1250 LDT bus in pass 1 parts. */

    /* Attempt a Secondary Bus Reset. */
    brctl = pci_conf_read(tag, PPB_BRCTL_INTERRUPT_REG);
    brctl |= PPB_BRCTL_SECONDARY_RESET;
    pci_conf_write(tag, PPB_BRCTL_INTERRUPT_REG, brctl);

    brctl = pci_conf_read(tag, PPB_BRCTL_INTERRUPT_REG);
    if ((brctl & PPB_BRCTL_SECONDARY_RESET) != 0) {
	int  i;
	/* Bit can be written, assume soft reset is implemented. */
	brctl &=~ PPB_BRCTL_SECONDARY_RESET;
	pci_conf_write(tag, PPB_BRCTL_INTERRUPT_REG, brctl);

	/* Add some delay (duration is a guess) */
	for (i = 0; i < delay; i++)
	    (void)pci_conf_read(tag, PPB_BRCTL_INTERRUPT_REG);
	/* Alternatively, wait for LinkFail or InitDone.  */
    }
}


/* LDT bridge and fabric initialization for a secondary chain */

int
ldt_chain_init (pcitag_t tag, int port, int bus, pci_flags_t flags)
{
    int  offset;
    int  double_ended;
    int  linkerr;
    pcireg_t cr, lr;
    int  ndev, no_probe;

#if 0
    /* This code may be necessary for LDT buses behind bridges (none
       of which yet exist) but seems to be a bad idea for the SB-1250
       LDT bus in pass 1 parts. Note that if we do reset, we must
       delay to give any attached devices a chance to (re)initialize
       per the PCI spec. */

    /* Attempt a Secondary Bus Reset. */
    ldt_link_reset(tag, 100);
#endif /* 0 */

    /* To avoid a chip erratum, we must prevent Type 0 configuration
       probes that get NXAs on a double hosted chain.  */
    no_probe = 0;

    offset = pci_find_ldt_cap(tag, LDT_SECONDARY);
    if (offset != 0) {
        linkerr = ldt_wait_ready(tag, offset, 0);

#if (LDT_DEBUG > 1)
	pci_tagprintf(tag, "bridge secondary:\n");
	ldt_show_cap(tag, offset, LDT_SECONDARY);
#endif
	if (linkerr) {
	    pci_tagprintf(tag, "secondary bad or never ready\n");
	    ldt_end_chain(tag, offset, 0);
	} else {
	    lr = ldt_get_link(tag, offset, 0);
	    if ((lr & LDT_LINKCTRL_INITDONE) != 0)
	        double_ended = ldt_fabric_init(tag, offset, port, bus, flags);
	    else {
		ldt_end_chain(tag, offset, 0);
		double_ended = 0;
	    }
	    cr = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
	    if (double_ended)
		cr |= LDT_COMMAND_DOUBLE_ENDED;
	    else
		cr &=~ LDT_COMMAND_DOUBLE_ENDED;
	    pci_conf_write(tag, offset + LDT_COMMAND_CAP_OFF, cr);

	    /* Rev 0.17 does not support dynamic link resizing. */
	    if ((flags & PCI_FLG_LDT_REV_017) == 0) {
		/* Issue a warm reset to update link frequencies. */
#if (LDT_DEBUG > 1)
	        pci_tagprintf(tag, "warm reset, bus %d\n", bus);
#endif
		cr = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
		cr |= LDT_COMMAND_WARM_RESET;
		pci_conf_write(tag, offset + LDT_COMMAND_CAP_OFF, cr);
		cr = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
		ldt_link_reset(tag, 100);
		ldt_wait_ready(tag, offset, 0);

		/* After reset, let secondary devices reinitialize. */
		cfe_sleep(CFE_HZ/2);

		ndev = ldt_fabric_reinit(port, bus);

		if (double_ended) {
		    cr = pci_conf_read(tag, offset + LDT_COMMAND_CAP_OFF);
		    cr |= LDT_COMMAND_DOUBLE_ENDED;
		    pci_conf_write(tag, offset + LDT_COMMAND_CAP_OFF, cr);

		    /* Bug workaround -- don't scan simple dual-hosted chain */
		    if (ndev == 0)
			no_probe = 1;
		}
	    }
	}
    }

    return no_probe;
}
#endif /* CFG_LDT */
