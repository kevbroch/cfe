/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  BCM1400-specific PCI support		File: bcm1480_pci_machdep.c
    *  
    *********************************************************************  
    *
    *  Copyright 2001,2002,2003,2004,2005
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

/*
 * Based in part on the algor p5064 version of pci_machdep.c,
 * from SiByte sources dated 20000824, which was distributed with
 * the copyright and license notices below:
 */

/* Very loosely based on: */
/*	$NetBSD: pci_machdep.c,v 1.17 1995/07/27 21:39:59 cgd Exp $	*/

/*
 * Copyright (c) 1994 Charles Hannum.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Charles Hannum.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * BCM1280/BCM1480 machine-specific functions for PCI autoconfiguration.
 */

#include "cfe.h"
#include "sbmips.h"
#include "bcm1480_regs.h"
#include "bcm1480_pci.h"
#include "bcm1480_ht.h"
#include "bcm1480_hsp.h"
#include "bcm1480_scd.h"
#include "lib_physio.h"
#include "env_subr.h"

#include "pcivar.h"
#include "pci_internal.h"
#include "pcireg.h"
#include "ldtreg.h"

#include "bcm1480_hsp_utils.h"


#ifndef CONFIG_HT
#define CONFIG_HT   1
#endif

const cons_t pci_optnames[] = {
    {"verbose",PCI_FLG_VERBOSE},
    {NULL,0}
};

extern int _pciverbose;


/* PCI regions in system physical (ZBbus) space.  See Figure 17. */

#define PCI_PORTMAX   (PCI_HOST_PORTS-1)

struct host_port {
    /* ZBbus space allocated for mapping to the standard PCI address spaces */
    uint32_t mem_space;
    uint32_t mem_space_size;
    uint32_t io_space;
    uint32_t io_space_size;
    uint32_t cfg_space;
    uint32_t cfg_space_size;

    /* PCI space available for configuration */
    uint32_t pci_mem_base;
    uint32_t pci_io_base;

    /* Bits for endian policy (0: match bytes, 1: match bits) */
    uint32_t mem_bit_endian;
    uint32_t io_bit_endian;
    uint32_t cfg_bit_endian;

    /* Match bits base for configuration (convenience variable) */
    physaddr_t cfg_base;
};

static struct host_port P[PCI_HOST_PORTS];
static int port_enabled[PCI_HOST_PORTS];

/* The current bus space */
static int               root;

static struct host_port *
pci_select_root (int bus_space)
{
    struct host_port *p;

    switch (bus_space) {
	case 0:    /* PCI interface (Figure 17) */
	    root = 0;
	    p = &P[0];

	    p->mem_space = A_BCM1480_PHYS_PCI_MEM_MATCH_BYTES;  /* 0x0030000000 */
	    p->mem_space_size = 0x0010000000;
	    p->io_space = A_BCM1480_PHYS_PCI_IO_MATCH_BYTES;    /* 0x002C000000 */
	    p->io_space_size = 0x0002000000;
	    p->cfg_space = A_BCM1480_PHYS_PCI_CFG_MATCH_BYTES;  /* 0x002E000000 */
	    p->cfg_space_size = 0x0001000000;

	    p->pci_mem_base = p->mem_space;             /* now transparent */
	    p->pci_io_base = 0x00000000;

	    p->mem_bit_endian = 0x0080000000;           /* now all the same */
	    p->io_bit_endian  = 0x0080000000;
	    p->cfg_bit_endian = 0x0080000000;

	    break;

	case 1:    /* HT interface (Figure 26) */
	    root = 1;
	    p = &P[1];

	    p->mem_space = A_BCM1480_PHYS_HT_MEM_MATCH_BYTES;   /* 0x0040000000 */
	    p->mem_space_size = 0x0020000000;
	    p->io_space = A_BCM1480_PHYS_HT_IO_MATCH_BYTES;     /* 0x00DC000000 */
	    p->io_space_size = 0x0020000000;
	    p->cfg_space = A_BCM1480_PHYS_HT_CFG_MATCH_BYTES;   /* 0x00DE000000 */
	    p->cfg_space_size = 0x0001000000;

	    p->pci_mem_base = p->mem_space;             /* transparent */
	    p->pci_io_base = 0x00000000;

	    p->mem_bit_endian = 0x0020000000;
	    p->io_bit_endian  = 0x0020000000;
	    p->cfg_bit_endian = 0x0020000000;

	    break;
	default:
	    return NULL;
    }

    p->cfg_base = PHYS_TO_XKSEG_UNCACHED(p->cfg_space | p->cfg_bit_endian);
    return p;
}


/* Templates for bus attributes. */

static const struct pci_bus bcm1400_pci_bus = {
	0,		/* minimum grant */
	255,		/* maximum latency */
	1,		/* devsel time = medium */
	1,		/* we support fast back-to-back */
	1,		/* we support prefetch */
	1,		/* we support 66 MHz */
	1,		/* we support 64 bit addressing and DAC */
	4000000,	/* bandwidth: in 0.25us cycles / sec */
	0,		/* initially no devices on bus */
};

static const struct pci_bus secondary_pci_bus = {
	0,		/* minimum grant */
	255,		/* maximum latency */
	0,		/* devsel time = unknown */
	0,		/* configure fast back-to-back */
	0,		/* we don't prefetch */
	0,		/* configure 66 MHz */
	0,		/* we don't support 64 bits */
	4000000,	/* bandwidth: in 0.25us cycles / sec */
	0,		/* initially no devices on bus */
};

/* Template for PCI BAR0 Map table configuration */
/* This maps 16MB region at address 0x0 to BAR0 */
static const struct bcm1480_inbw_conf bcm1480_bar0_conf = {
   0x0,       /* Base address(Physical) of the memory region to be mapped at BAR0 */
   
   0x0,       /* Offset from the Base address - Start of the region */

   0x1000000, /* Length of the region */

   0,         /* L2CA flag */

   0         /* Endian flag */
}; 

#define MAXBUS	10
static struct pci_bus _pci_bus[MAXBUS];
static int _pci_nbus = 0;

#define	BCM1400_PCI_MAKE_TAG(p,b,d,f)					\
    (((p) << 24) | ((b) << 16) | ((d) << 11) | ((f) << 8))
#define BCM1400_HOST_PORT(tag) (((tag) >> 24) & 0xFF)

#if defined(__MIPSEB)
/* This is for big-endian with a match bits policy. */
#define	BCM1400_CFG_ADDR(t, o, w)					\
   ((P[BCM1400_HOST_PORT(t)].cfg_base + ((t) & 0xFFFF00) + (o)) ^ (4 - (w)))
#elif defined(__MIPSEL)
/* This is for little-endian, either policy. */
#define	BCM1400_CFG_ADDR(t, o, w)					\
    (P[BCM1400_HOST_PORT(t)].cfg_base + ((t) & 0xFFFF00) + (o))
#else
#error "Must specifiy either MIPSEL or MIPSEB"
#endif

pcireg_t  pci_conf_read8(pcitag_t, int);
void	  pci_conf_write8(pcitag_t, int, pcireg_t);
#ifndef pci_conf_read32
#define pci_conf_read32  pci_conf_read
#endif
#ifndef pci_conf_write32
#define pci_conf_write32 pci_conf_write
#endif


/* Access functions */

/* The following should return the index of the last usable bus port
   (less than the maximum for the bcm1255, bcm1455, etc.) */
int
pci_maxport (void)
{
    uint64_t sysrev;

    sysrev = SBREADCSR(A_SCD_SYSTEM_REVISION);
    return ((G_SYS_PART(sysrev) & 0xF) == 0x6 ? 2 : 1) - 1;
}


/* The following must either fail or return the next sequential bus
   number to make secondary/subordinate numbering work correctly. */
int
pci_nextbus (int port)
{
    int bus = _pci_nbus;

    if (bus >= MAXBUS)
	return -1;
    _pci_nbus++;
    return bus;
}
  
int
pci_maxbus (int port)
{
    return _pci_nbus - 1;
}

struct pci_bus *
pci_businfo (int port, int bus)
{
    return (bus < _pci_nbus ? &_pci_bus[bus] : NULL);
}


/*
 * PCI address resources.
 * NB: initial limits for address allocation are assumed to be aligned
 * appropriately for PCI bridges (4K boundaries for I/O, 1M for memory).
 */

pcireg_t
pci_minmemaddr (int port)
{
    /* skip the 16MB reserved for ISA mem space */
    return P[port].pci_mem_base + 0x1000000;
}

pcireg_t
pci_maxmemaddr (int port)
{
    return P[port].pci_mem_base + P[port].mem_space_size;
}

pcireg_t
pci_minioaddr (int port)
{
    /* Skip the 32KB reserved for ISA i/o space. */
    return P[port].pci_io_base + 0x8000;
}

pcireg_t
pci_maxioaddr (int port)
{
    return P[port].pci_io_base + P[port].io_space_size;
}


/* The BCM1400 integrated host bridges. */

#define PCI_VENDOR_SIBYTE       0x166d
#define PCI_DEVICE_HTB_PRIM     0x0010
#define PCI_DEVICE_HTB_SEC      0x0011    

#define	BCM1400_PCI_BRIDGE	(BCM1400_PCI_MAKE_TAG(0,0,0,0))
#define	BCM1400_LDT_BRIDGE	(BCM1400_PCI_MAKE_TAG(1,0,4,0))

/* The BCM1400 integrated external PCI/HT bridges. */

#define BCM1400_EXT_BRIDGE(p)   (BCM1400_PCI_MAKE_TAG(1,0,(p),0))

static int bcm1400_in_device_mode;
static int bcm1400_ldt_slave_mode[BCM1480_HT_NUM_PORTS];

int eoi_implemented;    /* vestigal 1250 */


/* Convenience definitions for the PCI Host Bridge (PHB) */

#define M_BCM1480_PCI_RST_ASSERTS       (M_BCM1480_PCI_PERR_RST_ASSERT   | \
                                         M_BCM1480_PCI_DEVSEL_RST_ASSERT | \
                                         M_BCM1480_PCI_STOP_RST_ASSERT   | \
                                         M_BCM1480_PCI_TRDY_RST_ASSERT)


#define M_BCM1480_PCI_RST_STATS         (M_BCM1480_PCI_PERR_RST_STATUS   | \
                                         M_BCM1480_PCI_DEVSEL_RST_STATUS | \
                                         M_BCM1480_PCI_STOP_RST_STATUS   | \
                                         M_BCM1480_PCI_TRDY_RST_STATUS)

#define M_BCM1480_PHB_FCTRL_MEM_EN	(M_BCM1480_PHB_FCTRL_LOW_MEM_EN   | \
                                         M_BCM1480_PHB_FCTRL_UPPER_MEM_EN | \
                                         M_BCM1480_PHB_FCTRL_EXP_MEM_EN)

#define V_BCM1480_PHB_READHOST_DISABLE   0
#define V_BCM1480_PHB_READHOST_ENABLE    M_BCM1480_PHB_EXT_CONFIG_DIS


/*
 * PCI-X host bridge configuration
 */
static int
phb_host_init (void)
{
    uint64_t reset_ctl, reset_stat;
    uint64_t dll_ctl;
    enum {pci33, pci66, pcix66, pcix133} mode;
    enum {pci_bus, pcix_bus} protocol;
    unsigned int clock;
    int i;
    pcireg_t csr, icr;
    pcireg_t id;
    pcireg_t t;            /* used for reads that push writes */

    /* PCI: reset the busses rooted in this host bridge */
    reset_ctl = SBREADCSR(A_BCM1480_PCI_RESET);
    if ((reset_ctl & M_BCM1480_PCI_RESET_PIN) == 0) {
	reset_ctl |= M_BCM1480_PCI_RESET_PIN;
	SBWRITECSR(A_BCM1480_PCI_RESET, reset_ctl);
	cfe_usleep(100);
	}

    pci_clock_reset();

    /* PCI: adjust DLL parameters (XXX per designer, revisit on new revs?). */
    dll_ctl = SBREADCSR(A_BCM1480_PCI_DLL);
    dll_ctl &= ~M_BCM1480_PCI_DLL_STEP_SIZE;
    dll_ctl |= V_BCM1480_PCI_DLL_STEP_SIZE(0xF);
    SBWRITECSR(A_BCM1480_PCI_DLL, dll_ctl);
    cfe_usleep(100);

    reset_ctl &= ~M_BCM1480_PCI_PCIXCAP_PULLUP;
    SBWRITECSR(A_BCM1480_PCI_RESET, reset_ctl);
    cfe_usleep(100);
    reset_stat = SBREADCSR(A_BCM1480_PCI_RESET);
    if ((reset_stat & M_BCM1480_PCI_PCIXCAP_STATUS) != 0) {
	mode = pcix133;
	protocol = pcix_bus;
	}
    else {
	reset_ctl |= M_BCM1480_PCI_PCIXCAP_PULLUP;
	SBWRITECSR(A_BCM1480_PCI_RESET, reset_ctl);
	cfe_usleep(100);
	reset_stat = SBREADCSR(A_BCM1480_PCI_RESET);
	if ((reset_stat & M_BCM1480_PCI_PCIXCAP_STATUS) != 0) {
	    mode = pcix66;
	    protocol = pcix_bus;
	    }
	else {
	    protocol = pci_bus;
	    if ((reset_stat & M_BCM1480_PCI_M66EN_STATUS) != 0)
		mode = pci66;
	    else
		mode = pci33;
	    }
	reset_ctl &= ~M_BCM1480_PCI_PCIXCAP_PULLUP;
	SBWRITECSR(A_BCM1480_PCI_RESET, reset_ctl);
	}

    /* Choose the clock speed. */
    reset_ctl &= ~M_BCM1480_PCI_RST_ASSERTS;

    switch (mode) {
	case pci33:  default:
	    clock = pci_clock_select(33);
	    break;
	case pci66:
	    clock = pci_clock_select(66);
	    break;
	case pcix66:
	    reset_ctl |= M_BCM1480_PCI_STOP_RST_ASSERT;
	    clock = pci_clock_select(66);
	    break;
	case pcix133:
	    reset_ctl |= (M_BCM1480_PCI_TRDY_RST_ASSERT | 
			  M_BCM1480_PCI_STOP_RST_ASSERT);
	    clock = pci_clock_select(133);
	    break;
	}

    /* The bcm1480 et al. require an externally supplied PCI-X clock.
       In early revs of these chips, touching certain registers of the
       host bridge will hang if there is no clock.  The board-specific
       pci_clock_select should return 0 if no clock is supplied.  */
    if (clock == 0) {
	pci_tagprintf(BCM1400_PCI_BRIDGE, "PCI/PCI-X not configured\n");
	return -1;
	}
    pci_tagprintf(BCM1400_PCI_BRIDGE, "configuring bus for %dMHz PCI%s\n",
		  clock, (protocol == pcix_bus ? "-X" : ""));

    pci_clock_enable(1);
    cfe_usleep(1000);        /* Let clocks stabilize. */

    SBWRITECSR(A_BCM1480_PCI_RESET, reset_ctl);
    cfe_usleep(1000);        /* Set up Init Pattern */
    pci_clock_enable(1);
    cfe_usleep(1000);        /* Let clocks stabilize. */

    /* Release PCI reset */
    reset_ctl &= ~M_BCM1480_PCI_RESET_PIN;
    SBWRITECSR(A_BCM1480_PCI_RESET, reset_ctl);

    for (i = 1000; i > 0; i--) {
	reset_stat = SBREADCSR(A_BCM1480_PCI_RESET);
	if ((reset_stat & 0x3) == 0)
	    break;
	cfe_usleep(100);
	}
    if (i == 0)
	pci_tagprintf(BCM1400_PCI_BRIDGE, "PCI reset failed to clear\n");

    /* Delay here to guarantee 0.5 sec device initialization time. */
    cfe_sleep(CFE_HZ/2);
    
    /* The ResetIntr bit is always set here; clear it. */
    SBWRITECSR(A_BCM1480_PCI_RESET, reset_ctl | M_BCM1480_PCI_RESET_INTR);

#if (PCI_DEBUG != 0)
    pci_tagprintf(BCM1400_PCI_BRIDGE, "post-reset %08llx\n",
		  SBREADCSR(A_BCM1480_PCI_RESET) & 0xFFFFFFFF);
#endif

    /* PCI: disable and clear the BAR0 MAP registers */
    for (i = 0; i < BCM1480_PHB_MAPENTRIES; i++)
        pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_MAPBASE + 4*i, 0);

    /* PCI: set feature register to its default. */
    pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_FCTRL,
		     M_BCM1480_PHB_FCTRL_MEM_EN);

    /* PCI: enable bridge to PCI and PCI memory accesses, including
       write-invalidate, plus error handling */
    csr = PCI_COMMAND_MASTER_ENABLE | PCI_COMMAND_MEM_ENABLE |
          PCI_COMMAND_INVALIDATE_ENABLE |
          PCI_COMMAND_SERR_ENABLE |  PCI_COMMAND_PARITY_ENABLE;
    pci_conf_write32(BCM1400_PCI_BRIDGE, PCI_COMMAND_STATUS_REG, csr);

    /* PCI: clear errors */
    csr = pci_conf_read32(BCM1400_PCI_BRIDGE, PCI_COMMAND_STATUS_REG);
    csr |= PCI_STATUS_PARITY_ERROR | PCI_STATUS_SYSTEM_ERROR |
           PCI_STATUS_MASTER_ABORT | PCI_STATUS_MASTER_TARGET_ABORT |
           PCI_STATUS_TARGET_TARGET_ABORT | PCI_STATUS_PARITY_DETECT;
    pci_conf_write32(BCM1400_PCI_BRIDGE, PCI_COMMAND_STATUS_REG, csr);

    /* PCI: set up interrupt mapping */
    icr = pci_conf_read(BCM1400_PCI_BRIDGE, PCI_BPARAM_INTERRUPT_REG);
    icr &=~ (PCI_INTERRUPT_LINE_MASK << PCI_INTERRUPT_LINE_SHIFT);
    icr |= (pci_int_line(pci_int_map_0(BCM1400_PCI_BRIDGE))
	    << PCI_INTERRUPT_LINE_SHIFT);
    pci_conf_write(BCM1400_PCI_BRIDGE, PCI_BPARAM_INTERRUPT_REG, icr);

#if 1 /* XXX apparent work-around for bcm582x DMA */
    t = pci_conf_read(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_XACTCTRL);
    t &= ~(M_BCM1480_PHB_XACT_INB_RD_MAX_PREF |
	   M_BCM1480_PHB_XACT_INB_RD_LN_MAX_PREF |
	   M_BCM1480_PHB_XACT_INB_RD_MUL_MAX_PREF);
    t |= (V_BCM1480_PHB_XACT_INB_RD_MAX_PREF(2) |
	  V_BCM1480_PHB_XACT_INB_RD_LN_MAX_PREF(2) |
	  V_BCM1480_PHB_XACT_INB_RD_MUL_MAX_PREF(4));
    pci_conf_write(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_XACTCTRL, t);
#endif

    /* PCI: push the writes */
    t = pci_conf_read32(BCM1400_PCI_BRIDGE, PCI_ID_REG);

    /* PCI: set subsystem id */
    id = (PCI_VENDOR_SIBYTE << PCI_VENDOR_SHIFT)|(0x1280 << PCI_PRODUCT_SHIFT);
    pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_SUBSYSSET, id);

    /* PCI: enable bus-side access */
    pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_EXTCONFIGDIS,
		     V_BCM1480_PHB_READHOST_DISABLE);
    t = pci_conf_read32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_EXTCONFIGDIS);

    return 0;
}


static int
phb_device_init (void)
{
    pcireg_t id;
    pcireg_t t;
    int i;
    uint64_t reset_ctl, dll_ctl;
    int pcix;
    unsigned int clock;

    /* XXX what else needs to be done here? */

    /* PCI: adjust DLL parameters (per Ning). */
    dll_ctl = SBREADCSR(A_BCM1480_PCI_DLL);
    dll_ctl &= ~M_BCM1480_PCI_DLL_STEP_SIZE;
    dll_ctl |= V_BCM1480_PCI_DLL_STEP_SIZE(0xF);
    SBWRITECSR(A_BCM1480_PCI_DLL, dll_ctl);
    cfe_usleep(100);

    /* PCI: wait for the host to deassert reset. */
    for (i = 100000; i > 0; i--) {
	reset_ctl = SBREADCSR(A_BCM1480_PCI_RESET);
	if ((reset_ctl & M_BCM1480_PCI_RESET_PIN) == 0)
	    break;
	cfe_usleep(100);
	}
    if (i == 0) {
	pci_tagprintf(BCM1400_PCI_BRIDGE, "PCI/PCI-X reset failed to clear\n");
	return -1;
	}
    /* To get here, the system host must have released reset. */
    /* Clear PCI reset interrupt */
    SBWRITECSR(A_BCM1480_PCI_RESET, M_BCM1480_PCI_RESET_INTR);
    /* PCI: set subsystem id */
    id = (PCI_VENDOR_SIBYTE << PCI_VENDOR_SHIFT)|(0x1280 << PCI_PRODUCT_SHIFT);
    pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_SUBSYSSET, id);

    /* PCI: disable and clear the BAR0 MAP registers */
    for (i = 0; i < BCM1480_PHB_MAPENTRIES; i++)
        pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_MAPBASE + 4*i, 0);

    /* PCI: disable and clear the Outbound MAP registers */
    for (i = 0; i < BCM1480_PHB_OMAPENTRIES; i++) {
        pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_OMAP(i), 0);
        pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_OMAP(i)+4, 0);
    }

    /* PCI: set feature register for device mode. */
    pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_FCTRL,
		     M_BCM1480_PHB_FCTRL_UPPER_MEM_TR);

    /* PCI: Setup BAR0 MAP registers */
    pci_map_window(bcm1480_bar0_conf.pa, bcm1480_bar0_conf.offset, bcm1480_bar0_conf.len,
	       bcm1480_bar0_conf.l2ca, bcm1480_bar0_conf.endian);

    /* PCI: determine bus mode and speed. */
    reset_ctl = SBREADCSR(A_BCM1480_PCI_RESET);

    /* PCI: enable bus-side access */
    pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_EXTCONFIGDIS,
		     V_BCM1480_PHB_READHOST_DISABLE);
    t = pci_conf_read32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_EXTCONFIGDIS);

#if (PCI_DEBUG != 0)
    pci_tagprintf(BCM1400_PCI_BRIDGE, "post-reset %08llx\n",
		  reset_ctl & 0xFFFFFFFF);
#endif
    switch (reset_ctl & M_BCM1480_PCI_RST_STATS) {
	case M_BCM1480_PCI_TRDY_RST_STATUS:
	    pcix = 1;
	    clock = 66;
	    break;
	case M_BCM1480_PCI_STOP_RST_STATUS:
	    pcix = 1;
	    clock = 100;
	    break;
	case (M_BCM1480_PCI_STOP_RST_STATUS | M_BCM1480_PCI_TRDY_RST_STATUS):
	    pcix = 1;
	    clock = 133;
	    break;
	case 0:  default:
	    pcix = 0;
	    clock = ((reset_ctl & M_BCM1480_PCI_M66EN_STATUS) ? 66 : 33);
	    break;
	}
    pci_tagprintf(BCM1400_PCI_BRIDGE, "bus configured for %dMHz PCI%s\n",
		  clock, (pcix ? "-X" : ""));

    return 0;
}


/*
 * HT port initialization.  See Section 13, pp. 479-480.
 */

/* Implementation-specific definitions for the HT Host Bridge (HTD)
   Note that this is a bridge to the internal bus and logically
   appears as a PCI Host Bridge (no HT capabilities) */

#define M_BCM1480_HTD_ALL_PORTS(bit) (V_BCM1480_HTD_PORTCTRL_PORT0(bit) | \
                                      V_BCM1480_HTD_PORTCTRL_PORT1(bit) | \
                                      V_BCM1480_HTD_PORTCTRL_PORT2(bit))

#define M_BCM1480_HTD_PORTS_ACTIVE   (M_BCM1480_HTD_ALL_PORTS(M_BCM1480_HTD_PORT_ACTIVE))
#define M_BCM1480_HTD_PORTS_PRIMARY  (M_BCM1480_HTD_ALL_PORTS(M_BCM1480_HTD_PORT_IS_PRIMARY))
#define M_BCM1480_HTD_PORTS_RESET     (M_BCM1480_HTD_ALL_PORTS(M_BCM1480_HTD_PORT_LINK_RESET))
#define M_BCM1480_HTD_PORTS_LINKRESET (M_BCM1480_HTD_ALL_PORTS(M_BCM1480_HTD_PORT_LINKRESET_STATUS))

static const uint32_t ht_masks[8] = {
    0,
    V_BCM1480_HTD_PORTCTRL_PORT0(0xFF),
    V_BCM1480_HTD_PORTCTRL_PORT1(0xFF),
    V_BCM1480_HTD_PORTCTRL_PORT1(0xFF) | V_BCM1480_HTD_PORTCTRL_PORT0(0xFF),
    V_BCM1480_HTD_PORTCTRL_PORT2(0xFF),
    V_BCM1480_HTD_PORTCTRL_PORT2(0xFF) | V_BCM1480_HTD_PORTCTRL_PORT0(0xFF),
    V_BCM1480_HTD_PORTCTRL_PORT2(0xFF) | V_BCM1480_HTD_PORTCTRL_PORT1(0xFF),
    M_BCM1480_HTD_ALL_PORTS(0xFF)
};

/*
 * HT host device initialization.
 */
static unsigned int
htd_init (void)
{
    unsigned int ht_map;
    int i;
    int slave_mode[BCM1480_HT_NUM_PORTS];
    uint32_t ht_mask;
    pcireg_t csr, icr, id;
    pcireg_t port_ctl, prev_ctl;
    volatile pcireg_t t;      /* used for reads that push writes */
    int port;

    eoi_implemented = 1;      /* No EOI bug on the 1400 */

    /* HT: configure the high-speed ports. */
    ht_map = hsp_ht_init();

    /* Convert the bitmap to a register mask. */
    ht_mask = ht_masks[ht_map];
    
    /* Decide the mode of each port.

       XXX Currently, the notion of "slave" mode is overloaded.
       A port in slave mode could be configured as (Active, Primary),
       at least in a tree-structured bus topology.  Primary mode is
       not usable in early chips due to errata, and it is ignored
       for now (see #if's below); "slave" simply means that configuration
       code should not probe the corresponding bridge (see pci_canscan). */

    for (port = 0; port < BCM1480_HT_NUM_PORTS; port++)
	slave_mode[port] = (bcm1400_ldt_slave_mode[port] &&
			    HSP_PORT_MODE_HT(ht_map, port));

    /* HT: configure the external bridges. The mode for each is
       (Active, Secondary).  See note above. */

    port_ctl = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_PORTSCTRL);
    port_ctl &=~  M_BCM1480_HTD_PORTS_ACTIVE;
    port_ctl |=  ((M_BCM1480_HTD_PORTS_ACTIVE | M_BCM1480_HTD_PORTS_RESET)
		  & ht_mask);
    port_ctl &=~ (M_BCM1480_HTD_PORTS_PRIMARY & ht_mask);
#if 0
    for (port = 0; port < BCM1480_HT_NUM_PORTS; port++) {
	if (slave_mode[port])
	    port_ctl |= V_BCM1480_HTD_PORTCTRL_PORTX(port, M_BCM1480_HTD_PORT_IS_PRIMARY);
	}
#endif
    pci_conf_write(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_PORTSCTRL, port_ctl);
    t = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_PORTSCTRL);

    /* HT: set subsystem id */
    id = (PCI_VENDOR_SIBYTE << PCI_VENDOR_SHIFT)|(0x1280 << PCI_PRODUCT_SHIFT);
    pci_conf_write32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_SUBSYSSET, id);

    /* HT: deassert secondary port resets */
    port_ctl &=~ (M_BCM1480_HTD_PORTS_RESET & ht_mask);
#if 0
    for (port = 0; port < BCM1480_HT_NUM_PORTS; port++) {
	if (slave_mode[port])
	    port_ctl |= V_BCM1480_HTD_PORTCTRL_PORTX(port, M_BCM1480_HTD_PORT_LINK_RESET);
	}
#endif
    pci_conf_write(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_PORTSCTRL, port_ctl);
    t = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_PORTSCTRL);

    /* HT: deassert internal bus reset */
    t = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_INTBUSCTRL);
    t &=~ (M_BCM1480_HTD_INTBUS_RESET | M_BCM1480_HTD_INTBUS_WARM_R);
    pci_conf_write32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_INTBUSCTRL, t);
    t = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_INTBUSCTRL);

    /* HT: deassert primary port reset, if any. */
    prev_ctl = port_ctl;
#if 0
    for (port = 0; port < BCM1480_HT_NUM_PORTS; port++) {

	if (slave_mode[port])
	    port_ctl &=~ V_BCM1480_HTD_PORTCTRL_PORTX(port, M_BCM1480_HTD_PORT_LINK_RESET);
	}
    if (port_ctl != prev_ctl) {
	pci_conf_write(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_PORTSCTRL, port_ctl);
	t = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_PORTSCTRL);
	}
#endif

    /* HT: wait for resets to complete. */
    for (i = 10000; i > 0; i--) {
	t = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_PORTSCTRL);
	if ((t & (M_BCM1480_HTD_PORTS_LINKRESET & ht_mask)) == 0)
	    break;
	cfe_usleep(1000);
	}
    /* check for failure. */
    if (i == 0)
	pci_tagprintf(BCM1400_LDT_BRIDGE,
		      "port reset(s) failed to deassert (%08x).\n", t);
    for (i = 1000; i > 0; i--) {
	t = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_INTBUSCTRL);
	if ((t & M_BCM1480_HTD_INTBUS_RESET_STATUS) == 0)
	    break;
	cfe_usleep(100);
	}
    /* check for failure. */
    if (i == 0)
	pci_tagprintf(BCM1400_LDT_BRIDGE,
		      "internal bus reset(s) failed to deassert (%08x).\n", t);

    /* HT: disable and clear the BAR0 MAP registers */
    for (i = 0; i < HTD_MAPENTRIES; i++)
        pci_conf_write32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_MAP(i), 0);

    /* HT: XXX Set up the BARs.  These are placeholders */
    pci_conf_write32(BCM1400_LDT_BRIDGE, PCI_MAPREG(0), 0x60000000);
    pci_conf_write32(BCM1400_LDT_BRIDGE, PCI_MAPREG(1), 0x00000000); /* POHT */
    pci_conf_write32(BCM1400_LDT_BRIDGE, PCI_MAPREG(2), 0x70000000);

    pci_conf_write32(BCM1400_LDT_BRIDGE, PCI_MAPREG(4), 0x00000000);
    pci_conf_write32(BCM1400_LDT_BRIDGE, PCI_MAPREG(5), 0x00000000);

    /* HT: enable a split Low Access Range. */
    pci_conf_write32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_SPECCMDSTAT,
		     M_BCM1480_HTD_CMD_LOW_RANGE_EN |
		     M_BCM1480_HTD_CMD_LOW_RANGE_SPLIT);

    /* HT: clear any pending specific error bits (XXX revisit) */
    t = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_INTBUSCTRL);
    pci_conf_write32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_INTBUSCTRL, t);
    /* Clear reset to enable programming of BARs. */
    t &=~ (M_BCM1480_HTD_INTBUS_RESET | M_BCM1480_HTD_INTBUS_WARM_R);
    pci_conf_write32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_INTBUSCTRL, t);
    t = pci_conf_read32(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_INTBUSCTRL);

    /* HT: null interrupt mapping */
    icr = pci_conf_read(BCM1400_LDT_BRIDGE, PCI_BPARAM_INTERRUPT_REG);
    icr &=~ (PCI_INTERRUPT_LINE_MASK << PCI_INTERRUPT_LINE_SHIFT);
    icr |= (pci_int_line(0) << PCI_INTERRUPT_LINE_SHIFT);
    pci_conf_write(BCM1400_LDT_BRIDGE, PCI_BPARAM_INTERRUPT_REG, icr);

    /* HT: clear errors */
    csr = pci_conf_read32(BCM1400_LDT_BRIDGE, PCI_COMMAND_STATUS_REG);
    csr |= PCI_STATUS_MASTER_ABORT | PCI_STATUS_MASTER_TARGET_ABORT |
           PCI_STATUS_TARGET_TARGET_ABORT;
    pci_conf_write32(BCM1400_LDT_BRIDGE, PCI_COMMAND_STATUS_REG, csr);

    /* HT: enable bridge to internal bus */
    csr = PCI_COMMAND_MASTER_ENABLE | PCI_COMMAND_MEM_ENABLE;
    pci_conf_write32(BCM1400_LDT_BRIDGE, PCI_COMMAND_STATUS_REG, csr);

    /* HT: push the writes */
    t = pci_conf_read32(BCM1400_LDT_BRIDGE, PCI_ID_REG);

    return ht_map;
}


/*
 * HT external bridge initialization.
 */
static int
htb_init (unsigned int ht_map, unsigned linkfreq, unsigned linkwidth)
{
    unsigned freq_cap;        /* bit mask of advertised frequencies */
    pcireg_t cap;             /* capability register image */
    int port;

    /* Upgrade the frequency capabilities of the external bridges. */

    /* Map the maximum requested link frequency to a standard value. */
    if (linkfreq < 200)
	linkfreq = 200;
    else if (linkfreq > 800)
	linkfreq = 800;
    linkfreq = (linkfreq/100) * 100;
    if (linkfreq == 700) linkfreq = 600;   /* not a code point */

    /* Encode the capabilities as a bit vector (see Table 171) */
    freq_cap = 0x8000;                /* Vendor specific (compat mode) */
    switch (linkfreq) {   /* fall-through logic */
	case 800:  freq_cap |= (1 << LDT_FREQ_800);
	case 700:
	case 600:  freq_cap |= (1 << LDT_FREQ_600);
	case 500:  freq_cap |= (1 << LDT_FREQ_500);
	case 400:  freq_cap |= (1 << LDT_FREQ_400);
	case 300:  freq_cap |= (1 << LDT_FREQ_300);
	default:
	case 200:  freq_cap |= (1 << LDT_FREQ_200);
    }
    cap = (V_BCM1480_HTB_SPLINKFREQ_PLLFREQ(0)
	  | V_BCM1480_HTB_SPLINKFREQ_FREQCAPSET(freq_cap));

    /* Normalize the maximum link width (only 8 and 16 supported) */
    if (linkwidth >= 16) {
	linkwidth = 16;
	ldt_set_max_width(LDT_WIDTH_16);
	}
    else {
	linkwidth = 8;
	ldt_set_max_width(LDT_WIDTH_8);
	}

    for (port = 0; port < BCM1480_HT_NUM_PORTS; port++) {
	if (HSP_PORT_MODE_HT(ht_map, port))
	    pci_conf_write(BCM1400_EXT_BRIDGE(port),
			   R_BCM1480_HTB_SPECLINKFREQ, cap);
	}

    xprintf("HT maxima: %dMHz, %d bits\n", linkfreq, linkwidth);

    for (port = 0; port < BCM1480_HT_NUM_PORTS; port++) {
	if (bcm1400_ldt_slave_mode[port] && HSP_PORT_MODE_HT(ht_map, port))
	    xprintf("HT bridge %d in slave mode\n", port);
	}

    return 0;
}


/*
 * Called to initialise the host bridges at the beginning of time.
 */
int
pci_hwinit (int port, pci_flags_t flags)
{
    struct host_port *p;
    int rc;
    int i;
    uint64_t syscfg;

    if (port != 0 && port != 1)
	return -1;
    
    port_enabled[port] = 0;   /* defaults */
    rc = -1;
    p = pci_select_root(port);
    
    /* initialise global data */

    syscfg = SBREADCSR(A_SCD_SYSTEM_CFG);
    bcm1400_in_device_mode = ((syscfg & M_BCM1480_SYS_PCI_HOST) == 0);
    for (i = 0; i < BCM1480_HT_NUM_PORTS; i++)
	bcm1400_ldt_slave_mode[i] = ldt_slave_mode(i);

    _pci_nbus = 0;
    _pci_bus[_pci_nbus] = bcm1400_pci_bus;
    _pci_bus[_pci_nbus].port = port;
    _pci_nbus++;
    for (i = _pci_nbus; i < MAXBUS; i++)
	_pci_bus[i] = secondary_pci_bus;

    if (port == 0) {

	if (!bcm1400_in_device_mode) {
	    /* stop the BCM1400 from servicing any further PCI requests */
	    pci_conf_write32(BCM1400_PCI_BRIDGE, PCI_COMMAND_STATUS_REG, 0);

	    /* initialize the PCI host bridge */
	    rc = phb_host_init();
	} else {
	    /* mostly external initialization of the PCI host bridge */
	    rc = phb_device_init();
	}
    } else {
	unsigned linkfreq, maxfreq;
	const char *str;
	unsigned pll_div, sw_div;
	unsigned linkwidth;
	unsigned int ht_map;

	/* choose the HT link frequency, defaulting to 600 MHz */
	str = env_getenv("LDT_LINKFREQ");
	linkfreq = (str ? atoi(str) : 600);

	/* reduce the default if necessary to the maximum compatible with
	   the CPU and SCLK frequencies (Table 9). */
	pll_div = G_BCM1480_SYS_PLL_DIV(syscfg);
	sw_div = G_BCM1480_SYS_SW_DIV(syscfg);
	if (pll_div == 0 || sw_div == 0)
	    maxfreq = 200;        /* Undefined in Table 9; use HT minimum. */
	else {
	    /* See Table 9 for computation of SCLK from the CPU clock.
	       The maximum link frequency is limited to twice SCLK,
	       where.  SCLK = (100 * (pll_div/2)) / (sw_div/2) =
	       100*pll_div/sw_div.  */
	    maxfreq = 2*(100*pll_div)/sw_div;
	}
	if (linkfreq > maxfreq)
	    linkfreq = maxfreq;

	/* choose the HT maximum link width, defaulting to 16 bits */
	str = env_getenv("LDT_LINKWIDTH");
	linkwidth = (str ? atoi(str) : 16);

	/* stop the BCM1400 from servicing any further HT requests */
	pci_conf_write32(BCM1400_LDT_BRIDGE, PCI_COMMAND_STATUS_REG, 0);

	/* initialize the HT host bridge */
	ht_map = htd_init();

	/* initialize the internal HT bridges */
	rc = htb_init(ht_map, linkfreq, linkwidth);
    }

    if (rc == 0) {
        port_enabled[port] = 1;

	/* add some delay for devices to initialize after reset */
	cfe_sleep(CFE_HZ);
    }

    return 0;
}

/*
 * Called to update the host bridge after we've scanned each PCI device
 * and know what is possible.
 */
void
pci_hwreinit (int port, pci_flags_t flags)
{
    /* FastB2BEn is always clear for both host bridges.  The latency
       timer and cache line size are set by pci_setup_devices
       (pciconf.c) */

    /* XXX  Enable PCI read/write error interrupts? */
}


/* The following functions provide for device-specific setup required
   during configuration.  There is nothing SiByte-specific about them,
   and it would be better to do the packaging and registration in a
   more modular way. */

#define	PCI_VENDOR_ALSC			0x14D9
#define PCI_PRODUCT_ALSC_SP1011		0x0010
#define PCI_PRODUCT_ALSC_AS90L10208	0x9000
extern void sp1011_setup(pcitag_t tag, pci_flags_t flags);
extern void as90l10208_setup(pcitag_t tag, pci_flags_t flags);

#define	PCI_VENDOR_AMD			0x1022
#define PCI_PRODUCT_PLX_HT7520		0x7450
#define PCI_PRODUCT_PLX_HT7520_APIC	0x7451
extern void ht7520apic_preset(pcitag_t tag);
extern void ht7520apic_setup(pcitag_t tag);

#define	PCI_VENDOR_SERVERWORKS		0x1166		/* ServerWorks */
#define	PCI_PRODUCT_SW_BCM5780_HOST    	0x0205		/* BCM5780 HT1000 Legacy Host Bridge */
extern void bcm5780devs_enable_preset(pcitag_t tag);


/* Dispatch functions for device pre- and post-configuration hooks. */

/* Called for each hostbridge, to discover and scan secondary buses */
void
pci_businit_hostbridge (pcitag_t tag, pci_flags_t flags)
{
}

/* Called for each function prior to assigning PCI resources.
   Non-zero return means that no resource assignment should be
   done. */
int
pci_device_preset (pcitag_t tag)
{
    pcireg_t id;
    int skip;

    skip = 0;
    id = pci_conf_read(tag, PCI_ID_REG);

    switch (PCI_VENDOR(id)) {
	case PCI_VENDOR_SIBYTE:
	    /* Check for a host bridge seen internally, in which case
	       we don't want to allocate any address space for its
	       BARs. */
	    if (tag == BCM1400_PCI_BRIDGE || tag == BCM1400_LDT_BRIDGE)
		skip = 1;
	    break;
	case PCI_VENDOR_AMD:
	    if (PCI_PRODUCT(id) == PCI_PRODUCT_PLX_HT7520_APIC)
		ht7520apic_preset (tag);
	    break;
	case PCI_VENDOR_SERVERWORKS:
	    if (PCI_PRODUCT(id) == PCI_PRODUCT_SW_BCM5780_HOST) {
		bcm5780devs_enable_preset (tag); 
		}
	    break;

	default:
	    break;
    }
    return skip;
}


/* Called for each non-bridge (Type 0) function after assigning the BAR
   and InterruptLine (XXX check this) resources.. */
void
pci_device_setup (pcitag_t tag)
{
    pcireg_t id = pci_conf_read(tag, PCI_ID_REG);

    switch (PCI_VENDOR(id)) {
	case PCI_VENDOR_AMD:
	    if (PCI_PRODUCT(id) == PCI_PRODUCT_PLX_HT7520_APIC)
		ht7520apic_setup (tag);
	    break;
	default:
	    break;
    }
}

/* Called for each bridge (Type 1) function after configuring the
   secondary bus, to allow device-specific initialization. */
void
pci_bridge_setup (pcitag_t tag, pci_flags_t flags)
{
    pcireg_t id = pci_conf_read(tag, PCI_ID_REG);

    switch (PCI_VENDOR(id)) {
	case PCI_VENDOR_ALSC:
	    switch (PCI_PRODUCT(id)) {
		case PCI_PRODUCT_ALSC_SP1011:
		    sp1011_setup (tag, flags);
		    break;
		case PCI_PRODUCT_ALSC_AS90L10208:
		    as90l10208_setup (tag, flags);
		    break;
		default:
		    break;
	    }
	    break;
        case PCI_VENDOR_AMD:
	    /* The PLX ht7520 requires configuration of the
	       interrupt mapping, but it packages the IOAPIC as a
	       separate function, registers of which will not yet have
	       been initialized if the standard traversal order is
	       followed.  See next.  */
	    break;
	default:
	    break;
    }
}


/* Machine dependent access primitives and utility functions */

void
pci_flush (void)
{
    /* note: this is a noop for the BCM1400. */
}


pcitag_t
pci_make_tag (int port, int bus, int device, int function)
{
    return BCM1400_PCI_MAKE_TAG(port, bus, device, function);
}

void
pci_break_tag (pcitag_t tag,
	       int *portp, int *busp, int *devicep, int *functionp)
{
    if (portp) *portp = (tag >> 24) & PCI_PORTMAX;
    if (busp) *busp = (tag >> 16) & PCI_BUSMAX;
    if (devicep) *devicep = (tag >> 11) & PCI_DEVMAX;
    if (functionp) *functionp = (tag >> 8) & PCI_FUNCMAX;
}


int
pci_canscan (pcitag_t tag)
{
    int port, bus, device, function;

    pci_break_tag (tag, &port, &bus, &device, &function); 

    if (port > PCI_PORTMAX || !port_enabled[port])
	return 0;

    if (bus > PCI_BUSMAX || device > PCI_DEVMAX || function > PCI_FUNCMAX)
	return 0;

    switch (port) {
	case 0:
	    if (bus == 0 && bcm1400_in_device_mode)
		return 0;
	    break;
	case 1:
	    if (bus == 0) {
		int i;

		/* Don't scan any external bridges in slave mode. */
		for (i = 0; i < BCM1480_HT_NUM_PORTS; i++)
		    if (tag == BCM1400_EXT_BRIDGE(i))
			return !bcm1400_ldt_slave_mode[i];
		}
	    break;
	default:
	    return 0;
    }

    return 1;
}

int
pci_probe_tag(pcitag_t tag)
{
    physaddr_t addrp;
    pcireg_t data;

    if (!pci_canscan(tag))
	return 0;

    addrp = (physaddr_t) BCM1400_CFG_ADDR(tag, PCI_ID_REG, 4);

    /* An earlier version of this code cleared the MasterAbort and
       TargetAbort bits in the PCI host bridge, did the read, and
       looked for those bits to be set.  For the BCM1400, that's
       inappropriate because
	 - it's the wrong host bridge for devices behind HT.
	 - it loses status if testing the PCI host bridge itself.
       We rely on getting 0xffff when reading the vendor ID.  Note
       that this still has side effects on the host bridge registers.
    */

    data = phys_read32(addrp);  /* device + vendor ID */
    mips_wbflush();

    /* if it returned all vendor id bits set, it's not a device */
    return (PCI_VENDOR(data) != 0xffff);
}


/* Read/write access to PCI configuration registers.  For most
   applications, pci_conf_read<N> and pci_conf_write<N> are deprecated
   unless N = 32. */

static pcireg_t
pci_conf_readn(pcitag_t tag, int reg, int width)
{
    physaddr_t addrp;
    pcireg_t data;
#if (PCI_DEBUG != 0)
    int port, bus, device, function;

    if (reg & (width-1) || reg < 0 || reg >= PCI_REGMAX) {
	if (_pciverbose >= 1)
	    pci_tagprintf(tag, "pci_conf_readn: bad reg 0x%x\n", reg);
	return 0;
    }

    pci_break_tag(tag, &port, &bus, &device, &function); 
    if (port > PCI_PORTMAX
	|| bus > PCI_BUSMAX || device > PCI_DEVMAX || function > PCI_FUNCMAX) {
	if (_pciverbose >= 1)
	    pci_tagprintf(tag, "pci_conf_readn: bad tag 0x%x\n", tag);
	return 0;
    }
#endif /* PCI_DEBUG */

    mips_wbflush();

    addrp = (physaddr_t) BCM1400_CFG_ADDR(tag, reg, width);
    switch (width) {
    case 1:
	data = (pcireg_t) phys_read8(addrp);
	break;
    case 2:
	data = (pcireg_t) phys_read16(addrp);
	break;
    default:
    case 4:
	data = (pcireg_t) phys_read32(addrp);
	break;
    }

    mips_wbflush();

    return data;
}

pcireg_t
pci_conf_read8(pcitag_t tag, int reg)
{
    return pci_conf_readn(tag, reg, 1);
}

pcireg_t
pci_conf_read(pcitag_t tag, int reg)
{
    return pci_conf_readn(tag, reg, 4);
}

static void
pci_conf_writen(pcitag_t tag, int reg, pcireg_t data, int width)
{
    physaddr_t addrp;
#if (PCI_DEBUG != 0)
    int port, bus, device, function;

    if (reg & (width-1) || reg < 0 || reg > PCI_REGMAX) {
	if (_pciverbose >= 1)
	    pci_tagprintf(tag, "pci_conf_writen: bad reg 0x%x\n", reg);
	return;
    }

    pci_break_tag(tag, &port, &bus, &device, &function);
    if (port > PCI_PORTMAX
	|| bus > PCI_BUSMAX || device > PCI_DEVMAX || function > PCI_FUNCMAX) {
	if (_pciverbose >= 1)
	    pci_tagprintf(tag, "pci_conf_writen: bad tag 0x%x\n", tag);
	return;
    }
#endif /* PCI_DEBUG */

    mips_wbflush();

    addrp = (physaddr_t) BCM1400_CFG_ADDR(tag, reg, width);
    switch (width) {
    case 1:
	phys_write8(addrp, data);
	break;
    case 2:
	phys_write16(addrp, data);
	break;
    default:
    case 4:
	phys_write32(addrp, data);
	break;
    }

    mips_wbflush();
}

void
pci_conf_write8(pcitag_t tag, int reg, pcireg_t data)
{
    pci_conf_writen(tag, reg, data, 1);
}

void
pci_conf_write(pcitag_t tag, int reg, pcireg_t data)
{
    pci_conf_writen(tag, reg, data, 4);
}

/* Acked writes are intended primarily for updating the unitID field
   during HT fabric initialization.  The write changes the address of
   the target, so further accesses should be avoided until the write
   completes or times out.   */
int
pci_conf_write_acked(pcitag_t tag, int reg, pcireg_t data)
{
    int done;
    int port, bus;
    unsigned int count, prev_count;
    int  i;

    pci_break_tag(tag, &port, &bus, NULL, NULL);

    /* The following code uses the target_done counters in each host
       bridge.  It assumes that there is are no concurrent non-posted
       transactions on the relevant bus. */

    switch (port) {
	case 0:    /* The write will use the PCI Host Bridge */
	    prev_count = pci_conf_read(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_TGTDONE);
	    prev_count &= M_BCM1480_PHB_TGT_DONE_COUNTER;

	    pci_conf_write(tag, reg, data);

	    for (i = 0; i < 1000; i++) {
		count = pci_conf_read(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_TGTDONE);
		count &= M_BCM1480_PHB_TGT_DONE_COUNTER;
		if (count != prev_count)
		    break;
		}
	    done = (count != prev_count);
	    break;
	case 1:    /* The write will use the HT Host Bridge */
	    prev_count = pci_conf_read(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_TGTDONE);
	    prev_count &= M_BCM1480_HTD_TGT_DONE_COUNTER;

	    pci_conf_write(tag, reg, data);

	    for (i = 0; i < 1000; i++) {
		count = pci_conf_read(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_TGTDONE);
		count &= M_BCM1480_HTD_TGT_DONE_COUNTER;
		if (count != prev_count)
		    break;
		}
	    done = (count != prev_count);
	    break;
	default:  /* Otherwise, just write and read back. */
	    pci_conf_write(tag, reg, data);
	    (void) pci_conf_read(tag, reg);
	    done = 1;
	}

    return done;
}



int
pci_map_io(pcitag_t tag, int reg, pci_endian_t endian, phys_addr_t *pap)
{
    pcireg_t address;
    struct host_port *p;
    phys_addr_t pa;
    
    if (reg < PCI_MAPREG_START || reg >= PCI_MAPREG_END || (reg & 3)) {
	if (_pciverbose >= 1)
	    pci_tagprintf(tag, "pci_map_io: bad request\n");
	return -1;
    }
    
    address = pci_conf_read(tag, reg);
    
    if ((address & PCI_MAPREG_TYPE_IO) == 0) {
	if (_pciverbose >= 1)
	    pci_tagprintf(tag,
			  "pci_map_io: attempt to i/o map a memory region\n");
	return -1;
    }

    p = &P[BCM1400_HOST_PORT(tag)];
    pa = ((address & PCI_MAPREG_IO_ADDR_MASK) - p->pci_io_base) + p->io_space;
    if (endian == PCI_MATCH_BITS)
	pa |= p->io_bit_endian;      
    *pap = pa;
    return 0;
}

int
pci_map_mem(pcitag_t tag, int reg, pci_endian_t endian, phys_addr_t *pap)
{
    pcireg_t address;
    struct host_port *p;
    phys_addr_t pa;

    if (reg == PCI_MAPREG_ROM) {
	/* expansion ROM */
	address = pci_conf_read(tag, reg);
	if ((address & PCI_MAPREG_ROM_ENABLE) == 0) {
	    pci_tagprintf(tag, "pci_map_mem: attempt to map missing rom\n");
	    return -1;
	}
	pa = address & PCI_MAPREG_ROM_ADDR_MASK;
    } else {
	if (reg < PCI_MAPREG_START || reg >= PCI_MAPREG_END || (reg & 3)) {
	    if (_pciverbose >= 1)
		pci_tagprintf(tag, "pci_map_mem: bad request\n");
	    return -1;
	}
	
	address = pci_conf_read(tag, reg);
	
	if ((address & PCI_MAPREG_TYPE_IO) != 0) {
	    if (_pciverbose >= 1)
		pci_tagprintf(tag,
			      "pci_map_mem: attempt to memory map an I/O region\n");
	    return -1;
	}
	
	pa = address & PCI_MAPREG_MEM_ADDR_MASK;

	switch (address & PCI_MAPREG_MEM_TYPE_MASK) {
	case PCI_MAPREG_MEM_TYPE_32BIT:
	case PCI_MAPREG_MEM_TYPE_32BIT_1M:
	    break;
	case PCI_MAPREG_MEM_TYPE_64BIT:
	    if (reg + 4 < PCI_MAPREG_END)
	        pa |= ((phys_addr_t)pci_conf_read(tag, reg+4) << 32);
	    else {
	        if (_pciverbose >= 1)
		    pci_tagprintf(tag, "pci_map_mem: bad 64-bit reguest\n");
		return -1;
	    }
	    break;
	default:
	    if (_pciverbose >= 1)
		pci_tagprintf(tag, "pci_map_mem: reserved mapping type\n");
	    return -1;
	}
    }

    p = &P[BCM1400_HOST_PORT(tag)];
    pa = (pa - p->pci_mem_base) + p->mem_space;
    if (endian == PCI_MATCH_BITS)
	pa |= p->mem_bit_endian;      
    *pap = pa;
    return 0;
}


/* XXX This is restricts ISA devices to port 0 XXX */
#define ISAPORT_BASE(x)     (P[1].io_space + (x))

uint8_t
inb (unsigned int port)
{
    return phys_read8(ISAPORT_BASE(port));
}

uint16_t
inw (unsigned int port)
{
    return phys_read16(ISAPORT_BASE(port));
}

uint32_t
inl (unsigned int port)
{
    return phys_read32(ISAPORT_BASE(port));
}

void
outb (unsigned int port, uint8_t val)
{
    phys_write8(ISAPORT_BASE(port), val);
    mips_wbflush();
}

void
outw (unsigned int port, uint16_t val)
{
    phys_write16(ISAPORT_BASE(port), val);
    mips_wbflush();
}

void
outl (unsigned int port, uint32_t val)
{
    phys_write32(ISAPORT_BASE(port), val);
    mips_wbflush();
}


/* Management of inbound (BAR0) MAP table */

#define PHB_MAP_ENTRY_SPAN              (1 << 20)

int
pci_map_window(phys_addr_t pa,
	       unsigned int offset, unsigned int len,
	       int l2ca, int endian)
{
    unsigned int first, last;
    unsigned int i;
    uint32_t     addr;
    uint32_t     entry;

    if (len == 0)
        return 0;

    /* XXX Perhaps check for 1M multiples on offset and len? */

    first = offset / PHB_MAP_ENTRY_SPAN;
    last = (offset + (len-1)) / PHB_MAP_ENTRY_SPAN;

    if (last >= BCM1480_PHB_MAPENTRIES)
        return -1;

    addr = V_BCM1480_PHB_MAP_ADDR(pa / PHB_MAP_ENTRY_SPAN);
    for (i = first; i <= last; i++) {
	entry = (addr & M_BCM1480_PHB_MAP_ADDR) | M_BCM1480_PHB_MAP_ENABLE;
	if (l2ca)
	    entry |= M_BCM1480_PHB_MAP_L2CA;
	if (endian)
	    entry |= M_BCM1480_PHB_MAP_ENDIAN;
	pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_MAP(i), entry);
	addr += V_BCM1480_PHB_MAP_ADDR(1);
    }

    return 0;
}

int
pci_unmap_window(unsigned int offset, unsigned int len)
{
    unsigned int first, last;
    unsigned int i;

    if (len == 0)
        return 0;

    /* XXX Perhaps check for 1M multiples on offset and len? */

    first = offset / PHB_MAP_ENTRY_SPAN;
    if (first >= BCM1480_PHB_MAPENTRIES)
        return 0;

    last = (offset + (len-1)) / PHB_MAP_ENTRY_SPAN;
    if (last >= BCM1480_PHB_MAPENTRIES)
        last = BCM1480_PHB_MAPENTRIES - 1;

    for (i = first; i <= last; i++)
	pci_conf_write32(BCM1400_PCI_BRIDGE, R_BCM1480_PHB_MAP(i), 0);

    return 0;
}


/* Map PCI interrupts A, B, C, D into a value for the IntLine
   register.  For BCM1400, return the source number used by the
   interrupt mapper, or 0xff if none. */
uint8_t
pci_int_line(uint8_t pci_int)
{
    return (pci_int == 0) ? 0xff : (8 + (pci_int-1));
}


/* HT node routing (io and ccnuma).  These use HT configuration
   registers but are unrelated to traditional PCI/HT i/o buses.  */

void ht_node_route(int port, int source_node, int dest_node);

void
ht_node_route(int port, int source_node, int dest_node)
{
    pcireg_t cmd;
    pcireg_t map;
    unsigned int reg, shift;

    /* Program and enable our Full Access Bar */
    pci_conf_write(BCM1400_LDT_BRIDGE, PCI_MAPREG(4), 0);
    pci_conf_write(BCM1400_LDT_BRIDGE, PCI_MAPREG(5), source_node << 4);
    cmd = pci_conf_read(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_SPECCMDSTAT);
    cmd &=~ M_BCM1480_HTD_CMD_FULL_BAR_SPLIT;
    cmd |=  M_BCM1480_HTD_CMD_FULL_BAR_EN;
    pci_conf_write(BCM1400_LDT_BRIDGE, R_BCM1480_HTD_SPECCMDSTAT, cmd);

    /* Set our route to the destination. */
    reg = ((dest_node / 8 == 0) ?
	   R_BCM1480_HTB_NODEROUTING0 : R_BCM1480_HTB_NODEROUTING1);
    shift = (dest_node % 8) * 4;
    map = pci_conf_read(BCM1400_EXT_BRIDGE(port), reg);
    map |= (M_BCM1480_HTB_ROUTE_OVERRIDE_FOR_IO
	    | M_BCM1480_HTB_ROUTE_IS_ON_SEC_FOR_IO
	    | M_BCM1480_HTB_ROUTE_OVERRIDE_FOR_CC
	    | M_BCM1480_HTB_ROUTE_IS_ON_SEC_FOR_CC) << shift;
    pci_conf_write(BCM1400_EXT_BRIDGE(port), reg, map);
}


/* Primitives for manipulating routing tables.  These can be invoked
   on remote nodes and thus use node-prefixed addressing. */

#define NODE_ADDR(node,addr)  ((((uint64_t)(node)) << 36) | (addr))

static pcireg_t
hs_pci_conf_read(unsigned int node, pcitag_t tag, int reg)
{
    physaddr_t addrp;
    pcireg_t data;

    mips_wbflush();

    addrp = NODE_ADDR(node, (physaddr_t)BCM1400_CFG_ADDR(tag, reg, 4));
    data = (pcireg_t) phys_read32(addrp);

    mips_wbflush();

    return data;
}

static void
hs_pci_conf_write(unsigned int node, pcitag_t tag, int reg, pcireg_t data)
{
    physaddr_t addrp;

    mips_wbflush();

    addrp = NODE_ADDR(node, (physaddr_t)BCM1400_CFG_ADDR(tag, reg, 4));
    phys_write32(addrp, data);

    mips_wbflush();
}

void ht_route_swap(unsigned int peer_node, unsigned int dest_node,
		   unsigned int portA, unsigned int portB);

void
ht_route_swap(unsigned int peer_node, unsigned int dest_node,
	      unsigned int portA, unsigned int portB)
{
    int reg;
    pcitag_t tagA, tagB;
    uint32_t mask;
    uint32_t mapA, mapB, temp;

    reg = ((dest_node / 8 == 0) ?
	   R_BCM1480_HTB_NODEROUTING0 : R_BCM1480_HTB_NODEROUTING1);
    mask = 0x0F << ((dest_node % 8) * 4);    /* Select the nibble */

    tagA = BCM1400_EXT_BRIDGE(portA);
    tagB = BCM1400_EXT_BRIDGE(portB);

    /* Read remote HT configuration registers. */
    mapA = hs_pci_conf_read(peer_node, tagA, reg);
    mapB = hs_pci_conf_read(peer_node, tagB, reg);

    hs_pci_conf_write(peer_node, tagA, reg, mapA & ~mask);
    hs_pci_conf_write(peer_node, tagB, reg, mapB & ~mask);

    cfe_usleep(100);    /* Let link idle */

    temp = mapA;
    mapA &= ~mask;  mapA |= mapB & mask;
    mapB &= ~mask;  mapB |= temp & mask;

    hs_pci_conf_write(peer_node, tagA, reg, mapA);
    hs_pci_conf_write(peer_node, tagB, reg, mapB);
}


/* ccnuma management.  This also uses capabilities in HT configuration
   registers but is unrelated to traditional PCI/HT i/o buses.  */

void ht_ccn_enable(unsigned int node, unsigned int port, int enable);

#define S_VCSET_CFG_VCSETSUP         0
#define M_VCSET_CFG_VCSETSUP         _SB_MAKEMASK_32(8,S_VCSET_CFG_VCSETSUP)
#define V_VCSET_CFG_VCSETSUP(x)      _SB_MAKEVALUE_32(x,S_VCSET_CFG_VCSETSUP)
#define G_VCSET_CFG_VCSETSUP(x)      _SB_GETVALUE_32(x,S_VCSET_CFG_VCSETSUP,M_BCSET_CFG_VCSETSUP)

#define S_VCSET_CFG_L0ENBVCSET       16
#define M_VCSET_CFG_L0ENBVCSET       _SB_MAKEMASK_32(8,S_VCSET_CFG_L0ENBVCSET)
#define V_VCSET_CFG_L0ENBVCSET(x)    _SB_MAKEVALUE_32(x,S_VCSET_CFG_L0ENBVCSET)
#define G_VCSET_CFG_L0ENBVCSET(x)    _SB_GETVALUE_32(x,S_VCSET_CFG_L0ENBVCSET,M_BCSET_CFG_L0ENBVCSET)
#define K_VCSET_ALL                  ((1<<2) | (1<<4))

void
ht_ccn_enable(unsigned int node, unsigned int port, int enable)
{
    pcitag_t tag;
    pcireg_t vc_cfg;

    tag = BCM1400_EXT_BRIDGE(port);
    vc_cfg = hs_pci_conf_read(node, tag, R_BCM1480_HTB_VCSETCAP + 4);
    vc_cfg &=~ M_VCSET_CFG_L0ENBVCSET;
    if (enable)
	vc_cfg |= V_VCSET_CFG_L0ENBVCSET(K_VCSET_ALL);
    hs_pci_conf_write(node, tag, R_BCM1480_HTB_VCSETCAP + 4, vc_cfg);
}


