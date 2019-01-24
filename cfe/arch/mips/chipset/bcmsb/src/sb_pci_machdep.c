/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  Broadcom Silicon Backplane PCI support	File: sb_pci_machdep.c
    *  
    *  Author:  Ed Satterthwaite
    *  
    *********************************************************************  
    *
    *  Copyright 2003,2004
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
 * Silicon Backplane machine-specific functions for PCI autoconfiguration.
 */

#include "cfe.h"

#include "sb_bp.h"
#include "sb_pci.h"
#include "sb_utils.h"

#include "pcivar.h"
#include "pci_internal.h"
#include "pcireg.h"

#include "lib_try.h"

void pci_ext_arb_set(int enable);
void pci_ext_clk_set(int enable);

extern int _pciverbose;

const cons_t pci_optnames[] = {
    {"verbose", PCI_FLG_VERBOSE},
    {NULL, 0}
};


/* Templates for bus attributes. */

static const struct pci_bus init_pci_bus = {
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

#define MAXBUS	10
static struct pci_bus _pci_bus[MAXBUS];
static int _pci_nbus = 0;

static int _ext_arb = 0;
static int _ext_clk = 0;

#define PCI_MAKE_TAG(b,d,f) \
    (((b) << 16) | ((d) << 11) | ((f) << 8))

/* Access functions (XXX use (upgraded?) lib_physio) */

#define PTR_TO_PHYS(x) (PHYSADDR((uintptr_t)(x)))
#define PHYS_TO_PTR(a) ((uint8_t *)UNCADDR(a))

/* PCI "enumeration" space */

#define READCSR(x)    \
  (*(volatile uint32_t *)PHYS_TO_PTR(SB_PCI_BASE+(x)))
#define WRITECSR(x,v) \
  (*(volatile uint32_t *)PHYS_TO_PTR(SB_PCI_BASE+(x)) = (v))

/* PCI spaces mapped by the core in Host mode. */

#define SB_PCI_CFG_BASE  0x0C000000
#define SB_PCI_MEM_BASE  0x08000000

#define CFG_ADDR(x)      ((x) | SB_PCI_CFG_BASE)
#define MEM_ADDR(x)      ((x) | SB_PCI_MEM_BASE)

#define READCFG(x)       (*(volatile uint32_t *)PHYS_TO_PTR(CFG_ADDR(x)))
#define WRITECFG(x,v)    (*(volatile uint32_t *)PHYS_TO_PTR(CFG_ADDR(x)) = (v))


/* The following should return the index of the last usable bus port. */
int
pci_maxport (void)
{
    return PCI_HOST_PORTS - 1;
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
 *
 * The standard Silicon Backplane address map provides two
 * configurable PCI regions in host mode.  Thus it is not possible to
 * provide memory, i/o and configuration space simultaneously.  For
 * now, i/o space is not implemented.
 */

pcireg_t
pci_minmemaddr (int port)
{
    return SB_PCI_MEM_BASE;
}

pcireg_t
pci_maxmemaddr (int port)
{
    return SB_PCI_MEM_BASE + 0x04000000;
}

pcireg_t
pci_minioaddr (int port)
{
    return 0x00000000;
}

pcireg_t
pci_maxioaddr (int port)
{
    return 0x00000000;
}


/* The PCI core (a host bridge in Host mode) */

#define SB_PCI_BRIDGE     (PCI_MAKE_TAG(0,0,0))

/* Called to initialise the host bridge at the beginning of time. */
static void
phb_init (void)
{
    pcireg_t csr;

    /* stop the servicing of any further PCI */
    pci_conf_write(SB_PCI_BRIDGE, PCI_COMMAND_STATUS_REG, 0);
    cfe_usleep(100);

    /* Set up the BARs (BAR0 for cores, BAR1 for SDRAM) */
#ifdef SB_CHIPC_BASE
    pci_conf_write(SB_PCI_BRIDGE, PCI_PCIBAR0WINDOW_REG, SB_CHIPC_BASE);
#else
    pci_conf_write(SB_PCI_BRIDGE, PCI_PCIBAR0WINDOW_REG, SB_EXTIF_BASE);
#endif
    pci_conf_write(SB_PCI_BRIDGE, PCI_MAPREG(0), 0x20000000);
    pci_conf_write(SB_PCI_BRIDGE, PCI_PCIBAR1WINDOW_REG, 0x00000000);
    pci_conf_write(SB_PCI_BRIDGE, PCI_MAPREG(1), 0x00000000);
    pci_conf_write(SB_PCI_BRIDGE, PCI_BAR1BURSTCTRL_REG,
		   M_BAR1BURST_PE | M_BAR1BURST_WB);
    
    /* enable bridge to PCI and PCI memory accesses, including
       write-invalidate, plus error handling */
    csr = PCI_COMMAND_MASTER_ENABLE | PCI_COMMAND_MEM_ENABLE |
          PCI_COMMAND_INVALIDATE_ENABLE |
          PCI_COMMAND_SERR_ENABLE |  PCI_COMMAND_PARITY_ENABLE;
    pci_conf_write(SB_PCI_BRIDGE, PCI_COMMAND_STATUS_REG, csr);

    /* clear errors */
    csr = pci_conf_read(SB_PCI_BRIDGE, PCI_COMMAND_STATUS_REG);
    csr |= PCI_STATUS_PARITY_ERROR | PCI_STATUS_SYSTEM_ERROR |
           PCI_STATUS_MASTER_ABORT | PCI_STATUS_MASTER_TARGET_ABORT |
           PCI_STATUS_TARGET_TARGET_ABORT | PCI_STATUS_PARITY_DETECT;
    pci_conf_write(SB_PCI_BRIDGE, PCI_COMMAND_STATUS_REG, csr);

    (void)pci_conf_read(SB_PCI_BRIDGE, PCI_ID_REG);   /* push */
}


/* Allow us to work with external arbiter */
void
pci_ext_arb_set (int enable)
{
    _ext_arb = enable;
}

/* Allow us to work with external clock */
void
pci_ext_clk_set (int enable)
{
    _ext_clk = enable;
}


#ifndef PCI_CLOCK
#define PCI_CLOCK 33000000
#endif

int
pci_hwinit (int port, pci_flags_t flags)
{
    uint32_t ctrl;
    uint32_t cpu_clock;
    int i;

    /* Enable PCI clock and release reset */
    if (_ext_clk) {
        ctrl = M_PCICTL_OE;              /* enable reset output */
        }
    else {
        ctrl = M_PCICTL_OE | M_PCICTL_CE;/* enable the tristate drivers */
        WRITECSR(R_PCI_CONTROL, ctrl);
        ctrl |= M_PCICTL_CO;             /* enable the PCI clock */
        }
    WRITECSR(R_PCI_CONTROL, ctrl);
    cfe_usleep(100);                    /* delay 100 us */
    ctrl |= M_PCICTL_RO;                /* release reset */
    WRITECSR(R_PCI_CONTROL, ctrl);

    /* Set PCI clock frequency */
    cpu_clock = sb_clock();
    sb_setclock(cpu_clock, PCI_CLOCK);

    if (_ext_arb) {
        /* Enable the external arbiter */
        WRITECSR(R_PCI_ARB_CONTROL, M_PCIARB_EA);
        }
    else {
	/* Enable the internal arbiter.  PARK_LAST apparently locks the bus
	   when there is no response. */
	WRITECSR(R_PCI_ARB_CONTROL, M_PCIARB_IA);
	}

    /* Map the PCI memory window transparently. */
    WRITECSR(R_SB_TO_PCI_TRANSLATION0,
	     (SB_PCI_MEM_BASE & M_SBXLAT_UA) | M_SBXLAT_PE | M_SBXLAT_WB
	     | V_SBXLAT_AT(K_AT_RW));

    _pci_bus[_pci_nbus] = init_pci_bus;
    _pci_bus[_pci_nbus].port = port;
    _pci_nbus++;
    for (i = _pci_nbus; i < MAXBUS; i++)
	_pci_bus[i] = init_pci_bus;

    cfe_sleep(CFE_HZ/2);                /* let devices initialize */

    phb_init();

    /* Allow the standard PCI interrupts. */
    WRITECSR(R_INT_MASK, M_PCIINT_PA | M_PCIINT_PB);

    return 0;
}

/* Called to update the host bridge after we've scanned each PCI
   device and know what is possible. */
void
pci_hwreinit (int port, pci_flags_t flags)
{
}


/* The following functions provide for device-specific setup required
   during configuration.  There is nothing host-specific about them,
   and it would be better to do the packaging and registration in a
   more modular way. */

/* Dispatch functions for device pre- and post-configuration hooks. */

/* Called for each hostbridge, to discover and scan secondary buses */
void
pci_businit_hostbridge (pcitag_t tag, pci_flags_t flags)
{
}

/* Called for each function prior to assigning PCI resources.  */
int
pci_device_preset (pcitag_t tag)
{
    /* Check for a host bridge seen internally, in which case
       we don't want to allocate any address space for its BARs. */
    
    return (tag == SB_PCI_BRIDGE ? 1 : 0);
}


/* Called for each non-bridge (Type 0) function after assigning the
   BAR and InterruptLine resources. */
void
pci_device_setup (pcitag_t tag)
{
}

/* Called for each bridge (Type 1) function after configuring the
   secondary bus, to allow device-specific initialization. */
void
pci_bridge_setup (pcitag_t tag, pci_flags_t flags)
{
}


/* Machine dependent access primitives and utility functions */

void
pci_flush (void)
{
}


pcitag_t
pci_make_tag (int port, int bus, int device, int function)
{
    return PCI_MAKE_TAG(bus, device, function);
}

void
pci_break_tag (pcitag_t tag,
	       int *portp, int *busp, int *devicep, int *functionp)
{
    if (portp) *portp = 0;
    if (busp) *busp = (tag >> 16) & PCI_BUSMAX;
    if (devicep) *devicep = (tag >> 11) & PCI_DEVMAX;
    if (functionp) *functionp = (tag >> 8) & PCI_FUNCMAX;
}


/* Read/write access to PCI configuration registers.  Type0 addresses
   are used for bus 0, with a unary encoding of IDSEL in which device
   n is mapped to bit 16+n (16 possible devices, 0..15). */

int
pci_canscan (pcitag_t tag)
{
    int bus, device, function;

    pci_break_tag(tag, NULL, &bus, &device, &function); 
    if (bus > PCI_BUSMAX || device > PCI_DEVMAX || function > PCI_FUNCMAX)
	return 0;

    return (bus != 0 || (device >= 0 && device < 16));
}

int
pci_probe_tag(pcitag_t tag)
{
    pcireg_t data;
    jmpbuf_t *jb;

    if (!pci_canscan(tag))
	return 0;

    jb = exc_initialize_block();
    if (jb == NULL)
	return 0;

    if (exc_try(jb) == 0)
	data = pci_conf_read(tag, PCI_ID_REG);  /* bus error if no response */
    else
	data = 0xffffffff;

    exc_cleanup_block(jb);

    /* if it returned all vendor id bits set, it's not a device */
    return (PCI_VENDOR(data) != 0xffff);
}


/* Note that READCFG/WRITECFG uses below can generate bus errors that
   are not caught; guard calls with these functions with pci_probe_tag. */

pcireg_t
pci_conf_read(pcitag_t tag, int reg)
{
    int bus, device, function;
    pcireg_t data;
    uint32_t addr;

    pci_break_tag(tag, NULL, &bus, &device, &function);

    addr = (function << 8) | (reg & 0xFC);  /* common part */
    if (bus == 0) {
	if (device < 16) {
	    /* Generate a Type0 configuration cycle. */
	    addr |= (1 << (16 + device));
	    WRITECSR(R_SB_TO_PCI_TRANSLATION1,
		     (addr & M_SBXLAT_UA) | V_SBXLAT_AT(K_AT_CFG0_RW));
	    (void)READCSR(R_SB_TO_PCI_TRANSLATION1);  /* push */
	    addr &= ~M_SBXLAT_UA;
	    data = READCFG(addr);
	    }
	else
	    data = 0xFFFFFFFF;
	}
    else {
	/* Generate a Type1 configuration cycle. */
	addr |= (bus << 16) | (device << 11);
	WRITECSR(R_SB_TO_PCI_TRANSLATION1,
		 (addr & M_SBXLAT_UA) | V_SBXLAT_AT(K_AT_CFG1_RW));
	(void)READCSR(R_SB_TO_PCI_TRANSLATION1);  /* push */
	addr &= ~M_SBXLAT_UA;
	data = READCFG(addr);
	}

    return data;
}

void
pci_conf_write(pcitag_t tag, int reg, pcireg_t data)
{
    int bus, device, function;
    uint32_t addr;

    pci_break_tag(tag, NULL, &bus, &device, &function);

    addr = (function << 8) | (reg & 0xFC);  /* common part */
    if (bus == 0) {
	if (device < 16) {
	    /* Generate a Type0 configuration cycle. */
	    addr |= (1 << (16 + device));
	    WRITECSR(R_SB_TO_PCI_TRANSLATION1,
		     (addr & M_SBXLAT_UA) | V_SBXLAT_AT(K_AT_CFG0_RW));
	    (void)READCSR(R_SB_TO_PCI_TRANSLATION1);  /* push */
	    addr &= ~M_SBXLAT_UA;
	    WRITECFG(addr & ~SB_PCI_CFG_BASE, data);
	    }
	}
    else {
	/* Generate a Type1 configuration cycle. */
	addr |= (bus << 16) | (device << 11);
	WRITECSR(R_SB_TO_PCI_TRANSLATION1,
		 (addr & M_SBXLAT_UA) | V_SBXLAT_AT(K_AT_CFG1_RW));
	addr &= ~M_SBXLAT_UA;
	(void)READCSR(R_SB_TO_PCI_TRANSLATION1);  /* push */
	WRITECFG(addr, data);
	}
}

int
pci_conf_write_acked(pcitag_t tag, int reg, pcireg_t data)
{
    pci_conf_write(tag, reg, data);
    (void) pci_conf_read(tag, reg);
    return 1;
}


int
pci_map_io(pcitag_t tag, int reg, pci_endian_t endian, phys_addr_t *pap)
{
    pci_tagprintf(tag, "pci_map_io: i/o regions not supported\n");
    return -1;
}

int
pci_map_mem(pcitag_t tag, int reg, pci_endian_t endian, phys_addr_t *pap)
{
    pcireg_t address;
    phys_addr_t pa;

    if (reg == PCI_MAPREG_ROM) {
	/* expansion ROM */
	address = pci_conf_read(tag, reg);
	if ((address & PCI_MAPREG_ROM_ENABLE) == 0) {
	    pci_tagprintf(tag, "pci_map_mem: attempt to map missing rom\n");
	    return -1;
	    }
	pa = address & PCI_MAPREG_ROM_ADDR_MASK;
	}
    else {
	if (reg < PCI_MAPREG_START || reg >= PCI_MAPREG_END || (reg & 3)) {
	    if (_pciverbose != 0)
		pci_tagprintf(tag, "pci_map_mem: bad request\n");
	    return -1;
	    }
	
	address = pci_conf_read(tag, reg);
	
	if ((address & PCI_MAPREG_TYPE_IO) != 0) {
	    if (_pciverbose != 0)
		pci_tagprintf(tag, "pci_map_mem: attempt to memory map an I/O region\n");
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
		    if (_pciverbose != 0)
			pci_tagprintf(tag, "pci_map_mem: bad 64-bit reguest\n");
		    return -1;
		    }
		break;
	    default:
		if (_pciverbose != 0)
		    pci_tagprintf(tag, "pci_map_mem: reserved mapping type\n");
		return -1;
	    }
	}

    /* XXX Do something about endian.  Byte-swapping in cross-endian
       systems cannot really be handled here.  In the BCM47xx cores,
       swapping of DRAM accesses (only) is controlled by a bit in the
       DRAM address, not in the PCI address as in the BCM1250. */
    *pap = pa;
    return 0;
}


/* ISA-style i/o access (not supported) */

uint8_t
inb (unsigned int port)
{
    xprintf("inb: i/o regions not supported\n");
    return 0xFF;
}

uint16_t
inw (unsigned int port)
{
    xprintf("inw: i/o regions not supported\n");
    return 0xFFFF;
}

uint32_t
inl (unsigned int port)
{
    xprintf("inl: i/o regions not supported\n");
    return 0xFFFFFFFF;
}

void
outb (unsigned int port, uint8_t val)
{
    xprintf("outb: i/o regions not supported\n");
}

void
outw (unsigned int port, uint16_t val)
{
    xprintf("outw: i/o regions not supported\n");
}

void
outl (unsigned int port, uint32_t val)
{
    xprintf("outl: i/o regions not supported\n");
}


/* Interrupt slot wiring.  Eval boards apparently connect only INTA
   and INTB but use INTA<->INTA for all devices and slots.

   This should be board specific except that, for all boards, all PCI
   interrupts are reduced to a single backplane flag before being
   routed to the CPU. */

/* Map PCI interrupts A, B, C, D into a value for the IntLine
   register.  Return the source number used by the
   interrupt mapper, or 0xff if none. */
uint8_t
pci_int_line(uint8_t pci_int)
{
    uint32_t flag;

    flag = READCSR(R_SBTPSFLAG);
    return (pci_int == 0) ? 0xFF : G_SBTSF_FN(flag);
}
