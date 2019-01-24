/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  PPC Board Support Package: PCI         File: mpc824x_pci_machdep.c
    *  
    *  Author:  Ed Satterthwaite
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

/*
 * MPC824x machine-specific functions for PCI autoconfiguration.
 */

#include "lib_types.h"
#include "lib_printf.h"
#include "lib_string.h"
#include "cfe_timer.h"

#include "bsp_config.h"

#include "endian.h"
#include "addrspace.h"

#include "ppcdefs.h"
#include "mpc824x.h"

#include "pcivar.h"
#include "pci_internal.h"
#include "pcireg.h"

#if ((ENDIAN_BIG + ENDIAN_LITTLE) != 1)
#error "mpc824x_pci_machdep: system endian not set"
#endif

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

#define PCI_MAKE_TAG(b,d,f) \
    (((b) << 16) | ((d) << 11) | ((f) << 8))


/* PCI spaces as seen by the CPU in Host mode (Table 3-1). */

/* Core regions allocated for mapping to PCI address spaces. */
#define MPC824X_PCI_MEM_SPACE       0x80000000
#define MPC824X_PCI_MEM_SPACE_SIZE  0x7D000000
#define MPC824X_PCI_IO_SPACE        0xFE800000
#define MPC824X_PCI_IO_SPACE_SIZE   0x00400000

/* PCI regions mapped from the core addresses. */
#define MPC824X_PCI_MEM_BASE        0x80000000
#define MPC824X_PCI_IO_BASE         0x00800000

/* Window on configuration registers */
#define MPC824X_CONFIG_ADDR         0xFEC00000
#define MPC824X_CONFIG_DATA         0xFEE00000

#define MPC824X_PCI_ENABLE          0x80000000


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
 */

pcireg_t
pci_minmemaddr (int port)
{
    return MPC824X_PCI_MEM_BASE;
}

pcireg_t
pci_maxmemaddr (int port)
{
    return MPC824X_PCI_MEM_BASE + MPC824X_PCI_MEM_SPACE_SIZE;
}

pcireg_t
pci_minioaddr (int port)
{
    return MPC824X_PCI_IO_BASE;
}

pcireg_t
pci_maxioaddr (int port)
{
    return MPC824X_PCI_IO_BASE + MPC824X_PCI_IO_SPACE_SIZE;
}


/* The PCI core (a host bridge in Host mode) */

#define PPC_PCI_BRIDGE     (PCI_MAKE_TAG(0,0,0))

/* Called to initialise the host bridge at the beginning of time. */
static void
phb_init (void)
{
    pcireg_t csr;

    /* stop the servicing of any further PCI */
    pci_conf_write(PPC_PCI_BRIDGE, PCI_COMMAND_STATUS_REG, 0);
    cfe_usleep(100);

    /* XXX set up PMCR2, arbiter, PICR1 */

    /* enable bridge to PCI and PCI memory accesses, including
       write-invalidate, plus error handling */
    csr = PCI_COMMAND_MASTER_ENABLE | PCI_COMMAND_MEM_ENABLE |
          PCI_COMMAND_INVALIDATE_ENABLE |
          PCI_COMMAND_SERR_ENABLE |  PCI_COMMAND_PARITY_ENABLE;
    pci_conf_write(PPC_PCI_BRIDGE, PCI_COMMAND_STATUS_REG, csr);

    /* clear errors */
    csr = pci_conf_read(PPC_PCI_BRIDGE, PCI_COMMAND_STATUS_REG);
    csr |= PCI_STATUS_PARITY_ERROR | PCI_STATUS_SYSTEM_ERROR |
           PCI_STATUS_MASTER_ABORT | PCI_STATUS_MASTER_TARGET_ABORT |
           PCI_STATUS_TARGET_TARGET_ABORT | PCI_STATUS_PARITY_DETECT;
    pci_conf_write(PPC_PCI_BRIDGE, PCI_COMMAND_STATUS_REG, csr);

    (void)pci_conf_read(PPC_PCI_BRIDGE, PCI_ID_REG);   /* push */
}


int
pci_hwinit (int port, pci_flags_t flags)
{
    int i;

    /* Enable the internal arbiter.  PARK_LAST apparently locks the bus
       when there is no response. */
    /* XXX TBD, currently done in cpu init. */

    memcpy(&_pci_bus[_pci_nbus], &init_pci_bus, sizeof(struct pci_bus));
    _pci_bus[_pci_nbus].port = port;
    _pci_nbus++;
    for (i = _pci_nbus; i < MAXBUS; i++)
	memcpy(&_pci_bus[i], &init_pci_bus, sizeof(struct pci_bus));

    cfe_sleep(CFE_HZ/2);                /* let devices initialize */

    phb_init();
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
    
    return (tag == PPC_PCI_BRIDGE ? 1 : 0);
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
   n is mapped to AD bit n except that device 10 is mapped to bit 31
   (21 possible possible devices, 10..30).  The host bridge itself
   is mapped internally to bus 0, device 0. */

int
pci_canscan (pcitag_t tag)
{
    int bus, device, function;

    pci_break_tag(tag, NULL, &bus, &device, &function); 
    if (bus > PCI_BUSMAX || device > PCI_DEVMAX || function > PCI_FUNCMAX)
	return 0;

    return 1;
}

int
pci_probe_tag(pcitag_t tag)
{
    pcireg_t data;

    if (!pci_canscan(tag))
	return 0;

    data = pci_conf_read(tag, PCI_ID_REG);  /* bus error if no response */

    /* if it returned all vendor id bits set, it's not a device */
    return (PCI_VENDOR(data) != 0xffff);
}


pcireg_t
pci_conf_read(pcitag_t tag, int reg)
{
    pcireg_t data;
    uint32_t cfg;
    uint32_t addr = MPC824X_PCI_ENABLE | tag | reg;

#if ENDIAN_BIG
    __asm __volatile ("\
	addis  %0,0,0xfec0  \t\n \
	stwbrx %2,0,%0      \t\n \
	sync                \t\n \
	addis  %0,0,0xfee0  \t\n \
	lwbrx  %1,0,%0      \t\n \
	sync  " :
      "=r&" (cfg), "=r" (data) : "r" (addr));
#else
    __asm __volatile ("\
	addis  %0,0,0xfec0  \n\t \
	stw    %2,0(%0)     \n\t \
	sync                \n\t \
	addis  %0,0,0xfee0  \n\t \
	lw     %1,0(%0)     \n\t \
	sync  " :
      "=&r" (cfg), "=r" (data) : "r" (addr));
#endif

    return data;
}

void
pci_conf_write(pcitag_t tag, int reg, pcireg_t data)
{
    uint32_t addr = MPC824X_PCI_ENABLE | tag | reg;
    uint32_t cfg;

#if ENDIAN_BIG
    __asm __volatile ("\
	addis  %0,0,0xfec0  \t\n \
	stwbrx %2,0,%0      \t\n \
	sync                \t\n \
	addis  %0,0,0xfee0  \t\n \
	stwbrx %1,0,%0      \t\n \
	sync  " :
      "=&r" (cfg) : "r" (data), "r" (addr));
#else
    __asm __volatile ("\
	addis  %0,0,0xfec0  \n\t \
	stw    %2,0(%0)     \n\t \
	sync                \n\t \
	addis  %0,0,0xfee0  \n\t \
	stw    %1,0(%0)     \n\t \
	sync  " :
      "=&r" (cfg) : "r" (data), "r" (addr));
#endif
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
    pcireg_t address;
    phys_addr_t pa;
    
#if ENDIAN_BIG
    if (endian == PCI_MATCH_BITS) {
	if (_pciverbose != 0)
	    pci_tagprintf(tag, "pci_map_io: attempt to use MATCH BITS mode\n");
	return -1;
    }
#endif

    if (reg < PCI_MAPREG_START || reg >= PCI_MAPREG_END || (reg & 3)) {
	if (_pciverbose != 0)
	    pci_tagprintf(tag, "pci_map_io: bad request\n");
	return -1;
    }
    
    address = pci_conf_read(tag, reg);
    
    if ((address & PCI_MAPREG_TYPE_IO) == 0) {
	if (_pciverbose != 0)
	    pci_tagprintf(tag, "pci_map_io: attempt to i/o map a memory region\n");
	return -1;
    }

    pa = ((address & PCI_MAPREG_IO_ADDR_MASK) - MPC824X_PCI_IO_BASE)
         + MPC824X_PCI_IO_SPACE;
    *pap = pa;
    return 0;
}

int
pci_map_mem(pcitag_t tag, int reg, pci_endian_t endian, phys_addr_t *pap)
{
    pcireg_t address;
    phys_addr_t pa;

#if ENDIAN_BIG
    if (endian == PCI_MATCH_BITS) {
	if (_pciverbose != 0)
	    pci_tagprintf(tag, "pci_map_mem: attempt to use MATCH BITS mode\n");
	return -1;
    }
#endif

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
#if 0  /* XXX check for all zeros at reg+4 */
		    pa |= ((phys_addr_t)pci_conf_read(tag, reg+4) << 32);
#else
		    ;
#endif
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

    pa = (pa - MPC824X_PCI_MEM_BASE) + MPC824X_PCI_MEM_SPACE;
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


/* Map PCI interrupts A, B, C, D into the index in the IRQ vector, or
   0xff if none.  This makes assumptions about both the CPLD and the
   programming of the EPIC. */
uint8_t
pci_int_line(uint8_t pci_int)
{
    return pci_int;
}
