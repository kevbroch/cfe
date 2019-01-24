/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  Realtek RTL8139 Ethernet Driver		      File: dev_rtl8139.c
    *  
    *  Author:  Ed Satterthwaite
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

#ifdef CPUCFG_MEMCPY
#error "this doesn't work anymore."
extern void *CPUCFG_MEMCPY(void *dest, const void *src, size_t cnt);
#define blockcopy CPUCFG_MEMCPY
#else
#define blockcopy memcpy
#endif

#include "cfe_irq.h"

#include "net_enet.h"

#include "pcivar.h"
#include "pcireg.h"

#include "rtl8139.h"
#include "mii.h"


/* This is a driver for the Realtek 8139 10/100 MAC with integrated PHY.

   This SB1250 version takes advantage of DMA coherence.  It uses
   "preserve bit lanes" addresses for all register accesses but
   uses "preserve byte lanes" addresses for DMA.  */

#ifndef RTK_DEBUG
#define RTK_DEBUG 0
#endif

#if ((ENDIAN_BIG + ENDIAN_LITTLE) != 1)
#error "dev_rtl8139: system endian not set"
#endif

/* Set IPOLL to drive processing through the pseudo-interrupt
   dispatcher.  Set XPOLL to drive processing by an external polling
   agent.  Setting both is ok. */

#ifndef IPOLL
#define IPOLL 0
#endif
#ifndef XPOLL
#define XPOLL 1
#endif

#define MIN_ETHER_PACK  (ENET_MIN_PKT+ENET_CRC_SIZE)   /* size of min packet */
#define MAX_ETHER_PACK  (ENET_MAX_PKT+ENET_CRC_SIZE)   /* size of max packet */

/* Packet buffers.  For the RTL8139, a tx packet must be in a single
   contiguous buffer aligned to a 32-bit word boundary.  Since DMA
   does not write directly to packet buffers, alignment is not
   critical, but aligning to a cache line boundary can reduce lines
   touched on the copies. */

#define ETH_PKTPOOL_SIZE 32
#define ETH_PKTBUF_LEN      (((MAX_ETHER_PACK+31)/32)*32)

typedef struct eth_pkt_s {
    queue_t next;			/*  8 */
    uint8_t *buffer;			/*  4 */
    uint32_t flags;			/*  4 */
    int32_t length;			/*  4 */
    uint32_t unused[3];			/* 12 */
    uint8_t data[ETH_PKTBUF_LEN];
} eth_pkt_t;

#define CACHE_ALIGN       32
#define ETH_PKTBUF_LINES  ((sizeof(eth_pkt_t) + (CACHE_ALIGN-1))/CACHE_ALIGN)
#define ETH_PKTBUF_SIZE   (ETH_PKTBUF_LINES*CACHE_ALIGN)
#define ETH_PKTBUF_OFFSET (offsetof(eth_pkt_t, data))

#define ETH_PKT_BASE(data) ((eth_pkt_t *)((data) - ETH_PKTBUF_OFFSET))

static void
show_packet(char c, eth_pkt_t *pkt)
{
    int i;
    int n = (pkt->length < 32 ? pkt->length : 32);

    xprintf("%c[%4d]:", c, pkt->length);
    for (i = 0; i < n; i++) {
	if (i % 4 == 0)
	    xprintf(" ");
	xprintf("%02x", pkt->buffer[i]);
	}
    xprintf("\n");
}


/* Driver data structures */

typedef enum {
    eth_state_uninit,
    eth_state_off,
    eth_state_on, 
    eth_state_broken
} eth_state_t;

typedef struct rtl8139_softc {
    uint32_t membase;
    uint8_t irq;		/* interrupt mapping (used if IPOLL) */
    pcitag_t tag;               /* tag for configuration registers */

    uint8_t hwaddr[ENET_ADDR_LEN];                 
    uint16_t device;            /* chip device code */
    uint8_t revision;		/* chip revision and step */

    eth_state_t state;          /* current state */
    uint32_t intmask;           /* interrupt mask */

    /* These fields are set before calling rtl8139_hwinit */
    int linkspeed;		/* encodings from cfe_ioctl */
    int loopback;

    /* Packet free list */
    queue_t freelist;
    uint8_t *pktpool;
    queue_t rxqueue;

    uint8_t *rx_buffbase;

    unsigned int tx_pi;          /* pointers into the TSD/TSAD ring */
    unsigned int tx_ci;
    unsigned int tx_dmacnt;      /* entried on ring (shared, use CS_*) */
    uint32_t tx_thresh;

    cfe_devctx_t *devctx;

    int phy_check;
    uint16_t phy_status;

    /* Statistics */
    uint32_t inpkts;
    uint32_t outpkts;
    uint32_t interrupts;
    uint32_t rx_interrupts;
    uint32_t tx_interrupts;
    uint32_t bus_errors;
} rtl8139_softc;


/* Entry to and exit from critical sections (currently relative to
   interrupts only, not SMP) */

#if CFG_INTERRUPTS
#define CS_ENTER(sc) cfe_disable_irq(sc->irq)
#define CS_EXIT(sc)  cfe_enable_irq(sc->irq)
#else
#define CS_ENTER(sc) ((void)0)
#define CS_EXIT(sc)  ((void)0)
#endif


/* Prototypes */

static void rtl8139_ether_probe(cfe_driver_t *drv,
				unsigned long probe_a, unsigned long probe_b, 
				void *probe_ptr);


/* Address mapping macros.  Accesses in which the RTL8139 is the
   target are to registers and use match bits mode.  Accesses in which
   it is the initiator always assume little-endian responses and must
   use match bytes, per the macros below.  For big-endian hosts, the
   DMA status word must be byte-swapped. */

/* Note that PTR_TO_PHYS only works with 32-bit addresses, but then
   so does the RTL8139. */
#define PTR_TO_PHYS(x) (PHYSADDR((uintptr_t)(x)))
#define PHYS_TO_PTR(a) ((uint8_t *)KERNADDR(a))

/* The RLT8139 has no provision for big-endian DMA.  Thus, unlike most
   of the MAC drivers, this one must use match bytes addresses. */
#undef PHYS_TO_PCI
#undef PCI_TO_PHYS

#define PHYS_TO_PCI(a) ((uint32_t) (a))
#define PCI_TO_PHYS(a) ((uint32_t) (a))

#define PCI_TO_PTR(a)  (PHYS_TO_PTR(PCI_TO_PHYS(a)))
#define PTR_TO_PCI(x)  (PHYS_TO_PCI(PTR_TO_PHYS(x)))

#if ENDIAN_BIG
#define READCSR1(sc,csr) (phys_read8((sc)->membase + ((csr)^3)))
#define READCSR2(sc,csr) (phys_read16((sc)->membase + ((csr)^2)))
#else
#define READCSR1(sc,csr) (phys_read8((sc)->membase + (csr)))
#define READCSR2(sc,csr) (phys_read16((sc)->membase + (csr)))
#endif
#define READCSR4(sc,csr) (phys_read32((sc)->membase + (csr)))

#if ENDIAN_BIG
#define WRITECSR1(sc,csr,val) (phys_write8((sc)->membase + ((csr)^3), (val)))
#define WRITECSR2(sc,csr,val) (phys_write16((sc)->membase + ((csr)^2), (val)))
#else
#define WRITECSR1(sc,csr,val) (phys_write8((sc)->membase + (csr), (val)))
#define WRITECSR2(sc,csr,val) (phys_write16((sc)->membase + (csr), (val)))
#endif
#define WRITECSR4(sc,csr,val) (phys_write32((sc)->membase + (csr), (val)))

#if ENDIAN_BIG
static uint32_t
swap4(uint32_t x)
{
    uint32_t t;

    t = ((x & 0xFF00FF00) >> 8) | ((x & 0x00FF00FF) << 8);
    return (t >> 16) | ((t & 0xFFFF) << 16);
}
#else
#define swap4(x) (x)
#endif


/* Packet management */

static eth_pkt_t *
eth_alloc_pkt(rtl8139_softc *sc)
{
    eth_pkt_t *pkt;

    CS_ENTER(sc);
    pkt = (eth_pkt_t *) q_deqnext(&sc->freelist);
    CS_EXIT(sc);
    if (!pkt) return NULL;

    pkt->buffer = pkt->data;
    pkt->length = ETH_PKTBUF_LEN;
    pkt->flags = 0;

    return pkt;
}

static void
eth_free_pkt(rtl8139_softc *sc, eth_pkt_t *pkt)
{
    CS_ENTER(sc);
    q_enqueue(&sc->freelist, &pkt->next);
    CS_EXIT(sc);
}


static void
eth_initfreelist(rtl8139_softc *sc)
{
    int idx;
    uint8_t *ptr;
    eth_pkt_t *pkt;

    q_init(&sc->freelist);

    ptr = sc->pktpool;
    for (idx = 0; idx < ETH_PKTPOOL_SIZE; idx++) {
	pkt = (eth_pkt_t *) ptr;
	eth_free_pkt(sc, pkt);
	ptr += ETH_PKTBUF_SIZE;
	}
}


/* Utilities */

static const char *
rtl8139_devname(rtl8139_softc *sc)
{
    return (sc->devctx != NULL ? cfe_device_name(sc->devctx) : "eth?");
}


/*  Receive buffer processing.

   The RTL8139 does not use a descriptor ring.  Packets are DMA'd into
   an area of memory that's used as a packet ring buffer; packets are
   prefixed with a length/status word and placed end-to-end in the
   buffer, with wrap at the buffer boundaries.

   Roughly speaking, R_CAPR contains the consumer pointer and R_CBR
   contains the producer pointer, but there are complications not well
   documented in the data sheet: (1) R_CAPR trails the location of the
   status word by 16 bytes.  (2) R_CBR can point into a packet with
   DMA in progress, in which case the length will be read as 0xFFF0.  */

/* The following two definitions must be consistent.  */
#define RX_BUFFSIZE (64*1024)
#define RBLEN_CODE  K_RBLEN_64K

static void
rtl8139_rx_callback(rtl8139_softc *sc, eth_pkt_t *pkt)
{
    if (RTK_DEBUG) show_packet('>', pkt);   /* debug */

    CS_ENTER(sc);
    q_enqueue(&sc->rxqueue, &pkt->next);
    CS_EXIT(sc);
    sc->inpkts++;
}

static void
rtl8139_procrxbuffer(rtl8139_softc *sc)
{
    uint16_t capr, cbr;
    unsigned int ci, total;
    /*volatile*/ uint8_t *base = sc->rx_buffbase;

    capr = READCSR2(sc, R_CAPR);
    ci = (capr + 16) % RX_BUFFSIZE;
    cbr = READCSR2(sc, R_CBR);
    total = (ci <= cbr ? cbr : RX_BUFFSIZE + cbr) - ci;

    while ((READCSR1(sc, R_CR) & M_CR_BUFE) == 0) {
	eth_pkt_t *pkt;
	uint32_t status;
	size_t length, count, incr, n;
	
        status = swap4(*(uint32_t *)(base + ci));
	length = G_RS_LEN(status);
	ci = (ci + sizeof(uint32_t)) % RX_BUFFSIZE;
	total -= sizeof(uint32_t);

	if (length == 0xFFF0 || length > total)  /* DMA still in progress. */
	    break;

	incr = (length + 3) & ~3;
	total -= incr;
	if ((status & M_RS_ROK) != 0 && length > ENET_CRC_SIZE) {
	    pkt = eth_alloc_pkt(sc);
	    if (pkt == NULL) {
		/* If there are no free buffers, stop here.  Deadlock? */
		return;
		}
	    count = length;
	    if (count > MAX_ETHER_PACK) count = MAX_ETHER_PACK;
	    count -= ENET_CRC_SIZE;
	    pkt->length = count;
	    n = RX_BUFFSIZE - ci;
	    if (n > count)
	        n = count;
	    blockcopy(pkt->data, base + ci, n);
	    count -= n;
	    if (count > 0)
		blockcopy(pkt->data + n, base, count);

	    /* Release buffer space before the upcall. */
	    capr = (capr + sizeof(uint32_t) + incr) % RX_BUFFSIZE;
	    WRITECSR2(sc, R_CAPR, capr);

	    rtl8139_rx_callback(sc, pkt);
	    }
	else {
	    /* Some kind of error or runt packet.  Try skipping it. */
	    capr = (capr + sizeof(uint32_t) + incr) % RX_BUFFSIZE;
	    WRITECSR2(sc, R_CAPR, capr);
	    /* Perhaps we should reset and exit. */
	    }

	/* Do a readback to force the write and allow time for status
           to propagate to R_CR bits. */
	(void)READCSR2(sc, R_CAPR);   /* push, delay */

	ci = (ci + ((length + 3) & ~3)) % RX_BUFFSIZE;
	}
}


/*  Transmit ring processing.

    The RTL has 4 register pairs (R_TSDn, R_TSADn) that approximately
    implement a conventional descriptor ring of length 4.  */

static int
rtl8139_transmit(rtl8139_softc *sc, eth_pkt_t *pkt)
{
    int rv;
    unsigned pi = sc->tx_pi;

    if (RTK_DEBUG) show_packet('<', pkt);   /* debug */

    /* The RTL8139 does not auto-pad short packets */
    if (pkt->length < MIN_ETHER_PACK - ENET_CRC_SIZE) {
	unsigned int min = MIN_ETHER_PACK - ENET_CRC_SIZE;
	memset(pkt->buffer + pkt->length, 0, min - pkt->length);
	pkt->length = min;
	}

    CS_ENTER(sc);
    if (sc->tx_dmacnt < N_TSD) {
        uint32_t status;
	pci_addr_t addr = PTR_TO_PCI(pkt->buffer);

	WRITECSR4(sc, R_TSAD(pi), addr);
	status = V_TS_SIZE(pkt->length) | sc->tx_thresh;  /* clears OWN */
	WRITECSR4(sc, R_TSD(pi), status);
	(void)READCSR4(sc, R_TSD(pi));  /* push */
	sc->tx_dmacnt++;
	CS_EXIT(sc);
	sc->tx_pi = (sc->tx_pi + 1) % N_TSD;
	rv = 0;
	}
    else {
	CS_EXIT(sc);
	rv = -1;
	}

    sc->outpkts++;

    return rv;
}

static void
rtl8139_proctxring(rtl8139_softc *sc)
{
    unsigned ci = sc->tx_ci;
    unsigned count = sc->tx_dmacnt;
    uint32_t status;
    pci_addr_t addr;
    eth_pkt_t *pkt;

    /* Alternative: look for completion bits. */
    while (count > 0) {
        status = READCSR4(sc, R_TSD(ci));
	if ((status & M_TS_OWN) == 0)
	    break;
	addr = READCSR4(sc, R_TSAD(ci));
	pkt = ETH_PKT_BASE(PCI_TO_PTR(addr));
	eth_free_pkt(sc, pkt);
	count--;
	ci = (ci + 1) % N_TSD;
	}
    sc->tx_ci = ci;
    sc->tx_dmacnt = count;
}


static int
rtl8139_init(rtl8139_softc *sc)
{
    /* Allocate buffer pool */
    sc->pktpool = KMALLOC(ETH_PKTPOOL_SIZE*ETH_PKTBUF_SIZE, CACHE_ALIGN);
    eth_initfreelist(sc);
    q_init(&sc->rxqueue);

    /* Allocate an rx buffer */
    sc->rx_buffbase = KMALLOC(RX_BUFFSIZE + 16, 16);

    /* Initialize tx pointers */
    sc->tx_pi = sc->tx_ci = 0;
    sc->tx_dmacnt = 0;

    return 0;
}


static void
rtl8139_resetbuffers(rtl8139_softc *sc)
{
    unsigned ci = sc->tx_ci;
    unsigned count = sc->tx_dmacnt;
    uint32_t status;
    pci_addr_t addr;
    eth_pkt_t *pkt;
    
    while (count > 0) {
	status = READCSR4(sc, R_TSD(ci));
	status &= ~M_TS_SIZE;
	status |= M_TS_OWN;
        WRITECSR4(sc, R_TSD(ci), status);
	addr = READCSR4(sc, R_TSAD(ci));
	pkt = ETH_PKT_BASE(PCI_TO_PTR(addr));
	eth_free_pkt(sc, pkt);
	count--;
	ci = (ci + 1) % N_TSD;
	}

    sc->tx_pi = sc->tx_ci = 0;
    sc->tx_dmacnt = 0;
}


/* CRC */


/* EEPROM access (NYI) */

/* For now, assume that IDR is initialized correctly. */
static void
rtl8139_get_pm_addr(rtl8139_softc *sc, uint8_t buf[])
{
    int i;

    for (i = 0; i < ENET_ADDR_LEN; i++)
	buf[i] = READCSR1(sc, R_IDR(i));
}


/* MII access */

static void
mii_set_speed(rtl8139_softc *sc, int speed)
{
    /* NYI */
}

static void
mii_autonegotiate(rtl8139_softc *sc)
{
    uint16_t  status, remote;
    int linkspeed;

    linkspeed = ETHER_SPEED_UNKNOWN;

    /* Read twice to clear latching bits */
    status = READCSR2(sc, R_BMSR);
    status = READCSR2(sc, R_BMSR);
    sc->phy_status = status;

    if ((status & (BMSR_AUTONEG | BMSR_LINKSTAT)) !=
        (BMSR_AUTONEG | BMSR_LINKSTAT))
	status &= ~BMSR_ANCOMPLETE;

    xprintf("%s: Link speed: ", rtl8139_devname(sc));
    remote = READCSR2(sc, R_ANLPAR);
    if ((status & BMSR_ANCOMPLETE) != 0) {
	/* A link partner was negotiated... */

	if ((remote & ANLPAR_TXFD) != 0) {
	    xprintf("100BaseT FDX\n");
	    linkspeed = ETHER_SPEED_100FDX;
	    }
	else if ((remote & ANLPAR_TXHD) != 0) {
	    xprintf("100BaseT HDX\n");
	    linkspeed = ETHER_SPEED_100HDX;
	    }
	else if ((remote & ANLPAR_10FD) != 0) {
	    xprintf("10BaseT FDX\n");
	    linkspeed = ETHER_SPEED_10FDX;
	    }
	else if ((remote & ANLPAR_10HD) != 0) {
	    xprintf("10BaseT HDX\n");
	    linkspeed = ETHER_SPEED_10HDX;
	    }
	}
    else {
	/* no link partner convergence */
	xprintf("Unknown\n");
	linkspeed = ETHER_SPEED_UNKNOWN;
	}
    sc->linkspeed = linkspeed;

    /* clear latching bits */
    status = READCSR2(sc, R_BMSR);
}


static int
rtl8139_reset(rtl8139_softc *sc)
{
    uint8_t cr;
    int i;

    WRITECSR1(sc, R_CR, M_CR_RST);
    for (i = CFE_HZ/2; i > 0; i -= 5) {
	cfe_sleep(5);
	cr = READCSR1(sc, R_CR);
	if ((cr & M_CR_RST) == 0)
	    break;
	}
    return (i <= 0 ? -1 : 0);
}


static void
rtl8139_hwinit(rtl8139_softc *sc)
{
    if (sc->state == eth_state_uninit) {
        uint32_t rcr, tcr;

	rtl8139_reset(sc);

	/* This might seem too early, but rx and tx apparently must be
           enabled before the writes to their configuration registers
           are effective. */
	WRITECSR1(sc, R_CR, M_CR_TE | M_CR_RE);

	rcr = READCSR4(sc, R_RCR);
	rcr &= ~(M_RCR_RBLEN | M_RCR_MXDMA | M_RCR_RXFTH);
	rcr |= V_RCR_RBLEN(RBLEN_CODE);
	rcr |= V_RCR_MXDMA(K_MXDMA_UNLIMITED);
	rcr |= V_RCR_RXFTH(K_RXFTH_1024);
	rcr |= (M_RCR_APM | M_RCR_AB);
	WRITECSR4(sc, R_RCR, rcr);

	WRITECSR4(sc, R_RBSTART, PTR_TO_PCI(sc->rx_buffbase));

	tcr = READCSR4(sc, R_TCR);
	tcr &= ~(M_TCR_MXDMA | M_TCR_IFG);
	tcr |= V_TCR_MXDMA(K_MXDMA_2048);
	tcr |= V_TCR_IFG(K_802_3_IFG);
	WRITECSR4(sc, R_TCR, tcr);

	sc->tx_thresh = V_TS_ERTXTH(ENCODE_TXTH(32));

	WRITECSR1(sc, R_CONFIG1, M_CFG1_DVRLOAD);
	WRITECSR4(sc, R_MPC, 0);

	mii_autonegotiate(sc);
	
	sc->state = eth_state_on;
	}
}

  
static void
rtl8139_setspeed(rtl8139_softc *sc, int speed)
{
    /* XXX Not yet implemented - autonegotiation only. */
    (void)mii_set_speed;
}

static void
rtl8139_setloopback(rtl8139_softc *sc, int mode)
{
    /* XXX Not yet implemented. */
}


static void
rtl8139_isr(void *arg)
{
    rtl8139_softc *sc = (rtl8139_softc *)arg;
    uint16_t status;

#if IPOLL
    sc->interrupts++;
#endif

    for (;;) {

	/* Read and clear the interrupt status. */
	status = READCSR2(sc, R_ISR);
	status &= sc->intmask;
	if (status == 0)
	    break;

	WRITECSR2(sc, R_ISR, status);  /* write ones to clear (read doesn't) */

	if (status & (M_INT_ROK | M_INT_RER)) {
#if IPOLL
	    sc->rx_interrupts++;
#endif
	    rtl8139_procrxbuffer(sc);
	    }

	if (status & (M_INT_TOK | M_INT_TER)) {
#if IPOLL
	    sc->tx_interrupts++;
#endif
	    rtl8139_proctxring(sc);
	    }

	if (status & M_INT_PUN_LINKCHG) {
	    sc->intmask &= ~M_INT_PUN_LINKCHG;
#if IPOLL
	    WRITECSR2(sc, R_IMR, sc->intmask);
#endif
	    /* Check the PHY for link status change. */
	    sc->phy_check = 1;
	    }
	}

	/* XXX Handle SERR, etc. */
}

static void
rtl8139_checkphy(rtl8139_softc *sc)
{
    uint8_t mediastat;
    uint16_t status;

    mediastat = READCSR1(sc, R_MSR);
    status = READCSR2(sc, R_BMSR);
    if (status != sc->phy_status) {
	mii_autonegotiate(sc);
	}

    sc->intmask |= M_INT_PUN_LINKCHG;
#if IPOLL
    WRITECSR2(sc, R_IMR, sc->intmask);
#endif
}

static void
rtl8139_start(rtl8139_softc *sc)
{
    uint16_t isr;

    rtl8139_hwinit(sc);

    /* Set up loopback here */

    sc->intmask = 0;
    WRITECSR2(sc, R_IMR, 0);
    isr = READCSR2(sc, R_ISR);           /* clear any pending */
    WRITECSR2(sc, R_ISR, isr);

    sc->phy_check = 0;

    sc->intmask =  (M_INT_ROK | M_INT_RER
		    | M_INT_TOK | M_INT_TER
		    | M_INT_PUN_LINKCHG
		    | M_INT_SERR);

    /* XXX more? */

#if IPOLL
    cfe_request_irq(sc->irq, rtl8139_isr, sc, CFE_IRQ_FLAGS_SHARED, 0);
    WRITECSR2(sc, R_IMR, sc->intmask);
#endif

    WRITECSR1(sc, R_CR, M_CR_TE | M_CR_RE);
    sc->state = eth_state_on;
}

static void
rtl8139_stop(rtl8139_softc *sc)
{
    uint32_t cr;
    int count;

    /* Make sure that no further interrutps will be processed. */
    sc->intmask = 0;
    WRITECSR2(sc, R_IMR, 0);
    (void)READCSR2(sc, R_IMR);  /* push */
#if IPOLL
    cfe_free_irq(sc->irq, 0);
#endif

    WRITECSR1(sc, R_CR, 0);
    for (count = 0; count <= 10; count++) {
	cr = READCSR1(sc, R_CR);
	if ((cr & (M_CR_TE | M_CR_RE)) == 0)
	    break;
	cfe_sleep(CFE_HZ/10);
	}
    if (count > 10) {
	xprintf("%s: DMA idle not achieved\n", rtl8139_devname(sc));
	rtl8139_reset(sc);
	sc->state = eth_state_uninit;
	}
}


/* Declarations for CFE Device Driver Interface routines */

static int rtl8139_ether_open(cfe_devctx_t *ctx);
static int rtl8139_ether_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int rtl8139_ether_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int rtl8139_ether_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int rtl8139_ether_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int rtl8139_ether_close(cfe_devctx_t *ctx);
static void rtl8139_ether_poll(cfe_devctx_t *ctx, int64_t ticks);
static void rtl8139_ether_reset(void *softc);


/* CFE Device Driver dispatch structure */

const static cfe_devdisp_t rtl8139_ether_dispatch = {
    rtl8139_ether_open,
    rtl8139_ether_read,
    rtl8139_ether_inpstat,
    rtl8139_ether_write,
    rtl8139_ether_ioctl,
    rtl8139_ether_close,
    rtl8139_ether_poll,
    rtl8139_ether_reset
};


/* CFE Device Driver descriptor */

const cfe_driver_t rtl8139drv = {
    "RTL8139 Ethernet",
    "eth",
    CFE_DEV_NETWORK,
    &rtl8139_ether_dispatch,
    rtl8139_ether_probe
};


/* CFE Device Driver probe functions. */

static int
rtl8139_ether_attach(cfe_driver_t *drv, pcitag_t tag, int index)
{
    rtl8139_softc *sc;
    uint32_t device;
    uint32_t class;
    phys_addr_t pa;
    uint8_t promaddr[ENET_ADDR_LEN];
    char descr[100];

    device = pci_conf_read(tag, PCI_ID_REG);
    class = pci_conf_read(tag, PCI_CLASS_REG);

#if 1
    /* Use memory space for the CSRs */
    pci_map_mem(tag, PCI_MAPREG(1), PCI_MATCH_BITS, &pa);
#else
    /* Use i/o space for the CSRs */
    pci_map_io(tag, PCI_MAPREG(0), PCI_MATCH_BITS, &pa);
#endif

    sc = (rtl8139_softc *) KMALLOC(sizeof(rtl8139_softc), 0);

    if (sc == NULL) {
	xprintf("RTL8139: No memory to complete probe\n");
	return 0;
	}
    memset(sc, 0, sizeof(*sc));

    sc->membase = (uint32_t)pa;
    sc->irq = pci_conf_read(tag, PCI_BPARAM_INTERRUPT_REG) & 0xFF;
    sc->tag = tag;
    sc->device = PCI_PRODUCT(device);
    sc->revision = PCI_REVISION(class);
    sc->devctx = NULL;

#if 1
    sc->linkspeed = ETHER_SPEED_AUTO;    /* select autonegotiation */
#else
    sc->linkspeed = ETHER_SPEED_100FDX;  /* 100 Mbps, full duplex */
#endif
    sc->loopback = ETHER_LOOPBACK_OFF;

    rtl8139_init(sc);

    /* Use the address in EEPROM.  This will be read into the
       IDR0 .. IDR5 registers upon power up.  */
    rtl8139_get_pm_addr(sc, promaddr);
    memcpy(sc->hwaddr, promaddr, ENET_ADDR_LEN);

    sc->state = eth_state_uninit;

    xsprintf(descr, "%s at 0x%X (%a)",
	     drv->drv_description, sc->membase, sc->hwaddr);

    cfe_attach(drv, sc, NULL, descr);
    return 1;
}

static void
rtl8139_ether_probe(cfe_driver_t *drv,
		    unsigned long probe_a, unsigned long probe_b, 
		    void *probe_ptr)
{
    int n;

    n = 0;
    for (;;) {
	pcitag_t tag;

	if (pci_find_device(K_PCI_VENDOR_REALTEK, K_PCI_ID_RTL8139, n, &tag)
	    != 0)
	    break;
	rtl8139_ether_attach(drv, tag, n);
	n++;
	}
}


/* The functions below are called via the dispatch vector for the 8139. */

static int
rtl8139_ether_open(cfe_devctx_t *ctx)
{
    rtl8139_softc *sc = ctx->dev_softc;

    if (sc->state == eth_state_on)
	rtl8139_stop(sc);

    sc->devctx = ctx;

    sc->inpkts = sc->outpkts = 0;
    sc->interrupts = 0;
    sc->rx_interrupts = sc->tx_interrupts = 0;

    rtl8139_start(sc);

#if XPOLL
    rtl8139_isr(sc);
#endif

    return 0;
}

static int
rtl8139_ether_read(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    rtl8139_softc *sc = ctx->dev_softc;
    eth_pkt_t *pkt;
    int blen;

#if XPOLL
    rtl8139_isr(sc);
#endif

    if (sc->state != eth_state_on) return -1;

    CS_ENTER(sc);
    pkt = (eth_pkt_t *) q_deqnext(&(sc->rxqueue));
    CS_EXIT(sc);

    if (pkt == NULL) {
	buffer->buf_retlen = 0;
	return 0;
	}

    blen = buffer->buf_length;
    if (blen > pkt->length) blen = pkt->length;

    hs_memcpy_to_hs(buffer->buf_ptr, pkt->buffer, blen);
    buffer->buf_retlen = blen;

    eth_free_pkt(sc, pkt);

#if XPOLL
    rtl8139_isr(sc);
#endif

    return 0;
}

static int
rtl8139_ether_inpstat(cfe_devctx_t *ctx, iocb_inpstat_t *inpstat)
{
    rtl8139_softc *sc = ctx->dev_softc;

#if XPOLL
    rtl8139_isr(sc);
#endif

    if (sc->state != eth_state_on) return -1;

    /* We avoid an interlock here because the result is a hint and an
       interrupt cannot turn a non-empty queue into an empty one. */
    inpstat->inp_status = (q_isempty(&(sc->rxqueue))) ? 0 : 1;

    return 0;
}

static int
rtl8139_ether_write(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    rtl8139_softc *sc = ctx->dev_softc;
    eth_pkt_t *pkt;
    int blen;

#if XPOLL
    rtl8139_isr(sc);
#endif

    if (sc->state != eth_state_on) return -1;

    pkt = eth_alloc_pkt(sc);
    if (!pkt) return CFE_ERR_NOMEM;

    blen = buffer->buf_length;
    if (blen > pkt->length) blen = pkt->length;

    hs_memcpy_from_hs(pkt->buffer, buffer->buf_ptr, blen);
    pkt->length = blen;

    if (rtl8139_transmit(sc, pkt) != 0) {
	eth_free_pkt(sc,pkt);
	return CFE_ERR_IOERR;
	}

#if XPOLL
    rtl8139_isr(sc);
#endif

    return 0;
}

static int
rtl8139_ether_ioctl(cfe_devctx_t *ctx, iocb_buffer_t *buffer) 
{
    rtl8139_softc *sc = ctx->dev_softc;
    int   mode;
    int   speed;

    switch ((int)buffer->buf_ioctlcmd) {
	case IOCTL_ETHER_GETHWADDR:
	    hs_memcpy_to_hs(buffer->buf_ptr, sc->hwaddr, sizeof(sc->hwaddr));
	    return 0;

	case IOCTL_ETHER_SETHWADDR:
	    return -1;    /* not supported */

	case IOCTL_ETHER_GETSPEED:
	    speed = sc->linkspeed;
	    hs_memcpy_to_hs(buffer->buf_ptr,&speed,sizeof(speed));
	    return 0;

	case IOCTL_ETHER_SETSPEED:
	    hs_memcpy_from_hs(&speed,buffer->buf_ptr,sizeof(speed));
	    rtl8139_setspeed(sc, speed);
	    return -1;    /* not supported yet */

	case IOCTL_ETHER_GETLINK:
	    speed = sc->linkspeed;
	    hs_memcpy_to_hs(buffer->buf_ptr,&speed,sizeof(speed));
	    return 0;

	case IOCTL_ETHER_GETLOOPBACK:
	    speed = sc->loopback;
	    hs_memcpy_to_hs(buffer->buf_ptr,&speed,sizeof(speed));
	    return 0;

	case IOCTL_ETHER_SETLOOPBACK:
	    hs_memcpy_from_hs(&mode,buffer->buf_ptr,sizeof(mode));
	    sc->loopback = ETHER_LOOPBACK_OFF;  /* default */
	    if (mode == ETHER_LOOPBACK_INT || mode == ETHER_LOOPBACK_EXT) {
		rtl8139_setloopback(sc, mode);
		}
	    return -1;    /* not supported yet */

	default:
	    return -1;
	}
}

static int
rtl8139_ether_close(cfe_devctx_t *ctx)
{
    rtl8139_softc *sc = ctx->dev_softc;

    sc->state = eth_state_off;
    rtl8139_stop(sc);

    xprintf("%s: %d sent, %d received, %d interrupts\n",
	    rtl8139_devname(sc), sc->outpkts, sc->inpkts, sc->interrupts);
    xprintf("  %d rx interrupts, %d tx interrupts\n",
	    sc->rx_interrupts, sc->tx_interrupts);

    /* resynchronize buffers */
    rtl8139_resetbuffers(sc);

    sc->devctx = NULL;
#if 1  /* XXX Gentler resets don't work (yet?) */
    sc->state = eth_state_uninit;
#endif
    return 0;
}

static void
rtl8139_ether_poll(cfe_devctx_t *ctx, int64_t ticks)
{
    rtl8139_softc *sc = ctx->dev_softc;

    if (sc->phy_check) {
	sc->phy_check = 0;
	rtl8139_checkphy(sc);
	}
}

static void
rtl8139_ether_reset(void *softc)
{
    rtl8139_softc *sc = (rtl8139_softc *)softc;

    /* Turn off the Ethernet interface. */
    if (sc->state == eth_state_on)
      rtl8139_stop(sc);
    rtl8139_reset(sc);

    sc->state = eth_state_uninit;
}
