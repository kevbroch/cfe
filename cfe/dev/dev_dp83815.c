/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  NS DP83815 Ethernet Driver		      File: dev_dp83815.c
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
#error "this is broken now."
extern void *CPUCFG_MEMCPY(void *dest, const void *src, size_t cnt);
#define blockcopy CPUCFG_MEMCPY
#else
#define blockcopy memcpy
#endif
#include "cfe_irq.h"

#include "net_enet.h"

#include "pcivar.h"
#include "pcireg.h"

#include "dp83815.h"
#include "mii.h"

/* This is a driver for the National Semiconductor DP83815 (MacPhyter)
   10/100 MAC with integrated PHY.

   The current version has been developed for the Netgear FA311 and
   FA312 NICs.  These include an EEPROM with automatically loaded
   setup information that includes station address filtering.
   Operation without such an EEPROM has not been tested.

   The SB1250 version takes advantage of DMA coherence and uses
   "preserve bit lanes" addresses for all accesses that cross the
   ZBbus-PCI bridge.  For non-coherent memory systems, all addresses
   of descriptors are converted to do uncached access, and packet data
   is flushed as required.  */

#ifndef MACPHYTER_DEBUG
#define MACPHYTER_DEBUG 0
#endif
#ifndef MACPHYTER_TEST
#define MACPHYTER_TEST  0
#endif

#if ((ENDIAN_BIG + ENDIAN_LITTLE) != 1)
#error "dev_dp83815: system endian not set"
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

#define MIN_ETHER_PACK  (ENET_MIN_PKT+ENET_CRC_SIZE)   /* min packet size */
#define MAX_ETHER_PACK  (ENET_MAX_PKT+ENET_CRC_SIZE)   /* max packet size */

/* Packet buffers.  For the DP83815, an rx packet must be aligned to a
   32-bit word boundary, and we would like it aligned to a cache line
   boundary for performance.  Also, the buffers "should" be allocated
   in 32 byte multiples (5.3.2). */

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
#define ALIGN(n,align)    (((n)+((align)-1)) & ~((align)-1))

#define ETH_PKTBUF_LINES  ((sizeof(eth_pkt_t) + (CACHE_ALIGN-1))/CACHE_ALIGN)
#define ETH_PKTBUF_SIZE   (ETH_PKTBUF_LINES*CACHE_ALIGN)
#define ETH_PKTBUF_OFFSET (offsetof(eth_pkt_t, data))

#define ETH_PKT_BASE(data) ((eth_pkt_t *)((data) - ETH_PKTBUF_OFFSET))

/* packet flags */
#define ETH_TX_SETUP	 1     /* assumes Perfect Filtering format */

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


/* Descriptor structures.  NOTE: To avoid having descriptors straddle
   cache lines, we append a pad word, ignored by DMA, to each.  */

typedef struct rx_dscr {
    pci_addr_t rxd_link;
    uint32_t   rxd_cmdsts;
    pci_addr_t rxd_bufptr;
    uint32_t   rxd_pad;
} rx_dscr;
	
typedef struct tx_dscr {
    pci_addr_t txd_link;
    uint32_t   txd_cmdsts;
    pci_addr_t txd_bufptr;
    uint32_t   txd_pad;
} tx_dscr;


/* Driver data structures */

typedef enum {
    eth_state_uninit,
    eth_state_off,
    eth_state_on, 
    eth_state_broken
} eth_state_t;

#define ETH_PKTPOOL_SIZE 32

typedef struct dp83815_softc {
    uint32_t membase;
    uint8_t irq;		/* interrupt mapping (used if IPOLL) */
    pcitag_t tag;               /* tag for configuration registers */

    uint8_t hwaddr[ENET_ADDR_LEN];                 
    uint16_t device;            /* chip device code */
    uint8_t revision;		/* chip revision and step */

    eth_state_t state;          /* current state */
    uint32_t intmask;           /* interrupt mask */

    /* These fields are set before calling dp83815_hwinit */
    int linkspeed;		/* encodings from cfe_ioctl */
    int loopback;

    /* Packet free list */
    queue_t freelist;
    uint8_t *pktpool;
    queue_t rxqueue;

    /* The descriptor tables */
    uint8_t    *rxdscrmem;	/* receive descriptors */
    uint8_t    *txdscrmem;	/* transmit descriptors */

    /* These fields keep track of where we are in tx/rx processing */
    volatile rx_dscr *rxdscr_start;	/* beginning of ring */
    volatile rx_dscr *rxdscr_end;	/* end of ring */
    volatile rx_dscr *rxdscr_remove;	/* next one we expect DMA to use */
    volatile rx_dscr *rxdscr_add;	/* next place to put a buffer */
    int      rxdscr_onring;

    volatile tx_dscr *txdscr_start;	/* beginning of ring */
    volatile tx_dscr *txdscr_end;	/* end of ring */
    volatile tx_dscr *txdscr_remove;	/* next one we will use for tx */
    volatile tx_dscr *txdscr_add;	/* next place to put a buffer */

    cfe_devctx_t *devctx;

    /* These fields describe the PHY */
    int phy_addr;
    int phy_check;
    uint32_t phy_status;
    uint32_t phy_vendor;
    uint16_t phy_device;

    /* Statistics */
    uint32_t inpkts;
    uint32_t outpkts;
    uint32_t interrupts;
    uint32_t rx_interrupts;
    uint32_t tx_interrupts;
    uint32_t bus_errors;
} dp83815_softc;


/* Entry to and exit from critical sections (currently relative to
   interrupts only, not SMP) */

#if CFG_INTERRUPTS
#define CS_ENTER(sc) cfe_disable_irq(sc->irq)
#define CS_EXIT(sc)  cfe_enable_irq(sc->irq)
#else
#define CS_ENTER(sc) ((void)0)
#define CS_EXIT(sc)  ((void)0)
#endif


/* Driver parameterization */

#define MAXRXDSCR      32
#define MAXTXDSCR      32
#define MINRXRING	8


/* Prototypes */

static void dp83815_ether_probe(cfe_driver_t *drv,
				unsigned long probe_a, unsigned long probe_b, 
				void *probe_ptr);


/* Address mapping macros */

/* Note that PTR_TO_PHYS only works with 32-bit addresses, but then
   so does the dp83815. */
#define PTR_TO_PHYS(x) (PHYSADDR((uintptr_t)(x)))
#define PHYS_TO_PTR(a) ((uint8_t *)KERNADDR(a))

#define PCI_TO_PTR(a)  (PHYS_TO_PTR(PCI_TO_PHYS(a)))
#define PTR_TO_PCI(x)  (PHYS_TO_PCI(PTR_TO_PHYS(x)))

#define READCSR(sc,csr)      (phys_read32((sc)->membase + (csr)))
#define WRITECSR(sc,csr,val) (phys_write32((sc)->membase + (csr), (val)))


#define RESET_ADAPTER(sc)				\
	{						\
	/* XXX */                                       \
	}


/* Debugging */

static void
dumpstat(dp83815_softc *sc)
{
    xprintf("-- CR = %08X  CFG = %08x\n",
	    READCSR(sc, R_CR), READCSR(sc, R_CFG));
}

static void
dumpcsrs(dp83815_softc *sc)
{
    int reg;

    xprintf("-------------\n");
    for (reg = 0; reg < R_MIBC; reg += 4) {
	xprintf("CSR %02X = %08X\n", reg, READCSR(sc, reg));
	}
    xprintf("-------------\n");
}


/* Packet management */

/*  *********************************************************************
    *  ETH_ALLOC_PKT(sc)
    *  
    *  Allocate a packet from the free list.
    *  
    *  Input parameters: 
    *  	   sc - eth structure
    *  	   
    *  Return value:
    *  	   pointer to packet structure, or NULL if none available
    ********************************************************************* */
static eth_pkt_t *
eth_alloc_pkt(dp83815_softc *sc)
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


/*  *********************************************************************
    *  ETH_FREE_PKT(sc,pkt)
    *  
    *  Return a packet to the free list
    *  
    *  Input parameters: 
    *  	   sc - sbmac structure
    *  	   pkt - packet to return
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
static void
eth_free_pkt(dp83815_softc *sc, eth_pkt_t *pkt)
{
    CS_ENTER(sc);
    q_enqueue(&sc->freelist, &pkt->next);
    CS_EXIT(sc);
}


/*  *********************************************************************
    *  ETH_INITFREELIST(sc)
    *  
    *  Initialize the buffer free list for this mac.  The memory
    *  allocated to the free list is carved up and placed on a linked
    *  list of buffers for use by the mac.
    *  
    *  Input parameters: 
    *  	   sc - eth structure
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
static void
eth_initfreelist(dp83815_softc *sc)
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
dp83815_devname(dp83815_softc *sc)
{
    return (sc->devctx != NULL ? cfe_device_name(sc->devctx) : "eth?");
}


/* Descriptor ring management */

static int
dp83815_add_rcvbuf(dp83815_softc *sc, eth_pkt_t *pkt)
{
    volatile rx_dscr *rxd;
    volatile rx_dscr *nextrxd;

    rxd = sc->rxdscr_add;

    /* Figure out where the next descriptor will go */
    nextrxd = rxd+1;
    if (nextrxd == sc->rxdscr_end) {
	nextrxd = sc->rxdscr_start;
	}

    /* If the next one is the same as our remove pointer, the ring is
       considered full. */
    if (nextrxd == sc->rxdscr_remove) return -1;

    rxd->rxd_bufptr = PTR_TO_PCI(pkt->buffer);
    rxd->rxd_cmdsts = M_DES1_INTR | V_DES1_SIZE(ETH_PKTBUF_LEN);

    /* success, advance the pointer */
    sc->rxdscr_add = nextrxd;

    return 0;
}

static void
dp83815_fillrxring(dp83815_softc *sc)
{
    eth_pkt_t *pkt;

    CS_ENTER(sc);
    while (1) {
	if (sc->rxdscr_onring >= MINRXRING) {
	    CS_EXIT(sc);
	    break;
	    }
	CS_EXIT(sc);
	pkt = eth_alloc_pkt(sc);
	if (pkt == NULL) {
	    /* could not allocate a buffer */
	    break;
	    }
	if (dp83815_add_rcvbuf(sc, pkt) != 0) {
	    /* could not add buffer to ring */
	    eth_free_pkt(sc, pkt);
	    break;
	    }
	CS_ENTER(sc);
	sc->rxdscr_onring++;
	}
}


/*  *********************************************************************
    *  DP83815_RX_CALLBACK(sc, pkt)
    *  
    *  Receive callback routine.  This routine is invoked when a
    *  buffer queued for receives is filled. In this simple driver,
    *  all we do is add the packet to a per-MAC queue for later
    *  processing, and try to put a new packet in the place of the one
    *  that was removed from the queue.
    *  
    *  Input parameters: 
    *  	   sc - interface
    *  	   ptk - packet context (eth_pkt structure)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
static void
dp83815_rx_callback(dp83815_softc *sc, eth_pkt_t *pkt)
{
    if (MACPHYTER_DEBUG) show_packet('>', pkt);   /* debug */

    CS_ENTER(sc);
    q_enqueue(&sc->rxqueue, &pkt->next);
    CS_EXIT(sc);
    sc->inpkts++;
}


static void
dp83815_procrxring(dp83815_softc *sc)
{
    volatile rx_dscr *rxd;
    eth_pkt_t *pkt;
    eth_pkt_t *newpkt;
    uint32_t cmdsts;

    for (;;) {
	rxd = sc->rxdscr_remove;

	cmdsts = rxd->rxd_cmdsts;
	if ((cmdsts & M_DES1_OWN) == 0) {
	    /* end of ring, no more packets */
	    break;
	    }

	pkt = ETH_PKT_BASE(PCI_TO_PTR(rxd->rxd_bufptr));
	pkt->length = G_DES1_SIZE(cmdsts) - ENET_CRC_SIZE;

	/* Drop error packets */
	if (cmdsts & M_DES1_RX_ERRORS) {
#if MACPHYTER_DEBUG
	    if (pkt->length >= MIN_ETHER_PACK - ENET_CRC_SIZE)
		xprintf("%s: rx error %08X\n", dp83815_devname(sc), cmdsts);
#endif
	    newpkt = pkt;          /* recycle the buffer */
	    }
	else {
	    /* Pass up the packet */
	    dp83815_rx_callback(sc, pkt);

	    /* put a buffer back on the ring to replace this one */
	    newpkt = eth_alloc_pkt(sc);
	    }

	/* update the pointer, accounting for buffer wrap. */
	rxd++;
	if (rxd == sc->rxdscr_end)
	    rxd = sc->rxdscr_start;
	sc->rxdscr_remove = rxd;

	if (newpkt) {
	    /* The ring must have space now. */
	    dp83815_add_rcvbuf(sc, newpkt);
	    }
	else {
	    CS_ENTER(sc);
	    sc->rxdscr_onring--;
	    CS_EXIT(sc);
	    }
	}
}


static int
dp83815_add_txbuf(dp83815_softc *sc, eth_pkt_t *pkt)
{
    volatile tx_dscr *txd;
    volatile tx_dscr *nexttxd;

    txd = sc->txdscr_add;

    /* Figure out where the next descriptor will go */
    nexttxd = (txd+1);
    if (nexttxd == sc->txdscr_end) {
	nexttxd = sc->txdscr_start;
	}

    /* If the next one is the same as our remove pointer,
       the ring is considered full.  (it actually has room for
       one more, but we reserve the remove == add case for "empty") */

    if (nexttxd == sc->txdscr_remove) return -1;

    txd->txd_bufptr = PTR_TO_PCI(pkt->buffer);
    txd->txd_cmdsts = M_DES1_INTR | M_DES1_OWN | V_DES1_SIZE(pkt->length);

    /* success, advance the pointer */
    sc->txdscr_add = nexttxd;

    return 0;
}


static int
dp83815_transmit(dp83815_softc *sc,eth_pkt_t *pkt)
{
    int rv;

    if (MACPHYTER_DEBUG) show_packet('<', pkt);   /* debug */

    rv = dp83815_add_txbuf(sc, pkt);
    sc->outpkts++;

    WRITECSR(sc, R_CR, M_CR_TXE | M_CR_RXE);
    return rv;
}


static void
dp83815_proctxring(dp83815_softc *sc)
{
    volatile tx_dscr *txd;
    eth_pkt_t *pkt;
    uint32_t cmdsts;

    for (;;) {
	txd = sc->txdscr_remove;

	if (txd == sc->txdscr_add) {
	    /* ring is empty, no buffers to process */
	    break;
	    }

	cmdsts = txd->txd_cmdsts;
	if (cmdsts & M_DES1_OWN) {
	    /* Reached a packet still being transmitted */
	    break;
	    }

	/* Just free the packet */
	pkt = ETH_PKT_BASE(PCI_TO_PTR(txd->txd_bufptr));
	eth_free_pkt(sc, pkt);

	/* update the pointer, accounting for buffer wrap. */
	txd++;
	if (txd == sc->txdscr_end)
	    txd = sc->txdscr_start;

	sc->txdscr_remove = txd;
	}
}


static void
dp83815_initrings(dp83815_softc *sc)
{
    volatile tx_dscr *txd, *txn;
    volatile rx_dscr *rxd, *rxn;

    /* Claim ownership of all descriptors for the driver */

    for (txd = sc->txdscr_start; txd != sc->txdscr_end; txd++) {
	txn = txd + 1;
	if (txn == sc->txdscr_end) txn = sc->txdscr_start;
        txd->txd_link = PTR_TO_PCI(txn);
        txd->txd_cmdsts = 0;
	txd->txd_pad = 0;
	}
    for (rxd = sc->rxdscr_start; rxd != sc->rxdscr_end; rxd++) {
	rxn = rxd + 1;
	if (rxn == sc->rxdscr_end) rxn = sc->rxdscr_start;
	rxd->rxd_link = PTR_TO_PCI(rxn);
        rxd->rxd_cmdsts = M_DES1_OWN;
	rxd->rxd_pad = 0;
	}

    /* Init the ring pointers */

    sc->txdscr_add = sc->txdscr_remove = sc->txdscr_start;
    sc->rxdscr_add = sc->rxdscr_remove = sc->rxdscr_start;
    sc->rxdscr_onring = 0;

    /* Add stuff to the receive ring */

    dp83815_fillrxring(sc);
}


/* Allocate an integral number of cache lines suitable for DMA access. */
static uint8_t *
dma_alloc(size_t size, unsigned int align)
{
    uint8_t *base;
    size_t len = ALIGN(size, CACHE_ALIGN);

    base = KMALLOC(len, ALIGN(align, CACHE_ALIGN));
    if (base != NULL)
	CACHE_DMA_INVAL(base, len);
    return base;
}

static int
dp83815_init(dp83815_softc *sc)
{
    /* Allocate descriptor rings */
    sc->rxdscrmem = CACHE_DMA_SHARED(dma_alloc(MAXRXDSCR*sizeof(rx_dscr), CACHE_ALIGN));
    sc->txdscrmem = CACHE_DMA_SHARED(dma_alloc(MAXTXDSCR*sizeof(tx_dscr), CACHE_ALIGN));

    /* Allocate buffer pool */
    sc->pktpool = dma_alloc(ETH_PKTPOOL_SIZE*ETH_PKTBUF_SIZE, CACHE_ALIGN);
    eth_initfreelist(sc);
    q_init(&sc->rxqueue);

    /* Fill in pointers to the rings */
    sc->rxdscr_start = (volatile rx_dscr *) (sc->rxdscrmem);
    sc->rxdscr_end = sc->rxdscr_start + MAXRXDSCR;
    sc->rxdscr_add = sc->rxdscr_start;
    sc->rxdscr_remove = sc->rxdscr_start;
    sc->rxdscr_onring = 0;

    sc->txdscr_start = (volatile tx_dscr *) (sc->txdscrmem);
    sc->txdscr_end = sc->txdscr_start + MAXTXDSCR;
    sc->txdscr_add = sc->txdscr_start;
    sc->txdscr_remove = sc->txdscr_start;

    dp83815_initrings(sc);

    return 0;       
}


static void
dp83815_resetrings(dp83815_softc *sc)
{
    volatile tx_dscr *txd;
    volatile rx_dscr *rxd;
    eth_pkt_t *pkt;

    /* Free already-sent descriptors and buffers */
    dp83815_proctxring(sc);

    /* Free any pending but unsent */
    txd = sc->txdscr_remove;
    while (txd != sc->txdscr_add) {
	txd->txd_cmdsts &=~ M_DES1_OWN;
	pkt = ETH_PKT_BASE(PCI_TO_PTR(txd->txd_bufptr));
	eth_free_pkt(sc, pkt);

	txd++;
	if (txd == sc->txdscr_end)
	  txd = sc->txdscr_start;
        }
    sc->txdscr_add = sc->txdscr_remove;

    /* Discard any received packets as well as all free buffers */
    rxd = sc->rxdscr_remove;
    while (rxd != sc->rxdscr_add) {
	rxd->rxd_cmdsts |= M_DES1_OWN;
	pkt = ETH_PKT_BASE(PCI_TO_PTR(rxd->rxd_bufptr));
	eth_free_pkt(sc, pkt);
	
	rxd++;
	if (rxd == sc->rxdscr_end)
	    rxd = sc->rxdscr_start;
	CS_ENTER(sc);
	sc->rxdscr_onring--;
	CS_EXIT(sc);
	}

    /* Reestablish the initial state. */
    dp83815_initrings(sc);
}


#if 0 /* XXX Multicast filtering not yet implemented. */
/* CRCs */

static uint32_t
dp83815_crc32(const uint8_t *databuf, unsigned int datalen) 
{       
    unsigned int idx, bit, data;
    uint32_t crc;

    crc = 0xFFFFFFFFUL;
    for (idx = 0; idx < datalen; idx++)
	for (data = *databuf++, bit = 0; bit < 8; bit++, data >>= 1)
	    crc = (crc >> 1) ^ (((crc ^ data) & 1) ? ENET_CRC32_POLY : 0);
    return crc;
}

#define dp83815_mchash(mca)       (dp83815_crc32((mca), 6) & 0x1FF)
#endif
  

#if MACPHYTER_TEST
/* EEPROM access */

/* Current NICs use the EEPROM auto-load feature and there is no need
   for explicit EEPROM access.  The following routines are included
   for future applications and have been tested (Netgear FA311).  */

/*
 * The recommended EEPROM is the NM9306.
 * Delays below are chosen to meet specs for NS93C64 (slow M variant).
 * Current parts are faster.
 *     Reference:  NS Memory Data Book, 1994
 */

#define EEPROM_SIZE              (2*0x0C)
#define EEPROM_MAX_CYCLES        32

#define EEPROM_CMD_BITS          3
#define EEPROM_ADDR_BITS         6

#define K_EEPROM_READ_CMD        06
#define K_EEPROM_WRITE_CMD       05

#define EEPROM_CRC_INDEX         (EEPROM_SIZE-2)

#define EEPROM_WORD(rom,offset) ((rom)[offset] | ((rom)[offset+1] << 8))

static void
eeprom_idle_state(dp83815_softc *sc)
{
    uint32_t ctrl;
    unsigned int i;

    ctrl = READCSR(sc, R_MEAR);

    ctrl |= M_MEAR_EESEL;
    WRITECSR(sc, R_MEAR, ctrl);
    cfe_nsleep(100);           /* CS setup (Tcss=100) */

    /* Run the clock through the maximum number of pending read cycles */
    for (i = 0; i < EEPROM_MAX_CYCLES*2; i++) {
	ctrl ^= M_MEAR_EECLK;
	WRITECSR(sc, R_MEAR, ctrl);
	cfe_nsleep(1000);      /* SK period (Fsk=0.5MHz) */
	}

    /* Deassert EEPROM Chip Select */
    ctrl &=~ M_MEAR_EESEL;
    WRITECSR(sc, R_MEAR, ctrl);
    cfe_nsleep(50);            /* CS recovery (Tsks=50) */
}

static void
eeprom_send_command_bit(dp83815_softc *sc, unsigned int data)
{
    uint32_t  ctrl;

    ctrl = READCSR(sc, R_MEAR);

    /* Place the data bit on the bus */
    if (data == 1)
	ctrl |= M_MEAR_EEDI;
    else
	ctrl &=~ M_MEAR_EEDI;

    WRITECSR(sc, R_MEAR, ctrl);
    cfe_nsleep(360);                  /* setup: Tdis=200 */

    /* Now clock the data into the EEPROM */
    WRITECSR(sc, R_MEAR, ctrl | M_MEAR_EECLK);
    cfe_nsleep(900);                  /* clock high, Tskh=500 */
    WRITECSR(sc, R_MEAR, ctrl);
    cfe_nsleep(450);                  /* clock low, Tskl=250 */

    /* Now clear the data bit */
    ctrl &=~ M_MEAR_EEDI;             /* data invalid, Tidh=20 for SK^ */
    WRITECSR(sc, R_MEAR, ctrl);
    cfe_nsleep(270);                  /* min cycle, 1/Fsk=2000 */
}

static uint16_t
eeprom_read_bit(dp83815_softc *sc)
{
    uint32_t  ctrl;

    ctrl = READCSR(sc, R_MEAR);

    /* Generate a clock cycle before doing a read */
    WRITECSR(sc, R_MEAR, ctrl | M_MEAR_EECLK);     /* rising edge */
    cfe_nsleep(1000);                 /* clock high, Tskh=500, Tpd=1000 */
    WRITECSR(sc, R_MEAR, ctrl);                    /* falling edge */
    cfe_nsleep(1000);                 /* clock low, 1/Fsk=2000 */

    ctrl = READCSR(sc, R_MEAR);
    return ((ctrl & M_MEAR_EEDO) != 0 ? 1 : 0);
}

#define CMD_BIT_MASK (1 << (EEPROM_CMD_BITS+EEPROM_ADDR_BITS-1))

static uint16_t
eeprom_read_word(dp83815_softc *sc, unsigned int index)
{
    uint16_t command, word;
    uint32_t ctrl;
    unsigned int i;

    ctrl = READCSR(sc, R_MEAR) | M_MEAR_EESEL;

    /* Assert the EEPROM CS line */
    WRITECSR(sc, R_MEAR, ctrl);
    cfe_nsleep(100);           /* CS setup, Tcss = 100 */

    /* Send the read command to the EEPROM */
    command = (K_EEPROM_READ_CMD << EEPROM_ADDR_BITS) | index;
    for (i = 0; i < EEPROM_CMD_BITS+EEPROM_ADDR_BITS; i++) {
	eeprom_send_command_bit(sc, (command & CMD_BIT_MASK) != 0 ? 1 : 0);
	command <<= 1;
	}

    /* Now read the bits from the EEPROM (MSB first) */
    word = 0;
    for (i = 0; i < 16; ++i) {
	word <<= 1;
	word |= eeprom_read_bit(sc);
	}

    /* Clear the EEPROM CS Line,  CS hold, Tcsh = 0 */
    WRITECSR(sc, R_MEAR, ctrl &~ M_MEAR_EESEL);

    return word;
}


/****************************************************************************
 *  eeprom_checksum()
 *
 *  Calculate the checksum of the EEPROM and return it.  See Section
 *  4.2.4 for the algorithm.
 ***************************************************************************/

static uint16_t
eeprom_checksum(const uint8_t rom[])
{
    uint16_t sum;
    int i;

    sum = 0;
    for (i = 0; i < EEPROM_SIZE-1; i++)
	sum += rom[i];
    sum ^= 0xFF;
    return (((sum + 1) & 0xFF) << 8) | 0x55;
}


/****************************************************************************
 *  eeprom_read_all(sc, uint8_t dest)
 *
 *  Read the entire EEPROM into the srom array
 *
 *  Input parameters:
 *         sc - dp83815 state
 ***************************************************************************/

static int
eeprom_read_all(dp83815_softc *sc, uint8_t dest[])
{
    int  i;
    uint16_t cksum, temp;

    WRITECSR(sc, R_MEAR, M_MEAR_EESEL);

    eeprom_idle_state(sc);

    for (i = 0; i < EEPROM_SIZE/2; i++) {
	temp = eeprom_read_word(sc, i);
	dest[2*i] = temp & 0xFF;
	dest[2*i+1] = temp >> 8;
	}

    WRITECSR(sc, R_MEAR, 0);   /* CS hold, Tcsh=0 */

    cksum = eeprom_checksum(dest);;
    if (cksum != EEPROM_WORD(dest, EEPROM_CRC_INDEX)) {
	xprintf("%s: Invalid EEPROM CHECKSUM, calc %04x, stored %04x\n",
		dp83815_devname(sc),
		cksum, EEPROM_WORD(dest, EEPROM_CRC_INDEX));
	return 0/*-1*/;
	}
    return 0;
}

static int
eeprom_read_addr(const uint8_t rom[], uint8_t buf[])
{
    uint16_t s;
    unsigned offset, mask;
    int i, j;

    if (eeprom_checksum(rom) != EEPROM_WORD(rom, EEPROM_SIZE-2))
	return -1;

    s = 0;
    offset = 2*6; mask = 0x1;
    i = j = 0;
    do {
	s >>= 1;
	if ((EEPROM_WORD(rom, offset) & mask) != 0) s |= 0x8000;
	mask >>= 1;
	if (mask == 0) {
	    offset +=2;  mask = 0x8000;
	    }
	i++;
	if (i % 16 == 0) {
	    buf[j++] = s & 0xFF;
	    buf[j++] = s >> 8;
	    s = 0;
	    }
	} while (i < ENET_ADDR_LEN*8);

    return 0;
}
#endif /* MACPHYTER_TEST */

#if 0
static void
eeprom_dump(uint8_t srom[])
{
    int  i;

    xprintf("DP83815: EEPROM data:");
    for (i = 0; i < EEPROM_SIZE; i++) {
	if (i % 16 == 0)
	    xprintf("\n %02x: ", i);
	xprintf(" %02x", srom[i]);
	}
    xprintf("\n");
}
#else
#define eeprom_dump(srom)
#endif
  

static int
dp83815_get_pm_addr(dp83815_softc *sc, uint8_t buf[])
{
#ifdef BCM47XX   /* temporary patch */
  /* The 47xx parts apparently have trouble accessing the RFDR
     register and get bus errors.  For them, this function is a noop.
     The address passed into the probe routine is used instead, but
     since the station address is loaded into the PMATCH register, the
     two addresses must match for the MAC to receive unicast
     traffic. */
    return -1;
#else
    uint32_t rfcr;
    unsigned rfaddr;
    unsigned i;
    uint32_t rfdata;

    rfcr = READCSR(sc, R_RFCR);
    rfaddr = K_RFCR_PMATCH_ADDR;

    for (i = 0; i < ENET_ADDR_LEN/2; i++) {
	rfcr &=~ M_RFCR_RFADDR;
	rfcr |= V_RFCR_RFADDR(rfaddr);
	WRITECSR(sc, R_RFCR, rfcr);
	rfdata = READCSR(sc, R_RFDR);
	buf[2*i] = rfdata & 0xFF;
	buf[2*i+1] = (rfdata >> 8) & 0xFF;
	rfaddr += 2;
	}

    return 0;  
#endif /* BCM47XX */
}


#if MACPHYTER_TEST
/* MII access */

/* Current NICs use the internal PHY, which can be accessed more
   simply via internal registers.  The following routines are
   primarily for management access to an external PHY and are retained
   for future applications.  They have been tested on a Netgear FA311.  */

/****************************************************************************
 *                 MII access utility routines
 ***************************************************************************/

/* MII clock limited to 2.5 MHz (DP83815 allows 25 MHz), transactions
   end with MDIO tristated */

static void
mii_write_bits(dp83815_softc *sc, uint32_t data, unsigned int count)
{
    uint32_t   ctrl;
    uint32_t   bitmask;

    ctrl =  READCSR(sc, R_MEAR) & ~M_MEAR_MDC;
    ctrl |= M_MEAR_MDDIR;

    for (bitmask = 1 << (count-1); bitmask != 0; bitmask >>= 1) {
	ctrl &=~ M_MEAR_MDIO;
	if ((data & bitmask) != 0) ctrl |= M_MEAR_MDIO;
	WRITECSR(sc, R_MEAR, ctrl);

	cfe_nsleep(2000);     /* setup */
	WRITECSR(sc, R_MEAR, ctrl | M_MEAR_MDC);
	cfe_nsleep(2000);     /* hold */
	WRITECSR(sc, R_MEAR, ctrl);
	}
}

static void
mii_turnaround(dp83815_softc *sc)
{
    uint32_t  ctrl;

    ctrl = READCSR(sc, R_MEAR) &~ M_MEAR_MDDIR;

    /* stop driving data */
    WRITECSR(sc, R_MEAR, ctrl);
    cfe_nsleep(2000);       /* setup */
    WRITECSR(sc, R_MEAR, ctrl | M_MEAR_MDC);
    cfe_nsleep(2000);       /* clock high */
    WRITECSR(sc, R_MEAR, ctrl);

    /* read back and check for 0 here? */
}

/****************************************************************************
 *  mii_read_register
 *
 *  This routine reads a register from the PHY chip using the MII
 *  serial management interface.
 *
 *  Input parameters:
 *         index - index of register to read (0-31)
 *
 *  Return value:
 *         word read from register
 ***************************************************************************/

static uint16_t
mii_read_register(dp83815_softc *sc, unsigned int index)
{
    /* Send the command and address to the PHY.  The sequence is
       a synchronization sequence (32 1 bits)
       a "start" command (2 bits)
       a "read" command (2 bits)
       the PHY addr (5 bits)
       the register index (5 bits)
     */
    uint32_t  ctrl;
    uint16_t  word;
    int i;

    mii_write_bits(sc, 0xFF, 8);
    mii_write_bits(sc, 0xFFFFFFFF, 32);
    mii_write_bits(sc, MII_COMMAND_START, 2);
    mii_write_bits(sc, MII_COMMAND_READ, 2);
    mii_write_bits(sc, sc->phy_addr, 5);
    mii_write_bits(sc, index, 5);

    mii_turnaround(sc);

    ctrl = READCSR(sc, R_MEAR) &~ (M_MEAR_MDC | M_MEAR_MDDIR);
    word = 0;

    for (i = 0; i < 16; i++) {
	WRITECSR(sc, R_MEAR, ctrl);
	cfe_nsleep(2000);    /* clock width low */
	WRITECSR(sc, R_MEAR, ctrl | M_MEAR_MDC);
	cfe_nsleep(2000);    /* clock width high */
	WRITECSR(sc, R_MEAR, ctrl);
	cfe_nsleep(1000);    /* output delay */
	word <<= 1;
	if ((READCSR(sc, R_MEAR) & M_MEAR_MDIO) != 0)
	    word |= 0x0001;
	}

    return word;

    /* reset to output mode? */
}

/****************************************************************************
 *  mii_write_register
 *
 *  This routine writes a register in the PHY chip using the MII
 *  serial management interface.
 *
 *  Input parameters:
 *         index - index of register to write (0-31)
 *         value - word to write
 ***************************************************************************/

static void
mii_write_register(dp83815_softc *sc, unsigned int index, uint16_t value)
{
    mii_write_bits(sc, 0xFF, 8);
    mii_write_bits(sc, 0xFFFFFFFF, 32);
    mii_write_bits(sc, MII_COMMAND_START, 2);
    mii_write_bits(sc, MII_COMMAND_WRITE, 2);
    mii_write_bits(sc, sc->phy_addr, 5);
    mii_write_bits(sc, index, 5);
    mii_write_bits(sc, MII_COMMAND_ACK, 2);
    mii_write_bits(sc, value, 16);

    /* reset to input mode? */
}


static int
mii_probe(dp83815_softc *sc)
{
    int i;
    uint16_t id1, id2;

    /* Empirically, bit-banged access will return register 0 of the
       integrated PHY for all registers of all unpopulated PHY
       addresses. */
    for (i = 0; i < 32; i++) {
        sc->phy_addr = i;
        id1 = mii_read_register(sc, MII_PHYIDR1);
	id2 = mii_read_register(sc, MII_PHYIDR2);
	if ((id1 != 0x0000 && id1 != 0xFFFF) ||
	    (id2 != 0x0000 && id2 != 0xFFFF)) {
	    if (id1 != id2) {
		sc->phy_vendor = ((uint32_t)id1 << 6) | ((id2 >> 10) & 0x3F);
		sc->phy_device = (id2 >> 4) & 0x3F;
		return 0;
		}
	    }
	}
    return -1;
}

#if 0
static void
mii_dump(dp83815_softc *sc, const char *label)
{
    int i;
    uint16_t  r;

    xprintf("%s\n", label);
    for (i = 0; i <= 6; ++i) {
	r = mii_read_register(sc, i);
	xprintf("MII_REG%02x: %04x\n", i, r);
	}
    if (sc->phy_vendor == OUI_NAT_SEMI && sc->phy_device == DEV_DP83815) {
	r = mii_read_register(sc, 7);
	xprintf("MII_REG%02x: %04x\n", i, r);
	for (i = 0x10; i <= 0x16; ++i) {
	    r = mii_read_register(sc, i);
	    xprintf("MII_REG%02x: %04x\n", i, r);
	    }
	for (i = 0x19; i <= 0x1A; ++i) {
	    r = mii_read_register(sc, i);
	    xprintf("MII_REG%02x: %04x\n", i, r);
	    }
	}
}
#else
#define mii_dump(sc,label)
#endif


/* The following functions are suitable for explicit MII access.  */

static void
mii_set_speed(dp83815_softc *sc, int speed)
{
    uint16_t  control;

    control = mii_read_register(sc, MII_BMCR);

    control &=~ (BMCR_ANENABLE | BMCR_RESTARTAN);
    mii_write_register(sc, MII_BMCR, control);
    control &=~ (BMCR_SPEED0 | BMCR_SPEED1 | BMCR_DUPLEX);

    switch (speed) {
	case ETHER_SPEED_10HDX:
	default:
	    break;
	case ETHER_SPEED_10FDX:
	    control |= BMCR_DUPLEX;
	    break;
	case ETHER_SPEED_100HDX:
	    control |= BMCR_SPEED100;
	    break;
	case ETHER_SPEED_100FDX:
	    control |= BMCR_SPEED100 | BMCR_DUPLEX ;
	    break;
	}

    mii_write_register(sc, MII_BMCR, control);
}

static void
mii_autonegotiate(dp83815_softc *sc)
{
    uint16_t  control, status, cap;
    int  timeout;
    int linkspeed;

    linkspeed = ETHER_SPEED_UNKNOWN;

    /* Read twice to clear latching bits */
    status = mii_read_register(sc, MII_BMSR);
    status = mii_read_register(sc, MII_BMSR);
    mii_dump(sc, "query PHY");

    if ((status & (BMSR_AUTONEG | BMSR_LINKSTAT)) ==
        (BMSR_AUTONEG | BMSR_LINKSTAT))
	control = mii_read_register(sc, MII_BMCR);
    else {
	/* reset the PHY */
	mii_write_register(sc, MII_BMCR, BMCR_RESET);
	timeout = 3*CFE_HZ;
	for (;;) {
	    control = mii_read_register(sc, MII_BMCR);
	    if ((control && BMCR_RESET) == 0 || timeout <= 0)
		break;
	    cfe_sleep(CFE_HZ/2);
	    timeout -= CFE_HZ/2;
	    }
	if ((control & BMCR_RESET) != 0) {
	    xprintf("%s: PHY reset failed\n", dp83815_devname(sc));
	    return;
	    }

	status = mii_read_register(sc, MII_BMSR);
	cap = ((status >> 6) & (ANAR_TXFD | ANAR_TXHD | ANAR_10FD | ANAR_10HD))
	      | PSB_802_3;
	mii_write_register(sc, MII_ANAR, cap);
	control |= (BMCR_ANENABLE | BMCR_RESTARTAN);
	mii_write_register(sc, MII_BMCR, control);

	timeout = 3*CFE_HZ;
	for (;;) {
	    status = mii_read_register(sc, MII_BMSR);
	    if ((status & BMSR_ANCOMPLETE) != 0 || timeout <= 0)
		break;
	    cfe_sleep(CFE_HZ/2);
	    timeout -= CFE_HZ/2;
	    }
	mii_dump(sc, "done PHY");
	}

    xprintf("%s: Link speed: ", dp83815_devname(sc));
    if ((status & BMSR_ANCOMPLETE) != 0) {
	/* A link partner was negogiated... */

	uint16_t remote = mii_read_register(sc, MII_ANLPAR);

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
	/* no link partner negotiation */
	control &=~ (BMCR_ANENABLE | BMCR_RESTARTAN);
	mii_write_register(sc, MII_BMCR, control);
	xprintf("10BaseT HDX (assumed)\n");
	linkspeed = ETHER_SPEED_10HDX;
	if ((status & BMSR_LINKSTAT) == 0)
	    mii_write_register(sc, MII_BMCR, control);
	mii_set_speed(sc, linkspeed);
	}

    status = mii_read_register(sc, MII_BMSR);  /* clear latching bits */
    mii_dump(sc, "final PHY");
}
#endif /* MACPHYTER_TEST */


static void
dp83815_phyupdate(dp83815_softc *sc, uint32_t status)
{
    xprintf("%s: Link speed: ", dp83815_devname(sc));
    if ((status & M_CFG_LNKSTS) != 0) {
	switch (status & (M_CFG_SPEED100 | M_CFG_FDUP)) {
	    case (M_CFG_SPEED100 | M_CFG_FDUP):
		sc->linkspeed = ETHER_SPEED_100FDX;
		xprintf("100BaseT FDX\n");
		break;
	    case (M_CFG_SPEED100):
		sc->linkspeed = ETHER_SPEED_100HDX;
		xprintf("100BaseT HDX\n");
		break;
	    case (M_CFG_FDUP):
		sc->linkspeed = ETHER_SPEED_10FDX;
		xprintf("10BaseT FDX\n");
		break;
	    default:
		sc->linkspeed = ETHER_SPEED_10HDX;
		xprintf("10BaseT HDX\n");
		break;
	    }
	if ((status & M_CFG_SPEED100) != 0) {
	    uint32_t t;

	    /* This is a reputed fix that improves 100BT rx
	       performance on short cables with "a small number"
	       of DP83815 chips.  It comes from Bruce at NatSemi
	       via the Soekris support web page (see appended
	       note). */

	    WRITECSR(sc, R_PGSEL, 0x0001);
	    (void)READCSR(sc, R_PGSEL);   /* push */
	    t = READCSR(sc, R_DSPCFG);
	    WRITECSR(sc, R_DSPCFG, (t & 0xFFF) | 0x1000);
	    cfe_sleep(1);
	    t = READCSR(sc, R_TSTDAT) & 0xFF;
	    if ((t & 0x0080) == 0 || ((t > 0x00D8) && (t <= 0x00FF))) {
		WRITECSR(sc, R_TSTDAT, 0x00E8);
		t = READCSR(sc, R_DSPCFG);
		WRITECSR(sc, R_DSPCFG, t | 0x0020);
		}
	    WRITECSR(sc, R_PGSEL, 0);
	    }
	if ((status & M_CFG_FDUP) != (sc->phy_status & M_CFG_FDUP)) {
	    uint32_t txcfg, rxcfg;

	    txcfg = READCSR(sc, R_TXCFG);
	    rxcfg = READCSR(sc, R_RXCFG);
	    if (status & M_CFG_FDUP) {
		txcfg |= (M_TXCFG_CSI | M_TXCFG_HBI);
		rxcfg |= M_RXCFG_ATX;
		}
	    else {
		txcfg &= ~(M_TXCFG_CSI | M_TXCFG_HBI);
		rxcfg &= ~M_RXCFG_ATX;
		}
	    WRITECSR(sc, R_TXCFG, txcfg);
	    WRITECSR(sc, R_RXCFG, rxcfg);
	    }
	}
    else {
	xprintf("Unknown\n");	    
        }

    sc->phy_status = status;
}

static void
dp83815_hwinit(dp83815_softc *sc)
{
    if (sc->state == eth_state_uninit) {
	uint32_t cfg;
	uint32_t txcfg, rxcfg;
	uint32_t ready;
	int timeout;

        /* RESET_ADAPTER(sc); */
	sc->state = eth_state_off;
	sc->bus_errors = 0;

	cfg = READCSR(sc, R_CFG);
#if ENDIAN_BIG
	cfg |= M_CFG_BEM;   /* We will use match bits */
#endif
	WRITECSR(sc, R_CFG, cfg);

	sc->phy_status = 0;
	dp83815_phyupdate(sc, cfg & M_CFG_LNKSUMMARY);

	/* Set a maximum tx DMA burst length of 512 (128*4) bytes, a
	   fill threshold of 512 (16*32) and a drain threshold of 64
	   (2*32) bytes. */
	txcfg = READCSR(sc, R_TXCFG);
	txcfg &= ~(M_TXCFG_MXDMA | M_TXCFG_FLTH | M_TXCFG_DRTH);
	txcfg |= (M_TXCFG_ATP
		  | V_TXCFG_MXDMA(K_MXDMA_512)
		  | V_TXCFG_FLTH(16) | V_TXCFG_DRTH(2));
	WRITECSR(sc, R_TXCFG, txcfg);

	/* Set a maximum rx DMA burst length of 512 (128*4) bytes, and
	   an rx drain threshhold of 128 (16*8) bytes */
	rxcfg = READCSR(sc, R_RXCFG);
	rxcfg &= ~(M_RXCFG_MXDMA | M_RXCFG_DRTH);
	rxcfg |= (V_RXCFG_MXDMA(K_MXDMA_512) | V_RXCFG_DRTH(16));
	WRITECSR(sc, R_RXCFG, rxcfg);
	
#if MACPHYTER_TEST
	{
	    uint8_t srom[EEPROM_SIZE];
	    uint8_t addr[ENET_ADDR_LEN];

	    eeprom_read_all(sc, srom);
	    eeprom_dump(srom);
	    xprintf("  checksum %04x\n", eeprom_checksum(srom));
	    if (eeprom_read_addr(srom, addr) == 0)
	        xprintf("  addr: %02x-%02x-%02x-%02x-%02x-%02x\n",
		       addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	    mii_probe(sc);
	    xprintf("MII address %02x\n", sc->phy_addr);
	    mii_dump(sc, "DP83815 PHY:");
	    (void)mii_autonegotiate;
	    }
#endif /* MACPHYTER_TEST */

	/* XXX fix up rx filtering here.  We are relying on the EEPROM. */

	/* XXX This is inappropriate on a restart. */
	timeout = 2*CFE_HZ;
	ready = 0;
	for (;;) {
	    ready |= READCSR(sc, R_ISR);
	    if ((ready & (M_INT_TXRCMP | M_INT_RXRCMP))
		 == (M_INT_TXRCMP | M_INT_RXRCMP) || timeout <= 0)
		break;
	    cfe_sleep(CFE_HZ/10);
	    timeout -= CFE_HZ/10;
	    }
	if ((ready & M_INT_TXRCMP) == 0)
	    xprintf("%s: tx reset failed\n", dp83815_devname(sc));
	if ((ready & M_INT_RXRCMP) == 0)
	    xprintf("%s: rx reset failed\n", dp83815_devname(sc));
	}
}

static void
dp83815_setspeed(dp83815_softc *sc, int speed)
{
    /* XXX Not yet implemented - autonegotiation only. */
}

static void
dp83815_setloopback(dp83815_softc *sc, int mode)
{
    /* XXX Not yet implemented. */
}


static void
dp83815_isr(void *arg)
{
    dp83815_softc *sc = (dp83815_softc *)arg;
    uint32_t status;
    uint32_t isr;

#if IPOLL
    sc->interrupts++;
#endif

    for (;;) {

	/* Read (and clear) the interrupt status. */
	isr = READCSR(sc, R_ISR);
	status = isr & sc->intmask;

	/* if there are no more interrupts, leave now. */
	if (status == 0) break;

	/* Now, test each unmasked bit in the interrupt register and
           handle each interrupt type appropriately. */

	if (status & (M_INT_RTABT | M_INT_RMABT)) {
	    WRITECSR(sc, R_IER, 0);

	    xprintf("%s: bus error %08x\n", dp83815_devname(sc), status);
	    dumpstat(sc);
	    sc->bus_errors++;
	    if (sc->bus_errors >= 2) {
	        dumpcsrs(sc);
	        RESET_ADAPTER(sc);
		sc->state = eth_state_off;
		sc->bus_errors = 0;
	        }
#if IPOLL
	    else
	        WRITECSR(sc, R_IMR, sc->intmask);
#endif
	    }

	if (status & M_INT_RXDESC) {
#if IPOLL
	    sc->rx_interrupts++;
#endif
	    dp83815_procrxring(sc);
	    }

	if (status & M_INT_TXDESC) {
#if IPOLL
            sc->tx_interrupts++;
#endif
	    dp83815_proctxring(sc);
	    }

	if (status & (M_INT_TXURN | M_INT_RXORN)) {
	    if (status & M_INT_TXURN) {
		xprintf("%s: tx underrun, %08x\n", dp83815_devname(sc), isr);
		/* XXX Try to restart */
		}
	    if (status & M_INT_RXORN) {
		xprintf("%s: tx overrun, %08x\n", dp83815_devname(sc), isr);
		/* XXX Try to restart */
		}
	    }

	if (status & M_INT_PHY) {
	    sc->intmask &= ~ M_INT_PHY;
	    WRITECSR(sc, R_IMR, sc->intmask);
	    (void)READCSR(sc, R_MISR);     /* Clear at PHY */
	    sc->phy_check = 1;
	    }

	}
}

static void
dp83815_checkphy(dp83815_softc *sc)
{
    uint32_t cfg;
    uint32_t status;

    (void)READCSR(sc, R_MISR);     /* Clear at PHY */
    cfg = READCSR(sc, R_CFG);
    status = cfg & M_CFG_LNKSUMMARY;
    if (status != sc->phy_status) {
        /* XXX Can we really do the phy update with active rx and tx? */
	dp83815_phyupdate(sc, status);
	}

    sc->intmask |= M_INT_PHY;
    WRITECSR(sc, R_IMR, sc->intmask);
}


static void
dp83815_start(dp83815_softc *sc)
{
    dp83815_hwinit(sc);

    /* Set up loopback here */

    sc->intmask = 0;
    WRITECSR(sc, R_IER, 0);		/* no interrupts */
    WRITECSR(sc, R_IMR, 0);
    (void)READCSR(sc, R_ISR);           /* clear any pending */

    sc->phy_status = READCSR(sc, R_CFG) & M_CFG_LNKSUMMARY;
    sc->phy_check = 0;

    sc->intmask =  M_INT_RXDESC | M_INT_TXDESC;
    sc->intmask |= M_INT_RTABT | M_INT_RMABT | M_INT_RXORN | M_INT_TXURN;
    sc->intmask |= M_INT_PHY;

#if IPOLL
    cfe_request_irq(sc->irq, dp83815_isr, sc, CFE_IRQ_FLAGS_SHARED, 0);
    WRITECSR(sc, R_IMR, sc->intmask);
    WRITECSR(sc, R_IER, M_IER_IE);
#endif

    (void)READCSR(sc, R_MISR);         /* clear any pending */
    WRITECSR(sc, R_MISR, MISR_MSKJAB | MISR_MSKRF | MISR_MSKFHF | MISR_MSKRHF);
    WRITECSR(sc, R_MICR, MICR_INTEN);

    WRITECSR(sc, R_TXDP, PTR_TO_PCI(sc->txdscr_start));
    WRITECSR(sc, R_RXDP, PTR_TO_PCI(sc->rxdscr_start));

    WRITECSR(sc, R_MIBC, M_MIBC_ACLR);  /* zero hw MIB counters */

    WRITECSR(sc, R_CR, M_CR_TXE | M_CR_RXE);
    sc->state = eth_state_on;
}

static void
dp83815_stop(dp83815_softc *sc)
{
    uint32_t status;
    int count;

    /* Make sure that no further interrutps will be processed. */
    sc->intmask = 0;
    WRITECSR(sc, R_IER, 0);
    WRITECSR(sc, R_IMR, 0);

#if IPOLL
    (void)READCSR(sc, R_IER);   /* Push */
    cfe_free_irq(sc->irq, 0);
#endif

    WRITECSR(sc, R_CR, M_CR_TXD | M_CR_RXD);

    /* wait for any DMA activity to terminate */
    for (count = 0; count <= 13; count++) {
	status = READCSR(sc, R_CR);
	if ((status & (M_CR_TXE | M_CR_RXE)) == 0)
	    break;
	cfe_sleep(CFE_HZ/10);
	}
    if (count > 13) {
	xprintf("%s: idle state not achieved\n", dp83815_devname(sc));
	dumpstat(sc);
	RESET_ADAPTER(sc);
	sc->state = eth_state_uninit;
#if 1
	sc->linkspeed = ETHER_SPEED_AUTO;
#endif
	}
#if 0 /* XXX Not yet implemented. */
    else if (sc->loopback != ETHER_LOOPBACK_OFF) {
	dp83815_setloopback(sc, ETHER_LOOPBACK_OFF);
	}
#endif

    (void)READCSR(sc, R_ISR);   /* Clear any stragglers. */
}


/*  *********************************************************************
    *  Declarations for CFE Device Driver Interface routines
    ********************************************************************* */

static int dp83815_ether_open(cfe_devctx_t *ctx);
static int dp83815_ether_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int dp83815_ether_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int dp83815_ether_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int dp83815_ether_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int dp83815_ether_close(cfe_devctx_t *ctx);
static void dp83815_ether_poll(cfe_devctx_t *ctx, int64_t ticks);
static void dp83815_ether_reset(void *softc);

/*  *********************************************************************
    *  CFE Device Driver dispatch structure
    ********************************************************************* */

const static cfe_devdisp_t dp83815_ether_dispatch = {
    dp83815_ether_open,
    dp83815_ether_read,
    dp83815_ether_inpstat,
    dp83815_ether_write,
    dp83815_ether_ioctl,
    dp83815_ether_close,
    dp83815_ether_poll,
    dp83815_ether_reset
};

/*  *********************************************************************
    *  CFE Device Driver descriptor
    ********************************************************************* */

const cfe_driver_t dp83815drv = {
    "DP83815 Ethernet",
    "eth",
    CFE_DEV_NETWORK,
    &dp83815_ether_dispatch,
    dp83815_ether_probe
};


static int
dp83815_ether_attach(cfe_driver_t *drv, pcitag_t tag, uint8_t hwaddr[])
{
    dp83815_softc *sc;
    uint32_t device;
    uint32_t class;
    phys_addr_t pa;
    uint8_t promaddr[ENET_ADDR_LEN];
    char descr[100];
    uint32_t srr;

    device = pci_conf_read(tag, R_CFGID);
    class = pci_conf_read(tag, R_CFGRID);

#if 1
    /* Use memory space for the CSRs */
    pci_map_mem(tag, R_CFGMA, PCI_MATCH_BITS, &pa);
#else
    /* Use i/o space for the CSRs */
    pci_map_io(tag, R_CFGIOA, PCI_MATCH_BITS, &pa);
#endif

    sc = (dp83815_softc *) KMALLOC(sizeof(dp83815_softc), 0);

    if (sc == NULL) {
	xprintf("DP83815: No memory to complete probe\n");
	return 0;
	}
    memset(sc, 0, sizeof(*sc));

    sc->membase = (uint32_t)pa;
    sc->irq = pci_conf_read(tag, R_CFGINT) & 0xFF;
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
    memcpy(sc->hwaddr, hwaddr, ENET_ADDR_LEN);

    srr = READCSR(sc, R_SRR);
#if 0
    /* The NS data sheet recommends the following for "optimal
       performance" of CVNG parts.  Tested on a sample of one CVNG
       part on an NS "Macphyter Demo II" eval board, it seemed to
       produce slightly less reliable initial behavior. */
    if (G_SRR_REV(srr) == K_REV_CVNG) {
	/* Update PHY DSP registers per data sheet. */
	WRITECSR(sc, R_PGSEL, 0x0001);
	(void)READCSR(sc, R_PGSEL);   /* push */
	WRITECSR(sc, R_PMDCSR, 0x189C);
	WRITECSR(sc, R_TSTDAT, 0x0000);
	WRITECSR(sc, R_DSPCFG, 0x5040);
	WRITECSR(sc, R_SDCFG,  0x008C);
	}
#endif

    dp83815_init(sc);

    /* Prefer the address in EEPROM.  This will be read into the
       PMATCH register upon power up.  Unfortunately, how to test for
       completion of the auto-load (but see PTSCR_EELOAD_EN).  */
    if (dp83815_get_pm_addr(sc, promaddr) == 0) {
	memcpy(sc->hwaddr, promaddr, ENET_ADDR_LEN);
	}

    sc->state = eth_state_uninit;

    xsprintf(descr, "%s at 0x%X (%02x-%02x-%02x-%02x-%02x-%02x)",
	     drv->drv_description, sc->membase,
	     sc->hwaddr[0], sc->hwaddr[1], sc->hwaddr[2],
	     sc->hwaddr[3], sc->hwaddr[4], sc->hwaddr[5]);

    cfe_attach(drv, sc, NULL, descr);
    return 1;
}


/*  *********************************************************************
    *  DP83815_ETHER_PROBE(drv,probe_a,probe_b,probe_ptr)
    *  
    *  Probe and install drivers for all dp83815 Ethernet controllers.
    *  For each, create a context structure and attach to the
    *  specified network device.
    *  
    *  Input parameters: 
    *  	   drv - driver descriptor
    *  	   probe_a - not used
    *  	   probe_b - not used
    *  	   probe_ptr - string pointer to hardware address for the first
    *  	               MAC, in the form xx:xx:xx:xx:xx:xx
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
static void
dp83815_ether_probe(cfe_driver_t *drv,
		    unsigned long probe_a, unsigned long probe_b, 
		    void *probe_ptr)
{
    int n;
    uint8_t hwaddr[ENET_ADDR_LEN];                 

    if (probe_ptr)
	enet_parse_hwaddr((char *) probe_ptr, hwaddr);
    else {
	/* use default address 02-00-00-10-0B-00 */
	hwaddr[0] = 0x02;  hwaddr[1] = 0x00;  hwaddr[2] = 0x00;
	hwaddr[3] = 0x10;  hwaddr[4] = 0x0B;  hwaddr[5] = 0x00;
	}

    n = 0;
    for (;;) {
	pcitag_t tag;

	if (pci_find_device(K_PCI_VENDOR_NSC, K_PCI_ID_DP83815, n, &tag) != 0)
	    break;
	dp83815_ether_attach(drv, tag, hwaddr);
	n++;
	enet_incr_hwaddr(hwaddr, 1);
	}
}


/* The functions below are called via the dispatch vector for the 83815. */

/*  *********************************************************************
    *  DP83815_ETHER_OPEN(ctx)
    *  
    *  Open the Ethernet device.  The MAC is reset, initialized, and
    *  prepared to receive and send packets.
    *  
    *  Input parameters: 
    *  	   ctx - device context (includes ptr to our softc)
    *  	   
    *  Return value:
    *  	   status, 0 = ok
    ********************************************************************* */
static int
dp83815_ether_open(cfe_devctx_t *ctx)
{
    dp83815_softc *sc = ctx->dev_softc;

    if (sc->state == eth_state_on)
	dp83815_stop(sc);

    sc->devctx = ctx;

    sc->inpkts = sc->outpkts = 0;
    sc->interrupts = 0;
    sc->rx_interrupts = sc->tx_interrupts = 0;

    dp83815_start(sc);

#if XPOLL
    dp83815_isr(sc);
#endif

    return 0;
}

/*  *********************************************************************
    *  DP83815_ETHER_READ(ctx,buffer)
    *  
    *  Read a packet from the Ethernet device.  If no packets are
    *  available, the read will succeed but return 0 bytes.
    *  
    *  Input parameters: 
    *  	   ctx - device context (includes ptr to our softc)
    *      buffer - pointer to buffer descriptor.  
    *  	   
    *  Return value:
    *  	   status, 0 = ok
    ********************************************************************* */
static int
dp83815_ether_read(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    dp83815_softc *sc = ctx->dev_softc;
    eth_pkt_t *pkt;
    int blen;

#if XPOLL
    dp83815_isr(sc);
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

    CACHE_DMA_INVAL(pkt->buffer, blen);
    hs_memcpy_to_hs(buffer->buf_ptr, pkt->buffer, blen);
    buffer->buf_retlen = blen;

    eth_free_pkt(sc, pkt);
    dp83815_fillrxring(sc);

#if XPOLL
    dp83815_isr(sc);
#endif

    return 0;
}

/*  *********************************************************************
    *  DP83815_ETHER_INPSTAT(ctx,inpstat)
    *  
    *  Check for received packets on the Ethernet device
    *  
    *  Input parameters: 
    *  	   ctx - device context (includes ptr to our softc)
    *      inpstat - pointer to input status structure
    *  	   
    *  Return value:
    *  	   status, 0 = ok
    ********************************************************************* */
static int
dp83815_ether_inpstat(cfe_devctx_t *ctx, iocb_inpstat_t *inpstat)
{
    dp83815_softc *sc = ctx->dev_softc;

#if XPOLL
    dp83815_isr(sc);
#endif

    if (sc->state != eth_state_on) return -1;

    /* We avoid an interlock here because the result is a hint and an
       interrupt cannot turn a non-empty queue into an empty one. */
    inpstat->inp_status = (q_isempty(&(sc->rxqueue))) ? 0 : 1;

    return 0;
}

/*  *********************************************************************
    *  DP83815_ETHER_WRITE(ctx,buffer)
    *  
    *  Write a packet to the Ethernet device.
    *  
    *  Input parameters: 
    *  	   ctx - device context (includes ptr to our softc)
    *      buffer - pointer to buffer descriptor.  
    *  	   
    *  Return value:
    *  	   status, 0 = ok
    ********************************************************************* */
static int
dp83815_ether_write(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    dp83815_softc *sc = ctx->dev_softc;
    eth_pkt_t *pkt;
    int blen;

#if XPOLL
    dp83815_isr(sc);
#endif

    if (sc->state != eth_state_on) return -1;

    pkt = eth_alloc_pkt(sc);
    if (!pkt) return CFE_ERR_NOMEM;

    blen = buffer->buf_length;
    if (blen > pkt->length) blen = pkt->length;

    hs_memcpy_from_hs(pkt->buffer, buffer->buf_ptr, blen);
    pkt->length = blen;

    CACHE_DMA_SYNC(pkt->buffer, blen);
    if (dp83815_transmit(sc, pkt) != 0) {
	eth_free_pkt(sc,pkt);
	return CFE_ERR_IOERR;
	}

#if XPOLL
    dp83815_isr(sc);
#endif

    return 0;
}

/*  *********************************************************************
    *  DP83815_ETHER_IOCTL(ctx,buffer)
    *  
    *  Do device-specific I/O control operations for the device
    *  
    *  Input parameters: 
    *  	   ctx - device context (includes ptr to our softc)
    *      buffer - pointer to buffer descriptor.  
    *  	   
    *  Return value:
    *  	   status, 0 = ok
    ********************************************************************* */
static int
dp83815_ether_ioctl(cfe_devctx_t *ctx, iocb_buffer_t *buffer) 
{
    dp83815_softc *sc = ctx->dev_softc;
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
	    hs_memcpy_to_hs(buffer->buf_ptr,&speed,sizeof(int));
	    return 0;

	case IOCTL_ETHER_SETSPEED:
	    dp83815_stop(sc);
	    dp83815_resetrings(sc);
	    hs_memcpy_from_hs(&speed,buffer->buf_ptr,sizeof(int));
	    dp83815_setspeed(sc, speed);
	    dp83815_start(sc);
	    sc->state = eth_state_on;
	    return 0;

	case IOCTL_ETHER_GETLINK:
	    mode = sc->linkspeed;
	    hs_memcpy_to_hs(buffer->buf_ptr,&mode,sizeof(int));
	    return 0;

	case IOCTL_ETHER_GETLOOPBACK:
	    mode = sc->loopback;
	    hs_memcpy_to_hs(buffer->buf_ptr,&mode,sizeof(int));
	    return 0;

	case IOCTL_ETHER_SETLOOPBACK:
	    dp83815_stop(sc);
	    dp83815_resetrings(sc);
	    hs_memcpy_from_hs(&mode,buffer->buf_ptr,sizeof(int));
	    sc->loopback = ETHER_LOOPBACK_OFF;  /* default */
	    if (mode == ETHER_LOOPBACK_INT || mode == ETHER_LOOPBACK_EXT) {
		dp83815_setloopback(sc, mode);
		}
	    dp83815_start(sc);
	    sc->state = eth_state_on;
	    return 0;

	default:
	    return -1;
	}
}

/*  *********************************************************************
    *  DP83815_ETHER_CLOSE(ctx)
    *  
    *  Close the Ethernet device.
    *  
    *  Input parameters: 
    *  	   ctx - device context (includes ptr to our softc)
    *  	   
    *  Return value:
    *  	   status, 0 = ok
    ********************************************************************* */
static int
dp83815_ether_close(cfe_devctx_t *ctx)
{
    dp83815_softc *sc = ctx->dev_softc;

    sc->state = eth_state_off;
    dp83815_stop(sc);

    /* resynchronize descriptor rings */
    dp83815_resetrings(sc);

    xprintf("%s: %d sent, %d received, %d interrupts\n",
	    dp83815_devname(sc), sc->outpkts, sc->inpkts, sc->interrupts);
    if (IPOLL) {
	xprintf("  %d rx interrupts, %d tx interrupts\n",
		sc->rx_interrupts, sc->tx_interrupts);
	}

    sc->devctx = NULL;
    return 0;
}


/*  *********************************************************************
    *  DP83815_ETHER_POLL(ctx,ticks)
    *  
    *  TBD
    *  
    *  Input parameters: 
    *  	   ctx - device context (includes ptr to our softc)
    *      ticks- current time in ticks
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void
dp83815_ether_poll(cfe_devctx_t *ctx, int64_t ticks)
{
    dp83815_softc *sc = ctx->dev_softc;

    if (sc->phy_check) {
	sc->phy_check = 0;
	dp83815_checkphy(sc);
	}
}


/*  *********************************************************************
    *  DP83815_ETHER_RESET(softc)
    *  
    *  This routine is called when CFE is restarted after a 
    *  program exits.  We can clean up pending I/Os here.
    *  
    *  Input parameters: 
    *  	   softc - pointer to dp83815_softc
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void
dp83815_ether_reset(void *softc)
{
    dp83815_softc *sc = (dp83815_softc *)softc;

    /* Turn off the Ethernet interface. */

    /* RESET_ADAPTER(sc); */

    sc->state = eth_state_uninit;
}
