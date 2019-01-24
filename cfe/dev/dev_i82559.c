/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  i8255x Ethernet Driver			File: dev_i82559.c
    *  
    *  Author:  Ed Satterthwaite
    *  
    *********************************************************************  
    *
    *  Copyright 2003
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

#include "i82559.h"
#include "mii.h"

/*
   This is a driver for specific configurations of the Intel 8255x
   Ethernet controllers.  The chip design evolved substantially over
   time, and Intel uses a bewildering array of PCI product identifiers
   for this series.  The revision number appears to be a better guide
   to chip features:
          i82557      0x01 - 0x03
          i82558      0x04 - 0x05
          i82559      0x06 - 0x08
	  i82559S     0x09
          i82559ER    0x09
          i82550      0x0C - 0x0E
          i82551      0x0F - 0x10 (+?)
   The current version of this driver recognizes a limited number of
   NICs (product ids) and mostly restricts itself to features in the
   common subset.  It has been tested with the i82557 (Rev 2), i82559
   (Rev 8) and i82559S (Rev 9).

   This SB1250 version takes advantage of DMA coherence.  It uses
   "preserve bit lanes" addresses for all accesses to PCI space and
   CSRs, but "match byte lanes" must be used for all DMA accesses
   mastered by the 8255x.  */

#ifndef I82559_DEBUG
#define I82559_DEBUG 0
#endif

#if ((ENDIAN_BIG + ENDIAN_LITTLE) != 1)
#error "dev_i82559: system endian not set"
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

#define CACHE_ALIGN       32
#define ALIGN(n,align)    (((n)+((align)-1)) & ~((align)-1))

#define MIN_ETHER_PACK  (ENET_MIN_PKT+ENET_CRC_SIZE)   /* size of min packet */
#define MAX_ETHER_PACK  (ENET_MAX_PKT+ENET_CRC_SIZE)   /* size of max packet */


/* i8255x Control Blocks as struct's.  Since preserve-byte-lanes
   addressing is used for all DMA, 16 bit fields have the same offset
   in either endian mode, but the contents must be swapped if
   big-endian. */

typedef struct cb_hdr_s {               /* Common CB prefix (CU) */
    uint16_t cb_stat;
    uint16_t cb_cmd;
    uint32_t cb_link;
    /* command specific fields follow here */
} cb_hdr_t;

typedef struct tbd_s {                  /* Transmit Buffer Descriptor (TBD) */
    uint32_t tbd_addr;
    uint32_t tbd_count;
} tbd_t;

/* Transmit Control Blocks are defined to allow flexible mode and to
   accomodate extended TCBs.  The i82557 does not support extended
   TCBs, but the list of TBDs is still appended to the basic TCB. */

typedef struct tcb_s {                  /* Transmit CB (TCB, aka TxCB) */
    cb_hdr_t tcb_hdr;                   /*  8 */
    uint32_t tcb_tbdp;                  /*  4 */
    uint32_t tcb_count;                 /*  4 */
    tbd_t    tcb_tbd[2];                /* 16 */
} tcb_t;

#define TCB_STRIDE      ALIGN(sizeof(tcb_t), CACHE_ALIGN) 

typedef struct rfd_s {                  /* RFD structure (RU) */
    uint16_t rfd_stat;
    uint16_t rfd_cmd;
    uint32_t rfd_link;
    uint32_t rfd_rsvd;
    uint32_t rfd_count;
    /* data follows here */
} rfd_t;


/* Packet buffers.  For now, we use "simplified mode" for rx to
   support pre-82550 chips.  In that mode, the buffer immediately
   follows the RFD.  To allow sharing the buffer pool between rx and
   tx, space for prefixed control is included in all packet buffers.  */

#define ETH_PKTBUF_LEN  ALIGN(MAX_ETHER_PACK, CACHE_ALIGN)

typedef struct eth_pkt_s {
    queue_t next;			/*  8 */
    uint8_t *buffer;			/*  4 */
    uint16_t flags;			/*  2 */
    int16_t length;			/*  2 */
    rfd_t rfd;                          /* 16 */
    uint8_t data[ETH_PKTBUF_LEN];       /* cache aligned */
} eth_pkt_t;

#define ETH_PKTBUF_SIZE    ALIGN(sizeof(eth_pkt_t), CACHE_ALIGN)
#define ETH_PKTBUF_OFFSET  (offsetof(eth_pkt_t, data))
#define ETH_PKTRFD_OFFSET  (offsetof(eth_pkt_t, rfd))

#define BUF_PKT_BASE(data) ((eth_pkt_t *)((data) - ETH_PKTBUF_OFFSET))
#define RFD_PKT_BASE(rfd)  ((eth_pkt_t *)((uint8_t *)(rfd) - ETH_PKTRFD_OFFSET))

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

typedef enum {
    chip_i82557,
    chip_i82558,
    chip_i82559,
    chip_i82550,
    chip_i82551
} chip_t;

typedef struct i82559_softc_s i82559_softc;

struct i82559_softc_s {
    uint32_t membase;
    uint8_t irq;		/* interrupt mapping (used if IPOLL) */
    pcitag_t tag;               /* tag for configuration registers */

    uint8_t hwaddr[ENET_ADDR_LEN];

    uint16_t device;            /* chip device code */
    uint8_t revision;		/* chip revision and step (Table 3-7) */
    chip_t chip;                /* chip feature set */

    eth_state_t state;          /* current state */
    uint8_t intmask;            /* shadow interrupt mask */
    unsigned int tx_thresh;     /* current transmit threshold */

    cfe_devctx_t *devctx;

    /* These fields are set before calling i82559_hwinit */
    int linkspeed;		/* encodings from cfe_ioctl */
    int loopback;
    uint8_t linkstat;

    /* Packet buffer lists */
    queue_t freelist;
    queue_t rxqueue;
    uint8_t *pktpool;

    /* Head and tail pointers for the rx chain (of packet buffers). */
    volatile eth_pkt_t *rx_remove;
    volatile eth_pkt_t *rx_add;
    int rx_chained;

    /* Head and tail pointers for the tx ring (of transmit control blocks). */
    volatile tcb_t *tx_add;
    volatile tcb_t *tx_remove;
    volatile tcb_t *tx_suspend;    /* prev(tx_add) */
    uint8_t *tcbbase;
    volatile tcb_t *tx_ring;

    /* These fields describe the PHY */
    int phy_addr;
    uint32_t phy_vendor;
    uint16_t phy_device;
    uint16_t (*mii_read_register)(i82559_softc *sc, unsigned int index);
    void     (*mii_write_register)(i82559_softc *sc, unsigned int index,
				   uint16_t value);

    /* Statistics */
    uint32_t inpkts;
    uint32_t outpkts;
    uint32_t interrupts;
    uint32_t rx_interrupts;
    uint32_t tx_interrupts;
};


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

#define MIN_RX_CHAIN       16
#define TX_RING_SIZE       16
#define ETH_PKTPOOL_SIZE   (MIN_RX_CHAIN+16)


/* Access macros */

/* Note that PTR_TO_PHYS only works with 32-bit addresses, but then
   so does the i8255x. */
#define PTR_TO_PHYS(x) (PHYSADDR((uintptr_t)(x)))
#define PHYS_TO_PTR(a) ((uint8_t *)KERNADDR(a))

/* The i8255x does not have a big-endian option for DMA transactions
   that it masters.  Since most such transactions have byte-oriented
   data, "preserve byte lanes" addressing is used for such DMA. */
#undef PHYS_TO_PCI
#undef PCI_TO_PHYS
#define PHYS_TO_PCI(a) ((uint32_t) (a))
#define PCI_TO_PHYS(a) ((uint32_t) (a))

#define PCI_TO_PTR(a)  (PHYS_TO_PTR(PCI_TO_PHYS(a)))
#define PTR_TO_PCI(x)  (PHYS_TO_PCI(PTR_TO_PHYS(x)))


#define READCSR(sc,csr)        (phys_read32((sc)->membase + (csr)))
#define WRITECSR(sc,csr,val)   (phys_write32((sc)->membase + (csr), (val)))

#if ENDIAN_BIG
#define READCSR16(sc,csr)      (phys_read16((sc)->membase + ((csr)^2)))
#define WRITECSR16(sc,csr,val) (phys_write16((sc)->membase + ((csr)^2), (val)))

#define READCSR8(sc,csr)       (phys_read8((sc)->membase + ((csr)^3)))
#define WRITECSR8(sc,csr,val)  (phys_write8((sc)->membase + ((csr)^3), (val)))
#else
#define READCSR16(sc,csr)      (phys_read16((sc)->membase + (csr)))
#define WRITECSR16(sc,csr,val) (phys_write16((sc)->membase + (csr), (val)))

#define READCSR8(sc,csr)       (phys_read8((sc)->membase + (csr)))
#define WRITECSR8(sc,csr,val)  (phys_write8((sc)->membase + (csr), (val)))
#endif


/* Prototypes */

static void i82559_ether_probe(cfe_driver_t *drv,
			       unsigned long probe_a, unsigned long probe_b, 
			       void *probe_ptr);


/* Byte swap utilities. */

#if ENDIAN_BIG
#define HTOL2(x) \
    ((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))

static uint16_t
htol2(uint16_t x)
{
    return (uint16_t)((x >> 8) | (x << 8));
}

#define HTOL4(x) \
    ((((x) & 0x00FF) << 24) | \
     (((x) & 0xFF00) << 8)  | \
     (((x) >> 8) & 0xFF00)  | \
     (((x) >> 24) & 0x00FF))

static uint32_t
htol4(uint32_t x)
{
    uint32_t t;

    t = ((x & 0xFF00FF00) >> 8) | ((x & 0x00FF00FF) << 8);
    return (t >> 16) | ((t & 0xFFFF) << 16);
}
#else
#define HTOL2(x) (x)
#define htol2(x) (x)
#define HTOL4(x) (x)
#define htol4(x) (x)
#endif

#define LTOH2 HTOL2
#define ltoh2 htol2
#define LTOH4 HTOL4
#define ltoh4 htol4


/* Packet management */

static eth_pkt_t *
eth_alloc_pkt(i82559_softc *sc)
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
eth_free_pkt(i82559_softc *sc, eth_pkt_t *pkt)
{
    CS_ENTER(sc);
    q_enqueue(&sc->freelist, &pkt->next);
    CS_EXIT(sc);
}

static void
eth_initfreelist(i82559_softc *sc)
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
i82559_devname(i82559_softc *sc)
{
    return (sc->devctx != NULL ? cfe_device_name(sc->devctx) : "eth?");
}

static void
i82559_scb_command0(i82559_softc *sc, uint8_t cmd)
{
    uint8_t rv;

    do {     /* XXX Need a timeout. */
	rv = READCSR8(sc, R_SCB_CMD);
	} while (rv != K_CMD_ACCEPTED);
    WRITECSR8(sc, R_SCB_CMD, cmd);
}

static void
i82559_scb_command1(i82559_softc *sc, uint8_t cmd, uint32_t arg)
{
    uint8_t rv;

    do {     /* XXX Need a timeout. */
	rv = READCSR8(sc, R_SCB_CMD);
	} while (rv != K_CMD_ACCEPTED);
    WRITECSR(sc, R_SCB_PTR, arg);
    WRITECSR8(sc, R_SCB_CMD, cmd);
}


/* Descriptor chain/ring management */

/* Note: Currently, receive descriptors are managed as a list (chain).
   The manual is somewhat confusing about end-of-list state and
   recovery. */

static int
i82559_add_rcvbuf(i82559_softc *sc, eth_pkt_t *pkt)
{
    rfd_t *rfd = &pkt->rfd;

    rfd->rfd_cmd = HTOL2(M_RFD0_EL);
    rfd->rfd_stat = 0;
    rfd->rfd_link = HTOL4(NULL_LINK);
    rfd->rfd_count = htol4(V_RFD3_SIZE(ETH_PKTBUF_LEN));

    if (sc->rx_add != NULL) {
        volatile rfd_t *prev_rfd = &(sc->rx_add->rfd);

        prev_rfd->rfd_link = htol4(PTR_TO_PCI(rfd));
	prev_rfd->rfd_cmd &= ~HTOL2(M_RFD0_EL);
	sc->rx_add = pkt;
	}
    else {
        sc->rx_remove = sc->rx_add = pkt;
	}

    return 0;
}

static void
i82559_fillrxchain(i82559_softc *sc)
{
    eth_pkt_t *pkt;

    CS_ENTER(sc);
    while (1) {
	if (sc->rx_chained >= MIN_RX_CHAIN) {
	    CS_EXIT(sc);
	    break;
	    }
	CS_EXIT(sc);
	pkt = eth_alloc_pkt(sc);
	if (pkt == NULL) {
	    /* Out of free packet buffers */
	    break;
	    }
	if (i82559_add_rcvbuf(sc, pkt) != 0) {
	    eth_free_pkt(sc, pkt);
	    break;
	    }
	CS_ENTER(sc);
	sc->rx_chained++;
	}
}

static void
i82559_rx_callback(i82559_softc *sc, eth_pkt_t *pkt)
{
    if (I82559_DEBUG) show_packet('>', pkt);   /* debug */

    CS_ENTER(sc);
    q_enqueue(&sc->rxqueue, &pkt->next);
    CS_EXIT(sc);
    sc->inpkts++;
}


static void
i82559_procrxchain(i82559_softc *sc)
{
    volatile eth_pkt_t *rxp;
    volatile rfd_t *rfd;
    uint16_t stat;
    uint32_t link;
    eth_pkt_t *newpkt;

    rxp = sc->rx_remove;
    while (rxp != NULL) {
	rfd = &rxp->rfd;
	if ((rfd->rfd_stat & LTOH2(M_RFD0_C)) == 0) {
	    break;
	    }

	stat = ltoh2(rfd->rfd_stat);
	link = ltoh4(rfd->rfd_link);

	/* Drop error packets */
	if ((stat & M_RFD0_OK) == 0) { 
	    xprintf("%s: rx error %04x\n",
		    i82559_devname(sc), stat & M_RFD0_ERRS);
	    newpkt = (eth_pkt_t *)rxp;
	    }
	else {
	    /* Pass up the packet */
	    rxp->length = G_RFD3_COUNT(ltoh4(rfd->rfd_count));
	    i82559_rx_callback(sc, (eth_pkt_t *)rxp);

	    /* add a buffer to the chain to replace this one */
	    newpkt = eth_alloc_pkt(sc);
	    }

	if (link == NULL_LINK) {
	    rxp = NULL;
	    sc->rx_add = sc->rx_remove = NULL;
	    }
	else {
	    rxp = RFD_PKT_BASE(PCI_TO_PTR(link));
	    sc->rx_remove = rxp;
	    }

	if (newpkt) {
	    i82559_add_rcvbuf(sc, newpkt);
	    }
	else {
	    CS_ENTER(sc);
	    sc->rx_chained--;
	    CS_EXIT(sc);
	    }

	}
}

static void
i82559_rx_restart(i82559_softc *sc)
{
    if (sc->rx_remove != NULL) {
	volatile rfd_t *rfd;

	rfd = &sc->rx_remove->rfd;
	i82559_scb_command1(sc, V_SCB_RUC(K_RUC_START), PTR_TO_PCI(rfd));
	}
    /* else ??? XXX */
}


static int
i82559_add_txbuf(i82559_softc *sc, eth_pkt_t *pkt)
{
    volatile tcb_t *tcbp, *tcb;
    volatile tcb_t *tx_next;

    tcb = sc->tx_add;
    tx_next = (volatile tcb_t *)(PCI_TO_PTR(ltoh4(tcb->tcb_hdr.cb_link)));
    if (tx_next == sc->tx_remove)
	return -1;

    tcb->tcb_hdr.cb_cmd = HTOL2(V_CB0_CMD(K_CB_TRANSMIT) | M_TCB0_SF |
				M_CB0_S | M_CB0_I);
    tcb->tcb_hdr.cb_stat &=~ HTOL2(M_CB0_OK | M_CB0_C);

    tcb->tcb_count = htol4(V_TCB3_COUNT(0) | V_TCB3_TBDNUM(1) |
			   V_TCB3_THRESH(sc->tx_thresh));
    tcb->tcb_tbd[0].tbd_addr = htol4(PTR_TO_PCI(pkt->buffer));
    tcb->tcb_tbd[0].tbd_count = htol4(V_TBD1_COUNT(pkt->length) | M_TBD1_EL);
    tcb->tcb_tbd[1].tbd_addr = tcb->tcb_tbd[0].tbd_addr;  /* XXX NULL_LINK? */
    tcb->tcb_tbd[1].tbd_count = 0;

    tcbp = sc->tx_suspend;
    sc->tx_suspend = sc->tx_add;
    sc->tx_add = tx_next;

    tcbp->tcb_hdr.cb_cmd &= HTOL2(~M_CB0_S);
    return 0;
}

static int
i82559_transmit(i82559_softc *sc, eth_pkt_t *pkt)
{
    int rv;

    if (I82559_DEBUG) show_packet('<', pkt);

    rv = i82559_add_txbuf(sc, pkt);
    sc->outpkts++;
    i82559_scb_command0(sc, V_SCB_CUC(K_CUC_RESUME));

    return rv;
}


static void
i82559_proctxring(i82559_softc *sc)
{
    volatile tcb_t *txp;
    eth_pkt_t *pkt;

    for (;;) {
	txp = sc->tx_remove;

	if (txp == sc->tx_add)
	    break;
	if ((txp->tcb_hdr.cb_stat & HTOL2(M_CB0_C)) == 0)
	    break;

	if (G_CB0_CMD(ltoh2(txp->tcb_hdr.cb_cmd)) == K_CB_TRANSMIT) {
	    pkt = BUF_PKT_BASE(PCI_TO_PTR(ltoh4(txp->tcb_tbd[0].tbd_addr)));
	    eth_free_pkt(sc, pkt);
	    }

	txp = (volatile tcb_t *)(PCI_TO_PTR(ltoh4(txp->tcb_hdr.cb_link)));

	sc->tx_remove = txp;
	}
}


static void
i82559_initrings(i82559_softc *sc)
{
    volatile tcb_t *txp, *txn;
    int i;
    
    /* For tx, we make a permanent ring. */
    sc->tx_ring = (volatile tcb_t *)sc->tcbbase;
    txp = NULL;

    for (i = 0; i < TX_RING_SIZE; i++) {
	txn = (volatile tcb_t *)(sc->tcbbase + i*TCB_STRIDE);
	txn->tcb_hdr.cb_cmd = HTOL2(V_CB0_CMD(K_CB_NOP));
	txn->tcb_hdr.cb_stat = 0;
	txn->tcb_count = HTOL4(V_TCB3_TBDNUM(1));   /* always one buffer */

	/* The following fields are never updated. */
	if (sc->chip == chip_i82557) {  /* no Extended mode */
	    txn->tcb_tbdp = htol4(PTR_TO_PCI(txn->tcb_tbd));
	    }
	else {                          /* Extended Mode TxCB */
	    txn->tcb_tbdp = HTOL4(NULL_LINK);
	    }
	txn->tcb_tbd[1].tbd_count = 0;

	if (txp != NULL)
	    txp->tcb_hdr.cb_link = htol4(PTR_TO_PCI(txn));

	txp = txn;
	}
    txp->tcb_hdr.cb_link = htol4(PTR_TO_PCI(sc->tx_ring));

    sc->tx_add = sc->tx_ring;
    sc->tx_remove = sc->tx_suspend = txp;
    
    sc->rx_add = sc->rx_remove = NULL;
    i82559_fillrxchain(sc);
}


static int
i82559_init(i82559_softc *sc)
{
    /* Allocate buffer pool */
    sc->pktpool = KMALLOC(ETH_PKTPOOL_SIZE*ETH_PKTBUF_SIZE, CACHE_ALIGN);
    eth_initfreelist(sc);
    q_init(&sc->rxqueue);

    /* Allocate tx control blocks */
    sc->tcbbase = KMALLOC(TX_RING_SIZE*TCB_STRIDE, CACHE_ALIGN);

    i82559_initrings(sc);

    return 0;
}


static void
i82559_resetrings(i82559_softc *sc)
{
    /* NYI (XXX can leak rx packets) */
}


/* Serial ROM (EEPROM) access */

/*
 * Delays below (nsec) are chosen to meet specs for NS93C46 (slow M variant).
 * Current parts are faster.
 *     Reference:  NS Memory Data Book, 1994
 */

#define EEPROM_SIZE                128
#define EEPROM_MAX_CYCLES          32

#define EEPROM_CMD_BITS            3
#define EEPROM_ADDR_BITS           6

#define K_EEPROM_READ_CMD          06
#define K_EEPROM_WRITE_CMD         05
#define K_EEPROM_ERASE_CMD         07
#define K_EEPROM_EWEN_CMD          04   /* EWEN, EWDS, also WRAL, ERAL */

#define EEPROM_ADDR_INDEX          0    /* empirical */

#define EEPROM_WORD(rom,offset) ((rom)[offset] | ((rom)[offset+1] << 8))

static void
srom_idle_state(i82559_softc *sc)
{
    uint16_t eep;
    unsigned int i;

    eep = READCSR16(sc, R_EEPROM_CTL);

    eep |= M_PROM_EECS;
    WRITECSR16(sc, R_EEPROM_CTL, eep);
    cfe_nsleep(100);                  /* CS setup (Tcss=100) */

    /* Run the clock through the maximum number of pending read cycles */
    for (i = 0; i < EEPROM_MAX_CYCLES*2; i++) {
	eep ^= M_PROM_EESK;
	WRITECSR16(sc, R_EEPROM_CTL, eep);
	cfe_nsleep(1000);             /* SK period (Fsk=0.5MHz) */
	}

    /* Deassert SROM Chip Select */
    eep &=~ M_PROM_EECS;
    WRITECSR16(sc, R_EEPROM_CTL, eep);
    cfe_nsleep(50);                   /* CS recovery (Tsks=50) */
}

static void
srom_write_bit(i82559_softc *sc, unsigned int data)
{
    uint16_t  eep;

    eep = READCSR16(sc, R_EEPROM_CTL);

    /* Place the data bit on the bus */
    if (data == 1)
	eep |= M_PROM_EEDI;
    else
	eep &=~ M_PROM_EEDI;

    WRITECSR16(sc, R_EEPROM_CTL, eep);
    cfe_nsleep(360);                      /* setup: Tdis=200 */

    /* Now clock the data into the SROM */
    WRITECSR16(sc, R_EEPROM_CTL, eep | M_PROM_EESK);
    cfe_nsleep(900);                      /* clock high, Tskh=500 */
    WRITECSR16(sc, R_EEPROM_CTL, eep);
    cfe_nsleep(450);                      /* clock low, Tskl=250 */

    /* Now clear the data bit */
    eep &=~ M_PROM_EEDI;                  /* data invalid, Tidh=20 for SK^ */
    WRITECSR16(sc, R_EEPROM_CTL, eep);
    cfe_nsleep(270);                      /* min cycle, 1/Fsk=2000 */
}

static uint16_t
srom_read_bit(i82559_softc *sc)
{
    uint16_t  eep;

    eep = READCSR16(sc, R_EEPROM_CTL);

    /* Generate a clock cycle before doing a read */
    WRITECSR16(sc, R_EEPROM_CTL, eep | M_PROM_EESK);  /* rising edge */
    cfe_nsleep(1000);                 /* clock high, Tskh=500, Tpd=1000 */
    WRITECSR16(sc, R_EEPROM_CTL, eep);                    /* falling edge */
    cfe_nsleep(1000);                 /* clock low, 1/Fsk=2000 */

    eep = READCSR16(sc, R_EEPROM_CTL);
    return ((eep & M_PROM_EEDO) != 0 ? 1 : 0);
}

#define CMD_BIT_MASK (1 << (EEPROM_CMD_BITS+EEPROM_ADDR_BITS-1))

static uint16_t
srom_read_word(i82559_softc *sc, unsigned int index)
{
    uint16_t command, word;
    uint16_t eep;
    unsigned int i;

    eep = READCSR16(sc, R_EEPROM_CTL) | M_PROM_EECS;

    /* Assert the SROM CS line */
    WRITECSR16(sc, R_EEPROM_CTL, eep);
    cfe_nsleep(100);                /* CS setup, Tcss = 100 */

    /* Send the read command to the SROM */
    command = (K_EEPROM_READ_CMD << EEPROM_ADDR_BITS) | index;
    for (i = 0; i < EEPROM_CMD_BITS+EEPROM_ADDR_BITS; i++) {
	srom_write_bit(sc, (command & CMD_BIT_MASK) != 0 ? 1 : 0);
	command <<= 1;
	}

    /* Now read the bits from the SROM (MSB first) */
    word = 0;
    for (i = 0; i < 16; ++i) {
	word <<= 1;
	word |= srom_read_bit(sc);
	}

    /* Clear the SROM CS Line,  CS hold, Tcsh = 0 */
    WRITECSR16(sc, R_EEPROM_CTL, eep &~ M_PROM_EECS);

    return word;
}


/* Read the entire SROM into the srom array.
   XXX Assumes a 64-word EEPROM.  Size it dynamically (p 46)? */

static int
srom_read_all(i82559_softc *sc, uint8_t dest[])
{
    int  i;
    uint16_t temp;

    srom_idle_state(sc);

    for (i = 0; i < EEPROM_SIZE/2; i++) {
	temp = srom_read_word(sc, i);
	dest[2*i] = temp & 0xFF;
	dest[2*i+1] =temp >> 8;
	}

    WRITECSR16(sc, R_EEPROM_CTL, 0);   /* CS hold, Tcsh=0 */

    return 0;
}

static void
srom_dump(uint8_t srom[])
{
    int  i;

    xprintf("I82559: EEPROM data:");
    for (i = 0; i < EEPROM_SIZE; i++) {
	if (i % 16 == 0)
	    xprintf("\n %02x: ", i);
	xprintf(" %02x", srom[i]);
	}
    xprintf("\n");
}


/* MII access */

static uint16_t
mii_read(i82559_softc *sc, unsigned int reg)
{
    uint32_t cmd, status;
    uint32_t data;
    int timeout;

    cmd = (V_MDI_OP(K_MDI_OP_READ) |
           V_MDI_PHYADD(sc->phy_addr) | V_MDI_REGADD(reg));
    WRITECSR(sc, R_MDI_CTL, cmd);

    for (timeout = 5000; timeout > 0; timeout -= 100) {
	status = READCSR(sc, R_MDI_CTL);
	if ((status & M_MDI_R) != 0)
	    break;
	cfe_usleep(100);
	}

    if (timeout <= 0)
	return 0xFFFF;

    data = G_MDI_DATA(READCSR(sc, R_MDI_CTL));
    return data;
}

static void
mii_write(i82559_softc *sc, unsigned int reg, uint16_t value)
{
    uint32_t cmd, status;
    int timeout;

    cmd = (V_MDI_OP(K_MDI_OP_WRITE) |
           V_MDI_PHYADD(sc->phy_addr) | V_MDI_REGADD(reg) |
	   V_MDI_DATA(value));
    WRITECSR(sc, R_MDI_CTL, cmd);

    for (timeout = 5000; timeout > 0; timeout -= 100) {
	status = READCSR(sc, R_MDI_CTL);
	if ((status & M_MDI_R) != 0)
	    break;
	cfe_usleep(100);
	}
}

static int
mii_probe(i82559_softc *sc)
{
    int i;
    uint16_t id1, id2;

    for (i = 1; i < 32; i++) {
        sc->phy_addr = i;
        id1 = (*sc->mii_read_register)(sc, MII_PHYIDR1);
	id2 = (*sc->mii_read_register)(sc, MII_PHYIDR2);
	if ((id1 != 0x0000 && id1 != 0xFFFF) ||
	    (id2 != 0x0000 && id2 != 0xFFFF)) {
	    if (id1 != id2) {
		sc->phy_vendor = ((uint32_t)id1 << 6) | ((id2 >> 10) & 0x3F);
		sc->phy_device = (id2 >> 4) & 0x3F;
	        return 0;
		}
	    }
	}
    xprintf("mii_probe: No PHY found\n");
    sc->phy_addr = 0x1;   /* Try the default internal PHY */
    sc->phy_vendor = sc->phy_device = 0;
    return -1;
}

#if I82559_DEBUG
static void
mii_dump(i82559_softc *sc, const char *label)
{
    int i;
    uint16_t  r;

    xprintf("%s\n", label);
    xprintf("OUI: %08x, Part %02x\n", sc->phy_vendor, sc->phy_device);
    for (i = 0; i <= 6; ++i) {
	r = (*sc->mii_read_register)(sc, i);
	xprintf("MII_REG%02x: %04x\n", i, r);
	}
    if (sc->phy_vendor == OUI_INTEL && sc->phy_device == DEV_I82555) {
	r = (*sc->mii_read_register)(sc, 16);
	xprintf("MII_REG16: %04x\n", r);
	r = (*sc->mii_read_register)(sc, 27);
	xprintf("MII_REG27: %04x\n", r);
	}
}
#else
#define mii_dump(sc,label)
#endif

static void
mii_autonegotiate(i82559_softc *sc)
{
    uint16_t  control, status, cap;
    unsigned int  timeout;
    int linkspeed;
    int autoneg;

    linkspeed = ETHER_SPEED_UNKNOWN;

    /* Read twice to clear latching bits */
    status = (*sc->mii_read_register)(sc, MII_BMSR);
    status = (*sc->mii_read_register)(sc, MII_BMSR);
    if (I82559_DEBUG) mii_dump(sc, "query PHY");

    if ((status & (BMSR_AUTONEG | BMSR_LINKSTAT)) ==
        (BMSR_AUTONEG | BMSR_LINKSTAT))
	control = (*sc->mii_read_register)(sc, MII_BMCR);
    else {
	/* reset the PHY */
	(*sc->mii_write_register)(sc, MII_BMCR, BMCR_RESET);
	timeout = 3000;
	for (;;) {
	    control = (*sc->mii_read_register)(sc, MII_BMCR);
	    if ((control && BMCR_RESET) == 0) break;
	    cfe_sleep(CFE_HZ/2);
	    timeout -= 500;
	    if (timeout <= 0) break;
	    }
	if ((control & BMCR_RESET) != 0) {
	    xprintf("%s: PHY reset failed\n", i82559_devname(sc));
	    return;
	    }

	status = (*sc->mii_read_register)(sc, MII_BMSR);
	cap = ((status >> 6) & (ANAR_TXFD | ANAR_TXHD | ANAR_10FD | ANAR_10HD))
	      | PSB_802_3;
	(*sc->mii_write_register)(sc, MII_ANAR, cap);
	control |= (BMCR_ANENABLE | BMCR_RESTARTAN);
	(*sc->mii_write_register)(sc, MII_BMCR, control);

	timeout = 3000;
	for (;;) {
	    status = (*sc->mii_read_register)(sc, MII_BMSR);
	    if ((status & BMSR_ANCOMPLETE) != 0) break;
	    cfe_sleep(CFE_HZ/2);
	    timeout -= 500;
	    if (timeout <= 0) break;
	    }
	if (I82559_DEBUG) mii_dump(sc, "done PHY");
	}

    xprintf("%s: Link speed: ", i82559_devname(sc));
    if ((status & BMSR_ANCOMPLETE) != 0) {
	/* A link partner was negogiated... */

	uint16_t remote = (*sc->mii_read_register)(sc, MII_ANLPAR);

	autoneg = 1;
	if ((remote & ANLPAR_TXFD) != 0) {
	    xprintf("100BaseT FDX");
	    linkspeed = ETHER_SPEED_100FDX;	 
	    }
	else if ((remote & ANLPAR_TXHD) != 0) {
	    xprintf("100BaseT HDX");
	    linkspeed = ETHER_SPEED_100HDX;	 
	    }
	else if ((remote & ANLPAR_10FD) != 0) {
	    xprintf("10BaseT FDX");
	    linkspeed = ETHER_SPEED_10FDX;	 
	    }
	else if ((remote & ANLPAR_10HD) != 0) {
	    xprintf("10BaseT HDX");
	    linkspeed = ETHER_SPEED_10HDX;	 
	    }
	xprintf("\n");
	}
    else {
	/* no link partner negotiation */
	autoneg = 0;
	xprintf("Unknown, assuming 10BaseT\n");
	control &=~ (BMCR_ANENABLE | BMCR_RESTARTAN);
	(*sc->mii_write_register)(sc, MII_BMCR, control);
	linkspeed = ETHER_SPEED_10HDX;
	}

    if ((status & BMSR_LINKSTAT) == 0)
	(*sc->mii_write_register)(sc, MII_BMCR, control);
#if 0  /* NYI */
    mii_set_speed(sc, linkspeed, autoneg);
#endif

    status = (*sc->mii_read_register)(sc, MII_BMSR);  /* clear latching bits */
    if (I82559_DEBUG) mii_dump(sc, "final PHY");
}


/* Utilities */

#define CBALLOC(ext) \
    ((uint32_t *)KMALLOC(CB_HDR_BYTES+(ext), sizeof(uint32_t)))
#define CBFREE(cb)   KFREE((void *)cb)

/* Initiate an action command and wait for it to complete.  The common
   CB header is filled in here.  Any extension must be already be
   filled in correct DMA byte order. */
static int
i82559_polled_action(i82559_softc *sc, uint8_t op, uint32_t *cb)
{
    uint16_t cmd, status;
    int count;
    cb_hdr_t *cbh = (cb_hdr_t *)cb;

    cbh->cb_stat &= HTOL2(~(M_CB0_OK | M_CB0_C));
    cmd = ltoh2(cbh->cb_cmd);
    cmd &= ~(M_CB0_I | M_CB0_S | M_CB0_CMD);
    cmd |= (V_CB0_CMD(op) | M_CB0_EL);
    cbh->cb_cmd = htol2(cmd);

    cbh->cb_link = HTOL4(NULL_LINK);   /* Apparent end of chain encoding. */
  
    i82559_scb_command1(sc, V_SCB_CUC(K_CUC_START), PTR_TO_PCI(cb));

    count = 1000;
    do {
        cfe_usleep(10);
	status = ltoh2(cbh->cb_stat);
	} while ((status & M_CB0_C) == 0 && --count > 0);

    /* Setting EL asserts CI/CNA on completion. */
    WRITECSR8(sc, R_SCB_STATACK, M_SCB_CNA);

    return ((status & M_CB0_OK) != 0);
}

static void
i82559_dump(i82559_softc *sc)
{
    uint32_t buffer[149];
    uint32_t *cb;

    cb = CBALLOC(sizeof(uint32_t));
    if (cb != NULL) {
	uint8_t *bb;
	int i;

	cb[CB_HDR_WORDS+0] = htol4(PTR_TO_PCI(buffer));
	i82559_polled_action(sc, K_CB_DUMP, cb);
	CBFREE(cb);

	bb = (uint8_t *)buffer;
	xprintf("i82559: state dump:");
	for (i = 0; i < 149*4; i++) {
	    if (i % 16 == 0)
		xprintf("\n %04x:", i);
	    if (i % 4 == 0)
		xprintf(" ");
	    xprintf(" %02x", *bb++);
	    }
	xprintf("\n");
	}
}


static void
i82559_isr(void *arg)
{
    i82559_softc *sc = (i82559_softc *)arg;
    uint8_t status;

#if IPOLL
    sc->interrupts++;
#endif

    for (;;) {

	/* Read and clear the interrupt status. */
	status = READCSR8(sc, R_SCB_STATACK);
	WRITECSR8(sc, R_SCB_STATACK, status);
	if ((status & sc->intmask) == 0)
	    break;

	if (status & (M_SCB_FR | M_SCB_RNR)) {
#if IPOLL
	    sc->rx_interrupts++;
#endif
	    /* Process received packets. */
	    i82559_procrxchain(sc);
	    }
	if (status & M_SCB_RNR) {
	    /* Restart the receiver. */
	    i82559_rx_restart(sc);
	    }

	if (status & M_SCB_CX) {
#if IPOLL
	    sc->tx_interrupts++;
#endif
	    /* Process received packets. */
	    i82559_proctxring(sc);
	    }

	}
}


/* Required 1-bits and selected setting for the i82559 Configuration
   Bytes.  These match the mandatory and most of the recommended
   values in the manual.  Chip-specific adjustments are done below. */
static const uint8_t proto_config[24] = {
    0x16,    /*  0 */
    0x08,
    0x00,
    0x01,                 /* MWI Enable (not i82557) */
    0x00,    /*  4 */
    0x00,
    0x22,                 /* Extended TxCB (not i82557) */
    0x03,
    0x01,    /*  8 */      
    0x00,
    0x2E,
    0x00,
    0x61,    /* 12 */
    0x00,
    0xF2,
    0x48,
    0x00,    /* 16 */
    0x00,
    0xF3,
    0x80,
    0x3F,    /* 20 */
    0x05,
    0x00,    /* filler */
    0x00
};

#if 0  /* Observed after reset (i82557, rev 2): */
    0x00, 0x00, 0x2e, 0x00,    /*  8 - 11 */
    0x60, 0x00, 0xF2, 0xC8,    /* 12 - 15 */
    0x00, 0x40, 0xF2, 0x00,    /* 16 - 19 */
    0x3F, 0x05,                /* 20 - 21 */
#endif

#if 0  /* Observed after reset (i82559, rev 8): */
    0x01, 0x00, 0x2e, 0x00,
    0x60, 0xFF, 0xFF, 0xC8,
    0x00, 0x40, 0xF2, 0x00,
    0x3F, 0x05,
#endif

#if 0  /* Observed after reset (i82559S, rev 9): */
    0x01, 0x00, 0x2e, 0x00,
    0x60, 0xFF, 0xFF, 0xC8,
    0x00, 0x40, 0xF2, 0x80,
    0x3F, 0x05,
#endif

static void
i82559_configure(i82559_softc *sc)
{
    uint32_t *cb;
    uint8_t *config;

    cb = CBALLOC(sizeof(proto_config));
    if (cb != NULL) {
	config = (uint8_t *)&cb[CB_HDR_WORDS];
	memcpy(config, proto_config, sizeof(proto_config));

	/* Chip-specific adjustments */
	switch (sc->chip) {
	    case chip_i82557:
		config[3]  = 0x00;   /* no MWI */
		config[5]  = 0x32;   /* no Extended TxCB */
		config[12] = 0x60;   /* wait on tx only */
		config[17] = 0x40;   /* restore default */
		break;
	    default:
	        break;
	    }
	
	i82559_polled_action(sc, K_CB_CONFIGURE, cb);
	CBFREE(cb);
	}
}

static void
i82559_setaddress(i82559_softc *sc)
{
    uint32_t *cb;
    uint8_t *hwaddr;

    cb = CBALLOC(ENET_ADDR_LEN);
    if (cb != NULL) {
	hwaddr = (uint8_t *)&cb[CB_HDR_WORDS];
	memcpy(hwaddr, sc->hwaddr, ENET_ADDR_LEN);
	
	i82559_polled_action(sc, K_CB_ADDRSETUP, cb);
	CBFREE(cb);
	}
}

static void
i82559_hwinit(i82559_softc *sc)
{
    if (sc->state == eth_state_uninit) {
	sc->mii_read_register = mii_read;
	sc->mii_write_register = mii_write;

	mii_probe(sc);
	mii_autonegotiate(sc);
	if (sc->chip != chip_i82557 && sc->chip != chip_i82558) {
	    sc->linkstat = READCSR8(sc, R_GEN_STAT);
	    sc->linkstat &= (M_GSTAT_LINKUP | M_GSTAT_100 | M_GSTAT_FDX);
	    }

	/* Set up linear addressing. */
	i82559_scb_command1(sc, V_SCB_CUC(K_CUC_BASE), 0);
	i82559_scb_command1(sc, V_SCB_RUC(K_RUC_BASE), 0);

	if (I82559_DEBUG) i82559_dump(sc);

	i82559_configure(sc);
	i82559_setaddress(sc);
	sc->tx_thresh = 8;            /* 8*8 = 64 bytes */

	if (I82559_DEBUG) i82559_dump(sc);
	}
}


static void
i82559_start(i82559_softc *sc)
{
    uint8_t status;
    volatile tcb_t *tcb;
    volatile rfd_t *rfd;

    i82559_hwinit(sc);

    sc->intmask = 0;
    status = READCSR8(sc, R_SCB_STATACK);
    WRITECSR8(sc, R_SCB_STATACK, status);    /* clear any pending. */
    READCSR8(sc, R_SCB_STATACK);             /* push */

    sc->intmask = (M_SCB_FR | M_SCB_RNR | M_SCB_CX);

#if IPOLL
    cfe_request_irq(sc->irq, i82559_isr, sc, CFE_IRQ_FLAGS_SHARED, 0);
    WRITECSR8(sc, R_SCB_IC, 0);
#endif

    rfd = &sc->rx_remove->rfd;
    i82559_scb_command1(sc, V_SCB_RUC(K_RUC_START), PTR_TO_PCI(rfd));

    tcb = sc->tx_suspend;
    tcb->tcb_hdr.cb_cmd = HTOL2(V_CB0_CMD(K_CB_NOP) | M_CB0_S);
    tcb->tcb_hdr.cb_stat &= HTOL2(~(M_CB0_OK | M_CB0_C));
    i82559_scb_command1(sc, V_SCB_CUC(K_CUC_START), PTR_TO_PCI(tcb));

    sc->state = eth_state_on;
}

static void
i82559_stop(i82559_softc *sc)
{
    uint8_t status;

    /* Make sure that no further interrupts will be processed. */
    sc->intmask = 0;
    WRITECSR8(sc, R_SCB_IC, M_SCB_M);
    (void)READCSR8(sc, R_SCB_IC);   /* push */
    cfe_usleep(10);
    status = READCSR8(sc, R_SCB_STATACK);
    WRITECSR8(sc, R_SCB_STATACK, status);

#if IPOLL
    cfe_free_irq(sc->irq, 0);
#endif

    xprintf("%s: %d sent, %d received, %d interrupts\n",
	    i82559_devname(sc), sc->outpkts, sc->inpkts, sc->interrupts);
    if (IPOLL) {
	xprintf("  %d rx interrupts, %d tx interrupts\n",
		sc->rx_interrupts, sc->tx_interrupts);
	}
}


/* Declarations for CFE Device Driver Interface routines */

static int i82559_ether_open(cfe_devctx_t *ctx);
static int i82559_ether_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int i82559_ether_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int i82559_ether_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int i82559_ether_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int i82559_ether_close(cfe_devctx_t *ctx);
static void i82559_ether_poll(cfe_devctx_t *ctx, int64_t ticks);
static void i82559_ether_reset(void *softc);


/* CFE Device Driver dispatch structure */

const static cfe_devdisp_t i82559_ether_dispatch = {
    i82559_ether_open,
    i82559_ether_read,
    i82559_ether_inpstat,
    i82559_ether_write,
    i82559_ether_ioctl,
    i82559_ether_close,
    i82559_ether_poll,
    i82559_ether_reset
};

/* CFE Device Driver descriptor */

const cfe_driver_t i82559drv = {
    "i8255x Ethernet",
    "eth",
    CFE_DEV_NETWORK,
    &i82559_ether_dispatch,
    i82559_ether_probe
};


/* CFE Device Driver probe functions. */

static int
i82559_ether_attach(cfe_driver_t *drv, pcitag_t tag, int index)
{
    i82559_softc *sc;
    uint32_t device;
    uint32_t class;
    phys_addr_t pa;
    uint8_t eeprom[EEPROM_SIZE];
    const char *devname;
    char descr[100];

    device = pci_conf_read(tag, PCI_ID_REG);
    class = pci_conf_read(tag, PCI_CLASS_REG);

#if 1
    /* Use memory space for the CSRs */
    pci_map_mem(tag, PCI_MAPREG(0), PCI_MATCH_BITS, &pa);
#else
    /* Use i/o space for the CSRs */
    pci_map_io(tag, PCI_MAPREG(1), PCI_MATCH_BITS, &pa);
#endif

    sc = (i82559_softc *) KMALLOC(sizeof(i82559_softc), 0);

    if (sc == NULL) {
	xprintf("I82559: No memory to complete probe\n");
	return 0;
	}
    memset(sc, 0, sizeof(i82559_softc));

    sc->membase = (uint32_t)pa;
    sc->irq = pci_conf_read(tag, PCI_BPARAM_INTERRUPT_REG) & 0xFF;
    sc->tag = tag;
    sc->device = PCI_PRODUCT(device);
    sc->revision = PCI_REVISION(class);
    sc->devctx = NULL;

    sc->linkspeed = ETHER_SPEED_AUTO;    /* select autonegotiation */

    sc->state = eth_state_uninit;

    /* Intel recommends a software reset prior to any access. */
    WRITECSR(sc, R_PORT, V_PORT_FUNC(K_PORT_FUNC_SWRESET));
    cfe_usleep(10);
    WRITECSR8(sc, R_SCB_IC, M_SCB_M);    /* mask all interrupts */

    i82559_init(sc);

    srom_read_all(sc, eeprom);
    if (I82559_DEBUG) srom_dump(eeprom);
    memcpy(sc->hwaddr, &eeprom[EEPROM_ADDR_INDEX], ENET_ADDR_LEN);

    if (sc->device == K_PCI_ID_I82559ER) {
        devname = "i82559ER";
	sc->chip = chip_i82559;
	}    
    else {
	switch (sc->revision) {
	    case 0x01: case 0x02: case 0x03:
		devname = "i82557";
		sc->chip = chip_i82557;
		break;
	    case 0x04: case 0x05:
		devname = "i82558";
		sc->chip = chip_i82558;
		break;
	    case 0x06: case 0x07: case 0x08:
		devname = "i82559";
		sc->chip = chip_i82559;
		break;
	    case 0x09:
		devname = "i82559S";
		sc->chip = chip_i82559;
		break;
	    case 0x0C: case 0x0D: case 0x0E:
		devname = "i82550";
		sc->chip = chip_i82550;
		break;
	    case 0x0F: case 0x10:
		devname = "i82551";
		sc->chip = chip_i82551;
		break;
	    default:
		devname = "i8255x";
		sc->chip = chip_i82557;
		break;
	    }
	}
    xsprintf(descr, "%s Ethernet at 0x%X (%a)",
	     devname, sc->membase, sc->hwaddr);

    cfe_attach(drv, sc, NULL, descr);
    return 1;
}

static void
i82559_ether_probe(cfe_driver_t *drv,
		   unsigned long probe_a, unsigned long probe_b, 
		   void *probe_ptr)
{
    int index;
    int n;

    n = 0;
    index = 0;
    for (;;) {
	pcitag_t tag;
	pcireg_t device;

	if (pci_find_class(PCI_CLASS_NETWORK, index, &tag) != 0)
	    break;

	index++;

	device = pci_conf_read(tag, PCI_ID_REG);
	if (PCI_VENDOR(device) == K_PCI_VENDOR_INTEL) {
	    switch (PCI_PRODUCT(device)) {
		case K_PCI_ID_I82557:
		case K_PCI_ID_I82559ER:
		case K_PCI_ID_INBUSINESS:
		    i82559_ether_attach(drv, tag, n);
		    n++;
		    break;
		default:
		    break;
		}
	    }
	}
}


/* The functions below are called via the dispatch vector for the i82559. */

static int
i82559_ether_open(cfe_devctx_t *ctx)
{
    i82559_softc *sc = ctx->dev_softc;

    if (sc->state == eth_state_on)
	i82559_stop(sc);

    sc->devctx = ctx;

    sc->inpkts = sc->outpkts = 0;
    sc->interrupts = 0;
    sc->rx_interrupts = sc->tx_interrupts = 0;

    i82559_start(sc);

#if XPOLL
    i82559_isr(sc);
#endif

    return 0;
}

static int
i82559_ether_read(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    i82559_softc *sc = ctx->dev_softc;
    eth_pkt_t *pkt;
    int blen;

#if XPOLL
    i82559_isr(sc);
#endif

    if (sc->state != eth_state_on) return -1;

    CS_ENTER(sc);
    pkt = (eth_pkt_t *) q_deqnext(&(sc->rxqueue));
    CS_EXIT(sc);

    if (pkt == NULL) {
	buffer->buf_retlen = 0;
	}
    else {
	blen = buffer->buf_length;
	if (blen > pkt->length) blen = pkt->length;

	hs_memcpy_to_hs(buffer->buf_ptr, pkt->buffer, blen);
	buffer->buf_retlen = blen;

	eth_free_pkt(sc, pkt);
	i82559_fillrxchain(sc);
	}

#if XPOLL
    i82559_isr(sc);
#endif

    return 0;
}

static int
i82559_ether_inpstat(cfe_devctx_t *ctx, iocb_inpstat_t *inpstat)
{
    i82559_softc *sc = ctx->dev_softc;

#if XPOLL
    i82559_isr(sc);
#endif

    if (sc->state != eth_state_on) return -1;

    /* We avoid an interlock here because the result is a hint and an
       interrupt cannot turn a non-empty queue into an empty one. */
    inpstat->inp_status = (q_isempty(&(sc->rxqueue))) ? 0 : 1;

    return 0;
}

static int
i82559_ether_write(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    i82559_softc *sc = ctx->dev_softc;
    eth_pkt_t *pkt;
    int blen;

#if XPOLL
    i82559_isr(sc);
#endif

    if (sc->state != eth_state_on) return -1;

    pkt = eth_alloc_pkt(sc);
    if (!pkt) return CFE_ERR_NOMEM;

    blen = buffer->buf_length;
    if (blen > pkt->length) blen = pkt->length;

    hs_memcpy_from_hs(pkt->buffer, buffer->buf_ptr, blen);
    pkt->length = blen;

    if (i82559_transmit(sc, pkt) != 0) {
	eth_free_pkt(sc, pkt);
	return CFE_ERR_IOERR;
	}

#if XPOLL
    i82559_isr(sc);
#endif

    return 0;
}

static int
i82559_ether_ioctl(cfe_devctx_t *ctx, iocb_buffer_t *buffer) 
{
    i82559_softc *sc = ctx->dev_softc;
    int speed;

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
	    return -1;     /* NYI */

	case IOCTL_ETHER_GETLINK:
	    speed = sc->linkspeed;
	    hs_memcpy_to_hs(buffer->buf_ptr,&speed,sizeof(int));
	    return 0;

	case IOCTL_ETHER_GETLOOPBACK:
	    speed = sc->loopback;
	    hs_memcpy_to_hs(buffer->buf_ptr,&speed,sizeof(int));
	    return 0;

	case IOCTL_ETHER_SETLOOPBACK:
	    return -1;     /* NYI */

	default:
	    return -1;
	}
}

static int
i82559_ether_close(cfe_devctx_t *ctx)
{
    i82559_softc *sc = ctx->dev_softc;

    sc->state = eth_state_off;
    i82559_stop(sc);

    /* resynchronize the control blocks */
    i82559_resetrings(sc);

    sc->devctx = NULL;
    return 0;
}

static void
i82559_ether_poll(cfe_devctx_t *ctx, int64_t ticks)
{
    i82559_softc *sc = ctx->dev_softc;

#if XPOLL
    i82559_isr(sc);
#endif

    if (sc->chip != chip_i82557 && sc->chip != chip_i82558) {
	uint8_t linkstat = READCSR8(sc, R_GEN_STAT);

	linkstat &= (M_GSTAT_LINKUP | M_GSTAT_100 | M_GSTAT_FDX);
	if (linkstat != sc->linkstat) {
	    if ((linkstat & M_GSTAT_LINKUP) == 0) {
		if ((sc->linkstat & M_GSTAT_LINKUP) != 0) {
		    xprintf("%s: Link speed: Unknown (down)\n",
			    i82559_devname(sc));
		    }
		}
	    else {
		xprintf("%s: Link speed: %sBaseT %s\n",	i82559_devname(sc),
			((linkstat & M_GSTAT_100) != 0 ? "100" : "10"),
			((linkstat & M_GSTAT_FDX) != 0 ? "FDX" : "HDX"));
		}
	    sc->linkstat = linkstat;
	    }
	}
}

static void
i82559_ether_reset(void *softc)
{
    /* NYI */
}
