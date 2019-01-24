/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Adaptec AIC-6915 (10/100 EthernetMAC) driver	File: dev_aic6915.c
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

#include "cfe_irq.h"

#include "pcivar.h"
#include "pcireg.h"

#include "aic6915.h"
#include "mii.h"

/* This is a driver for the Adaptec AIC6915 ("Starfire") 10/100 MAC.

   Reference:
     AIC-6915 Ethernet LAN Controller, Programmer's Manual
     Adaptec, Inc., 691 South Milpitas Boulevard, Milpitas CA, 1998

   The current version has been developed for the Adaptec 62011/TX
   single-port NIC with SEEQ/LSI 80220 PHY.  The card is strapped to
   preload information from a serial EEPROM at power up.

   This is a simple version for understanding the chip.  Optimizations
   can come later.

   This BCM1250 version takes advantage of DMA coherence and uses
   "preserve bit lanes" addresses for all accesses that cross the
   ZBbus-PCI bridge. */

#ifndef AIC_DEBUG
#define AIC_DEBUG 0
#endif

/* Set IPOLL to drive processing through the pseudo-interrupt
   dispatcher.  Set XPOLL to drive processing by an external polling
   agent.  Setting both is ok. */

#ifndef IPOLL
#define IPOLL 1
#endif
#ifndef XPOLL
#define XPOLL 0
#endif

#define ENET_ADDR_LEN	6		/* size of an ethernet address */
#define MIN_ETHER_PACK  64              /* min size of a packet */
#define MAX_ETHER_PACK  1518		/* max size of a packet */
#define VLAN_TAG_LEN    4               /* VLAN type plus tag */
#define CRC_SIZE	4		/* size of CRC field */

/* Packet buffers.  For the AIC-6915, a receive packet buffer must be
   aligned to a 32-bit word boundary.  We would like it aligned to a
   cache line boundary for performance.  Note that the IP/TCP header
   will be 16-bit, but not 32-bit, aligned with this constraint. */

#define ETH_PKTBUF_LEN      (((MAX_ETHER_PACK+31)/32)*32)

#if __long64
typedef struct eth_pkt_s {
    queue_t next;			/* 16 */
    uint8_t *buffer;			/*  8 */
    uint32_t flags;			/*  4 */
    int32_t length;			/*  4 */
    uint8_t data[ETH_PKTBUF_LEN];
} eth_pkt_t;
#else
typedef struct eth_pkt_s {
    queue_t next;			/*  8 */
    uint8_t *buffer;			/*  4 */
    uint32_t flags;			/*  4 */
    int32_t length;			/*  4 */
    uint32_t unused[3];			/* 12 */
    uint8_t data[ETH_PKTBUF_LEN];
} eth_pkt_t;
#endif

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


/* How often to check MII status */
#define MII_POLL_INTERVAL (1*CFE_HZ)

/* How often to accumulate PCI usage statistics */
#define PCI_POLL_INTERVAL (CFE_HZ/8)

static void aic_ether_probe(cfe_driver_t *drv,
			    unsigned long probe_a, unsigned long probe_b, 
			    void *probe_ptr);


/* AIC-6915 Hardware Data Structures 
   XXX These work for 1250 big endian, 32-bit physical addressing.
   XXX Should move to a header file? */

/* AIC-6915 Ring Sizes */

#define RX_RING_ENTRIES_256    256
#define RX_RING_ENTRIES_2K     2048

#define TX_RING_MAXSIZE        16384

#define COMP_RING_ENTRIES      1024

/* AIC-6915 Receive Descriptors (producer ring).  Page 2-4.  There are
   2 types, only 32-bit host addressing is supported for now.  */

typedef struct aic_rx_dscr0_s {
    uint32_t  bufptr;
} aic_rx_dscr0_t;

#define M_RX_DSCR_ADDR         0xfffffffc
#define M_RX_DSCR_END          0x00000002
#define M_RX_DSCR_VALID        0x00000001

/* AIC-6915 Transmit Descriptors (producer ring).  Pages 3-5 to 3-6.
   There are 5 types, only buffer descriptors and 32-bit addressing
   are supported for now. */

typedef struct aic_tx_dscr1_s {
    uint16_t flags;
    uint16_t length;
    uint32_t bufptr;
} aic_tx_dscr1_t;

#define M_FLAG_ID                 0xf000
#define V_FLAG_ID_TX              0xb000

#define M_FLAG_INTR               0x0800
#define M_FLAG_END                0x0400
#define M_FLAG_CALTCP             0x0200
#define M_FLAG_CRCEN              0x0100
#define S_FLAG_NBUFFERS           0
#define M_FLAG_NBUFFERS           0x00ff

/* AIC-6915 Completion Descriptors.  RX and TX can be configured for
   shared or disjoint rings.  Leading bits of word 0 distinguish.
   When intermixed, types must be chosen to give identical sizes.  */

#define M_COMP_TAG                0xc0000000
#define V_COMP_TAG_NONE           0x00000000
#define V_COMP_TAG_RX             0x40000000
#define V_COMP_TAG_TX             0x80000000

/* AIC-6915 Receive Completion Descriptors.  Pages 2-5 to 2-9.
   There are 4 types, selected globally and not self-identifying. */

typedef struct aic_rx_comp_dscr0_s {
    uint16_t status1; 
    uint16_t length;
}  aic_rx_comp_dscr0_t;

typedef struct aic_rx_comp_dscr1_s {
    uint16_t status1; 
    uint16_t length;
    uint16_t status2;
    uint16_t vlan;
}  aic_rx_comp_dscr1_t;

typedef struct aic_rx_comp_dscr2_s {
    uint16_t status1; 
    uint16_t length;
    uint16_t status2;
    uint16_t tcp_cksum;
}  aic_rx_comp_dscr2_t;

typedef struct aic_rx_comp_dscr3_s {
    uint16_t status1; 
    uint16_t length;
    uint16_t status2;
    uint16_t status3;
    uint16_t tcp_cksum;
    uint16_t vlan;
    uint32_t timestamp;
}  aic_rx_comp_dscr3_t;

/* Fields of Status1 */
#define M_STATUS1_TAG             0xc000
#define V_STATUS1_TAG_RX          0x4000
#define M_STATUS1_OK              0x2000
#define M_STATUS1_FIFOFULL        0x1000
#define M_STATUS1_Q               0x0800
#define M_END_INDEX               0x07ff  /* in entries */

/* Fields of Status2 */
#define M_STATUS2_PERFECT         0x8000
#define M_STATUS2_HASH            0x4000
#define M_STATUS2_CRCERR          0x2000
#define M_STATUS2_ISL_CRCERR      0x1000
#define M_STATUS2_DRIBBLE         0x0800
#define M_STATUS2_CODEERR         0x0400
#define M_STATUS2_VLAN            0x0200
#define M_STATUS2_CKSUM_OK        0x0100
#define M_STATUS2_CKSUM_BAD       0x0080
#define M_STATUS2_PART_CKSUM_OK   0x0040
#define M_STATUS2_FRAG            0x0020
#define M_STATUS2_TCP             0x0010
#define M_STATUS2_UDP             0x0008
#define M_STATUS2_TYPE            0x0007
#define K_STATUS2_TYPE_UNKNOWN    0x0
#define K_STATUS2_TYPE_IPV4       0x1
#define K_STATUS2_TYPE_IPV6       0x2
#define K_STATUS2_TYPE_IPX        0x3
#define K_STATUS2_TYPE_ICMP       0x4
#define K_STATUS2_TYPE_UNSUPP     0x5

/* Fields of Status3 */
#define M_STATUS3_ISL             0x8000
#define M_STATUS3_PAUSE           0x4000
#define M_STATUS3_CONTROL         0x2000
#define M_STATUS3_HEADER          0x1000
#define M_STATUS3_TRAILER         0x0800
#define M_START_INDEX             0x07ff  /* in entries */

/* AIC-6915 Transmit Completion Descriptors (return ring).  Pages 3-10 to 3-11.
   There are 2 types */

typedef struct aic_tx_comp_dscr0_s {
    uint16_t stamp; 
    uint16_t qref;
}  aic_tx_comp_dscr0_t;

typedef struct aic_tx_comp_dscr1_s {
    uint16_t status; 
    uint16_t qref;
}  aic_tx_comp_dscr1_t;

#define M_COMP_TYPE               0x02000000
#define V_COMP_TYPE_DMA           0x00000000
#define V_COMP_TYPE_TX            0x20000000

/* DMA Complete Entries */
#define M_STAMP_TAG               0xc000
#define M_STAMP_TYPE              0x2000
#define M_STAMP_TIME              0x1fff

/* Transmit Complete Entries */
#define M_STATUS_TAG              0xc000
#define M_STATUS_TYPE             0x2000
#define M_STATUS_PAUSED           0x1000
#define M_STATUS_PAUSE            0x0800
#define M_STATUS_CONTROL          0x0400
#define M_STATUS_UNDERRUN         0x0200
#define M_STATUS_OVERSIZE         0x0100
#define M_STATUS_LATECOLL         0x0080
#define M_STATUS_EXSCOLL          0x0040
#define M_STATUS_EXSDEFER         0x0020
#define M_STATUS_DEFERRED         0x0010
#define M_STATUS_SUCCESS          0x0008
#define M_STATUS_FIELDRANGE       0x0004
#define M_STATUS_FIELDLENGTH      0x0002
#define M_STATUS_CRCERR           0x0001

/* Common */
#define M_QREF_Q                  0x8000
#define M_QREF_OFFSET             0x7FFF    /* in bytes */

/* End of AIC-6915 defined data structures */


typedef enum {
    eth_state_uninit,
    eth_state_off,
    eth_state_on, 
} eth_state_t;

/* All rings must be aligned to a 256-byte boundary.  Since all sizes
   are multiples of 256 bytes, they are allocated contiguously and
   aligned once.  For now, we use single rings (Q1) for rx and rx
   complete. */

#define RX_RING_ENTRIES           RX_RING_ENTRIES_256
#define TX_RING_ENTRIES           256

typedef struct aic_rings_s {
    aic_rx_dscr0_t       rx_dscr[RX_RING_ENTRIES];
    aic_rx_comp_dscr3_t  rx_done[COMP_RING_ENTRIES];
    aic_tx_dscr1_t       tx_dscr[TX_RING_ENTRIES];      /* explicit end bit */
    aic_tx_comp_dscr1_t  tx_done[COMP_RING_ENTRIES];
} aic_rings_t;

typedef struct aic_ether_s {
    uint32_t  regbase;
    uint8_t irq;
    pcitag_t tag;		/* tag for configuration registers */

    uint8_t   hwaddr[6];
    uint16_t  device;           /* chip device code */
    uint8_t revision;           /* chip revision */

    /* current state */
    eth_state_t state;
    uint32_t intmask;

    /* packet lists */
    queue_t freelist;
    uint8_t *pktpool;
    queue_t rxqueue;

    /* rings */
    aic_rx_dscr0_t      *rx_dscr;
    aic_rx_comp_dscr3_t *rx_done;
    aic_tx_dscr1_t      *tx_dscr;
    aic_tx_comp_dscr1_t *tx_done;
    aic_rings_t         *rings;        /* must be 256-byte aligned */

    cfe_devctx_t *devctx;

    /* PHY access */
    int      phy_addr;         /* normally 1 */
    uint16_t phy_status;
    uint16_t phy_ability;

    /* MII polling control */
    int      mii_polling;
    uint64_t mii_polltime;

    /* driver statistics */
    uint32_t inpkts;
    uint32_t outpkts;
    uint32_t interrupts;
    uint32_t rx_interrupts;
    uint32_t tx_interrupts;

    /* cumulative bus usage statistics (Tables 7-30 and 7-31) */
    uint64_t pci_polltime;
    uint32_t pci_latency;
    uint32_t int_latency;
    uint64_t pci_slave;
    uint64_t pci_master;
    uint64_t pci_data;
} aic_ether_t;


/* Address mapping macros */

#define PTR_TO_PHYS(x) (K0_TO_PHYS((uintptr_t)(x)))
#define PHYS_TO_PTR(a) ((uint8_t *)PHYS_TO_K0(a))

#define PCI_TO_PTR(a)  (PHYS_TO_PTR(PCI_TO_PHYS(a)))
#define PTR_TO_PCI(x)  (PHYS_TO_PCI(PTR_TO_PHYS(x)))


/* Chip access macros */

#define READCSR(sc,csr)      (phys_read32((sc)->regbase+(csr)))
#define WRITECSR(sc,csr,val) (phys_write32((sc)->regbase+(csr), (val)))


/* Entry to and exit from critical sections (currently relative to
   interrupts only, not SMP) */

#if CFG_INTERRUPTS
#define CS_ENTER(sc) cfe_disable_irq(sc->irq)
#define CS_EXIT(sc)  cfe_enable_irq(sc->irq)
#else
#define CS_ENTER(sc) ((void)0)
#define CS_EXIT(sc)  ((void)0)
#endif


static void
dumpseq(aic_ether_t *sc, int start, int next)
{
    int offset, i, j;
    int columns = 4;
    int lines = (((next - start)/4 + 1) + 3)/columns;
    int step = lines*4;

    offset = start;
    for (i = 0; i < lines; i++) {
	xprintf("\nCSR");
	for (j = 0; j < columns; j++) {
	    if (offset + j*step < next)
		xprintf(" %04X: %08X ",
			offset+j*step, READCSR(sc, offset+j*step));
	    }
	offset += 4;
	}
    xprintf("\n");
}

static void
dumpcsrs(aic_ether_t *sc, const char *legend)
{
    xprintf("%s:\n", legend);
    xprintf("-----Ctrl----");
    /* PCI control registers */
    dumpseq(sc, 0x40, 0x50);
    xprintf("---General---");
    /* Ethernet functional registers */
    dumpseq(sc, 0x70, 0xF8);
    xprintf("-----MAC-----");
    /* MAC registers */
    dumpseq(sc, 0x5000, 0x5018);
    xprintf("-------------\n");
}


/* Packet management */

#define ETH_PKTPOOL_SIZE  64
#define MIN_RX_DSCRS      32

static eth_pkt_t *
eth_alloc_pkt(aic_ether_t *sc)
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
eth_free_pkt(aic_ether_t *sc, eth_pkt_t *pkt)
{
    CS_ENTER(sc);
    q_enqueue(&sc->freelist, &pkt->next);
    CS_EXIT(sc);
}

static void
eth_initfreelist(aic_ether_t *sc)
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
aic_devname(aic_ether_t * sc)
{
    return (sc->devctx != NULL ? cfe_device_name(sc->devctx) : "eth?");
}

/* The delay loop uses uncached PCI reads, each of which requires at
   least 3 PCI bus clocks (90 ns at 33 MHz) to complete.  The actual
   delay will be longer.  */
#define PCI_MIN_DELAY  90

static void
aic_spin(aic_ether_t *sc, long usec)
{
#if 0  /* XXX Can't use this to time PCI resets */
    long  delay;
    volatile uint32_t t;

    for (delay = 1000*usec; delay > 0; delay -= PCI_MIN_DELAY)
	t = pci_conf_read(sc->tag, PCI_ID_REG);
#else
    cfe_sleep(2);
#endif
}


/* Descriptor ring management */

static void
aic_fillrxring(aic_ether_t *sc)
{
    uint32_t ptrs;
    unsigned rx_dscr_ci, rx_dscr_pi;
    unsigned rx_on_ring;
    eth_pkt_t *pkt;

    ptrs = READCSR(sc, R_RxDescQueue1Ptrs);
    rx_dscr_ci = (ptrs & M_RxDescConsumer) >> S_RxDescConsumer;
    rx_dscr_pi = (ptrs & M_RxDescProducer) >> S_RxDescProducer;
    if (rx_dscr_pi >= rx_dscr_ci)
	rx_on_ring = rx_dscr_pi - rx_dscr_ci;
    else
	rx_on_ring = (rx_dscr_pi + RX_RING_ENTRIES) - rx_dscr_ci;

    while (rx_on_ring < MIN_RX_DSCRS) {       /* XXX Check this */
	pkt = eth_alloc_pkt(sc);
	if (pkt == NULL) {
	    /* could not allocate a buffer */
	    break;
	    }
	sc->rx_dscr[rx_dscr_pi].bufptr =
	  PTR_TO_PCI(pkt->buffer) & M_RX_DSCR_ADDR;
	rx_dscr_pi = (rx_dscr_pi + 1) % RX_RING_ENTRIES;
	rx_on_ring++;
	}
    
    ptrs &=~ M_RxDescProducer;
    ptrs |= rx_dscr_pi << S_RxDescProducer;
    WRITECSR(sc, R_RxDescQueue1Ptrs, ptrs);
}

static void
aic_rx_callback(aic_ether_t *sc, eth_pkt_t *pkt)
{
    if (AIC_DEBUG) show_packet('>', pkt);

    CS_ENTER(sc);
    q_enqueue(&sc->rxqueue, &pkt->next);
    CS_EXIT(sc);
    sc->inpkts++;
}

static int
aic_procrxring(aic_ether_t *sc)
{
    uint32_t ptrs;
    unsigned rx_comp_ci, rx_comp_pi;
    int consumed;
    aic_rx_comp_dscr3_t *rxc;
    aic_rx_dscr0_t *rxd;
    eth_pkt_t   *pkt;

    /* Assumes that only Q1 is being used */
    ptrs = READCSR(sc, R_CompletionQueueProducerIndex);
    rx_comp_pi = G_RxCompletionQ1ProducerIndex(ptrs);
    ptrs = READCSR(sc, R_CompletionQueueConsumerIndex);
    rx_comp_ci = G_RxCompletionQ1ConsumerIndex(ptrs);
    consumed = 0;

    while (rx_comp_ci != rx_comp_pi) {
	rxc = &(sc->rx_done[rx_comp_ci]);
	rxd = &(sc->rx_dscr[rxc->status1 & M_END_INDEX]);

	pkt = ETH_PKT_BASE(PCI_TO_PTR(rxd->bufptr & M_RX_DSCR_ADDR));
	pkt->length = rxc->length;
	if ((rxc->status1 & M_STATUS1_OK) != 0)
	    aic_rx_callback(sc, pkt);
	else {
#if 1
	    xprintf("%s: rx error %04X\n", aic_devname(sc), rxc->status2);
#endif
	    eth_free_pkt(sc, pkt);
	}

	consumed++;
        rx_comp_ci = (rx_comp_ci + 1) % COMP_RING_ENTRIES;
	}

    /* Update the completion ring pointers */
    if (consumed != 0) {
	ptrs &= ~M_RxCompletionQ1ConsumerIndex;
	ptrs |= V_RxCompletionQ1ConsumerIndex(rx_comp_ci);
	WRITECSR(sc, R_CompletionQueueConsumerIndex, ptrs);
	}

    /* Refill the descriptor ring */
    aic_fillrxring(sc);
    
    return consumed;
}


static int
aic_transmit(aic_ether_t *sc, eth_pkt_t *pkt)
{
    uint32_t ptrs;
    unsigned tx_dscr_ci, tx_dscr_pi, next_pi;
    aic_tx_dscr1_t *txd;

    if (AIC_DEBUG) show_packet('<', pkt);

    ptrs = READCSR(sc, R_TxDescQueueConsumerIndex);
    tx_dscr_ci = (ptrs & M_LoPrTxConsumerIndex) >> S_LoPrTxConsumerIndex;
    ptrs = READCSR(sc, R_TxDescQueueProducerIndex);
    tx_dscr_pi = (ptrs & M_LoPrTxProducerIndex) >> S_LoPrTxProducerIndex;

    /* Pointers are in units of 8 bytes */
    next_pi = (tx_dscr_pi + sizeof(aic_tx_dscr1_t)/8) % TX_RING_ENTRIES;
    if (next_pi == tx_dscr_ci)   /* Ring full */
	return -1;

    txd = &(sc->tx_dscr[(tx_dscr_pi << 3)/sizeof(aic_tx_dscr1_t)]);
    txd->bufptr = PTR_TO_PCI(pkt->buffer);
    txd->length = pkt->length;

    ptrs &= ~M_LoPrTxProducerIndex;
    ptrs |= next_pi << S_LoPrTxProducerIndex;
    WRITECSR(sc, R_TxDescQueueProducerIndex, ptrs);

    sc->outpkts++;
    return 0;
}


static int
aic_proctxring(aic_ether_t *sc)
{
    uint32_t ptrs;
    unsigned tx_comp_ci, tx_comp_pi;
    int consumed;
    aic_tx_comp_dscr1_t *txc;
    aic_tx_dscr1_t *txd;
    unsigned index;
    eth_pkt_t   *pkt;

    ptrs = READCSR(sc, R_CompletionQueueProducerIndex);
    tx_comp_pi = G_TxCompletionProducerIndex(ptrs);
    ptrs = READCSR(sc, R_CompletionQueueConsumerIndex);
    tx_comp_ci = G_TxCompletionConsumerIndex(ptrs);
    consumed = 0;

    while (tx_comp_ci != tx_comp_pi) {
	txc = &(sc->tx_done[tx_comp_ci]);
	index = (txc->qref & M_QREF_OFFSET) / sizeof(aic_tx_dscr1_t);
	txd = &(sc->tx_dscr[index]);

	pkt = ETH_PKT_BASE(PCI_TO_PTR(txd->bufptr));
	eth_free_pkt(sc, pkt);

	consumed++;
        tx_comp_ci = (tx_comp_ci + 1) % COMP_RING_ENTRIES;
	}

    /* Update the completion ring pointers */
    if (consumed != 0) {
	ptrs &= ~M_TxCompletionConsumerIndex;
	ptrs |= V_TxCompletionConsumerIndex(tx_comp_ci);
	WRITECSR(sc, R_CompletionQueueConsumerIndex, ptrs);
	}

    return consumed;
}


static void
aic_init(aic_ether_t *sc)
{
    int i;

    /* Allocate buffer pool */
    sc->pktpool = KMALLOC(ETH_PKTPOOL_SIZE*ETH_PKTBUF_SIZE, CACHE_ALIGN);
    eth_initfreelist(sc);
    q_init(&sc->rxqueue);

    /* The tx_dscr ring uses an explicit END bit */
    for (i = 0; i < TX_RING_ENTRIES; i++) {
	sc->tx_dscr[i].flags = (V_FLAG_ID_TX
				| M_FLAG_INTR
				| M_FLAG_CRCEN
				| (1 << S_FLAG_NBUFFERS));
	}
    sc->tx_dscr[TX_RING_ENTRIES-1].flags |= M_FLAG_END;
}


/* MII access functions.  */

#define MII_MAX_RETRIES 10000

static uint16_t
mii_read_register(aic_ether_t *sc, int phy, int index)
{
    uint32_t csr;
    uint32_t val;
    int   i;

    for (i = 0; i < MII_MAX_RETRIES; i++) {
	val = READCSR(sc, R_MIIStatus);
	if ((val & M_MIIStatus_MiiBusy) == 0)
	    break;
	}	
    if (i == MII_MAX_RETRIES)
	xprintf("%s: mii_read_register: MII always busy\n", aic_devname(sc));

    csr = R_MIIRegistersAccessPort + ((PHY_REGISTERS*phy + index) << 2);
    for (i = 0; i < MII_MAX_RETRIES; i++) {
	val = READCSR(sc, csr);
	if ((val & M_MiiDataValid) != 0)
	    break;
	}	
    if (i == MII_MAX_RETRIES)
	xprintf("%s: mii_read_register: data never valid\n", aic_devname(sc));

    return (val & M_MiiRegDataPort) >> S_MiiRegDataPort;
}

static void
mii_write_register(aic_ether_t *sc, int phy, int index, uint16_t value)
{
    uint32_t csr;
    uint32_t val;
    int   i;

    for (i = 0; i < MII_MAX_RETRIES; i++) {
	val = READCSR(sc, R_MIIStatus);
	if ((val & M_MIIStatus_MiiBusy) == 0)
	    break;
	}
    if (i == MII_MAX_RETRIES)
	xprintf("%s: mii_write_register: always busy\n", aic_devname(sc));

    csr = R_MIIRegistersAccessPort + ((PHY_REGISTERS*phy + index) << 2);
    WRITECSR(sc, csr, value);
}

static int
mii_probe(aic_ether_t *sc)
{
    int i;
    uint16_t id1, id2;

    for (i = 0; i < 32; i++) {
        id1 = mii_read_register(sc, i, MII_PHYIDR1);
	id2 = mii_read_register(sc, i, MII_PHYIDR2);
	if ((id1 != 0x0000 && id1 != 0xFFFF) ||
	    (id2 != 0x0000 && id2 != 0xFFFF)) {
	    if (id1 != id2) return i;
	    }
	}
    return -1;
}

#if 0
#define OUI_SEEQ    0x00A07D
#define IDR_SEEQ    0x0005BE     /* OUI bit-reversed within each byte */
#define PART_80220  0x03

static void
mii_dump(aic_ether_t *sc, const char *label)
{
    int i;
    uint16_t  r;
    uint32_t  idr, part;

    xprintf("%s\n", label);
    idr = part = 0;

    /* Required registers */
    for (i = 0; i <= 5; ++i) {
	r = mii_read_register(sc, sc->phy_addr, i);
	xprintf("MII_REG%02X: %04X\n", i, r);
	if (i == MII_PHYIDR1) {
	    idr |= r << 6;
	    }
	else if (i == MII_PHYIDR2) {
	    idr |= (r >> 10) & 0x3F;
	    part = (r >> 4) & 0x3F;
	    }
	}

    /* Extensions */
    if (idr == IDR_SEEQ && part == PART_80220) {
	for (i = 0x10; i <= 0x14; ++i) {
	    r = mii_read_register(sc, sc->phy_addr, i);
	    xprintf("MII_REG%02X: %04X\n", i, r);
	    }
	}
}
#else
#define mii_dump(sc,label)
#endif

static int
mii_poll(aic_ether_t *sc)
{
    uint16_t  status, ability;

    /* BMSR has read-to-clear bits; read twice.  */
    
    status = mii_read_register(sc, sc->phy_addr, MII_BMSR);
    status = mii_read_register(sc, sc->phy_addr, MII_BMSR);
    ability = mii_read_register(sc, sc->phy_addr, MII_ANLPAR);

    if (status != sc->phy_status || ability != sc->phy_ability) {
        sc->phy_status = status;
	sc->phy_ability = ability;
	return 1;
	}
    return 0;
}

static void
mii_set_speed(aic_ether_t *sc, int speed)
{
    uint16_t  control;

    control = mii_read_register(sc, sc->phy_addr, MII_BMCR);

    control &=~ (BMCR_ANENABLE | BMCR_RESTARTAN);
    mii_write_register(sc, sc->phy_addr, MII_BMCR, control);
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

    mii_write_register(sc, sc->phy_addr, MII_BMCR, control);
}

static void
mii_autonegotiate(aic_ether_t *sc)
{
    uint16_t  control, status, remote;
    unsigned int  timeout;
    int linkspeed;

    linkspeed = ETHER_SPEED_UNKNOWN;

    /* Read twice to clear latching bits */
    status = mii_read_register(sc, sc->phy_addr, MII_BMSR);
    status = mii_read_register(sc, sc->phy_addr, MII_BMSR);
    mii_dump(sc, "query PHY");

    if ((status & (BMSR_AUTONEG | BMSR_LINKSTAT)) ==
        (BMSR_AUTONEG | BMSR_LINKSTAT))
	control = mii_read_register(sc, sc->phy_addr, MII_BMCR);
    else {
	timeout = 3*CFE_HZ;
	for (;;) {
	    status = mii_read_register(sc, sc->phy_addr, MII_BMSR);
	    if ((status & BMSR_ANCOMPLETE) != 0 || timeout <= 0)
		break;
	    cfe_sleep(CFE_HZ/2);
	    timeout -= CFE_HZ/2;
	    }
	}

    remote = mii_read_register(sc, sc->phy_addr, MII_ANLPAR);
    
    xprintf("%s: Link speed: ", aic_devname(sc));
    if ((status & BMSR_ANCOMPLETE) != 0) {
	/* A link partner was negogiated... */
        uint32_t config1;
	uint32_t ipgt;

	config1 = READCSR(sc, R_MacConfig1);
	ipgt = READCSR(sc, R_BkToBkIPG);
	ipgt &=~ M_IPGT;

	if ((remote & ANLPAR_TXFD) != 0) {
	    xprintf("100BaseT FDX\n");
	    config1 |= M_FullDuplex;
	    ipgt |= (K_IPGT_FDX << S_IPGT);
	    linkspeed = ETHER_SPEED_100FDX;	 
	    }
	else if ((remote & ANLPAR_TXHD) != 0) {
	    xprintf("100BaseT HDX\n");
	    config1 &= ~M_FullDuplex;
	    ipgt |= (K_IPGT_HDX << S_IPGT);
	    linkspeed = ETHER_SPEED_100HDX;	 
	    }
	else if ((remote & ANLPAR_10FD) != 0) {
	    xprintf("10BaseT FDX\n");
	    config1 |= M_FullDuplex;
	    ipgt |= (K_IPGT_FDX << S_IPGT);
	    linkspeed = ETHER_SPEED_10FDX;	 
	    }
	else if ((remote & ANLPAR_10HD) != 0) {
	    xprintf("10BaseT HDX\n");
	    config1 &= ~M_FullDuplex;
	    ipgt |= (K_IPGT_HDX << S_IPGT);
	    linkspeed = ETHER_SPEED_10HDX;	 
	    }

	/* Can this be done with the DMA enabled? */
	WRITECSR(sc, R_BkToBkIPG, ipgt);
	WRITECSR(sc, R_MacConfig1, config1);
	WRITECSR(sc, R_MacConfig1, config1 | M_SoftRst);
	cfe_sleep(1);
	WRITECSR(sc, R_MacConfig1, config1);
	}
    else {
	/* no link partner negotiation */
	xprintf("Unknown\n");
	linkspeed = ETHER_SPEED_UNKNOWN;
	remote = 0;
	}

    /* clear latching bits */
    status = mii_read_register(sc, sc->phy_addr, MII_BMSR);
    sc->phy_status = status;
    sc->phy_ability = remote;

    mii_dump(sc, "final PHY");

}


/* Bus utilization statistics */

static void
aic_initstats(aic_ether_t *sc)
{
    sc->pci_latency = sc->int_latency = 0;
    sc->pci_slave = sc->pci_master = 0;
    sc->pci_data = 0;

    WRITECSR(sc, R_PCIMonitor1, 0);
    WRITECSR(sc, R_PCIMonitor2, 0);

    sc->pci_polltime = (uint64_t)cfe_ticks + PCI_POLL_INTERVAL;
}

static void
aic_pcistats(aic_ether_t *sc)
{
    uint32_t monitor1, monitor2;
    uint32_t t;

    /* The following actions are not atomic but should be close enough
       for collecting stats.  */
    monitor1 = READCSR(sc, R_PCIMonitor1);
    monitor2 = READCSR(sc, R_PCIMonitor2);
    WRITECSR(sc, R_PCIMonitor1, 0);
    WRITECSR(sc, R_PCIMonitor2, 0);

    t = (monitor1 & M_PCIBusMaxLatency) >> S_PCIBusMaxLatency;
    if (t > sc->pci_latency)
	sc->pci_latency = t;
    t = (monitor1 & M_PCIIntMaxLatency) >> S_PCIIntMaxLatency;
    if (t > sc->int_latency)
	sc->int_latency = t;

    t = (monitor1 & M_PCISlaveBusUtilization) >> S_PCISlaveBusUtilization;
    sc->pci_slave += (uint64_t) t;
    t = (monitor2 & M_PCIMasterBusUtilization) >> S_PCIMasterBusUtilization;
    sc->pci_master += (uint64_t) t;
    t = (monitor2 & M_ActiveTransferCount) >> S_ActiveTransferCount;
    sc->pci_data += (uint64_t) t;
}


/* The following functions collectively implement the recommended
   AIC-6915 Initialization Procedure (Section 8) */

static int
aic_reset(aic_ether_t *sc)
{
    uint32_t pci_config;
    uint32_t status;
    uint32_t mac_config;
    uint32_t bac_ctrl;

    /* Reset PHY (3) - skipped, see aic_initlink */

    /* Disable DMA, MAC (4) */
    WRITECSR(sc, R_GeneralEthernetCtrl, 0);

    /* Software reset (5) */
    pci_config = READCSR(sc, R_PCIDeviceConfig);
    WRITECSR(sc, R_PCIDeviceConfig, 0);
    aic_spin(sc, 2);      /* at least 2 usec */
    WRITECSR(sc, R_PCIDeviceConfig, M_SoftReset);
    aic_spin(sc, 2);
    WRITECSR(sc, R_PCIDeviceConfig, pci_config);
    aic_spin(sc, 2);

    /* Clear PCI status (6) */
    status = pci_conf_read(sc->tag, PCI_COMMAND_STATUS_REG);
    pci_conf_write(sc->tag, PCI_COMMAND_STATUS_REG, status);
    
    /* Configure MAC (7) */
    mac_config = READCSR(sc, R_MacConfig1);
    mac_config |= M_PadEn;
    mac_config |= M_FullDuplex;   /* Default, updated by autonegotiation. */
    WRITECSR(sc, R_MacConfig1, mac_config);
    mac_config |= M_SoftRst;
    WRITECSR(sc, R_MacConfig1, mac_config);
    mac_config = READCSR(sc, R_MacConfig1);
    mac_config &= ~M_SoftRst;
    WRITECSR(sc, R_MacConfig1, mac_config);

    bac_ctrl = READCSR(sc, R_BacControl);
    bac_ctrl &= ~M_DataSwapMode;
    bac_ctrl |= V_DataSwapMode_BE;
    WRITECSR(sc, R_BacControl, bac_ctrl);

    return 0;
}

static int
aic_coldreset(aic_ether_t *sc)
{
    pcireg_t cmd;

    /* only cold reset needs steps 1 and 2 */

    /* Enable memory, also clear R/WC status bits (1) */
    cmd = pci_conf_read(sc->tag, PCI_COMMAND_STATUS_REG);
    cmd |= PCI_COMMAND_MEM_ENABLE | PCI_COMMAND_MASTER_ENABLE;
    pci_conf_write(sc->tag, PCI_COMMAND_STATUS_REG, cmd);

    aic_reset(sc);       /* continue with warm reset */
    return 0;
}

static int
aic_rxinit(aic_ether_t *sc)
{
    uint32_t ctrl;
    unsigned offset;

    /* Initialize the Rx Completion rings.  Q1 only and 32-bit
       addressing for now.  */
    WRITECSR(sc, R_RxCompletionQueue1Ctrl,
	     (PTR_TO_PCI(sc->rx_done) & M_RxCompletionBaseAddress)
	     | (3 << S_RxCompletionType)
	     | (7 << S_RxCompletionThreshold));    /* XXX check */

    /* Initialize ring indices */
    WRITECSR(sc, R_CompletionQueueConsumerIndex, 0);
    WRITECSR(sc, R_CompletionQueueProducerIndex, 0);

    /* Increase the maximum burst size. */
    ctrl = READCSR(sc, R_RxDmaCtrl);
    ctrl &= ~M_RxBurstSize;
    ctrl |= (16 << S_RxBurstSize);

    /* Initialize the Rx Descriptor rings.  Q1 only and 32-bit
       addressing for now. */
    WRITECSR(sc, R_RxDescQueue1Ctrl,
	     (ETH_PKTBUF_LEN << S_RxBufferLength)
	     | (4 << S_RxMinDescriptorsThreshold));   /* XXX check */
    WRITECSR(sc, R_RxDescQueueHighAddress, 0);
    WRITECSR(sc, R_RxDescQueue1LowAddress, PTR_TO_PCI(sc->rx_dscr));
    /* Initialize pointers */
    WRITECSR(sc, R_RxDescQueue1Ptrs, 0);

    /* Set up address filters (see Table 7-108 for format) */    
    for (offset = 0; offset < PERFECT_ADDRESS_ENTRIES*4; offset++)
        WRITECSR(sc, R_PerfectAddressBase + 4*offset, 0);
    for (offset = 0; offset < 3; offset++) {
	uint32_t bytes;
        
	bytes = (sc->hwaddr[4-2*offset] << 8) | (sc->hwaddr[5-2*offset]);
	WRITECSR(sc, R_PerfectAddressBase + 4*offset, bytes);
	}
    WRITECSR(sc, R_RxAddressFilteringCtrl,
	     (K_PerfectFiltering_16 << S_PerfectFilteringMode)
	     | (K_HashFiltering_Off << S_HashFilteringMode)
	     | M_PassBroadcast);

    return 0;
}

static int
aic_txinit(aic_ether_t *sc)
{
    uint32_t ctrl;

    /* Initialize the Tx Descriptor rings.  32-bit addressing for now.  */
    WRITECSR(sc, R_TxDescQueueCtrl,
	     (1 << S_TxDescType)
	     | (0 << S_SkipLength)
	     | (8 << S_TxDmaBurstSize)
	     | (2 << S_TxHighPriorityFifoThreshold));  /* XXX check */
    /* Use the low priority ring */
    WRITECSR(sc, R_TxDescQueueHighAddr, 0);
    WRITECSR(sc, R_LoPrTxDescQueueBaseAddr, PTR_TO_PCI(sc->tx_dscr));

    /* Empty rings */
    WRITECSR(sc, R_TxDescQueueProducerIndex, 0);
    WRITECSR(sc, R_TxDescQueueConsumerIndex, 0);

    /* Configure for interrupt on DMA Complete */
    ctrl = READCSR(sc, R_TxFrameCtrl);
    ctrl &= ~M_DmaCompletionAfterTransmitComplete;
    WRITECSR(sc, R_TxFrameCtrl, ctrl);

    /* Initialize the Tx Completion rings. */
    WRITECSR(sc, R_CompletionQueueHighAddr, 0);
    WRITECSR(sc, R_TxCompletionQueueCtrl,
	     (PTR_TO_PCI(sc->tx_done) & M_TxCompletionBaseAddress)
	     | (7 << S_TxCompletionQueueThreshold));   /* XXX check */

    /* Initialize ring indices (again) */
    WRITECSR(sc, R_CompletionQueueConsumerIndex, 0);
    WRITECSR(sc, R_CompletionQueueProducerIndex, 0);

    return 0;
}


static int
aic_statsinit(aic_ether_t *sc)
{
    int i;

    for (i = 0; i < STATISTICS_COUNT; i++)
	WRITECSR(sc, R_StatisticsBase + 4*i, 0);

    return 0;
}


static void
aic_initlink(aic_ether_t *sc)
{
    sc->phy_addr = mii_probe(sc);
    if (sc->phy_addr < 0) {
	xprintf("%s: no PHY found\n", aic_devname(sc));
	return;  
	}
    if (1)   /* XXX Support only autonegotiation for now */
	mii_autonegotiate(sc);
    else
	mii_set_speed(sc, ETHER_SPEED_10HDX);
    sc->mii_polltime = (uint64_t)cfe_ticks + MII_POLL_INTERVAL;

    sc->mii_polling = 0;
}


static void
aic_enable(aic_ether_t *sc)
{
    uint32_t pci_config;

    /* Prime the Rx ring.  This must be done before enabling interrupts. */
    aic_fillrxring(sc);

    /* Configure and enable interrupts */
    sc->intmask = M_RxQ1DoneInt | M_TxFrameCompleteInt | M_TxDMADoneInt;
    (void) READCSR(sc, R_InterruptStatus);   /* Clear read-to-clear bits */
    WRITECSR(sc, R_InterruptEn, sc->intmask);
    pci_config = READCSR(sc, R_PCIDeviceConfig);
    pci_config |= M_IntEnable;
    WRITECSR(sc, R_PCIDeviceConfig, pci_config);

    /* Enable DMA and MAC */
    WRITECSR(sc, R_GeneralEthernetCtrl,
	     M_RxDmaEn | M_ReceiveEn | M_TxDmaEn | M_TransmitEn);
}

static void
aic_disable(aic_ether_t *sc)
{
    uint32_t ctrl;

    ctrl = READCSR(sc, R_GeneralEthernetCtrl);
    ctrl &= ~(M_RxDmaEn | M_ReceiveEn | M_TxDmaEn | M_TransmitEn);
    WRITECSR(sc, R_GeneralEthernetCtrl, ctrl);

    sc->intmask = 0;
    WRITECSR(sc, R_InterruptEn, sc->intmask);
}


static void
aic_hwinit(aic_ether_t *sc)
{
    if (sc->state == eth_state_uninit) {
	aic_coldreset(sc);
	aic_rxinit(sc);
	aic_txinit(sc);
	aic_statsinit(sc);

	aic_initlink(sc);

	sc->state = eth_state_off;
#if AIC_DEBUG
	dumpcsrs(sc, "end init");
#else
	(void)dumpcsrs;
#endif
	}
}


static void
aic_isr(void *arg)
{
    aic_ether_t *sc = (aic_ether_t *)arg;
    int received, sent;
    uint32_t status;

    if (IPOLL) sc->interrupts++;

    received = sent = 0;

    for (;;) {
	status = READCSR(sc, R_InterruptStatus);

	if ((status & sc->intmask) == 0)
	    break;

	if (status & M_RxQ1DoneInt) {
	    if (IPOLL) sc->rx_interrupts++;  
	    received += aic_procrxring(sc);
	    }

	/* XXX choose only one tx completion interrupt. */
	if (status & (M_TxFrameCompleteInt | M_TxDMADoneInt)) {
	    if (IPOLL) sc->tx_interrupts++;  
	    sent += aic_proctxring(sc);
	    }
	}
}


static void
aic_start(aic_ether_t *sc)
{
    aic_hwinit(sc);

    sc->intmask = 0;
#if IPOLL
    cfe_request_irq(sc->irq, aic_isr, sc, CFE_IRQ_FLAGS_SHARED, 0);
#endif

    aic_enable(sc);
    sc->state = eth_state_on;
}

static void
aic_stop(aic_ether_t *sc)
{
    aic_disable(sc);

#if IPOLL
    cfe_free_irq(sc->irq, 0);
#endif
}


static int aic_ether_open(cfe_devctx_t *ctx);
static int aic_ether_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int aic_ether_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int aic_ether_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int aic_ether_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int aic_ether_close(cfe_devctx_t *ctx);
static void aic_ether_poll(cfe_devctx_t *ctx, int64_t ticks);
static void aic_ether_reset(void *softc);

const static cfe_devdisp_t aic_ether_dispatch = {
    aic_ether_open,
    aic_ether_read,
    aic_ether_inpstat,
    aic_ether_write,
    aic_ether_ioctl,
    aic_ether_close,	
    aic_ether_poll,
    aic_ether_reset
};

cfe_driver_t aic6915drv = {
    "AIC-6915 Ethernet",
    "eth",
    CFE_DEV_NETWORK,
    &aic_ether_dispatch,
    aic_ether_probe
};


static int
aic_ether_attach(cfe_driver_t *drv, pcitag_t tag, int index)
{
    aic_ether_t *sc;
    char descr[80];
    phys_addr_t pa;
    uint32_t base;
    uint32_t addr;
    pcireg_t device, class;
    int i;

    pci_map_mem(tag, PCI_MAPREG(0), PCI_MATCH_BITS, &pa);
    base = (uint32_t) pa;

    sc = (aic_ether_t *) KMALLOC(sizeof(aic_ether_t), 0);
    if (sc == NULL) {
	xprintf("AIC-6915: No memory to complete probe\n");
	return 0;
	}
    memset(sc, 0, sizeof(aic_ether_t));

    /* All descriptor rings must be aligned to a 256-byte boundary.  */
    sc->rings = (aic_rings_t *) KMALLOC(sizeof(aic_rings_t), 256);
    if (sc->rings == NULL) {
	xprintf("AIC-6915: No memory for descriptor rings\n");
	KFREE(sc);
	return 0;
	}
    memset(sc->rings, 0, sizeof(aic_rings_t));
    sc->rx_dscr = sc->rings->rx_dscr;
    sc->rx_done = sc->rings->rx_done;
    sc->tx_dscr = sc->rings->tx_dscr;
    sc->tx_done = sc->rings->tx_done;

    sc->regbase = base + K_AIC_REG_OFFSET;

    sc->irq = pci_conf_read(tag, PCI_BPARAM_INTERRUPT_REG) & 0xFF;

    device = pci_conf_read(tag, PCI_ID_REG);
    class = pci_conf_read(tag, PCI_CLASS_REG);

    sc->tag = tag;
    sc->device = PCI_PRODUCT(device);
    sc->revision = PCI_REVISION(class);
    sc->devctx = NULL;

    /* Assume on-chip firmware has initialized the MAC address.
       Empirically, attempts to read the EEPROM directly give bus
       errors. */
    addr = READCSR(sc, R_MacAddr2);
    for (i = 0; i < 2; i++)
	sc->hwaddr[i] = (addr >> (8*(1-i))) & 0xff;
    addr = READCSR(sc, R_MacAddr1);
    for (i = 0; i < 4; i++)
	sc->hwaddr[2+i] = (addr >> (8*(3-i))) & 0xff;

    aic_init(sc);

    sc->state = eth_state_uninit;

    xsprintf(descr, "%s at 0x%X (%a)",
	     drv->drv_description, base, sc->hwaddr);

    cfe_attach(drv, sc, NULL, descr);
    return 1;
}

static void
aic_ether_probe(cfe_driver_t *drv,
		unsigned long probe_a, unsigned long probe_b, 
		void *probe_ptr)
{
    int n;

    n = 0;
    for (;;) {
	pcitag_t tag;

	if (pci_find_device(K_PCI_VENDOR_ADAPTEC, K_PCI_ID_AIC6915, n, &tag)
	    != 0)
	   break;
	aic_ether_attach(drv, tag, n);
	n++;
	}
}


/* The functions below are called via the dispatch vector for the AIC-6915 */

static int
aic_ether_open(cfe_devctx_t *ctx)
{
    aic_ether_t *sc = ctx->dev_softc;

    if (sc->state == eth_state_on)
	aic_stop(sc);

    sc->devctx = ctx;

    sc->inpkts = sc->outpkts = 0;
    sc->interrupts = 0;
    sc->rx_interrupts = sc->tx_interrupts = 0;

    aic_start(sc);
    aic_initstats(sc);

    if (XPOLL) aic_isr(sc);
    return 0;
}

static int
aic_ether_read(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    aic_ether_t *sc = ctx->dev_softc;
    eth_pkt_t *pkt;
    int blen;

    if (XPOLL) aic_isr(sc);

    if (sc->state != eth_state_on) return -1;

    CS_ENTER(sc);
    pkt = (eth_pkt_t *) q_deqnext(&sc->rxqueue);
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

    if (XPOLL) aic_isr(sc);
    return 0;
}

static int
aic_ether_inpstat(cfe_devctx_t *ctx, iocb_inpstat_t *inpstat)
{
    aic_ether_t *sc = ctx->dev_softc;

    if (XPOLL) aic_isr(sc);

    if (sc->state != eth_state_on) return -1;

    /* We avoid an interlock here because the result is a hint and an
       interrupt cannot turn a non-empty queue into an empty one. */
    inpstat->inp_status = (q_isempty(&(sc->rxqueue))) ? 0 : 1;

    return 0;
}

static int
aic_ether_write(cfe_devctx_t *ctx, iocb_buffer_t *buffer)
{
    aic_ether_t *sc = ctx->dev_softc;
    eth_pkt_t *pkt;
    int blen;

    if (XPOLL) aic_isr(sc);

    if (sc->state != eth_state_on) return -1;

    pkt = eth_alloc_pkt(sc);
    if (!pkt) return CFE_ERR_NOMEM;

    blen = buffer->buf_length;
    if (blen > pkt->length) blen = pkt->length;

    hs_memcpy_from_hs(pkt->buffer, buffer->buf_ptr, blen);
    pkt->length = blen;

    if (aic_transmit(sc, pkt) != 0) {
	eth_free_pkt(sc,pkt);
	return CFE_ERR_IOERR;
	}

    if (XPOLL) aic_isr(sc);
    return 0;
}

static int
aic_ether_ioctl(cfe_devctx_t *ctx, iocb_buffer_t *buffer) 
{
    aic_ether_t *sc = ctx->dev_softc;

    switch ((int)buffer->buf_ioctlcmd) {
	case IOCTL_ETHER_GETHWADDR:
	    hs_memcpy_to_hs(buffer->buf_ptr, sc->hwaddr, sizeof(sc->hwaddr));
	    return 0;

	default:
	    return -1;
	}
}

static int
aic_ether_close(cfe_devctx_t *ctx)
{
    aic_ether_t *sc = ctx->dev_softc;

    sc->state = eth_state_off;
    aic_stop(sc);

    xprintf("%s: max latency: pci %d, int %d\n",
	    aic_devname(sc), sc->pci_latency, sc->int_latency);
    xprintf("  %lld active in %lld master, %lld slave\n",
	    sc->pci_data, sc->pci_master, (sc->pci_slave + 32)/64);

    xprintf("%s: %d sent, %d received, %d interrupts\n",
	    aic_devname(sc), sc->outpkts, sc->inpkts, sc->interrupts);
    xprintf("  %d rx interrupts, %d tx interrupts\n",
	    sc->rx_interrupts, sc->tx_interrupts);

    sc->devctx = NULL;
    return 0;
}

static void
aic_ether_poll(cfe_devctx_t *ctx, int64_t ticks)
{
    aic_ether_t *sc = ctx->dev_softc;
    int changed;

    if (sc->state != eth_state_uninit) {
	uint64_t now = cfe_ticks;

	if (now >= sc->pci_polltime) {
	    aic_pcistats(sc);
	    sc->pci_polltime = now + PCI_POLL_INTERVAL;
	    }

	if (now >= sc->mii_polltime) {
	    changed = mii_poll(sc);
	    if (changed) {
		mii_autonegotiate(sc);
		}
	    sc->mii_polltime = (uint64_t)cfe_ticks + MII_POLL_INTERVAL;
	    }
	}
}

static void
aic_ether_reset(void *softc)
{
    aic_ether_t *sc = (aic_ether_t *)softc;

    /* Turn off the Ethernet interface. */

    if (sc->state == eth_state_on)
	aic_stop(sc);

    sc->state = eth_state_uninit;
}
