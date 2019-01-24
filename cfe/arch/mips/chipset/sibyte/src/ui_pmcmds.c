/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Packet Manager commands			File: ui_pmcmds.c
    *  
    *  Commands and test stuff to check out the packet manager
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

#define _PM_DEBUG_

#include "cfe.h"
#include "sbmips.h"
#include "ui_command.h"

#include "lib_hssubr.h"
#include "lib_try.h"
#include "lib_memfuncs.h"
#include "lib_physio.h"
#include "env_subr.h"
#include "sb1250_defs.h"

#include "pcivar.h"
#include "pci_internal.h"
#include "pcireg.h"
#include "ldtreg.h"

#include "bcm1480_regs.h"
#include "bcm1480_pm.h"
#include "bcm1480_hr.h"
#include "bcm1480_ht.h"
#include "bcm1480_hsp.h"
#include "bcm1480_scd.h"

#include "bcm1480_hsp_utils.h"

#include "ui_bitfields.h"

typedef volatile uint32_t sbeth_port_t;
typedef uint32_t sbeth_physaddr_t;
#define SBETH_PORT(x) PHYS_TO_K1(x)
#if CFG_L2_RAM
#define SBETH_VTOP(x) ((sbeth_physaddr_t)0xD0000000 + (sbeth_physaddr_t)(x))
#else
#define SBETH_VTOP(x) (K1_TO_PHYS((sbeth_physaddr_t)(x)))
#endif

#define WRITECSR(x,y) phys_write64(x,y)
#define READCSR(x) phys_read64(x)


/*  *********************************************************************
    *  Constants 
    ********************************************************************* */

#define REFCLK 		100	/* in MHz */
#define SPI4_MHZ	400	/* in MHz */

#define DEFAULT_COUNT	1	/* historically was 5, simpler at 1 */

#define DEFAULT_ACTIVE_CHANNELS 16	// Enable 16 channels so can test all
				// (-chan=* flag to pm init overrides this)

#define DEFAULT_PATH 15		// the code that is returned by next_dest debug
#define NO_COOKIE -1		// arbitrary key value


static unsigned long rand(void)
{
    static unsigned long seed = 1;
    long x, hi, lo, t;

    x = seed;
    hi = x / 127773;
    lo = x % 127773;
    t = 16807 * lo - 2836 * hi;
    if (t <= 0) t += 0x7fffffff;
    seed = t;
    return t;
}

//------------------------------------------------------------------------------
// Some more routines to fill data different ways and verify it

const int UniquePrime = 2399;		// is it?

// returns '1' if data is okay
static int verifyData(uint16_t* ptr, int byteCount) {
	int i;
	int error = 0;
	uint16_t exp = ptr[0];
	for (i=1; i<byteCount/2; i++) {
		exp = (exp + UniquePrime) & 0xffff;
		if (ptr[i] != exp) {
			printf("bad data [%d]=0x%x expected 0x%x\n",
				i, ptr[i], exp);
			error = 1;
		}
	}
	return !error;
}

// must provide a seed value
static void fillData(uint16_t* ptr, int byteCount, uint16_t val) {
	int i;
	ptr[0] = val;
	for (i=1; i<byteCount/2; i++) {
		val = (val + UniquePrime) & 0xffff;
		ptr[i] = val;
	}
}

static void fillCustomPat(int dpat, uint8_t* pkt, uint8_t* data, int pktlen) {
	int i;
	int c = 0;
	uint16_t* d = (uint16_t*) data;
	uint16_t* p = (uint16_t*) pkt;
	for (i=0; i<pktlen/2; i++) {
		int word;
		word = ((dpat >> (15-c)) & 1) ? 0xffff : 0x0000;
		d[i] = word;
		p[i] = word;
		c++;
		c &= 0xf;
	}
}

static void fillDataPatRepeat(uint32_t pat, uint8_t* pkt, uint8_t* data,
	int pktlen) {
	// repeat this 32-bits over and over in the packet
	int i;
	uint32_t* d = (uint32_t*) data;
	uint32_t* p = (uint32_t*) pkt;
	for (i=0; i<pktlen/4; i++) {
		d[i] = pat;
		p[i] = pat;
	}
}

static void fillDataPatByWord(int patid, uint8_t* pkt, uint8_t* data, int pktlen) {
    switch (patid) {
    case 1: {
	// 64 bytes of ff, then 64 bytes of zero
	int j;
	int c = 0;
	for (j=0; j<pktlen; j++) {
		int byte;
		byte = ((c >> 5) & 1) ? 0x00 : 0xff;
		data[j] = byte;
		pkt[j] = byte;
		c++;
	}
	break;
    }
    case 2: {
	int j;
	int c = 0;
	for (j=0; j<pktlen; j++) {
		int byte;
		byte = ((c >> 5) & 1) ? 0x00 : 0x80;
		data[j] = byte;
		pkt[j] = byte;
		c++;
	}
	break;
    }
    case 3: {
	int j;
	int c = 0;
	for (j=0; j<pktlen; j++) {
		int byte;
		byte = ((c >> 5) & 1) ? 0x55 : 0xaa;
		data[j] = byte;
		pkt[j] = byte;
		c++;
	}
	break;
    }
    case 4: {
	int j;
	int c = 0;
	for (j=0; j<pktlen; j++) {
		int byte;
		byte = (c & 0x1f) ? 0xff : 0x00;	// one zero every 64
		data[j] = byte;
		pkt[j] = byte;
		c++;
	}
	break;
    }
    case 5: {
	int j;
	int c = 0;
	for (j=0; j<pktlen; j++) {
		int byte;
		byte = (c & 0x1f) ? 0xff : 0x7f;	// one zero every 64
		data[j] = byte;
		pkt[j] = byte;
		c++;
	}
	break;
    }
    default:
    	printf("ERROR: program error!\n");
    }
}

static void fillRandom(uint8_t* pkt, uint8_t* data, int pktlen) {
	int i;
	for (i=0; i<pktlen; i++) {
		uint8_t byte = rand() & 0xff00 >> 8;
		data[i] = byte;
		pkt[i] = byte;
	}
}

/*  *********************************************************************
    *  Prototypes
    ********************************************************************* */

int ui_init_pmcmds(void);
extern int dumpmem(hsaddr_t addr,hsaddr_t dispaddr,int length,int wlen);

/*  *********************************************************************
    *  Data
    ********************************************************************* */


static bitfield_t dscrerrs[] = {
    {M_BCM1480_PM_DSCR0_PE,"PE"},
    {M_BCM1480_PM_DSCR0_LE,"LE"},
    {M_BCM1480_PM_DSCR0_SE,"SE"},
    {0,NULL}};

static int hsp_ports = 0;		/* all ports in use */

/*  *********************************************************************
    *  Descriptor structure
    ********************************************************************* */

typedef struct pmdscr_s {
    uint64_t  dscr_a;
    uint64_t  dscr_b;
} pmdscr_t;

#define PM_NRINGS 32		// really this is fixed--don't change it

// #define PM_RINGSIZE 16
#define PM_RINGSIZE 8		// eight is enough (well within 1% of 16)

/*
 * Allow larger PM packets for only a few board variants which have been 
 * give larger heaps
 */
#if (CFG_HEAP_SIZE >= 4096)
#define PKTBUFSIZE 4096
#else
#define PKTBUFSIZE 64
#endif

#define RANDOM_MAX_PKTSIZE 1500
#define DEFAULT_PKTSIZE 64

#define NUMPKTS 512		// Now able to alloc this # of packets


typedef struct pmring_s {
    pmdscr_t *dscr;
    void **ctx;
    int ringsize;
    int qno;
    int next_add;
    int next_rem;
    int onring;
} pmring_t;

typedef struct packet_s {
    struct packet_s *next;
    unsigned int len;
    int res0,res1;
    int next_dest;
    uint8_t  pkt[PKTBUFSIZE];
    uint8_t  pad[32];
    
} packet_t;

static inline void dump_packet(packet_t* pkt) {
	dumpmem((hsaddr_t)(long)pkt->pkt,(hsaddr_t)(long)pkt->pkt,pkt->len,4);
}

static void save_data(packet_t* pkt, uint8_t* data) {
	int i;
	for (i=0; i<pkt->len; i++) {
		data[i] = pkt->pkt[i];
	}
}

static void write_cookie(packet_t* pkt, int offset, uint32_t v) {
	if (offset + 4 > pkt->len) {
		printf("ERROR: bad cookie write\n");
	}
	// may be at a byte offset so do each byte (big-endian)
	pkt->pkt[offset] = (v >> 24);
	pkt->pkt[offset + 1] = (v >> 16);
	pkt->pkt[offset + 2] = (v >> 8);
	pkt->pkt[offset + 3] = v;
}

static packet_t *pm_pkt_pool = NULL;
static packet_t *pm_pkts[NUMPKTS];

static pmring_t *pmi_rings[PM_NRINGS];
static pmring_t *pmo_rings[PM_NRINGS];

static int map16 = 0;			// either map8 or map16 mode

static uint8_t *scratch_data[ PM_RINGSIZE * PM_NRINGS ];




/*  *********************************************************************
    *  Packet Manager stuff
    ********************************************************************* */


static inline void pm_pkt_free(packet_t *pkt)
{
    pkt->next = pm_pkt_pool;
    pm_pkt_pool = pkt;
}

static inline packet_t *pm_pkt_alloc(void)
{
    packet_t *pkt = pm_pkt_pool;

    if (pkt) {
	pm_pkt_pool = pkt->next;
	return pkt;
	}
    else {
	return NULL;
	}
}

static void pm_init_pktpool(void)
{
    int idx,j;
    packet_t *pkt;


    /* scratch area used for data verify step */
    memset(scratch_data, 0, sizeof (scratch_data));
    for (idx = 0; idx < PM_RINGSIZE * PM_NRINGS ; idx++) 
    {
        scratch_data[idx] = KMALLOC( PKTBUFSIZE,32);
        if ( scratch_data[idx] == NULL ) 
        {
            printf("Failed alloc of pm scratch buffers %d packets\n", idx-1);
            printf("pm test can not run (4MB heap needed. Change bsp_config.h) \n");
            goto ERROR_EXIT;
        }
    }

    for (idx = 0; idx < NUMPKTS; idx++) 
    {
        // printf("pkt[%d]\n", idx);
        pkt = KMALLOC(sizeof(packet_t),32);		// the real malloc
        if (!pkt) {
            printf("used up all memory with %d packets, okay\n", idx-1);
            printf("pm test will still run, but other commands may die\n");
            break;
        }
        pm_pkts[idx] = pkt;
        for (j = 0; j < sizeof(pkt->pkt); j+=4) {
            pkt->pkt[j+0] =  0xde;		// the '0xdeadbeef' init value
            pkt->pkt[j+1] =  0xad;
            pkt->pkt[j+2] =  0xbe;
            pkt->pkt[j+3] =  0xef;
	    }
        pm_pkt_free(pkt);			// a hack--puts into our pool
	}

ERROR_EXIT:
    return;
}

static void pm_free_pktpool(void)
{
    int idx;

    /* scratch area used for data verify step */
    for (idx = 0; idx < PM_RINGSIZE * PM_NRINGS ; idx++) 
    {
        if (scratch_data[idx])
        {
            KFREE(scratch_data[idx]);
            scratch_data[idx] = NULL;
        }
    }

    for (idx = 0; idx < NUMPKTS; idx++) 
    {
        if (pm_pkts[idx]) KFREE(pm_pkts[idx]);
        pm_pkts[idx] = NULL;
	}
    pm_pkt_pool = NULL;

}

static void pm_free_ring(pmring_t *ring)
{
    if (ring->dscr) KFREE(ring->dscr);
    if (ring->ctx) KFREE(ring->ctx);
    KFREE(ring);
}

static int pm_init_pmo_queue(pmring_t *ring)
{
    /* Set up Ring Base and Size */
    WRITECSR(A_BCM1480_PMO_LCL_REGISTER(ring->qno,R_BCM1480_PM_BASE_SIZE), 
	     (V_BCM1480_PM_BASE(SBETH_VTOP(ring->dscr)) |
	      V_BCM1480_PM_SIZE(ring->ringsize)));

    /* Enable Queue Configuration Register */
    WRITECSR(A_BCM1480_PMO_LCL_REGISTER(ring->qno,R_BCM1480_PM_CONFIG0),
	     (M_BCM1480_PM_QUEUE_ENABLE | M_BCM1480_PM_DS_CACHE_EN));

    /* Set up L2 Cacheability */
    WRITECSR(A_BCM1480_PMO_LCL_REGISTER(ring->qno,R_BCM1480_PM_CACHEABILITY),
	     V_BCM1480_PM_HDR_SIZE(511));

    /* 
     * Setup Interrupt Watermark Register
     * WRITECSR(A_BCM1480_PMO_LCL_REGISTER(ring->qno,R_BCM1480_PM_INT_WMK),
     * (V_BCM1480_PM_LOW_WATERMARK(lwm)|
     * V_BCM1480_PM_HIGH_WATERMARK(hwm)));
     */

    /* 
     * Initialize Interrupt Config Register 
     *
     * int_priority_val = int_priority ? M_BCM1480_PM_INT_PRIORITY : 0;
     * int_cnfg_val = V_BCM1480_PM_INT_CORE_ID(cpuid) | int_priority_val | 
     * V_BCM1480_PM_INT_PKT_CNT(int_pkt_cnt);
     * WRITECSR(A_BCM1480_PMO_LCL_REGISTER(ring->qno,R_BCM1480_PM_INT_CNFG), int_cnfg_val);
     */

    /* Initialize Descriptor Merge Timer */
    WRITECSR(A_BCM1480_PMO_LCL_REGISTER(ring->qno,R_BCM1480_PM_DESC_MERGE_TIMER), 16);

    return 0;
}

static int pm_init_pmi_queue(pmring_t *ring)
{
    /* Set up Ring Base and Size */
    WRITECSR(A_BCM1480_PMI_LCL_REGISTER(ring->qno,R_BCM1480_PM_BASE_SIZE),
	     (V_BCM1480_PM_BASE(SBETH_VTOP(ring->dscr)) |
	      V_BCM1480_PM_SIZE(ring->ringsize)));

    /* Enable Queue Configuration Register */
    WRITECSR(A_BCM1480_PMI_LCL_REGISTER(ring->qno,R_BCM1480_PM_CONFIG0),
	     (M_BCM1480_PM_QUEUE_ENABLE | M_BCM1480_PM_DS_CACHE_EN));

    /* Set up L2 Cacheability */
    WRITECSR(A_BCM1480_PMI_LCL_REGISTER(ring->qno,R_BCM1480_PM_CACHEABILITY),
	     V_BCM1480_PM_HDR_SIZE(511));

    /*
     * Setup Interrupt Watermark Register
     *
     *  WRITECSR(A_BCM1480_PMI_LCL_REGISTER(ring->qno,R_BCM1480_PM_INT_WMK),(V_BCM1480_PM_LOW_WATERMARK(lwm)|
     *  V_BCM1480_PM_HIGH_WATERMARK(hwm)));
     */

    /* 
     * Initialize Interrupt Config Register
     *      int_priority_val = int_priority ? M_BCM1480_PM_INT_PRIORITY : 0;
     *      int_cnfg_val = V_BCM1480_PM_INT_CORE_ID(cpuid) | int_priority_val | 
     *      V_BCM1480_PM_INT_PKT_CNT(int_pkt_cnt);
     *      WRITECSR(A_BCM1480_PMI_LCL_REGISTER(ring->qno,R_BCM1480_PM_INT_CNFG), int_cnfg_val);
     */

    /* 
     * Initialize Descriptor Merge Timer
     *
     * WRITECSR(A_BCM1480_PMI_LCL_REGISTER(ring->qno,R_BCM1480_PM_DESC_MERGE_TIMER), desc_merge_timer);
     */

    /* 
     * Update Global Debug Mode Register
     *
     *      debug_mode_val = debug_mode ? M_BCM1480_PM_DEBUG_MODE : 0;
     *      read_priority_val = read_priority ? M_BCM1480_PM_READ_PRIORITY : 0;
     *      write_priority_val = write_priority ? M_BCM1480_PM_WRITE_PRIORITY : 0;
     *      WRITECSR(A_BCM1480_PM_GLOBALDEBUGMODE_PMI, debug_mode_val | read_priority_val |
     *      write_priority_val );
     */

    return 0;
}

static pmring_t *pm_init_ring(int qno,int ringsize)
{
    int idx;
    pmring_t *ring;

    // printf("allocating pm ring[%d], %d entries\n", qno, ringsize);
    ring = (pmring_t *) KMALLOC(sizeof(pmring_t),0);
    if (!ring) return NULL;

    ring->dscr = (pmdscr_t *) KMALLOC(ringsize*sizeof(pmdscr_t),32);
    if (!ring->dscr) {
    	KFREE(ring);
    	return NULL;
    }
    ring->ctx = (void **) KMALLOC(ringsize*sizeof(void *),8);
    if (!ring->ctx) {
    	KFREE(ring->dscr);
    	KFREE(ring);
    	return NULL;
    }
    ring->qno = qno;
    ring->ringsize = ringsize;
    ring->next_add = 0;
    ring->next_rem = 0;
    ring->onring = 0;

    for (idx = 0; idx < ringsize; idx++) {
	ring->dscr[idx].dscr_a = M_BCM1480_PM_DSCR0_HW;
	ring->dscr[idx].dscr_b = 0;
	ring->ctx[idx] = 0;
	}

    return ring;
}

static int pm_add_rxbuf(pmring_t *ring,packet_t *pkt)
{
    pmdscr_t *dscr = ring->dscr + ring->next_add;
    int newadd;

    pkt->len = sizeof(pkt->pkt);

    newadd = ring->next_add + 1;
    if (newadd >= ring->ringsize) newadd = 0;
    if (newadd == ring->next_rem) {
	printf("Ring %d: RX ring is full\n",ring->qno);
	return 0;
	}

    ring->ctx[ring->next_add] = pkt;
    dscr->dscr_a = M_BCM1480_PM_DSCR0_HW | V_BCM1480_PM_DSCR0_BUFFER_LENGTH(pkt->len);
    dscr->dscr_b = V_BCM1480_PM_DSCR1_BUFFER_ADDR(SBETH_VTOP(&(pkt->pkt[0])));

    ring->next_add = newadd;
    ring->onring++;

    return 1;
}

static inline void pm_start_rx(pmring_t *ring,int cnt)
{
    WRITECSR(A_BCM1480_PMI_LCL_REGISTER(ring->qno,R_BCM1480_PM_CNT), V_BCM1480_PM_COUNT(cnt));
}

static int pm_add_txbuf(pmring_t *ring,packet_t *pkt)
{
    pmdscr_t *dscr = ring->dscr + ring->next_add;
    int newadd;
    /* 
     * Defeat including upper nibble (bit 7-4) which are "or in" by
     * hardware for final IVC value.  Below setting of IVC really
     * doesn;t affect hardware IVC.   It is set by PMO port/ivc cfg
     * mapping value.
     * -Fixes a problem with PT loopback screening test
     */
    int ivc = ring->qno & 0xF;
    int nxt_dest = 0;

    newadd = ring->next_add + 1;
    if (newadd >= ring->ringsize) newadd = 0;
    if (newadd == ring->next_rem) {
	printf("Ring %d: TX Ring is full\n",ring->qno);
	return 0;
	}

    ring->ctx[ring->next_add] = pkt;
    dscr->dscr_a = M_BCM1480_PM_DSCR0_HW | M_BCM1480_PM_DSCR0_SOP | 
	M_BCM1480_PM_DSCR0_EOP | V_BCM1480_PM_DSCR0_BUFFER_LENGTH(pkt->len);
    dscr->dscr_b = V_BCM1480_PM_DSCR1_BUFFER_ADDR(SBETH_VTOP(&(pkt->pkt[0]))) | 
	V_BCM1480_PM_DSCR1_IVC(ivc) | V_BCM1480_PM_DSCR1_NEXT_DEST(nxt_dest);

#ifdef _PM_DEBUG_
	// printf("TX dscr->a=0x%llx b=0x%llx ring=%p\n", dscr->dscr_a, dscr->dscr_b, ring);
#endif

    ring->next_add = newadd;
    ring->onring++;

    return 1;
}

static inline void pm_start_tx(pmring_t *ring,int cnt)
{
    WRITECSR(A_BCM1480_PMO_LCL_REGISTER(ring->qno,R_BCM1480_PM_CNT), V_BCM1480_PM_COUNT(cnt));
}

static int pm_dequeue_rxbuf(pmring_t *ring,packet_t **ppkt)
{
    packet_t *pkt;
    uint64_t status;
    volatile pmdscr_t *dscr = ring->dscr + ring->next_rem;

    if (!(dscr->dscr_a & M_BCM1480_PM_DSCR0_HW) && (ring->next_rem != ring->next_add)) {
	status = dscr->dscr_a;
	if (dscr->dscr_a & (M_BCM1480_PM_DSCR0_PE | M_BCM1480_PM_DSCR0_SE | M_BCM1480_PM_DSCR0_LE)) {
	    printf("Ring %d: DscrErr: %016llX [ %s]\n",ring->qno,status,showfields(dscrerrs,status));
	    }
	pkt = ring->ctx[ring->next_rem];
	pkt->len = G_BCM1480_PM_DSCR0_BUFFER_LENGTH(dscr->dscr_a);
	pkt->next_dest = G_BCM1480_PM_DSCR1_NEXT_DEST(dscr->dscr_b);
	*ppkt = pkt;
	ring->next_rem++;
	ring->onring--;
	if (ring->next_rem == ring->ringsize) ring->next_rem = 0;
	return 1;
	}
    return 0;
}

static int pm_dequeue_txbuf(pmring_t *ring,packet_t **ppkt)
{
    packet_t *pkt;
    volatile pmdscr_t *dscr = ring->dscr + ring->next_rem;

    if (!(dscr->dscr_a & M_BCM1480_PM_DSCR0_HW) && (ring->next_rem != ring->next_add)) {
	pkt = ring->ctx[ring->next_rem];
	*ppkt = pkt;
	ring->next_rem++;
	if (ring->next_rem == ring->ringsize) ring->next_rem = 0;
	ring->onring--;
	return 1;
	}
    return 0;
}


static void pm_free_rings(void)
{
    int idx;

    for (idx = 0; idx < PM_NRINGS; idx++) {
	if (pmi_rings[idx]) pm_free_ring(pmi_rings[idx]);
	if (pmo_rings[idx]) pm_free_ring(pmo_rings[idx]);
	pmi_rings[idx] = NULL;
	pmo_rings[idx] = NULL;
	}
}

static void pm_drain_rings(void)
{
    int idx;
    packet_t *pkt,*tpkt;
    int cnt;

    for (idx = 0; idx < PM_NRINGS; idx++) 
    {
        if (pmo_rings[idx]) 
        {
            while (pm_dequeue_txbuf(pmo_rings[idx],&pkt)) 
            {
                printf("PMO(%d) TX buffer flushed\n", idx );
                pm_pkt_free(pkt);
            }
        }
        if (pmi_rings[idx] && pmo_rings[idx]) 
        {
            cnt = 0;
            do 
            {
                cnt = 0;
                while (pm_dequeue_rxbuf(pmi_rings[idx],&pkt)) 
                {
                    printf("PMI(%d) RX buffer flushed\n", idx );
                    pm_add_rxbuf(pmi_rings[idx],pkt); 
                    pm_start_rx(pmi_rings[idx],1);
                    cnt++;
                }
                /* Keep trying to drain TX buffers while we do this, in case we're
                   in loopback mode and the tx buffers are waiting */
                while (pm_dequeue_txbuf(pmo_rings[idx],&tpkt)) 
                {
                    printf("PMO(%d) TX buffer flushed\n", idx ); 
                    pm_pkt_free(tpkt);
                }
            
            } while (cnt > 0);
        }
	}
}

// in "map8" mode we give all three ports each 8 channels
static void setup_map8(int cross) {
	uint64_t value;

	// outbound (the ID24 part is a don't care--shove it to HSP2[15:8]
	value = (V_BCM1480_PM_MAP_DEST_ID0(0) |
		V_BCM1480_PM_MAP_DEST_ID8(1) |
		V_BCM1480_PM_MAP_DEST_ID16(2) |
		V_BCM1480_PM_MAP_DEST_ID24(2) |  M_BCM1480_PM_MAP_DEST_HALF24 |
		0);
	WRITECSR(A_BCM1480_PM_PMO_MAPPING, value);

	// inbound (HSP012[15:8] are don't cares--shove them all to pm[31:24])
	if (cross) {
    WRITECSR(A_BCM1480_HR_REGISTER(0,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_15_8) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_31_24)));

	    WRITECSR(A_BCM1480_HR_REGISTER(1,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_7_0) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_31_24)));
	}
	else {
	    WRITECSR(A_BCM1480_HR_REGISTER(0,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_7_0) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_31_24)));

	    WRITECSR(A_BCM1480_HR_REGISTER(1,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_15_8) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_31_24)));
	}

	WRITECSR(A_BCM1480_HR_REGISTER(2,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_23_16) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_31_24)));
}

// in "map16" mode we are able to use all 16 channels for ports 0 and 1 only
static void setup_map16(int cross) {
	uint64_t value;

	// outbound
	value = (V_BCM1480_PM_MAP_DEST_ID0(0) |
		V_BCM1480_PM_MAP_DEST_ID8(0) | M_BCM1480_PM_MAP_DEST_HALF8 |
		V_BCM1480_PM_MAP_DEST_ID16(1) |
		V_BCM1480_PM_MAP_DEST_ID24(1) |  M_BCM1480_PM_MAP_DEST_HALF24 |
		0);
	WRITECSR(A_BCM1480_PM_PMO_MAPPING, value);

	// inbound (HSP2[15:0] is don't care--shove them all to pm[15:0])
	if (cross) {
	    WRITECSR(A_BCM1480_HR_REGISTER(0,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_23_16) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_31_24)));

	    WRITECSR(A_BCM1480_HR_REGISTER(1,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_7_0) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_15_8)));
	}
	else {
	    WRITECSR(A_BCM1480_HR_REGISTER(0,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_7_0) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_15_8)));

	    WRITECSR(A_BCM1480_HR_REGISTER(1,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_23_16) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_31_24)));
	}

	WRITECSR(A_BCM1480_HR_REGISTER(2,R_BCM1480_HR_MAPPING),
		(V_BCM1480_HR_RX2PMI_MAP_LO(K_BCM1480_HR_RX2PMI_MAP_7_0) |
		V_BCM1480_HR_RX2PMI_MAP_HI(K_BCM1480_HR_RX2PMI_MAP_15_8)));
}

// returns '1' if error
static int init_pm_ring_block(int start, int count) {
	int idx;
	for (idx = start; idx < start+count; idx++) {
		pmi_rings[idx]  = pm_init_ring(idx,PM_RINGSIZE);
		pmo_rings[idx]  = pm_init_ring(idx,PM_RINGSIZE);
		if (!pmi_rings[idx] || !pmo_rings[idx]) return 1;
	}
	return 0;
}

//------------------------------------------------------------------------------
//-HR block utility functions and test programs
// General, works for both SPI-4 and PoHT mode
// Tests just port0.  Requires loopback mode (cable straight back to port 0)
// Assumes pm init -map16 mode so that we can test all 16 VCs of the port.
// Glen 3/20/06

#define HR_DEFAULT_PKTLEN 64

// simplify argument passing to all these routines	FIX remove?
typedef struct hr_s 
{
	sbeth_physaddr_t base_addr;	// FIX remove
	uint32_t debug;
	uint32_t num_ch;
    uint32_t pktlen;
} hr_t;
hr_t hr;

static void set_header_ptr(int port, int offset) {
	sbeth_physaddr_t addr;
	addr = A_BCM1480_HR_BASE(port) + R_BCM1480_HR_CFG;
	uint64_t value;
	value = READCSR(addr);
	value &= ~(M_BCM1480_HR_HEADER_PTR);		// clear old one
	value |= V_BCM1480_HR_HEADER_PTR(offset);	// set new one
	WRITECSR(addr, value);
}

static void turn_on_next_dest_debug_mode(int port) {
	sbeth_physaddr_t addr;
	addr = A_BCM1480_HR_BASE(port) + R_BCM1480_HR_CFG;
	uint64_t value;
	value = READCSR(addr);

	value |= M_BCM1480_HR_SELECT_PTNUM_TO_TAG;
	WRITECSR(addr, value);
}

static void turn_off_next_dest_debug_mode(int port) {
	sbeth_physaddr_t addr;
	addr = A_BCM1480_HR_BASE(port) + R_BCM1480_HR_CFG;
	uint64_t value;
	value = READCSR(addr);

	value &= ~M_BCM1480_HR_SELECT_PTNUM_TO_TAG;
	WRITECSR(addr, value);
}

// create a direct mapping rule from IVC to OVC
// creates both the 'rule' and the 'path' that relies on that rule matching
// the rule is put in row 'i' and the path in row 'j'
static void hr_write_rule_ivc(int i, int j, uint16_t ivc, uint16_t ovc) {
	uint64_t value;

	value = ivc | (0xffULL << 32);
	WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_OP(i), value);

	value = (V_BCM1480_HR_RULE_TYPE_WORD_OFST_0(0) |
		M_BCM1480_HR_RULE_TYPE_SEL_0);
	WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_TYPE(i), value);

	value = V_BCM1480_HR_ENABLE(1 << (i)) |
		V_BCM1480_HR_TEST(0xffff) |
		V_BCM1480_HR_PATH_DATA_DEST(3) |	// PMI
		V_BCM1480_HR_PATH_DATA_NDEST(0) |
		V_BCM1480_HR_PATH_TYPE(0) |		// OVC mode
		V_BCM1480_HR_PATH_DATA_VC(ovc);
	WRITECSR(hr.base_addr + R_BCM1480_HR_PATH(j), value);
}


// create a direct mapping rule to forward a IVC out a 
// destination Port and OVC
// creates both the 'rule' and the 'path' that relies on that rule matching
// the rule is put in row 'i' and the path in row 'j'
static void hr_write_rule_ivc_port_forward(int i, int j, uint16_t ivc, uint16_t ovc, uint16_t port) {
	uint64_t value;

	value = ivc | (0xffULL << 32);
	WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_OP(i), value);

	value = (V_BCM1480_HR_RULE_TYPE_WORD_OFST_0(0) |
		M_BCM1480_HR_RULE_TYPE_SEL_0);
	WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_TYPE(i), value);

	value = V_BCM1480_HR_ENABLE(1 << (i)) |
		V_BCM1480_HR_TEST(0xffff) |
		V_BCM1480_HR_PATH_DATA_DEST(port) |	// output for 0 - 2
		V_BCM1480_HR_PATH_DATA_NDEST(0) |
		V_BCM1480_HR_PATH_TYPE(0) |		// OVC mode
		V_BCM1480_HR_PATH_DATA_VC(ovc);
	WRITECSR(hr.base_addr + R_BCM1480_HR_PATH(j), value);
}


// create a mapping for a bit-pattern in the packet
// creates both the 'rule' and the 'path' that relies on that rule matching
// the rule is put in row 'i' and the path in row 'j'
static void hr_write_rule_pattern(int i, int j, uint32_t en, uint32_t op,
	int word_offset, int ovc) {
	uint64_t value;

	value = op | (((uint64_t) en) << 32);
	WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_OP(i), value);

	value = V_BCM1480_HR_RULE_TYPE_WORD_OFST_0(word_offset);
	WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_TYPE(i), value);

	value = V_BCM1480_HR_ENABLE(1 << (i)) |
		V_BCM1480_HR_TEST(0xffff) |
		V_BCM1480_HR_PATH_DATA_DEST(3) |	// PMI
		V_BCM1480_HR_PATH_DATA_NDEST(0) |
		V_BCM1480_HR_PATH_TYPE(0) |		// OVC mode
		V_BCM1480_HR_PATH_DATA_VC(ovc);
	WRITECSR(hr.base_addr + R_BCM1480_HR_PATH(j), value);
}

static void hr_default_to_pmi(int ovc) {
	sbeth_physaddr_t addr = hr.base_addr + R_BCM1480_HR_PATH_DEFAULT;
	uint64_t value;
	value = V_BCM1480_HR_PATH_DATA_DEST(3)  |	// PMI
		V_BCM1480_HR_PATH_DATA_NDEST(0) |
		V_BCM1480_HR_PATH_TYPE(0) |		// OVC mode
		V_BCM1480_HR_PATH_DATA_VC(ovc);
	WRITECSR(addr, value);
}

/*
static void hr_default_to_tx(int tx, int ovc) {
	uint64_t value;
	value = V_BCM1480_HR_PATH_DATA_DEST(tx)  |	// to this 'tx' port
		V_BCM1480_HR_PATH_DATA_NDEST(0) |
		V_BCM1480_HR_PATH_TYPE(0) |		// OVC mode
		V_BCM1480_HR_PATH_DATA_VC(ovc);
	WRITECSR(hr.base_addr + R_BCM1480_HR_PATH_DEFAULT, value);
}
*/

static void hr_clear(void) {
	uint64_t value = 0;
	int i;
	for (i=0; i<16; i++) {
		WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_OP(i), value);
		WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_TYPE(i), value);
		WRITECSR(hr.base_addr + R_BCM1480_HR_PATH(i), value);
	}
	hr_default_to_pmi(31);	// send default to PMI queue 31 (drop)

    /* Reset header offset to 0 */
    set_header_ptr(0, 0);

}

// use the 16 rule/path entries to create a mapping that passes any
// incoming packet through to the PMI on the VC that it came in on
// (you might think this would be the default, but it takes a special
// mapping to create this situation)
// both rule and path go on row 'i' for VC 'i'
static void hr_set_passthrough(void) {
	// passthrough map
	int i;
	for (i=0; i<16; i++) {
		hr_write_rule_ivc(i, i, i, i);
	}
}

// returns '1' if error
static int check_data(packet_t* pkt, uint8_t* data) {
	int i;
	int err = 0;
	for (i=0; i<pkt->len; i++) {
		if (pkt->pkt[i] != data[i]) {
			printf("	[%d]=0x%x expected 0x%x\n",
				pkt->pkt[i], data[i]);
			err = 1;
		}
	}
	return err;
}

// returns '1' if error
static int hr_send_one(int txch, int rxch, int expected_path, int cookie_offset,
	uint32_t cookie, uint32_t pktlen) {
	packet_t *txpkt;
	packet_t *rxpkt;
	uint8_t data[PKTBUFSIZE];
	txpkt = pm_pkt_alloc();
	fillRandom(txpkt->pkt, data, pktlen);
	txpkt->len = pktlen;
	if (cookie_offset >= 0) {
		if (hr.debug) {
			printf("writing cookie 0x%x at offset [%d]\n", cookie,
				cookie_offset);
		}
		write_cookie(txpkt, cookie_offset, cookie);
	}
	if (hr.debug) {
		printf("Packet contents:\n");
		dump_packet(txpkt);
	}
	save_data(txpkt, data);		// a final time to capture cookies
	pm_add_txbuf(pmo_rings[txch], txpkt);
	pm_start_tx(pmo_rings[txch], 1);
	if (hr.debug) {
		printf("TX    pm[%d] pktlen=%d\n", txch, txpkt->len);
	}

	int err = 0;
	int cnt = 0;
	int waiting = 2;		// get both tx and rx processed
	uint64_t start = SBREADCSR(A_BCM1480_SCD_ZBBUS_CYCLE_COUNT);
	while (waiting) {
		if (pm_dequeue_txbuf(pmo_rings[txch],&txpkt)) {
			waiting--;
			pm_pkt_free(txpkt);
		}

		if (pm_dequeue_rxbuf(pmi_rings[rxch],&rxpkt)) {
			waiting--;
			if (hr.debug) {
				printf("   RX pm[%d] pktlen=%d path=%d\n",
					rxch, rxpkt->len, rxpkt->next_dest);
			}
			err |= check_data(rxpkt, data);

			// only check path # if requested to do so by caller
			if (expected_path >= 0) {
				if (rxpkt->next_dest != expected_path) {
					printf("ERROR expected path %d\n",
						expected_path);
					err = 1;
				}
			}

			pm_add_rxbuf(pmi_rings[rxch],rxpkt);
			pm_start_rx(pmi_rings[rxch], 1);
			cnt++;
		}
		uint64_t now = SBREADCSR(A_BCM1480_SCD_ZBBUS_CYCLE_COUNT);
		if (now - start > 100000000) {
			// 200 ms, give up
			printf("ERROR: packet not received on %d\n", rxch);
			pm_drain_rings();
				// get back to a sane state for further testing
			break;
		}
	}
	if (waiting > 0) err = 1;
	return err;
}

static inline int hr_default_rule(void) {
	printf("Default rule in table with passthrough info...\n");
	int err = 0;
	int ch;
	for (ch=0; ch<hr.num_ch; ch++) {
		hr_clear();
		hr_default_to_pmi(ch);
		err |= hr_send_one(ch, ch, DEFAULT_PATH, NO_COOKIE, 0, hr.pktlen);
	}
	return err;
}

static inline int hr_passthrough(void) {
	printf("Single packets on each channel in passthrough...\n");
	int i;
	int err = 0;
	hr_set_passthrough();
	for (i=0; i<19; i++) {		// to test uneven ring wrap
		int ch = i % hr.num_ch;
		err |= hr_send_one(ch, ch, ch, NO_COOKIE, 0, hr.pktlen);
	}
	return err;
}

static inline int hr_single_passthrough(void) {
	printf("A single rule in table with passthrough info...\n");
	int err = 0;
	int i;

	// in slot 'i' in the rule table
	for (i=0; i<16; i++) {
	    int j;
	    for (j=0; j<16; j++) {
		int ch;
		// put a passthrough map for OVC 'ch' to come back to IVC 'ch'
		for (ch=0; ch<hr.num_ch; ch++) {
			hr_clear();
			hr_write_rule_ivc(i, j, ch, ch);
			err |= hr_send_one(ch, ch, j, NO_COOKIE, 0, hr.pktlen);
		}
	    }
	}
	return err;
}


//--------------------------------------------------
/**
 
 This port manage diagnostic tests the hash and route functionality
 by building a rule and path table which mapped to a linear tree
 routing rule.  The rule table is filled in with 4 or 8 rules checking
 for patterns in the lower nibble.  Below describes the map8 case

     HR: Rule Table   
     Index Comp Value    Enable 
     0     0x00000001   0x00000001
     1     0x00000002   0x00000002
     2     0x00000004   0x00000004
     3     0x00000008   0x00000008

     HR: Match Table
     Index   Bin Encoding     Output
             (X don't care)   PMI-Q
     0       XXX0             0
     1       XX01             1
     2       X011             2
     3       0111             3
     4       1111             4
     D       <other>          5

 The test builds this above table, and then sendings packets in on PMO-0 
 with all possible bit combinations of the lower nibble and verifies the
 final destiniation route is correct.  This will will be one of the 
 mentioned PMI queues

 @ingroup PMDIAG

*/
//--------------------------------------------------
static inline uint32_t hr_test_rule_linear_tree(void) 
{
	printf("A linear tree routing table ...\n");
    uint32_t err = 0;
	uint32_t i, j;
    uint32_t operand       = 0;
    uint32_t enable        = 0;
    uint32_t cookie        = 0;
    uint32_t path          = 0;
    uint32_t word_offset   = 0;
    uint64_t reg_val ;

    /* Clear hash and route tables */
    hr_clear();

	/*
     * in slot 'i' in the rule table, tree depth equals to number of
     * channels
     */
	for (i=0; i < hr.num_ch ; i++) 
    {
        operand = 0x1 << i;
        enable  = 0x1 << i;
        reg_val = operand  | (((uint64_t) enable) << 32);
        WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_OP(i), reg_val);

        reg_val = V_BCM1480_HR_RULE_TYPE_WORD_OFST_0( word_offset);
        WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_TYPE(i), reg_val);
    }

	// in slot 'i' in the path table
	for (i = 0; i < ((hr.num_ch / 2) + 1); i++) 
    {
        if (i == 0)
        {
            path   = 0x0000;
            enable = 0x0001;
        }
        else
        {
            path   = (path << 1)   | 0x1;
            enable = (enable << 1) | 0x1;    
        }

        reg_val = V_BCM1480_HR_ENABLE(enable) |
                  V_BCM1480_HR_TEST(path) |
                  V_BCM1480_HR_PATH_DATA_DEST(3) |	// PMI
                  V_BCM1480_HR_PATH_DATA_NDEST(0) |
                  V_BCM1480_HR_PATH_TYPE(0) |		// OVC mode
		          V_BCM1480_HR_PATH_DATA_VC( i );
        WRITECSR(hr.base_addr + R_BCM1480_HR_PATH(i), reg_val);
    }

    /* Set default PMI to one above programmed paths */
    hr_default_to_pmi(  ((hr.num_ch / 2) + 1) );

	for (i=0; i < ((hr.num_ch / 2) + 1); i++) 
    {	
        if (i == 0)
        {
            cookie   = 0x0000;
        }
        else
        {
            cookie   = (cookie << 1)   | 0x1;
        }
        
		err |= hr_send_one(0, i, i, word_offset, cookie, hr.pktlen);
	}


    /* 
     * cycle through all possible bit combinations 
     * Look for bit matches of known paths 
     */
	for (i=0; i < hr.num_ch; i++) 
    {
        /* 
         * check to see if pattern matches known path,
         * If no match is found, it will be PMO of the 
         * deafult path
         */
        path        = 0;
        enable      = 0x1;
        for (j = 0; j < ((hr.num_ch / 2) + 1); j++)
        {
            if ((i & enable) == path)
            {
                break;
            }
            else
            {
                path   = (path << 1)   | 0x1;
                enable = (enable << 1) | 0x1;    
            }
        }

		err |= hr_send_one(i, 
                           j, 
                           (j == ((hr.num_ch / 2) + 1) ? 31 : j), 
                           word_offset, 
                           i,
                           hr.pktlen);
	}
 
	return err;
}

//--------------------------------------------------
/**
 
 This port manager diagnostic tests the hash and route functionality
 by building a rule and path table which mapped to a full balanced
 binary tree.  The rule table is filled in with 4 rules checking
 for 8 patterns in the each nibble.  Below describes the map8 case

     HR: Rule Table   
     Index Comp Value    Enable 
     0     0x00000080   0x000000FF
     1     0x00008000   0x0000FF00
     2     0x00800000   0x00FF0000
     3     0x80000000   0xFF000000

     HR: Match Table
     Index   Bin Encoding     Output
             (X don't care)   PMI-Q
     0       0000             0     (Default route no 8s)
     1       0001             1
     2       0010             2
     3       0011             3
     4       0100             4
     ...
     15      1111             15

 The test builds this above table, and then sendings packets in on PMO-0 
 with all possible bit combinations of the lower nibble and verifies the
 final destiniation route is correct.  This will be one of the 
 mentioned PMI queues

 @ingroup PMDIAG

*/
//--------------------------------------------------
static inline uint32_t hr_test_rule_balanced_tree_crazy_8s(void) 
{
	printf("A full balanced binary tree routing table - crazy 8s ...\n");
    uint32_t err = 0;
	uint32_t i, j;
    uint32_t operand       = 0;
    uint32_t enable        = 0;
    uint32_t cookie        = 0;
    uint32_t path          = 0;
    uint32_t word_offset   = 0;
    uint64_t reg_val ;

    /* Clear hash and route tables */
    hr_clear();

	/*
     * in slot 'i' in the rule table, tree depth equals to number of
     * channels
     */
	for (i=0; i < sizeof(uint32_t) ; i++) 
    {
        operand = 0x80UL << (i * 8);
        enable  = 0xFFUL << (i * 8);
        reg_val = operand  | (((uint64_t) enable) << 32);
        WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_OP(i), reg_val);

        reg_val = V_BCM1480_HR_RULE_TYPE_WORD_OFST_0( word_offset);
        WRITECSR(hr.base_addr + R_BCM1480_HR_RULE_TYPE(i), reg_val);
    }

	// in slot 'i' in the path table. compute all possible combinations
	for (i = 0; i < sizeof(uint32_t) * sizeof(uint32_t); i++) 
    {
        path   = i;
        enable = 0xF;    

        reg_val = V_BCM1480_HR_ENABLE(enable) |
                  V_BCM1480_HR_TEST(path) |
                  V_BCM1480_HR_PATH_DATA_DEST(3) |	// PMI
                  V_BCM1480_HR_PATH_DATA_NDEST(0) |
                  V_BCM1480_HR_PATH_TYPE(0) |		// OVC mode
		          V_BCM1480_HR_PATH_DATA_VC( i );
        WRITECSR(hr.base_addr + R_BCM1480_HR_PATH(i), reg_val);
    }

    /* 
     * cycle through all possible '8's combinations 
     * Look for bit matches of known paths 
     */
	for (i=0; i < sizeof(uint32_t) * sizeof(uint32_t); i++) 
    {
        cookie =  (i & 0x8) ? 0x80000000 : 0 ;
        cookie |= (i & 0x4) ? 0x00800000 : 0 ;
        cookie |= (i & 0x2) ? 0x00008000 : 0 ;
        cookie |= (i & 0x1) ? 0x00000080 : 0 ;

		err |= hr_send_one(0, i, i, word_offset, cookie, hr.pktlen);
	}

    /* 
     * cycle through more bit combinations 
     * Look for bit matches of known paths 
     */
	for (i=0; i <= 0x80808080 ; i += 0x800080) 
    {
        /* Check to see if current index is one a special route */
        path = 0;
        for (j = 0; j < sizeof(uint32_t); j++)
        { 
            if ( ( ( i >> (j * 8) ) & 0xFF) == 0x80 ) 
            {
                path |= (1 << j);
            }
        }

		err |= hr_send_one(0, path, path, word_offset, i, hr.pktlen);
	}
 
	return err;
}


//--------------------------------------------------
/**
 
 This port manager diagnostic tests the hash and route functionality
 by building a set of overlapping comparision rules to set the 
 priority ordering of the HR Path table.  
 The cookie will be 0x8BADF00D and several rules will be defined which 
 match subsets of this word. The first match of FOOD should hit over all 
 others following lower priority matches. Below describes the table setups

     HR: Rule Table   
     Index Comp Value    Enable 
     0     0x0000F00D   0x0000FFFF
     1     0x8BAD0000   0xFFFF0000
     2     0x600D0000   0xFFFF0000
     3     0x600DF00D   0xFFFFFFFF
     4     0x8BADF00D   0xFFFFFFFF


     HR: Match Table
     Index   Bin Encoding     Output
             (X don't care)   PMI-Q
     0       00001             0     (600DFOOD:Match but lower priority) 
     1       00010             1     (8BADFOOD:Match but lower priority) 
     2       00100             2     (600DFOOD:Match but lower priority) 
     3       01000             3     (500DFOOD:GOODFOOD match)  
     4       10000             4     (8BADFOOD:BADFOOD match)  

 The test builds this above table, and then sendings packets in on PMO-0 
 with sub pattern 600DFOOD, 0000F00D, 8BAD0000 to verify direct matched, and then
 send 8BADFOOD to verify high priority match.

 NOTE: Highest priority is HIGHEST table index.

 @ingroup PMDIAG

*/
//--------------------------------------------------
static inline uint32_t hr_test_rule_priority_match(void) 
{
	printf("A priority match test ...\n");
    uint32_t err = 0;
	uint32_t i;
    uint32_t word_offset   = 0;
    static uint32_t pr_vals[][3] = { 
      /* Enable      Operand     Expected PMO  */
        {0x0000FFFF, 0x0000F00D, 0},
        {0xFFFF0000, 0x8BAD0000, 1},
        {0xFFFF0000, 0x600D0000, 2},
        {0xFFFFFFFF, 0x600DF00D, 3},
        {0xFFFFFFFF, 0x8BADF00D, 4} };

    uint32_t rule_cnt = sizeof( pr_vals ) / (3 * sizeof(uint32_t)); 

    /* Clear hash and route tables */
    hr_clear();

    for (i = 0; i < rule_cnt; i++)
    {
        hr_write_rule_pattern(i, i, pr_vals[i][0], pr_vals[i][1], word_offset, i);
    }

    for (i = 0; i < rule_cnt; i++)
    {
        err |= hr_send_one(0, pr_vals[i][2], pr_vals[i][2], word_offset, pr_vals[i][1], hr.pktlen);
    }

	return err;
}


static inline int hr_single_passthrough_plus_constant(void) {
	printf("A single rule in table with simple IVC->OVC map +3 ...\n");
	int err = 0;
	int i;
	for (i=0; i<16; i++) {
	    int j;
	    for (j=0; j<16; j++) {
		int ch;
		// put a passthrough map for OVC 'ch' to come back to IVC 'ch'
		for (ch=0; ch<hr.num_ch; ch++) {
			int rxch = ch + 3;
			if (rxch >= hr.num_ch) rxch -= hr.num_ch;

			hr_clear();
			hr_write_rule_ivc(i, j, ch, rxch);

			err |= hr_send_one(ch, rxch, j, NO_COOKIE, 0, hr.pktlen);
		}
	    }
	}
	return err;
}




static inline int hr_bitmask_one(int offset) {
	int err = 0;

	
	uint32_t op = rand();
	uint32_t en = rand();

	uint32_t cookie = op;

	int i = 7;	// rule row, arbitrary
	int j = 9;	// path row, arbitrary
	int word_offset = 0;
	int ovc = 3;	// arbitrary

	if (hr.debug) {
		printf("en=0x%x op=0x%x cookie=0x%x\n", en, op, cookie);
	}

	hr_clear();

    set_header_ptr(0, offset);
	hr_write_rule_pattern(i, j, en, op, word_offset, ovc);

	err |= hr_send_one(0, ovc, j, offset, cookie, 1024 );
	return err;
}

static int hr_bitmask(void) {
	printf("Test of immediate op compare to cookie...\n");
	int err = 0;
	int i;
	for (i=0; i < 255; i++) 
    {
		err |= hr_bitmask_one(i);
	}

	return err;
}



// a suite of tests performed on HSP port 0 in loopback mode
static int ui_cmd_pm_hr(ui_cmdline_t *cmd,int argc,char *argv[])
{
	if (!hsp_ports) {
		printf("PM not initialized.  Do \"pm init -map16 0\" first.\n");
		return -1;
	}
	if (!(hsp_ports & 0x1)) return ui_showusage(cmd);
		// assuming we have port 0 config'd

	// can run in either mode
	if (map16) hr.num_ch = 16;
	else hr.num_ch = 8;

	hr.debug = cmd_sw_isset(cmd,"-debug");
	hr.base_addr = A_BCM1480_HR_BASE(0);
	int err = 0;

	turn_on_next_dest_debug_mode(0);

	//----------------------------------------------------------------------
	printf("Running suite of HR tests...\n");

	err |= hr_default_rule();
	err |= hr_passthrough();
	err |= hr_single_passthrough();
	err |= hr_single_passthrough_plus_constant();
    err |= hr_test_rule_linear_tree();
    err |= hr_test_rule_balanced_tree_crazy_8s();
    //hr.debug = 1;
    err |= hr_test_rule_priority_match(); 
    err |= hr_bitmask();


	//----------------------------------------------------------------------
	if (err) {
		printf("HR tests FAIL\n");
	}
	else {
		printf("HR tests PASS\n");
	}

	// put things back to the default so tests like "pm test" will run
	hr_set_passthrough();
	turn_off_next_dest_debug_mode(0);

	return 0;
}



//------------------------------------------------------------------------------

static int ui_cmd_pm_init(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int port;
    int i;
    int j; 
    int idx;
    int cnt;
    sbeth_physaddr_t base_addr, hr_cfg_addr, tx_next_addr_addr;
    uint64_t value;
    volatile pcireg_t t;      /* used for reads that push writes */
    packet_t *pkt;
    char *x;
    int spi4_pllmult = SPI4_MHZ / (REFCLK / 2);
    uint64_t cpu_mhz;
    uint64_t sw_mhz;
    uint64_t spi4_mhz = SPI4_MHZ;
    int portmask;
    int cross;
    int active_channels;
    int rstat;

    /*
     * Don't let us do this again
     */

    if (hsp_ports != 0) {
        printf("The Packet Manager & High-Speed ports are already initialized.\n");
        return -1;
	}

    /*
     * Collect a list of which ports we're going to init
     */
    portmask = 0;
    for (cnt = 0; cnt < BCM1480_HSP_NUM_PORTS; cnt++) {
        x = cmd_getarg(cmd,cnt);
        if (x) {
            j = atoi(x);
            portmask |= 1<<j;
	    }
	}
    if (portmask == 0) portmask = 0x7;	/* default to all ports */
    hsp_ports = portmask;

    cross = cmd_sw_isset(cmd,"-cross");

    rstat = !cmd_sw_isset(cmd,"-no_rstat");	// default is rstat for loopback

    map16 = cmd_sw_isset(cmd,"-map16");
    if (map16) {
    	printf("map16 mode: PM0-15 to HSP0, PM16-31 to HSP1, HSP2 unused\n");
    }
    else {
    	printf("map8 mode: PM0-7 to HSP0, PM8-15 to HSP1, PM16-23 to HSP2\n");
    }

    // minor optimization to use only 8 channels if we have to--the
    // active_channels could also be left at 16 for both map8 and map16
    if (cmd_sw_value(cmd,"-chan",&x)) {
        active_channels = atoi(x);
    }
    else if (map16) {
    	active_channels = 16;
    }
    else {
    	active_channels = 8;
    }
    printf("Configuring RAM for %d active channels\n",
           active_channels);

    if (cmd_sw_value(cmd,"-mhz",&x)) {
        spi4_mhz = atoi(x);
	}

    cpu_mhz = G_BCM1480_SYS_PLL_DIV(READCSR(A_SCD_SYSTEM_CFG)) * (REFCLK/2);
    sw_mhz = cpu_mhz * 2 / G_BCM1480_SYS_SW_DIV(READCSR(A_SCD_SYSTEM_CFG));

    spi4_pllmult = spi4_mhz / (REFCLK/2);

    printf("CPUCLK: %dMHz   SWCLK: %dMHz   SPI4CLK: %dMHz\n",cpu_mhz,sw_mhz,spi4_mhz);

    /*
     * Clean up from a previous invocation
     */

    pm_free_pktpool();
    pm_free_rings();		// sets all ring pointers to NULL (required)
    
    int err = 0;
    if (map16) {
    	if (hsp_ports & 0x1) err |= init_pm_ring_block(0, 16);
    	if (hsp_ports & 0x2) err |= init_pm_ring_block(16, 16);
    }
    else {
    	if (hsp_ports & 0x1) err |= init_pm_ring_block(0, 8);
    	if (hsp_ports & 0x2) err |= init_pm_ring_block(8, 8);
    	if (hsp_ports & 0x4) err |= init_pm_ring_block(16, 8);
    }
    pm_init_pktpool();

    if (err || !pm_pkt_pool) {
        printf("Memory allocation failed\n");
        return -1;
	}

    /*
     * Force the TX ports back into reset state
     */

    for (port = 0; port < BCM1480_HSP_NUM_PORTS; port++) {
        if (!(portmask & (1<<port))) continue;		/* skip ports we're not using */
        value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_CFG_0));
        value |= M_BCM1480_HSP_TX_PORT_RESET;
        WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_CFG_0),value);
        value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CFG_0));
        value |= M_BCM1480_HSP_RX_PORT_RESET;
        WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CFG_0),value);
	}

    /* 
     * Initialize each port, including the H&R Blocks 
     */
    for (port = 0; port < BCM1480_HSP_NUM_PORTS; port++) {

        if (!(portmask & (1<<port))) continue;		/* skip ports we're not using */

        int spi4 = HSP_IS_PORT_SPI4_MODE(port);

        printf("Initializing TX/RX %d (%s)\n",port,spi4 ? "SPI4" : "PoHT");

        /*
         * Using the # of active channels we're planning to support, carve
         * up the TX and RX memory to give everyone a share.
         */
        hsp_pm_ram_n_flow_alloc( port, active_channels );

        /*
         * If the port is in SPI4 mode, do the SPI4 port initialization.
         * If the port is in PoHT mode, the HT has already been initialized for us.
         */

        if (spi4) {
            hsp_spi4_init_port(port, spi4_mhz, active_channels, rstat);
	    }
        else
        {
            /*
             * Issue a HT LINK_RESET through the brdge confg space to the ports 
             * force new rx credit to be established
             */
            hsp_ht_port_link_reset( port );
        }
	    
        /*
         * Do Hash and Route initialization.
         */

        base_addr = A_BCM1480_HR_BASE(port);
        hr_cfg_addr = base_addr + R_BCM1480_HR_CFG;
        value = V_BCM1480_HR_HEADER_PTR(0)  |
            M_BCM1480_HR_HDR_PTR_IMMD;
        WRITECSR(hr_cfg_addr, value);

        hr.base_addr = base_addr;
        hr.debug = 0;
        hr.pktlen = HR_DEFAULT_PKTLEN;
        hr_default_to_pmi(0);
        hr_set_passthrough();

        if (!spi4) {
            for (j = 0; j < 8; j++) {
                /* TX Next Addr Register */
                tx_next_addr_addr = A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_NEXT_ADDR_REGISTER(j));  

                // original code said set to 0x4000_0000 but think this is wrong
                value =  (V_BCM1480_HSP_TX_NEXT_ADDR_EVEN(0x400000) |
                          V_BCM1480_HSP_TX_NEXT_ADDR_ODD(0x400000));

                // setting both address above and BAR1 below to zero works too
                // value = 0;

                WRITECSR(tx_next_addr_addr, value);
            }

            // value = 0;
            value = 0x40000000;

            pci_conf_write(BCM1480_LDT_BRIDGE, PCI_MAPREG(1), value);   /* POHT = 0x4000_0000 */
            /* HT: push the writes */
            t = pci_conf_read(BCM1480_LDT_BRIDGE, PCI_ID_REG);
            pci_conf_write(BCM1480_EXTx_BRIDGE(port),0x78,1<<18);
            t = pci_conf_read(BCM1480_LDT_BRIDGE, PCI_ID_REG);

            /*
              Code put in to try to initialize secondary as well as primary
              if (port == 0) 
              pci_conf_write(BCM1480_EXTx_BRIDGE_BUS1(port),0x78,1<<18);
              if (port == 1) 
              pci_conf_write(BCM1480_EXTx_BRIDGE_BUS2(port),0x78,1<<18);
              t = pci_conf_read(BCM1480_LDT_BRIDGE, PCI_ID_REG);
            */ 
	    }

	}

    if (map16) {
        setup_map16(cross);
    }
    else {
        setup_map8(cross);
    }

    printf("Initializing PMI and PMO Queues\n");
    for (idx = 0; idx < PM_NRINGS; idx++) {
        if (pmo_rings[idx]) {
            pm_init_pmi_queue(pmi_rings[idx]);
            pm_init_pmo_queue(pmo_rings[idx]);
	    }
	}

    /* Initialize The PMI Ring */
    for (idx = 0; idx < PM_NRINGS; idx++) {
    	if (!pmo_rings[idx]) continue;

        int cnt = 0;
        for (i = 0; i < PM_RINGSIZE-1; i++) {
            pkt = pm_pkt_alloc();
            if (pkt) cnt += pm_add_rxbuf(pmi_rings[idx],pkt);
	    }
        pm_start_rx(pmi_rings[idx],cnt);
	}

    /* Display TX/RX calibration, if in SPI4 mode */

    for (port = 0; port < BCM1480_HSP_NUM_PORTS; port++) {
        uint64_t reg,txreg,rxreg;

        if (!(portmask & (1<<port))) continue;

        reg = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CFG_0));

        if (G_BCM1480_HSP_LINK_MODE(reg) == K_BCM1480_HSP_LINK_MODE_SPI4) {  
            txreg = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_CALIBRATION));
            rxreg = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_CALIBRATION));
            printf("SPI4 Status, Port %d:  TX:%s   RX:%s\n",port,
                   (txreg & M_BCM1480_HSP_CAL_CALFIN) ? "Cal_Finished" : "Not_Cal     ",
                   (rxreg & M_BCM1480_HSP_CAL_CALFIN) ? "Cal_Finished" : "Not_Cal     ");
	    }
	}

    
    return 0;

}


static void print_context(uint8_t* actual, uint8_t* expected, int i,
	int pktlen) {

	i = i/2;
	int lower = 0;
	int behind_span = 5;
	int ahead_span = 1;
		// turned out this showed errors because bits not in yet

	if (i - behind_span > 0) lower = i - behind_span;

	int upper = (pktlen/2) - 1;
	if (i + ahead_span < upper) upper = i + ahead_span;

	printf("Showing received data: (this bit-error marked with *)\n");

	uint16_t* act = (uint16_t*) actual;
	uint16_t* exp = (uint16_t*) expected;
	int j;
	for (j=lower; j<upper; j++) {
		uint16_t a = act[j];
		uint16_t e = exp[j];
		printf("[%d,%d]   0x%04x   ", j*2, j*2+1, a);
		int k;
		for (k=15; k>=0; k--) {
			int abit = ((a >> k) & 1);
			int ebit = ((e >> k) & 1);
			printf("%s%s ", abit ? "1" : "0",
				(abit == ebit) ? " " : "*");
		}
		printf("\n");
	}
}
  
static int ui_cmd_pm_test(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int i,idx;
    int num_to_send = PM_RINGSIZE;
    packet_t *pkt;
    int cont;
    uint64_t rxcnt = 0;
    int statcnt = 0;
    int rxonly;
    int pktlen;
    int randlen;
    int dump;
    int context;
    int num_context = 1;
    int patid = 0;
    int use_patid = 0;
    int data_pattern = 0;
    uint32_t use_data_pattern = 0;
    int dpat = 0;
    int use_dpat = 0;
    int num_errors = 0;
    int print_interval;
    int verify;
    int verify_fast;
    int tx_index = 0;		// okay if not lined up with ring's next_add
    int rx_index = 0;
    char *x;
    static pmring_t *_pmi_rings[PM_NRINGS];
    static pmring_t *_pmo_rings[PM_NRINGS];
    int txcount[PM_NRINGS];
    int rxcount[PM_NRINGS];
    char ringid[PM_NRINGS][40];
    int nrings = 0;
    int cnt;
    uint64_t cpu_mhz;
    int portmask = 0;
    int pkt_count = 0;
    int vc = 1;
    int vc_start = 0;
    int b_exit_on_max = 0;
    uint64_t max_packet_count = 0;

    cpu_mhz = G_BCM1480_SYS_PLL_DIV(READCSR(A_SCD_SYSTEM_CFG)) * (REFCLK/2);

    rxonly = cmd_sw_isset(cmd,"-recv");

    for (idx = 0; idx < BCM1480_HSP_NUM_PORTS; idx++) {
        x = cmd_getarg(cmd,idx);
        if (!x) break;
        i = atoi(x);
        portmask |= (1<<i);
	}
    if (portmask == 0) {		// default to all initialized ports
    	portmask = hsp_ports;
	}

    portmask &= hsp_ports;

    nrings = 0;
    if (cmd_sw_value(cmd,"-vc",&x)) vc = atoi(x);
    if (cmd_sw_value(cmd,"-vc_start",&x)) vc_start = atoi(x);
    if (map16) {
    	if (vc < 1 || (vc_start + vc) > 16) {
            printf("ERROR: Bad vc range %d-%d, must include at least one value\n",
                   vc_start, vc_start + vc - 1);
            printf("and be in range 0-15 for map16 mode\n");
            return -1;
        }
        printf("map16 mode:  will send packets on pm queues:\n");
    	if (portmask & 0x1) {
            for (idx = vc_start; idx < vc_start + vc; idx++) {
                _pmi_rings[nrings] = pmi_rings[idx];
                _pmo_rings[nrings] = pmo_rings[idx];
                sprintf(ringid[nrings], "pm[%d] --> HSP0[%d]", idx, idx);
                nrings++;
            }
        }
    	if (portmask & 0x2) {
            for (idx = 16 + vc_start; idx < 16 + vc_start + vc; idx++) {
                _pmi_rings[nrings] = pmi_rings[idx];
                _pmo_rings[nrings] = pmo_rings[idx];
                sprintf(ringid[nrings], "pm[%d] --> HSP1[%d]", idx, idx - 16);
                nrings++;
            }
        }
    }
    else {
    	if (vc < 1 || (vc_start + vc) > 8) {
            printf("ERROR: Bad vc range %d-%d, must include at least one value\n",
                   vc_start, vc_start + vc - 1);
            printf("and be in range 0-7 for map8 mode\n");
            return -1;
        }
        printf("map8 mode:  will send packets on pm queues:\n");
    	if (portmask & 0x1) {
            for (idx = vc_start; idx < vc_start + vc; idx++) {
                _pmi_rings[nrings] = pmi_rings[idx];
                _pmo_rings[nrings] = pmo_rings[idx];
                sprintf(ringid[nrings], "pm[%d] --> HSP0[%d]", idx, idx);
                nrings++;
            }
        }
    	if (portmask & 0x2) {
            for (idx = vc_start + 8; idx < 8 + vc_start + vc; idx++) {
                _pmi_rings[nrings] = pmi_rings[idx];
                _pmo_rings[nrings] = pmo_rings[idx];
                sprintf(ringid[nrings], "pm[%d] --> HSP1[%d]", idx, idx - 8);
                nrings++;
            }
        }
    	if (portmask & 0x4) {
            for (idx = vc_start + 16; idx < 16 + vc_start + vc; idx++) {
                _pmi_rings[nrings] = pmi_rings[idx];
                _pmo_rings[nrings] = pmo_rings[idx];
                sprintf(ringid[nrings], "pm[%d] --> HSP2[%d]", idx, idx - 16);
                nrings++;
            }
        }
    }
    if (nrings == 0) {
    	printf("no rings available, portmask=0x%x\n", portmask);
        return -1;
	}
    for (idx = 0; idx < nrings; idx++) {
    	printf("	<%02d>  %s\n", idx, ringid[idx]);
	}

    pktlen = DEFAULT_PKTSIZE;
    if (cmd_sw_value(cmd,"-len",&x)) pktlen = atoi(x);
    if (pktlen > PKTBUFSIZE) {
    	pktlen = PKTBUFSIZE;
    	printf("clipping packet size to max of %d bytes\n", pktlen);
    }
    randlen = cmd_sw_isset(cmd,"-randlen");

    dump = cmd_sw_isset(cmd,"-dump");

    verify = cmd_sw_isset(cmd,"-verify");
    verify_fast = cmd_sw_isset(cmd,"-verify_fast");

    /* default to 1, but allow more context debug print as well */
    context = cmd_sw_isset(cmd,"-context");
    if (cmd_sw_value(cmd,"-context",&x)) num_context = atoi(x);
    if (context) {
    	printf("will print error context for %d errors, then stop\n",
               num_context);
    }

    if (cmd_sw_value(cmd,"-patid",&x)) {
        use_patid = 1;
        patid = atoi(x);
	}

    if (cmd_sw_value(cmd,"-data",&x)) {
        use_data_pattern = 1;
        data_pattern = atoi(x);
	}

    if (cmd_sw_value(cmd,"-pat",&x)) {
    	use_dpat = 1;
    	dpat = atoi(x);
        printf("Using custom data pattern: 0x%x\n", dpat);
        printf("for 16-sequential words, with each bit having same value\n");
    }

   if (cmd_sw_value(cmd,"-max",&x)) {
    	b_exit_on_max = 1;
    	max_packet_count = (uint64_t) atoi(x);
        printf("Will exit after: 0x%x packets sent\n", max_packet_count);
    }

    cont = rxonly || cmd_sw_isset(cmd,"-continuous");

    num_to_send = DEFAULT_COUNT;
    if (cmd_sw_value(cmd,"-count",&x)) num_to_send = atoi(x);
    if (num_to_send > (PM_RINGSIZE-1)) {
    	printf("clipping -count size to max of %d packets\n", num_to_send);
    	num_to_send = PM_RINGSIZE-1;
    }

    if (verify || verify_fast) {
    	// print more often so user doesn't think the terminal is hung
    	print_interval = 10000;
    }
    else {
    	print_interval = 1000000;
    }

    if (!hsp_ports) {
        printf("PM not initialized.  Do 'pm init' first.\n");
        return -1;
	}

    /* Drain pending TX completes and refill RX rings */
    pm_drain_rings();

    for (idx = 0; idx < nrings; idx++) {
    	txcount[idx] = 0;
    	rxcount[idx] = 0;
    }

    cnt = 0;
    if (!rxonly) {
        printf("Sending packets...\n");
        for (idx = 0; idx < nrings; idx++) {
            int tx_count = 0;
            for (i = 0; i < num_to_send; i++) {
                pkt = pm_pkt_alloc();
                if (pkt) {
                    if (verify_fast) {
                        fillData((uint16_t*) pkt->pkt, pktlen, i);
                    }
                    else if (use_dpat) {
                        fillCustomPat(dpat, pkt->pkt, scratch_data[tx_index], pktlen);
                    }
                    else if (use_data_pattern) {
                        fillDataPatRepeat(data_pattern, pkt->pkt, scratch_data[tx_index], pktlen);
                    }
                    else if (use_patid) {
                        fillDataPatByWord(patid, pkt->pkt, scratch_data[tx_index], pktlen);
                    }
                    else {		// random data
                        fillRandom(pkt->pkt,  scratch_data[tx_index], pktlen);
                        tx_index++;
                        if (tx_index >= (PM_RINGSIZE * PM_NRINGS)) tx_index = 0;
                        pkt_count = 0;
                    }
                    pkt->len = pktlen;

                    pm_add_txbuf(_pmo_rings[idx],pkt);
                    tx_count++;
                }
                else {
                    printf("WARNING--out of pkt store\n");
                }
            }
            if (tx_count > 0) {
                pm_start_tx(_pmo_rings[idx],tx_count);
            }
            txcount[idx] += num_to_send;
            cnt += num_to_send;
	    }
        printf("Sent %d packets total on %d queues\n",cnt,nrings);
	}


    if ((cont) || (b_exit_on_max)) {
        uint64_t start_time,end_time;
        uint64_t rate,ttlbits;
        int running = 1;

        start_time = SBREADCSR(A_BCM1480_SCD_ZBBUS_CYCLE_COUNT);

        /*
         * NOTE:  Below loop currently has RX check before check on TX done and 
         * resend.  This is needed in order to correctly order and track tx with
         * rx packets (tx_index will match rx_index packets).  The verify step will
         * ONLY work correctly if above ordering is mantained.    
         */
        while (running) 
        {
            for (idx = 0; idx < nrings; idx++) 
            {
                /* If an rx completed, put it back on the receive ring */
                while (pm_dequeue_rxbuf(_pmi_rings[idx],&pkt) &&
                       /* check for max packet count exit condition */
                       !( (b_exit_on_max) && (rxcnt >= max_packet_count))) 
                {
                    if (dump) {
                        printf("## Ring %d: Packet Received, len=%d\n",idx,
                               pkt->len);
                        dump_packet(pkt);
                    }
                    if (verify_fast) {
                        verifyData((uint16_t*) pkt->pkt, pkt->len);
                        // will print own error message
                    }
                    else if (verify) {
                        int i;
                        for (i=0; i<pkt->len; i++) {
                            if (pkt->pkt[i] != scratch_data[rx_index][i]) {
                                num_errors++;
                                printf("ERROR [%d] byte [%d] got 0x%x expected "
                                       "0x%x (%d pkts)\n",
                                       rx_index, i, pkt->pkt[i], scratch_data[rx_index][i],
                                       pkt_count);
                                pkt_count = 0;
                                if (context && num_errors <= num_context) {
                                    print_context(pkt->pkt, scratch_data[rx_index], i,
                                                  pktlen);
                                }
                            }
                        }
                        if (context && (num_errors >= num_context)) {
                            running = 0;
                        }
                        rx_index++;
                        pkt_count++;
                        if (rx_index >= (PM_RINGSIZE * PM_NRINGS)) rx_index = 0;
                    }
                    pm_add_rxbuf(_pmi_rings[idx],pkt);
                    pm_start_rx(_pmi_rings[idx],1);
                    rxcnt++;
                    rxcount[idx]++;
                    if ((rxcnt % print_interval) == 0) printf("%lld\n",rxcnt);
                }

                if (!rxonly) 
                {
                    /* Keep up attempting to send the requested queue depth */
                    for (i = 0; i < num_to_send; i++) 
                    {
                        /* If a tx completed, send it again */
                        if (pm_dequeue_txbuf(_pmo_rings[idx],&pkt))
                        {
                            pkt->len = randlen ? (16+(rand() %
                                                      (RANDOM_MAX_PKTSIZE-16))) : pktlen;

                            if (verify) {		// performance doesn't matter
                                if (use_dpat) {
                                    fillCustomPat(dpat, pkt->pkt, scratch_data[tx_index],
                                                  pktlen);
                                }
                                else if (use_data_pattern) {
                                    fillDataPatRepeat(data_pattern, pkt->pkt, scratch_data[tx_index], pktlen);
                                }
                                else if (use_patid) {
                                    fillDataPatByWord(patid, pkt->pkt, scratch_data[tx_index], pktlen);
                                }
                                else {		// test with new random data
                                    int j;
                                    for (j=0; j<pktlen; j++) {
                                        uint8_t byte = rand() & 0xff00 >> 8;
                                        scratch_data[tx_index][j] = byte;
                                        pkt->pkt[j] = byte;
                                    }
                                }

                                tx_index++;
                                if (tx_index >= (PM_RINGSIZE * PM_NRINGS)) tx_index = 0;
                            }

                            pm_add_txbuf(_pmo_rings[idx],pkt);
                            pm_start_tx(_pmi_rings[idx],1);
                            txcount[idx] += 1;
                        }
                    }
                }
            }

            statcnt++;
            if (statcnt > print_interval / 16) {
                if (console_status()) {
                    running = 0;
                    break;
                }
                statcnt = 0;
            }

            /* check for max packet count exit condition */
            if ((b_exit_on_max) && (rxcnt >= max_packet_count))
            {
                running = 0;
                break;
            }
	    }

        end_time = SBREADCSR(A_BCM1480_SCD_ZBBUS_CYCLE_COUNT);

        printf("%lld packets (%lld bytes) transferred in %lld ZClks\n",rxcnt,
               rxcnt*pktlen,end_time-start_time);

        ttlbits = rxcnt*pktlen*8;

        /* Zclk counts at 1 tick per 2 core clocks, or at cpu_mhz/2 */

        rate = (ttlbits * (cpu_mhz/2)) /  (end_time - start_time);

        printf("Rate: %lld Mbits/s,  %lld pkts/sec\n",rate,(rxcnt * 250000000)/(end_time-start_time));

	}
    else {
        printf("Receiving packets...\n");
	
        idx = 0;
        int running = 1;
        while (running && cnt > 0) {
            for (idx = 0; idx < nrings; idx++) {
                if (console_status()) {
                    running = 0;
                    break;
                }
                if (pm_dequeue_rxbuf(_pmi_rings[idx],&pkt)) {
                    if (dump) {
                        printf("## Packet Received, ring=%d len=%d\n",
                               idx, pkt->len);
                        dumpmem((hsaddr_t)(long)pkt->pkt,(hsaddr_t)(long)pkt->pkt,pkt->len,4);
                    }
                    cnt--;
                    pm_add_rxbuf(_pmi_rings[idx],pkt);
                    pm_start_rx(_pmi_rings[idx],1);
                    rxcount[idx]++;
                }
            }
	    }

        if ((cnt == 0)) printf("All packets were received\n");
	}

    printf("Packet statistics summary:\n");
    printf("	<id>  channel              tx pkts   rx pkts \n");
    for (idx = 0; idx < nrings; idx++) {
    	printf("	<%02d>  %s      %d        %d\n", idx, ringid[idx],
               txcount[idx], rxcount[idx]);
	}

    printf("	Number of verify errors:<%d>\n", num_errors);

    return 0;
}

static int ui_cmd_pm_showdescr(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int idx;
    pmring_t *pmi_ring;
    pmring_t *pmo_ring;
    char *x;

    if (!hsp_ports) {
	printf("PM not initialized.  Do 'pm init' first.\n");
	return -1;
	}

    x = cmd_getarg(cmd,0);
    if (!x) x = "0";
    idx = atoi(x);
    if (idx > PM_NRINGS) idx = PM_NRINGS;

    pmi_ring = pmi_rings[idx];
    pmo_ring = pmo_rings[idx];

    printf("PMI Queue %d---  base:%p, entries:%d, add/rem:%d/%d\n",pmi_ring->qno,
	   pmi_ring->dscr,pmi_ring->ringsize,pmi_ring->next_add,pmi_ring->next_rem);
    // hw = READCSR(A_BCM1480_PMI_LCL_REGISTER(pmi_ring->qno,R_BCM1480_PM_LAST));
    	// this register is broken in Ax/B0, not showing to avoid confusion

    for (idx = 0; idx < pmi_ring->ringsize; idx++) {
	printf(" PMI Dscr %d:  %016llX %016llX%s%s\n",idx,
	       pmi_ring->dscr[idx].dscr_a,
	       pmi_ring->dscr[idx].dscr_b,
	       (idx == pmi_ring->next_add) ? " <-add" : "",
	       (idx == pmi_ring->next_rem) ? " <-rem" : "");
	}

    printf("PMO Queue %d---  base:%p, entries:%d, add/rem:%d/%d\n",pmo_ring->qno,
	   pmo_ring->dscr,pmo_ring->ringsize,pmo_ring->next_add,pmo_ring->next_rem);

    for (idx = 0; idx < pmo_ring->ringsize; idx++) {
	printf(" PMO Dscr %d:  %016llX %016llX%s%s\n",idx,
	       pmo_ring->dscr[idx].dscr_a,
	       pmo_ring->dscr[idx].dscr_b,
	       (idx == pmo_ring->next_add) ? " <-add" : "",
	       (idx == pmo_ring->next_rem) ? " <-rem" : "");
	}
	   
    return 0;

}


static int ui_cmd_pm_route_forward(ui_cmdline_t *cmd,int argc,char *argv[])
{
//    char *x;
    uint32_t src_port, dest_port, i;

    if (!cmd_getarg(cmd,1)) return ui_showusage(cmd);
 
    src_port  = atoi(cmd_getarg(cmd,0));
    dest_port = atoi(cmd_getarg(cmd,1));

    if ( (src_port > 2) || (dest_port > 2) )
    {
        printf("Invalid source (%d)  or destination port(%d) index\n",
               src_port, dest_port);
        ui_showusage(cmd);
    }

    hr.base_addr = A_BCM1480_HR_BASE(src_port);

    for (i = 0; i < 16; i++)
    {
        hr_write_rule_ivc_port_forward(i, i, i, i, dest_port);
    }

    return 0;
}


/*  *********************************************************************
    *  ui_init_pmcmds()
    *  
    *  Add PM-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */


int ui_init_pmcmds(void)
{
    cmd_addcmd("pm init",
	       ui_cmd_pm_init,
	       NULL,
	       "initialize packet manager for some or all high speed ports (HSP)",
	       "pm init [0] [1] [2]",
	       "-map8;Map PMI/PMO to HSP 0,1,2 in groups of 8 (default)|"
	       "-map16;Map PMI/PMO to HSP 0,1 in groups of 16 (HSP 2 not used)|"
	       "-cross;RX-to-PMI map for cross-connected cables, HSP 0-1 only|"
	       "-chan=*;Active channels, for RAM alloc and SPI4 calendar (default 16)|"
	       "-no_rstat;Do not set RSTAT_POLARITY (to run in non-loopback mode)|"
	       "-mhz=*;Set SPI-4 clock freqency in MHz");

    cmd_addcmd("pm test",
	       ui_cmd_pm_test,
	       NULL,
	       "Test packet manager for some or all high speed ports (HSP)",
	       "pm test [0] [1] [2]",
	       "-dump;Dump received data|"
	       "-continuous;Run until a keystroke|"
	       "-count=*;Send the specified number of packets per pm queue|"
           "-max=*; Run until max packets are sent |" 
	       "-vc=*;Number of virtual channels per HSP (default 1, max 8 or 16)|"
	       "-vc_start=*;Start at this VC on each HSP (default 0)|"
	       "-randlen;Use random packet length|"
	       "-recv;Don't send, just receive|"
	       "-verify_fast;Verify packet data with stride pattern|"
	       "-verify;Verify packet data with random pattern|"
	       "-context=*;Print error bytes (only for -verify (-continuous or -max) mode)|"
	       "-patid=*;Data pattern id--see source code for values and meaning|"
	       "-data=*;Data pattern, 32-bits repeated in packet, e.g. -pat=0x0123abcd|"
	       "-pat=*;Data pattern, 16-bit, each bit specifies 16-bits of 0xffff or 0x0000|"
	       "-len=*;Use specified packet length");

    cmd_addcmd("pm hr",	
	       ui_cmd_pm_hr,
	       NULL,
	       "Hash and route functional test (requires \"pm init -map16 0\")",
	       "pm hr",
	       "-debug;Debug info including packet data");

    cmd_addcmd("pm showdescr",	
	       ui_cmd_pm_showdescr,
	       NULL,
	       "Show pending descriptors",
	       "pm showdescr",
	       "");

    cmd_addcmd("pm route forward",	
	       ui_cmd_pm_route_forward,
	       NULL,
	       "Set up routing to forward rcvd packets back out HSP ports",
	       "pm route forward <source_port> <destition_port>\n"
           "Set up switch to route all RX packets on a source port\n"
           "back a TX destination HSP port\n "
           "source_port (0-2) destination_port (0-2)",
	       "");

    return 0;
}



