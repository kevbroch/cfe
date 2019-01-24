/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    * HSP Configuration file			File:   bcm1480_hsp_utils.c
    *  
    * This file contains initalization and configuration routines for 
    * the BCM1480 High Speed Port Interface.  This routine will
    * be responsible for  managing the allocation of TX/RX buffer space
    * and initialization of associate -phil cnt' registers.   
    * 
    * This file have been originally developed under the CFE environment.
    * 
    *  Code was leveraged from 'bcm1480_pci_machdep.c' and 'ui_pmcmd.c' 
    *
    *  Author:  Kean Hurley
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2004,2005,2006
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


/***********************************************************
*
* Included files
*
***********************************************************/
#include "cfe.h"
#include "sbmips.h"

#include "lib_physio.h"

#include "pcivar.h"
#include "pci_internal.h"
#include "pcireg.h"
#include "ldtreg.h"
#include "pci_cfg.h"

#include "bcm1480_regs.h"
#include "bcm1480_ht.h"
#include "bcm1480_hsp.h"
#include "bcm1480_scd.h"

#include "bcm1480_hsp_utils.h"

/***********************************************************
*
* Global variables
*
***********************************************************/

/***********************************************************
*
* Defined values, enumerated types, macros, typedefs
* internal to this file
*
***********************************************************/

#if 0 // For future allocation manager
typedef struct bcm1480_hsp_buf_mgr
{
    uint32_t cli_alloc_flags[BCM1480_HT_NUM_PORTS];
    uint32_t free_rx_buf_idx[BCM1480_HT_NUM_PORTS];
    uint32_t free_tx_buf_idx[BCM1480_HT_NUM_PORTS];
}  bcm1480_hsp_buf_mgr_t;

typedef struct bcm1480_hsp_io_buf_info 
{
    uint32_t rx_npc_cmd_bufs;
    uint32_t rx_npc_data_bufs;
    uint32_t rx_pc_cmd_bufs;
    uint32_t rx_pc_data_bufs;
    uint32_t rx_rsp_data_bufs;
    uint32_t tx_npc_bufs;
    uint32_t tx_pc_bufs;
    uint32_t tx_rsp_bufs;
} bcm1480_hsp_buf_io_info_t;


typedef struct bcm1480_hsp_ccnuma_buf_info 
{
    uint32_t rx_prb_bufs;
    uint32_t rx_ack_bufs;
    uint32_t rx_wb_bufs;
    uint32_t rx_cfill_bufs;
    uint32_t rx_crd_bufs;
    uint32_t tx_prb_bufs;
    uint32_t tx_ack_bufs;
    uint32_t tx_wb_bufs;
    uint32_t tx_cfill_bufs;
    uint32_t tx_crd_bufs;
} bcm1480_hsp_ccnuma_buf_info_t;

#endif

#define WRITECSR(x,y) phys_write64(x,y)
#define READCSR(x) phys_read64(x)

#define CALENDAR_M 	1	// there is a hw bug--can't have LEN too small
#define CALENDAR_LEN 	16	// we keep this 16 even for fewer channels

#define REFCLK 		100	/* in MHz */

/***********************************************************
*
* Static variables
*
***********************************************************/
// Documented


/***********************************************************
*
* Function prototypes of static functions
*
***********************************************************/
#ifndef DOXYGEN_IGNORE_ALWAYS


#endif


/***********************************************************
*
* Functions
*
***********************************************************/

/*
 * HT port initialization.  See Section 13, pp. 479-480.
 */

/**
    @fn hsp_ht_ram_alloc ( uint32_t port );
*/
void hsp_ht_ram_alloc ( uint32_t port )
{
    /* HSP: Allocate RX and TX buffers.  */
    /* NB: Floor and ceiling fields in all ramalloc registers are
       swapped relative to the original documentation.  */

    /* The following allocations for HT i/o and ccnuma are recommended
       defaults per PR3491 and PR3492 */

    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_HT_RAMALLOC_0),
		 (V_BCM1480_HSP_RX_NPC_CMD_FLOOR(0x300)   |
		  V_BCM1480_HSP_RX_NPC_CMD_CEILING(0x307)));

#ifdef _BCM1480_PASS1_WORKAROUNDS_
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_HT_RAMALLOC_1),
		 (V_BCM1480_HSP_RX_PC_CMD_FLOOR(0x348)    |
		  V_BCM1480_HSP_RX_PC_CMD_CEILING(0x34F)  |
		  V_BCM1480_HSP_RX_PRB_FLOOR(0x000)       |
		  V_BCM1480_HSP_RX_PRB_CEILING(0x000)));
#else
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_HT_RAMALLOC_1),
		 (V_BCM1480_HSP_RX_PC_CMD_FLOOR(0x348)    |
		  V_BCM1480_HSP_RX_PC_CMD_CEILING(0x34F)  |
		  V_BCM1480_HSP_RX_PRB_FLOOR(0x378)       |
		  V_BCM1480_HSP_RX_PRB_CEILING(0x387)));
#endif

    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_HT_RAMALLOC_2),
		 (V_BCM1480_HSP_RX_ACK_FLOOR(0x388)       |
		  V_BCM1480_HSP_RX_ACK_CEILING(0x397)     |
		  V_BCM1480_HSP_RX_WB_FLOOR(0x398)        |
		  V_BCM1480_HSP_RX_WB_CEILING(0x3C1)));
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_HT_RAMALLOC_3),
		 (V_BCM1480_HSP_RX_CFILL_FLOOR(0x3C2)     |
		  V_BCM1480_HSP_RX_CFILL_CEILING(0x3EE)   |
		  V_BCM1480_HSP_RX_CRD_FLOOR(0x3EF)       |
		  V_BCM1480_HSP_RX_CRD_CEILING(0x3FF)));
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_HT_RAMALLOC_4),
		 (V_BCM1480_HSP_RX_NPC_DAT_FLOOR(0x308)   |
		  V_BCM1480_HSP_RX_NPC_DAT_CEILING(0x32F) |
		  V_BCM1480_HSP_RX_RSP_DAT_FLOOR(0x330)   |
		  V_BCM1480_HSP_RX_RSP_DAT_CEILING(0x347)));
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_HT_RAMALLOC_5),
		 (V_BCM1480_HSP_RX_PC_DAT_FLOOR(0x350)    |
		  V_BCM1480_HSP_RX_PC_DAT_CEILING(0x377)));

    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_NPC_RAMALLOC),
		 V_BCM1480_HSP_TX_NPC_FLOOR(0xC0) |        
		 V_BCM1480_HSP_TX_NPC_CEILING(0xC9));
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_RSP_RAMALLOC),
        V_BCM1480_HSP_TX_RSP_FLOOR(0xCA) |    /* KPH: (D1 - CA) + 1 = 8, which is good   */
        V_BCM1480_HSP_TX_RSP_CEILING(0xD1));  /* need to be divs by 4 (RSP sz 4*16)      */
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_PC_RAMALLOC),
		 V_BCM1480_HSP_TX_PC_FLOOR(0xD3) |    /* KPH: (E1 - D3) + 1 = F, which is good   */
		 V_BCM1480_HSP_TX_PC_CEILING(0xE1));  /* need to be divs by 5 (PC sz 5*16)       */ 
                                              /* D2 unused,  but no bigge for now        */
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_HTCC_RAMALLOC_0),
		 V_BCM1480_HSP_TX_PRB_FLOOR(0xE2)   |
		 V_BCM1480_HSP_TX_PRB_CEILING(0xE5) |
		 V_BCM1480_HSP_TX_ACK_FLOOR(0xE6)   |
		 V_BCM1480_HSP_TX_ACK_CEILING(0xE9));
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_HTCC_RAMALLOC_1),
		 V_BCM1480_HSP_TX_WB_FLOOR(0xEA)    |
		 V_BCM1480_HSP_TX_WB_CEILING(0xF2)  |
		 V_BCM1480_HSP_TX_CFILL_FLOOR(0xF3) |
		 V_BCM1480_HSP_TX_CFILL_CEILING(0xFB));
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_HTCC_RAMALLOC_2),
		 V_BCM1480_HSP_TX_CRD_FLOOR(0xFC)   |
		 V_BCM1480_HSP_TX_CRD_CEILING(0xFF));
}


/**
    @fn void hsp_ht_ram_alloc ( uint32_t port );
*/
void hsp_ht_flow_alloc (uint32_t port)
{
    /* HSP: Program the flow control registers.  */

    /* The following allocations for HT i/o and ccnuma are recommended
       faults per PR3491 and PR3492. */

    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_HTIO_RXPHITCNT),
		 (V_BCM1480_HSP_RX_NPC_CMD_PHITCNT(0x08) |
		  V_BCM1480_HSP_RX_NPC_DAT_PHITCNT(0x08) |
		  V_BCM1480_HSP_RX_RSP_DAT_PHITCNT(0x06) |
		  V_BCM1480_HSP_RX_PC_CMD_PHITCNT(0x08)  |
		  V_BCM1480_HSP_RX_PC_DAT_PHITCNT(0x08)));

#ifdef _BCM1480_PASS1_WORKAROUNDS_
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_HTCC_RXPHITCNT),
		 (V_BCM1480_HSP_TX_HTCC_PRB_PHITCNT(0x01)     |
		  V_BCM1480_HSP_TX_HTCC_ACK_PHITCNT(0x10)     |
		  V_BCM1480_HSP_TX_HTCC_WB_PHITCNT(0x0E)      |
		  V_BCM1480_HSP_TX_HTCC_CFILL_PHITCNT(0x0F)   |
		  V_BCM1480_HSP_TX_HTCC_CRD_PHITCNT(0x11)));
#else
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_HTCC_RXPHITCNT),
		 (V_BCM1480_HSP_TX_HTCC_PRB_PHITCNT(0x10)     |
		  V_BCM1480_HSP_TX_HTCC_ACK_PHITCNT(0x10)     |
		  V_BCM1480_HSP_TX_HTCC_WB_PHITCNT(0x0E)      |
		  V_BCM1480_HSP_TX_HTCC_CFILL_PHITCNT(0x0F)   |
		  V_BCM1480_HSP_TX_HTCC_CRD_PHITCNT(0x11)));
#endif

    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_HTIO_TXPHITCNT),
		 (V_BCM1480_HSP_TX_NPC_PHITCNT(0x0A)     |
          V_BCM1480_HSP_TX_RSP_PHITCNT(0x08)     |  /* KPH: 2 * (4 RSP sz 4*16)    */
		  V_BCM1480_HSP_TX_PC_PHITCNT(0x0F)));      /* KPH: 3 * (5 PC Cmd sz 5*16) */
    WRITECSR(A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_HTCC_TXPHITCNT),
		 (V_BCM1480_HSP_TX_HTCC_PRB_PHITCNT(0x04)     |
		  V_BCM1480_HSP_TX_HTCC_ACK_PHITCNT(0x04)     |
		  V_BCM1480_HSP_TX_HTCC_WB_PHITCNT(0x09)      |
		  V_BCM1480_HSP_TX_HTCC_CFILL_PHITCNT(0x09)   |
		  V_BCM1480_HSP_TX_HTCC_CRD_PHITCNT(0x04)));

   /* mpl: setting POHT up here */

    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_PKT_RAMALLOC(0)),
		 V_BCM1480_HSP_RX_RAMCEILING_0(0x03F)|
		 V_BCM1480_HSP_RX_RAMFLOOR_0(0x000));

    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_PKT_RAMALLOC(0)),
		 V_BCM1480_HSP_TX_RAMCEILING_0(0x00F)|
		 V_BCM1480_HSP_TX_RAMFLOOR_0(0x000));
    
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_PKT_RXPHITCNT(0)),
		 V_BCM1480_HSP_RX_PHITCNT_0(0x10));

    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_PKT_TXPHITCNT(0)),
		 V_BCM1480_HSP_TX_PHITCNT_0(0x10));
}

/**
    @fn uint32_t hsp_ht_init (void)
*/
uint32_t hsp_ht_init (void)
{
    int i;
    unsigned int ht_map;

    /* HSP: Discover the ports in HT mode. */
    ht_map = 0;
    for (i = 0; i < BCM1480_HT_NUM_PORTS; i++) {
        if ( HSP_IS_PORT_HT_MODE(i) )
        {
            ht_map |= (1 << i);
        }
	}

    /* HSP: Allocate RX and TX buffers.  */
    for (i = 0; i < BCM1480_HT_NUM_PORTS; i++) {
	if (HSP_PORT_MODE_HT(ht_map, i))
	    hsp_ht_ram_alloc(i);
	}

    /* HSP: Program the flow control registers.  */
    for (i = 0; i < BCM1480_HT_NUM_PORTS; i++) {
	if (HSP_PORT_MODE_HT(ht_map, i))
	    hsp_ht_flow_alloc(i);
	}

    return ht_map;
}

/**
    @fn void hsp_pm_ram_n_flow_alloc(uint32_t port_idx, uint32_t num_active );
*/
void hsp_pm_ram_n_flow_alloc(uint32_t port, uint32_t num_active )
{
    int total_tx_ram,total_rx_ram;
    int per_tx_channel,per_rx_channel;
    int i;
    int cur_floor;
    uint64_t ramalloc[8];
    uint64_t phitcnt[4];
    uint64_t watermark[8];
    int almostfull,almostempty;

    /* XXX this relies on a hard-coded agreement between the HT init code
    	(same as the PCI init) and the code here.  We know that HT init
	has put all the ccNUMA and PoHT at 0x300 and above for RX and
	0xc0 and above for TX. */

    total_rx_ram = 1024 - 256;		/* number of 16-byte blocks */
    total_tx_ram = 256 - 64;		/* number of 16-byte blocks */

    per_tx_channel = total_tx_ram / num_active;
    per_rx_channel = total_rx_ram / num_active;

    /*
     * Set the "almost full" watermark to 512 bytes (32 blocks) or 1/2 the
     * per channel allocation, whichever is smaller.  Set the "almost empty"
     * watermark to 1/4 the allocation.
     */

    almostfull = per_rx_channel / 2;
    if (almostfull > 32) almostfull = per_rx_channel - 32;

    almostempty = per_rx_channel / 4;

    /* This shouldn't happen, but we want to prevent the chip from going insane. */
    if (almostempty > almostfull) almostempty = almostfull / 2;


    printf("** TX: alloc=%d    RX: alloc=%d full=%d empty=%d\n",per_tx_channel,
	   per_rx_channel,almostfull,almostempty);

    /*
     * Do RX side
     */

    cur_floor = 0;
    memset(ramalloc,0,sizeof(ramalloc));
    memset(phitcnt,0,sizeof(phitcnt));
    memset(watermark,0,sizeof(watermark));

    for (i = 0; i < num_active; i++) {
	switch (i & 1) {
	    case 0:
		ramalloc[i/2] |= V_BCM1480_HSP_RX_RAMCEILING_0(cur_floor+per_rx_channel-1)|
		    V_BCM1480_HSP_RX_RAMFLOOR_0(cur_floor);
		break;
	    case 1:
		ramalloc[i/2] |= V_BCM1480_HSP_RX_RAMCEILING_1(cur_floor+per_rx_channel-1)|
		    V_BCM1480_HSP_RX_RAMFLOOR_1(cur_floor);
		break;
	    }

	switch (i & 1) {
	    case 0:
		watermark[i/2] |= V_BCM1480_HSP_RX_ALMOSTEMPTY_EVEN(almostempty) |
		    V_BCM1480_HSP_RX_ALMOSTFULL_EVEN(almostfull);
		break;
	    case 1:
		watermark[i/2] |= V_BCM1480_HSP_RX_ALMOSTEMPTY_ODD(almostempty) |
		    V_BCM1480_HSP_RX_ALMOSTFULL_ODD(almostfull);
		break;
	    }

	switch (i & 3) {
	    case 0:
		phitcnt[i/4] |= V_BCM1480_HSP_RX_PHITCNT_0(per_rx_channel/4);
		break;
	    case 1:
		phitcnt[i/4] |= V_BCM1480_HSP_RX_PHITCNT_1(per_rx_channel/4);
		break;
	    case 2:
		phitcnt[i/4] |= V_BCM1480_HSP_RX_PHITCNT_2(per_rx_channel/4);
		break;
	    case 3:
		phitcnt[i/4] |= V_BCM1480_HSP_RX_PHITCNT_3(per_rx_channel/4);
		break;
	    }

	cur_floor += per_rx_channel;

	}

    /*
     * Now transfer these calculations into the RX registers.
     */

    for (i = 0; i < 8; i++) {
	WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_PKT_RAMALLOC(i)),ramalloc[i]);
	}

    for (i = 0; i < 8; i++) {
	WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI_WATERMARK(i)),watermark[i]);
	}

    for (i = 0; i < 4; i++) {
	WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_PKT_RXPHITCNT(i)),phitcnt[i]);
	}

    /*
     * Now do the TX side.
     */


    cur_floor = 0;
    memset(ramalloc,0,sizeof(ramalloc));
    memset(phitcnt,0,sizeof(phitcnt));

    for (i = 0; i < num_active; i++) {
	if (i & 1) {
	    ramalloc[i/2] |= V_BCM1480_HSP_TX_RAMCEILING_1(cur_floor+per_tx_channel-1)|
		V_BCM1480_HSP_TX_RAMFLOOR_1(cur_floor);
	    }
	else {
	    ramalloc[i/2] |= V_BCM1480_HSP_TX_RAMCEILING_0(cur_floor+per_tx_channel-1)|
		V_BCM1480_HSP_TX_RAMFLOOR_0(cur_floor);
	    }

	switch (i & 3) {
	    case 0:
		phitcnt[i/4] |= V_BCM1480_HSP_TX_PHITCNT_0(per_tx_channel);
		break;
	    case 1:
		phitcnt[i/4] |= V_BCM1480_HSP_TX_PHITCNT_1(per_tx_channel);
		break;
	    case 2:
		phitcnt[i/4] |= V_BCM1480_HSP_TX_PHITCNT_2(per_tx_channel);
		break;
	    case 3:
		phitcnt[i/4] |= V_BCM1480_HSP_TX_PHITCNT_3(per_tx_channel);
		break;
	    }

	cur_floor += per_tx_channel;

	}


    /*
     * Now transfer these calculations into the TX registers.
     */

    for (i = 0; i < 8; i++) {
	WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_PKT_RAMALLOC(i)),ramalloc[i]);
	}

    for (i = 0; i < 4; i++) {
	WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_PKT_TXPHITCNT(i)),phitcnt[i]);
	}

}


/**
    @fn void hsp_ht_port_link_reset( uint32_t port );
*/
void hsp_ht_port_link_reset( uint32_t port )
{
    volatile pcireg_t cmd, ctrl;  

    if (port > BCM1480_HT_NUM_PORTS)
    {
        printf("ht_port_link_reset: Invalid port %d parameter\n");
        goto ERROR_EXIT;
    }

    /* 
     * Now issue a soft reset to force the link to renegotiate the new speed and reestablish
     * credit
     */
    ctrl = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_LDT_BRCMD);
    ctrl |= LDT_COMMAND_WARM_RESET;
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), R_LDT_BRCMD, ctrl);
    cmd = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), PPB_BRCTL_INTERRUPT_REG);
    cmd |= M_PPB_BRCTL_SECBUSRESET;
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), PPB_BRCTL_INTERRUPT_REG, cmd);

    cfe_usleep(1000);

    cmd = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), PPB_BRCTL_INTERRUPT_REG);
    cmd &= ~M_PPB_BRCTL_SECBUSRESET;
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), PPB_BRCTL_INTERRUPT_REG, cmd);

ERROR_EXIT:
    
    return;
}


/**
    @fn void hsp_spi4_init_port(uint32_t port,
                        uint32_t mhz,
                        uint32_t active_channels,
                        uint32_t rstat);
*/
void hsp_spi4_init_port(uint32_t port,
                        uint32_t mhz,
                        uint32_t active_channels,
                        uint32_t rstat)
{
    uint64_t value;
    int spi4_pllmult;
    uint64_t cpu_mhz;
    uint64_t sw_mhz;
    uint64_t spi4_mhz = mhz;

    cpu_mhz = G_BCM1480_SYS_PLL_DIV(READCSR(A_SCD_SYSTEM_CFG)) * (REFCLK/2);
    sw_mhz = cpu_mhz * 2 / G_BCM1480_SYS_SW_DIV(READCSR(A_SCD_SYSTEM_CFG));

    spi4_pllmult = spi4_mhz / (REFCLK/2);

    /* enable the calendar */

    uint64_t v0, v1;

    // do all 16 if we're doing more than one (could optimize
    // further if we really need to, for 2 VCs for example)
    if (active_channels > 1) {
    	v0 = 0x0706050403020100LL;
    	v1 = 0x0F0E0D0C0B0A0908LL;
    }
    else {
    	v0 = 0;		// VC 0 for all 16 calendar slots
    	v1 = 0;
    }
    printf("writing calendar 0x%x 0x%x\n", v0, v1);
    // most general calendar--each VC gets a slot
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_CALENDAR_0),v0);
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_CALENDAR_1),v1);

    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CALENDAR_0),v0);
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CALENDAR_1),v1);

    value = V_BCM1480_HSP_RX_CALENDAR_LEN(CALENDAR_LEN) |
		V_BCM1480_HSP_RX_CALENDAR_M(CALENDAR_M) |
		V_BCM1480_HSP_RX_ALPHA(4000);

    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CFG_1),value);

    value = V_BCM1480_HSP_TX_MAXBURST1(8) |
		V_BCM1480_HSP_TX_MAXBURST2(4) |
		V_BCM1480_HSP_TX_CALENDAR_LEN(CALENDAR_LEN) |
		V_BCM1480_HSP_TX_CALENDAR_M(CALENDAR_M) |
		V_BCM1480_HSP_TX_ALPHA(4000) |
		V_BCM1480_HSP_TX_ACTIVE_CHANNELS(active_channels);

    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_CFG_1),value);

    /* Enable status register */
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_PORT_INT_EN),
		     (M_BCM1480_HSP_TX_INT_TSTATTIMEOUT |
		      M_BCM1480_HSP_TX_INT_DIP2RXERR |
		      M_BCM1480_HSP_TX_INT_SPI4RESET));
		      
    /* Watch for all RX errors */
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_PORT_INT_EN),
		     (M_BCM1480_HSP_RX_INT_PERVCERR |
		      M_BCM1480_HSP_RX_INT_EOPABORT |
		      M_BCM1480_HSP_RX_INT_SPI4PROTOERR |
		      M_BCM1480_HSP_RX_INT_ESTOREOVERFLOW |
		      M_BCM1480_HSP_RX_INT_ALPHATRAINERR |
		      M_BCM1480_HSP_RX_INT_DIP4ERROR |
		      M_BCM1480_HSP_RX_INT_HRERROR |
		      M_BCM1480_HSP_RX_INT_INTOVERFLOW));


    /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* 
     * Do the RX SPI4 config register.  Be sure to enable divide-by-4 mode
     * if running < 200Mhz
     */

    value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CFG_0));

    value &= ~(M_BCM1480_HSP_RX_PLL_MULTIPLIER | M_BCM1480_HSP_RX_PLL_DIV_4);

    if (spi4_mhz < 200) {
		value |= V_BCM1480_HSP_RX_PLL_MULTIPLIER(spi4_pllmult*4) | M_BCM1480_HSP_RX_PLL_DIV_4;
		}
    else {
		value |= V_BCM1480_HSP_RX_PLL_MULTIPLIER(spi4_pllmult);
		}

    value &= ~_SB_MAKEMASK1(1);		/* Clear reserved bit */

    if (rstat) {
		/* For loopbackEnable either RX or TX rstat polarity, not both */
        value |= M_BCM1480_HSP_RSTAT_POLARITY;
		printf("HSP[%d] Setting M_BCM1480_HSP_RSTAT_POLARITY for "
			"loopback\n", port);
        }

    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CFG_0),value);

    /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /*
     * TX SPI4 Training Format Register 
     */
 
    /* The following register values makes TX to send some data out!! Please don't change!! */
    value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_TRAINING_FMT));
    value &= ~M_BCM1480_HSP_TX_TXPREFBURSTSZ;
    value |= V_BCM1480_HSP_TX_TXPREFBURSTSZ(2);
    value &= ~M_BCM1480_HSP_TX_TXMAXBURSTSZ;
    value |= V_BCM1480_HSP_TX_TXMAXBURSTSZ(5);
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_TRAINING_FMT),value);
    value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_TRAINING_FMT));


    /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /*
     * Fix up the calibration registers.  For TX, turn off the "STARTCAL2"
     * bit.  It shouldn't be set, but it is.  For RX, disable "No Calibration"
     * mode to allow the RX side to do its calibrations.
     */

    value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_CALIBRATION));
    value &= ~M_BCM1480_HSP_CAL_STARTCAL2;
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_CALIBRATION),value);

    value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_CALIBRATION));
    value &= ~M_BCM1480_HSP_CAL_NO_CALIB;
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_CALIBRATION),value);

    /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /*
     * Do the SPI4 TX Config register 0.  This register contains PLL settings
     * and error count tolerances.  
     */

    value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_CFG_0));
    value &= ~M_BCM1480_HSP_TX_RST_STATCNT;  
    value |= V_BCM1480_HSP_TX_RST_STATCNT(0);

    /*
     * For the PLL, make sure the ratios are all sane.  If the spi4 clock
     * is less than 200MHz, be sure to turn on the "divide-by-4" mode
     * and run the PLL 4x faster.
     */

    value &= ~(M_BCM1480_HSP_TX_PLL_MULTIPLIER | M_BCM1480_HSP_TX_TX_PLL_DIV_4);

    if (spi4_mhz < 200) {
		value |= V_BCM1480_HSP_TX_PLL_MULTIPLIER(spi4_pllmult*4) | M_BCM1480_HSP_TX_TX_PLL_DIV_4;
		}
    else {
		value |= V_BCM1480_HSP_TX_PLL_MULTIPLIER(spi4_pllmult);
		}

    /* 
     * TSTATCLK is 1/4 the SPI clock.  If (TSTATCLK/4)/SWCLK is < 1/4, turn 
     * on TSTAT slow mode.
     */
    if ((spi4_mhz / sw_mhz) < 4) {
		value |= M_BCM1480_HSP_TX_TSTAT_SLOW_MODE;
		}

    /*  'n' DIP-2 Errors causes TX to go to Link Training */
    value &= ~M_BCM1480_HSP_TX_DIP2_ERRLIMIT;
    value |= V_BCM1480_HSP_TX_DIP2_ERRLIMIT(1);
//	    value |= M_BCM1480_HSP_TX_TSTAT_POLARITY;


    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_CFG_0),value);

    /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

    cfe_usleep(10000);

    /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

            /* 
     * Bring ports out of reset.
     */

    value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_CFG_0));
    value &= ~M_BCM1480_HSP_TX_PORT_RESET;
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_CFG_0),value);

            /* Enable RX Port */	    
    value = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CFG_0));
    value &= ~M_BCM1480_HSP_RX_PORT_RESET;
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_CFG_0),value);
}


/**
    @fn uint32_t hsp_ht_reset_errors(uint32_t port);
*/
uint32_t hsp_ht_reset_errors(uint32_t port)
{
    pcireg_t csr;

    /*
     * Clear standard PCI errors on primary status register 
     */
    csr = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), PCI_COMMAND_STATUS_REG);
    csr |= PCI_STATUS_PARITY_ERROR | PCI_STATUS_SYSTEM_ERROR |
           PCI_STATUS_MASTER_ABORT | PCI_STATUS_MASTER_TARGET_ABORT |
           PCI_STATUS_TARGET_TARGET_ABORT | PCI_STATUS_PARITY_DETECT;
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), PCI_COMMAND_STATUS_REG, csr);

    /*
     * Clear standard PCI errors on secondary (bridge) status register 
     */
    csr = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), PPB_IO_STATUS_REG);
    csr |= PCI_STATUS_PARITY_ERROR | PCI_STATUS_SYSTEM_ERROR |
           PCI_STATUS_MASTER_ABORT | PCI_STATUS_MASTER_TARGET_ABORT |
           PCI_STATUS_TARGET_TARGET_ABORT | PCI_STATUS_PARITY_DETECT;
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), PPB_IO_STATUS_REG, csr);


    /*
     * Clear Link Fail and CRC status bits in bridge link status register
     */
    csr = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKCTRL);
    csr |= M_BCM1480_HTB_LINKCTRL_LINKFAIL | M_BCM1480_HTB_LINKCTRL_CRCERR;
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKCTRL, csr);


    /*
     * Clear Protocol, Overflow, and End of Chain status bits 
     */
    csr = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKFREQERR);
    csr |= M_BCM1480_HTB_LINKFQERR_PROTERR | M_BCM1480_HTB_LINKFQERR_OVFLERR |
        M_BCM1480_HTB_LINKFQERR_EOCERR;
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKFREQERR, csr);


    /*
     * Clear Chain Fail(Sync flood) and Respond error status bits 
     */
    csr = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_ERRHNDL);
    csr |= M_BCM1480_HTB_ERRHNDL_CHNFAIL | M_BCM1480_HTB_ERRHNDL_RSPERR;
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_ERRHNDL, csr);

    return (0);
}


/**
    uint32_t hsp_ht_enable_sync_flood_on_errors(uint32_t port, uint32_t enable);
*/
uint32_t hsp_ht_enable_sync_flood_on_errors(uint32_t port, uint32_t enable)
{
    pcireg_t csr;

    /*
     * Enable / Disable Sync Flood on CRC  
     */
    csr = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKCTRL);
    if (enable)
    {
        csr |= M_BCM1480_HTB_LINKCTRL_CRCFLEN;
    }
    else
    {
        csr &= ~M_BCM1480_HTB_LINKCTRL_CRCFLEN;
    }
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKCTRL, csr);

    /*
     * Enable / Disable Sync Flood on Protocol and Overflow errors  
     */
    csr = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_ERRHNDL);
    if (enable)
    {
        csr |= M_BCM1480_HTB_ERRHNDL_PROFLEN | M_BCM1480_HTB_LINKFQERR_OVFLERR ;
    }
    else
    {
        csr &= ~(M_BCM1480_HTB_ERRHNDL_PROFLEN | M_BCM1480_HTB_LINKFQERR_OVFLERR);
    }
    pci_conf_write32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_ERRHNDL, csr);

    return (0);

}


/**
    uint32_t hsp_ht_check_for_errors(uint32_t port);
*/
uint32_t hsp_ht_check_for_errors(uint32_t port)
{
    pcireg_t reg;
    uint32_t b_error = 0;

    /*
     * Check standard PCI errors on primary status register 
     */
    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), PCI_COMMAND_STATUS_REG);
    b_error |= ( reg & (PCI_STATUS_PARITY_ERROR | PCI_STATUS_SYSTEM_ERROR |
                       PCI_STATUS_MASTER_ABORT | PCI_STATUS_MASTER_TARGET_ABORT |
                       PCI_STATUS_TARGET_TARGET_ABORT | PCI_STATUS_PARITY_DETECT)) ? 1 : 0;
    /*
     * Check standard PCI errors on secondary (bridge) status register 
     */
    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), PPB_IO_STATUS_REG);
    b_error |= ( reg & (PCI_STATUS_PARITY_ERROR | PCI_STATUS_SYSTEM_ERROR |
                       PCI_STATUS_MASTER_ABORT | PCI_STATUS_MASTER_TARGET_ABORT |
                       PCI_STATUS_TARGET_TARGET_ABORT | PCI_STATUS_PARITY_DETECT)) ? 1 : 0;

    /*
     * Check Link Fail and CRC status bits in bridge link status register
     */
    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKCTRL);
    b_error |= ( reg & ( M_BCM1480_HTB_LINKCTRL_LINKFAIL | M_BCM1480_HTB_LINKCTRL_CRCERR)) ? 1 : 0;

    /*
     * Check Protocol, Overflow, and End of Chain status bits 
     */
    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKFREQERR);
    b_error |= ( reg & ( M_BCM1480_HTB_LINKFQERR_PROTERR | M_BCM1480_HTB_LINKFQERR_OVFLERR |
        M_BCM1480_HTB_LINKFQERR_EOCERR)) ? 1 : 0;

    /*
     * Check Chain Fail(Sync flood) and Respond error status bits 
     */
    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_ERRHNDL);
     b_error |= ( reg & ( M_BCM1480_HTB_ERRHNDL_CHNFAIL | M_BCM1480_HTB_ERRHNDL_RSPERR )) ? 1 : 0;

    return (b_error);    
}



/**
    uint32_t hsp_spi4_reset_errors(uint32_t port)
*/
uint32_t hsp_spi4_reset_errors(uint32_t port)
{
    
    /* Clear TX irq status register */
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_PORT_INT_STATUS),
		     (M_BCM1480_HSP_TX_INT_TSTATTIMEOUT |
		      M_BCM1480_HSP_TX_INT_DIP2RXERR |
		      M_BCM1480_HSP_TX_INT_SPI4RESET));
		      
    /* Clear RX irq for all RX errors */
    WRITECSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_PORT_INT_STATUS),
		     (M_BCM1480_HSP_RX_INT_PERVCERR |
		      M_BCM1480_HSP_RX_INT_EOPABORT |
		      M_BCM1480_HSP_RX_INT_SPI4PROTOERR |
		      M_BCM1480_HSP_RX_INT_ESTOREOVERFLOW |
		      M_BCM1480_HSP_RX_INT_ALPHATRAINERR |
		      M_BCM1480_HSP_RX_INT_DIP4ERROR |
		      M_BCM1480_HSP_RX_INT_HRERROR |
		      M_BCM1480_HSP_RX_INT_INTOVERFLOW));    

    return(0);
}





/**
    uint32_t hsp_spi4_check_for_errors(uint32_t port)
*/
uint32_t hsp_spi4_check_for_errors(uint32_t port)
{
    uint32_t b_error = 0;
    uint64_t csr;


    /* Check TX irq status register */
    csr = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_PORT_INT_STATUS) );
        
    b_error |= (csr & (M_BCM1480_HSP_TX_INT_TSTATTIMEOUT |
                       M_BCM1480_HSP_TX_INT_DIP2RXERR |
                       M_BCM1480_HSP_TX_INT_SPI4RESET)) ? 1 : 0;
		      
    /* Check RX irq for all RX errors */
    csr = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_PORT_INT_STATUS) );

    b_error |= (csr & (M_BCM1480_HSP_RX_INT_PERVCERR |
                        M_BCM1480_HSP_RX_INT_EOPABORT |
                        M_BCM1480_HSP_RX_INT_SPI4PROTOERR |
                        M_BCM1480_HSP_RX_INT_ESTOREOVERFLOW |
                        M_BCM1480_HSP_RX_INT_ALPHATRAINERR |
                        M_BCM1480_HSP_RX_INT_DIP4ERROR |
                        M_BCM1480_HSP_RX_INT_HRERROR |
                        M_BCM1480_HSP_RX_INT_INTOVERFLOW)) ? 1 : 0;    
     
     
    return (b_error);
}



