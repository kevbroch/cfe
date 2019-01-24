/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  USB Ethernet				File: usbeth.h
    *  
    *  Driver for USB Ethernet devices.
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2005
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

#ifndef _USBETH_H__
#define _USBETH_H__

/*  *********************************************************************
    *  USB-Ethernet adapter driver includes
    ********************************************************************* */

/* **************************************
   *  CATC Netmate adapter
   ************************************** */

#define CATC_MCAST_TBL_ADDR         0xFA80      //in Netmate's SRAM

#define CATC_GET_MAC_ADDR           0xF2
#define CATC_SET_REG                0xFA
#define CATC_GET_REG                0xFB
#define CATC_SET_MEM                0xFC

#define CATC_TX_BUF_CNT_REG         0x20
#define CATC_RX_BUF_CNT_REG         0x21
#define CATC_ADV_OP_MODES_REG       0x22
#define CATC_RX_FRAME_CNT_REG       0x24

#define CATC_ETH_CTRL_REG           0x60
#define CATC_ENET_STATUS_REG        0x61
#define CATC_ETH_ADDR_0_REG         0x67        // Byte #0 (leftmost)
#define CATC_LED_CTRL_REG           0x81


/* *******************************************************************
   * Admtek ADM8511 (Pegasus II) register and bit definitions.
   *
   * Reference:
   *   ADM 8511 Pegasus II USB / Fast Ethernet with MII Interface,
   *     Version 2.04
   *   ADMtek Incorporated, Hsinchu, Taiwan
   ******************************************************************* */

#define PEG_SET_REG                0xF1
#define PEG_GET_REG                0xF0

#define R_PEG_ETH_CTL0             0x00
#define R_PEG_ETH_CTL1             0x01
#define R_PEG_ETH_CTL2             0x02
#define R_PEG_MCAST_TBL            0x08
#define R_PEG_MAC_ADDR_0           0x10
#define R_PEG_EEPROM_OFS           0x20
#define R_PEG_EEPROM_DATA          0x21
#define R_PEG_EEPROM_CTL           0x23
#define R_PEG_PHY_ADDR             0x25
#define R_PEG_PHY_DATA             0x26		//& 0x27 for 2 bytes
#define R_PEG_PHY_CTRL             0x28
#define R_PEG_INT_PHY  	           0x7b
#define R_PEG_GPIO0                0x7e
#define R_PEG_GPIO1                0x7f

/* PHY Access Control Register (0x28) */
#define PEG_PHY_WRITE              0x20
#define PEG_PHY_READ               0x40


/* *******************************************************************
   * Realtek RTL8150 register and bit definitions.
   *
   * Reference:
   *   Realtek Single-Chip USB to Fast Ethernet Controller with
   *     MII Interface RTL8150L(M), Version 1.40
   *   Realtek Semiconductor Corporation, Hsinchu, Taiwan, 2002/2/18.
   ******************************************************************* */

#define RTEK_REG_ACCESS            0x05

#define R_RTEK_MAC                 0x0120
#define R_RTEK_CMD                 0x012E
#define R_RTEK_TXCFG               0x012F
#define R_RTEK_RXCFG               0x0130
#define R_RTEK_TXSTAT              0x0132
#define R_RTEK_RXSTAT              0x0133

/* Command Register (0x012E) */
#define RTEK_AUTOLOAD              0x01
#define RTEK_TXENABLE              0x04
#define RTEK_RXENABLE              0x08
#define RTEK_RESET                 0x10

/* Receive Configuration Register (0x0130) */
#define RTEK_MACADDR               0x04
#define RTEK_BCASTADDR             0x08


/* *******************************************************************
   * Kawasaki LSI KL5KUSB101B register and bit definitions.
   *
   * Reference: inferred from *BSD kue drivers.
   ******************************************************************* */

/* Generic commands */
#define KLSI_SEND_SCAN             0xFF

/* MAC-specific commands */
#define KLSI_GET_ETH_DESC          0x00
#define KLSI_SET_MCAST_FILTER      0x01
#define KLSI_SET_PKT_FILTER        0x02
#define KLSI_SET_MAC               0x06
#define KLSI_GET_MAC               0x07
#define KLSI_SET_URB_SIZE          0x08
#define KLSI_SET_SOFS              0x09

/* Structure returned by GET_ETH_DESC */
typedef struct klsi_ether_desc_s {
    uint8_t  klsi_len;
    uint8_t  klsi_rsvd0;
    uint8_t  klsi_rsvd1;
    uint8_t  klsi_macaddr[6];
    uint8_t  klsi_etherstats[4];
    uint8_t  klsi_maxseg[2];
    uint8_t  klsi_mcastfilt[2];
    uint8_t  klsi_rsvd2;
} klsi_ether_desc_t;

/* Bit masks for SET_PKT_FILTER */
#define KLSI_RX_PROMISC            0x0001
#define KLSI_RX_ALLMULTI           0x0002
#define KLSI_RX_UNICAST            0x0004
#define KLSI_RX_BROADCAST          0x0008
#define KLSI_RX_MULTICAST          0x0010


/*  *********************************************************************
    *  Device register/unregister
    ********************************************************************* */

typedef struct usbeth_disp_s {
    int (*read)(void *,hsaddr_t buf);
    int (*inpstat)(void *);
    int (*write)(void *,hsaddr_t buf,int len);
    int (*getaddr)(void *,hsaddr_t addr);
} usbeth_disp_t;

int usbeth_register(usbeth_disp_t *disp,void *softc);
void usbeth_unregister(void *softc);

#endif /*__USBETH_H_ */
