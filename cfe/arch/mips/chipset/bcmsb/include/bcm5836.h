/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  SOC chip definitions			File: bcm5836.h
    *
    *  Constants and macros specific to this SOC
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

#ifndef _BCM5836_H_
#define _BCM5836_H_

/* Define the chip, to match legacy #ifdef's. */
#define BCM5836  1

/* Define the chip family */
#define BCM47XX  1
#define BCM47xx  1

/* BCM5836 Address map */
#define BCM5836_SDRAM		0x00000000 /* 0-128MB Physical SDRAM */
#define BCM5836_PCI_MEM		0x08000000 /* Host Mode PCI mem space (64MB)*/
#define BCM5836_PCI_CFG		0x0c000000 /* Host Mode PCI cfg space (64MB)*/
#define BCM5836_PCI_DMA		0x40000000 /* Client Mode PCI mem space (1GB)*/
#define	BCM5836_SDRAM_SWAPPED	0x10000000 /* Byteswapped Physical SDRAM */
#define BCM5836_ENUM		0x18000000 /* Beginning of core enum space */

/* BCM5836 Core register space */
#define BCM5836_REG_CHIPC	0x18000000 /* Chipcommon  registers */
#define BCM5836_REG_EMAC0	0x18001000 /* Ethernet MAC0 core registers */
#define BCM5836_REG_EMAC1	0x18002000 /* Ethernet MAC1 core registers */
#define BCM5836_REG_USB		0x18003000 /* USB core registers */
#define BCM5836_REG_PCI		0x18004000 /* PCI core registers */
#define BCM5836_REG_MIPS33	0x18005000 /* MIPS core registers */
#define BCM5836_REG_CODEC	0x18006000 /* AC97 Codec Core registers */
#define BCM5836_REG_IPSEC	0x18007000 /* BCM582x CryptoCore registers */
#define BCM5836_REG_MEMC	0x18008000 /* MEMC core registers */

#define BCM5836_REG_UARTS       (BCM5836_REG_CHIPC + 0x300) /* UART regs */

#define	BCM5836_EJTAG		0xff200000 /* MIPS EJTAG space (2M) */

/* Internal 16550-compatible UARTs */
#define BCM5836_UART0		(BCM5836_REG_UARTS + 0x00000000)
#define BCM5836_UART1		(BCM5836_REG_UARTS + 0x00000100)

/* Registers common to MIPS33 Core used in 5365 and 4704 */
#define MIPS33_EXTIF_REGION           0x1A000000 /* Chipcommon EXTIF region*/
#define MIPS33_FLASH_REGION_AUX       0x1C000000 /* FLASH Region 2*/
#define MIPS33_FLASH_REGION           0x1FC00000 /* Boot FLASH Region  */

/* bcm4704 mapping to generic sb_bp identifiers */
/* XXX It would be better to discover this dynamically. */

/* BSP Abstraction, pickup names via bsp_config.h. */

#define SB_ENUM_BASE            BCM5836_ENUM

#define SB_CHIPC_BASE           BCM5836_REG_CHIPC
#define SB_ENET0_BASE           BCM5836_REG_EMAC0
#define SB_ENET1_BASE           BCM5836_REG_EMAC1
#define SB_IPSEC_BASE           BCM5836_REG_IPSEC
#define SB_USB_BASE             BCM5836_REG_USB
#define SB_PCI_BASE             BCM5836_REG_PCI
#define SB_MIPS33_BASE          BCM5836_REG_MIPS33
#define SB_MEMC_BASE            BCM5836_REG_MEMC

#define SB_EXTIF_SPACE          MIPS33_EXTIF_REGION
#define SB_FLASH_SPACE          MIPS33_FLASH_REGION

#endif /* _BCM5836_H_ */
