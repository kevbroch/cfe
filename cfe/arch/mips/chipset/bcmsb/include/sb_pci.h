/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PCI core definitions                  	File: sb_pci.c
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

#ifndef _SBPCI_H_
#define _SBPCI_H_

/*
 * Register and bit definitions for the PCI core (OCP ID 0x804)
 *
 * The PCI core can be operated in either host or device (aka client)
 * mode.  Interpretations of some registers are different in the two
 * modes.  In host mode, the core serves as the PCI host bridge for
 * the internal CPU.  In device mode, there is an external master
 * responsible for PCI configuration and the bcm47xx appears as a PCI
 * device. Some derivative parts the same core but in device mode only.
 *
 * In addition to the registers in enumeration space, the PCI core
 * provides an extended set of PCI configuration registers accessible
 * only to the system host via PCI configuration cycles.
 */

#define K_PCI_VENDOR_BROADCOM  0x14e4

/* PCI Enumeration Space (Section 10.4) */

#define R_PCI_CONTROL           0x000    /* not device mode */
#define R_BIST_STATUS           0x00C
#define R_PCI_ARB_CONTROL       0x010
#define R_INT_STATUS            0x020
#define R_INT_MASK              0x024
#define R_SB_TO_PCI_MAILBOX     0x028

#define R_BROADCAST_ADDRESS     0x050
#define R_BROADCAST_DATA        0x054

#define R_GPIO_INPUT            0x060    /* BCM440x */
#define R_GPIO_OUTPUT           0x064    /* BCM440x */
#define R_GPIO_OUT_EN           0x068    /* BCM440x */
#define R_GPIO_CONTROL          0x06C    /* BCM440x */

#define R_SB_TO_PCI_TRANSLATION0 0x100
#define R_SB_TO_PCI_TRANSLATION1 0x104
#define R_SB_TO_PCI_TRANSLATION2 0x108   /* undocumented for BCM440x */

/* SPROM Shadow Area */
#define SROM_OFFSET              0x800
#define SROM_SIZE                0x100


/* PCI Configuration Registers */

/* Standard Configuration Registers (Section 10.5.1, see pci.h) */

/* Power Managment Capability (Section 10.5.2) */

#define PCI_PMC_REG            0x40
#define PCI_PMCSR_REG          0x44
#define PCI_PMDATA_REG         0x48

/* Device Specific Configuration Space Registers (10.5.3) */

#define PCI_PCIBAR0WINDOW_REG  0x80
#define PCI_PCIBAR1WINDOW_REG  0x84
#define PCI_SPROMCONTROL_REG   0x88
#define PCI_BAR1BURSTCTRL_REG  0x8C
#define PCI_PCIINTSTATUS_REG   0x90
#define PCI_PCIINTMASK_REG     0x94
#define PCI_SBMAILBOX_REG      0x98
#define PCI_BACKPLANEADDR_REG  0xA0
#define PCI_BACKPLANEDATA_REG  0xA4

/* Extended PCI Configuration Space (XXX reconcile with above) */

#define R_PCI_BAR0_WINDOW       0x080
#define R_PCI_BAR1_WINDOW       0x084
#define R_BAR1_BURST_CONTROL    0x08C
#define R_PCI_INT_STATUS        0x090
#define R_PCI_INT_MASK          0x094
#define R_PCI_TO_SB_MAILBOX     0x098

#define R_BACKPLANE_ADDRESS     0x0A0
#define R_BACKPLANE_DATA        0x0A4


/* PCI Core Enumeration Space Registers */

/* PCICTL: PCI Control Register (0x000, R/W) */

#define M_PCICTL_OE             _DD_MAKEMASK1(0)        /* PCIResetOutputEn */
#define M_PCICTL_RO             _DD_MAKEMASK1(1)        /* PCIResetOutput */
#define M_PCICTL_CE             _DD_MAKEMASK1(2)        /* PCIClockOutputEn */
#define M_PCICTL_CO             _DD_MAKEMASK1(3)        /* PCIClockOutput */

/* PCIARB: PCI Arbiter Control Register (0x010, R/W) */

#define M_PCIARB_IA             _DD_MAKEMASK1(0)        /* InternalArbiter */
#define M_PCIARB_EA             _DD_MAKEMASK1(1)        /* ExternalArbiter */

#define S_PCIARB_PI             2                       /* ParkID */
#define M_PCIARB_PI             _DD_MAKEMASK(2,S_PCIARB_PI)
#define V_PCIARB_PI(x)          _DD_MAKEVALUE(x,S_PCIARB_PI)
#define G_PCIARB_PI(x)          _DD_GETVALUE(x,S_PCIARB_PI,M_PCIARB_PI)
#define K_ARB_PARK_GNT0         0x0
#define K_ARB_PARK_GNT1         0x1
#define K_ARB_PARK_INT          0x2
#define K_ARB_PARK_LAST         0x3

/* ISR: Interrupt Status Register (0x020, R/W) */
/* IMR: Interrupt Mask Register   (0x024, R/W) */

#define M_PCIINT_PA             _DD_MAKEMASK1(0)        /* PCIIntA */
#define M_PCIINT_PB             _DD_MAKEMASK1(1)        /* PCIIntB */
#define M_PCIINT_PS             _DD_MAKEMASK1(2)        /* PCISerr */
#define M_PCIINT_PP             _DD_MAKEMASK1(3)        /* PCIPerr */
#define M_PCIINT_PM             _DD_MAKEMASK1(4)        /* PCIPME */

#define S_PCIINT_F0             8                       /* Function0 */
#define M_PCIINT_F0             _DD_MAKEMASK(2,S_PCIINT_F0)
#define V_PCIINT_F0(x)          _DD_MAKEVALUE(x,S_PCIINT_F0)
#define G_PCIINT_F0(x)          _DD_GETVALUE(x,S_PCIINT_F0,M_PCIINT_F0)

#define S_PCIINT_F1             10                      /* Function1 */
#define M_PCIINT_F1             _DD_MAKEMASK(2,S_PCIINT_F1)
#define V_PCIINT_F1(x)          _DD_MAKEVALUE(x,S_PCIINT_F1)
#define G_PCIINT_F1(x)          _DD_GETVALUE(x,S_PCIINT_F1,M_PCIINT_F1)

#define S_PCIINT_F2             12                      /* Function2 */
#define M_PCIINT_F2             _DD_MAKEMASK(2,S_PCIINT_F2)
#define V_PCIINT_F2(x)          _DD_MAKEVALUE(x,S_PCIINT_F2)
#define G_PCIINT_F2(x)          _DD_GETVALUE(x,S_PCIINT_F2,M_PCIINT_F2)

#define S_PCIINT_F3             14                      /* Function3 */
#define M_PCIINT_F3             _DD_MAKEMASK(2,S_PCIINT_F3)
#define V_PCIINT_F3(x)          _DD_MAKEVALUE(x,S_PCIINT_F3)
#define G_PCIINT_F3(x)          _DD_GETVALUE(x,S_PCIINT_F3,M_PCIINT_F3)

/* SBMBOX: System Backplane to PCI Mailbox Register (0x028, WO) */

#define S_SBMBOX_F0             8                      /* Function0 */
#define M_SBMBOX_F0             _DD_MAKEMASK(2,S_SBMBOX_F0)
#define V_SBMBOX_F0(x)          _DD_MAKEVALUE(x,S_SBMBOX_F0)
#define G_SBMBOX_F0(x)          _DD_GETVALUE(x,S_SBMBOX_F0,M_SBMBOX_F0)

#define S_SBMBOX_F1             10                      /* Function1 */
#define M_SBMBOX_F1             _DD_MAKEMASK(2,S_SBMBOX_F1)
#define V_SBMBOX_F1(x)          _DD_MAKEVALUE(x,S_SBMBOX_F1)
#define G_SBMBOX_F1(x)          _DD_GETVALUE(x,S_SBMBOX_F1,M_SBMBOX_F1)

#define S_SBMBOX_F2             12                      /* Function2 */
#define M_SBMBOX_F2             _DD_MAKEMASK(2,S_SBMBOX_F2)
#define V_SBMBOX_F2(x)          _DD_MAKEVALUE(x,S_SBMBOX_F2)
#define G_SBMBOX_F2(x)          _DD_GETVALUE(x,S_SBMBOX_F2,M_SBMBOX_F2)

#define S_SBMBOX_F3             14                      /* Function3 */
#define M_SBMBOX_F3             _DD_MAKEMASK(2,S_SBMBOX_F3)
#define V_SBMBOX_F3(x)          _DD_MAKEVALUE(x,S_SBMBOX_F3)
#define G_SBMBOX_F3(x)          _DD_GETVALUE(x,S_SBMBOX_F3,M_SBMBOX_F3)

/* SBXLAT: System Backplane to PCI Translation 0 Register (0x100, R/W) */
/* SBXLAT: System Backplane to PCI Translation 1 Register (0x104, R/W) */
/* SBXLAT: System Backplane to PCI Translation 2 Register (0x108, R/W) */

#define S_SBXLAT_AT             0                       /* AccessType */
#define M_SBXLAT_AT             _DD_MAKEMASK(2,S_SBXLAT_AT)
#define V_SBXLAT_AT(x)          _DD_MAKEVALUE(x,S_SBXLAT_AT)
#define G_SBXLAT_AT(x)          _DD_GETVALUE(x,S_SBXLAT_AT,M_SBXLAT_AT)
#define K_AT_RW                 0
#define K_AT_IO_RW              1
#define K_AT_CFG0_RW            2
#define K_AT_CFG1_RW            3

#define M_SBXLAT_PE             _DD_MAKEMASK1(2)        /* PrefetchEn */
#define M_SBXLAT_WB             _DD_MAKEMASK1(3)        /* WriteBurstEn */

/* SBXLAT0,1 span 64MB */
#define S_SBXLAT_UA             26                      /* UpperAddress */
#define M_SBXLAT_UA             _DD_MAKEMASK(6,S_SBXLAT_UA)
#define V_SBXLAT_UA(x)          _DD_MAKEVALUE(x,S_SBXLAT_UA)
#define G_SBXLAT_UA(x)          _DD_GETVALUE(x,S_SBXLAT_UA,M_SBXLAT_UA)

/* SBXLAT2 spans 1GB */
#define S_SBXLAT2_UA            30                      /* UpperAddress */
#define M_SBXLAT2_UA            _DD_MAKEMASK(2,S_SBXLAT2_UA)
#define V_SBXLAT2_UA(x)         _DD_MAKEVALUE(x,S_SBXLAT2_UA)
#define G_SBXLAT2_UA(x)         _DD_GETVALUE(x,S_SBXLAT2_UA,M_SBXLAT2_UA)


/* Device Specific PCI Configuration Registers */

/* SPROMCTL: SPROM Control Register (0x88, R/W) */

#define S_SPROMCTL_SS           0                       /* SPROMSize */
#define M_SPROMCTL_SS           _DD_MAKEMASK(2,S_SPROMCTL_SS)
#define V_SPROMCTL_SS(x)        _DD_MAKEVALUE(x,S_SPROMCTL_SS)
#define G_SPROMCTL_SS(x)        _DD_GETVALUE(x,S_SPROMCTL_SS,M_SPROMCTL_SS)
#define K_SPROM_SIZE_128        0
#define K_SPROM_SIZE_512        1
#define K_SPROM_SIZE_2048       2

#define M_SPROMCTL_SB           _DD_MAKEMASK1(2)        /* SPROMBlank */
#define M_SPROMCTL_SL           _DD_MAKEMASK1(3)        /* SPROMLocked */
#define M_SPROMCTL_SW           _DD_MAKEMASK1(4)        /* SPROMWriteEn */

/* BAR1BURST: Bar 1 Burst Control Register (0x8C, R/W) */

#define M_BAR1BURST_WB          _DD_MAKEMASK1(0)        /* WriteBufferEn */
#define M_BAR1BURST_PE          _DD_MAKEMASK1(1)        /* PrefetchEn */
#define M_BAR1BURST_ET          _DD_MAKEMASK1(4)        /* TransactionTimer */

/* SBISR: PCI Interrupt Status Register (0x90, R/W) */
/* SBIMR: PCI Interrupt Mask Register   (0x94, R/W) */

#define S_SBINT_BM              0                       /* SBMailbox */
#define M_SBINT_BM              _DD_MAKEMASK(2,S_SBINT_BM)
#define V_SBINT_BM(x)           _DD_MAKEVALUE(x,S_SBINT_BM)
#define G_SBINT_BM(x)           _DD_GETVALUE(x,S_SBINT_BM,M_SBINT_BM)

#define M_SBINT_SB               _DD_MAKEMASK1(2)        /* SBError */

/* The following on PCI 2.3 versions (Rev >= 6) only. */
#define S_SBINT_IM              8                       /* SBIntMask */
#define M_SBINT_IM              _DD_MAKEMASK(8,S_SBINT_IM)
#define V_SBINT_IM(x)           _DD_MAKEVALUE(x,S_SBINT_IM)
#define G_SBINT_IM(x)           _DD_GETVALUE(x,S_SBINT_IM,M_SBINT_IM)

/* PCIMBOX: PCI to System Backplane Mailbox Register (0x98, WO) */

#define S_PCIMBOX_BM            0                       /* SBMailbox */
#define M_PCIMBOX_BM            _DD_MAKEMASK(2,S_PCIMBOX_BM)
#define V_PCIMBOX_BM(x)         _DD_MAKEVALUE(x,S_PCIMBOX_BM)
#define G_PCIMBOX_BM(x)         _DD_GETVALUE(x,S_PCIMBOX_BM,M_PCIMBOX_BM)

#endif /* _SBPCI_H_ */
