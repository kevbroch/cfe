/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  BCM440X (10/100 Ethernet MAC) registers       File: bcm4401.h
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

#ifndef _BCM4400_H_
#define _BCM4400_H_

/*
 * Register and bit definitions for the Broadcom BCM440X family
 * of 10/100 MACs with integrated PHY.  Similar controllers are used
 * in certain SOC products.
 */

/*
 * References:
 * 
 *   Programmer's Guide BCM440X PCI 10/100 Ethernet and
 *     V.90/V.92 Controller, 440X-PG00-R
 *   Broadcom Corp., 16215 Alton Parkway, Irvine CA, 08/23/02
 *
 *   BCM4401 PCI 10/100 Ethernet Controller Data Sheet, 4401-DS104-R
 *   Broadcom Corp., 16215 Alton Parkway, Irvine CA, 1/30/03
 */

#define _DD_MAKEMASK1(n) (1 << (n))
#define _DD_MAKEMASK(v,n) ((((1)<<(v))-1) << (n))
#define _DD_MAKEVALUE(v,n) ((v) << (n))
#define _DD_GETVALUE(v,n,m) (((v) & (m)) >> (n))

#define K_PCI_VENDOR_BROADCOM  0x14e4
#define K_PCI_ID_BCM4401       0x4401     /* Rev A0, A1 */
#define K_PCI_ID_BCM4401_B     0x170C     /* Rev B0 and later */
#define K_PCI_ID_BCM4402       0x4402     /* used by HND */

/* PCI Configuration Registers (440X extensions) */

#define PCI_PMC_REG            0x40
#define PCI_PMCSR_REG          0x44
#define PCI_PMDATA_REG         0x48

#define PCI_PCIBAR0WINDOW_REG  0x80
#define PCI_PCIBAR1WINDOW_REG  0x84
#define PCI_SPROMCONTROL_REG   0x88
#define PCI_BAR1BURSTCTRL_REG  0x8C
#define PCI_PCIINTSTATUS_REG   0x90
#define PCI_PCIINTMASK_REG     0x94
#define PCI_SBMAILBOX_REG      0x98
#define PCI_BACKPLANEADDR_REG  0xA0
#define PCI_BACKPLANEDATA_REG  0xA4


/* SBISR: PCI Interrupt Status Register (0x90, R/W) */
/* SBIMR: PCI Interrupt Mask Register   (0x94, R/W) */

#define S_SBINT_BM              0                       /* SBMailbox */
#define M_SBINT_BM              _DD_MAKEMASK(2,S_SBINT_BM)
#define V_SBINT_BM(x)           _DD_MAKEVALUE(x,S_SBINT_BM)
#define G_SBINT_BM(x)           _DD_GETVALUE(x,S_SBINT_BM,M_SBINT_BM)

#define M_SBINT_SB               _DD_MAKEMASK1(2)       /* SBError */

#define S_SBINT_IM              8                       /* SBIntMask */
#define M_SBINT_IM              _DD_MAKEMASK(8,S_SBINT_IM)
#define V_SBINT_IM(x)           _DD_MAKEVALUE(x,S_SBINT_IM)
#define G_SBINT_IM(x)           _DD_GETVALUE(x,S_SBINT_IM,M_SBINT_IM)


/*
 * The 440x products contain multiple cores that communicate via a
 * system backplane internal to the chip.  The 4401 includes a MAC
 * core (with indirect access to the internal PHY) and a PCI core.
 * The 4402 adds a codec core, which appears in 4401 enumeration space.
 *
 * Each core occupies 4K (0x1000) bytes of "enumeration" space.  Each
 * includes a configuration region at offset 0xF00 consisting of
 * registers to identify the core and map its resources.  Some cores,
 * including the MAC, have a standard DMA engine; if so, its registers
 * appear at offset 0x200.
 *
 * In theory, the positioning of address spaces for each core is
 * programmable; in practice, a standard configuration is loaded from
 * EEPROM on reset (see data sheet), and the MAC core's enumeration
 * space is thereafter accessible via BAR 0.  */


/* System backplane addresses of cores */

#ifndef SB_PCI_BASE		/* XXX this should not be here */
#define SB_ENET_BASE            0x18000000
#define SB_CODEC_BASE           0x18001000    /* bond-out on 4402 only */
#define SB_PCI_BASE             0x18002000
#endif


/* Ethernet 10/100 Core (Section 5, Table 102) */

#define R_DEV_CONTROL           0x000
#define R_BIST_STATUS           0x00C
#define R_WAKEUP_LENGTH         0x010
#define R_INT_STATUS            0x020
#define R_INT_MASK              0x024
#define R_GP_TIMER              0x028
#define R_ENET_FT_ADDR          0x090
#define R_ENET_FT_DATA          0x094
#define R_EMAC_XMT_MAX_BURST    0x0A0
#define R_EMAC_RCV_MAX_BURST    0x0A4
#define R_EMAC_CONTROL          0x0A8
#define R_INT_RECV_LAZY         0x100

/* Core DMA Engine Registers (Section 3) */

#define R_XMT_CONTROL           0x200
#define R_XMT_ADDR              0x204
#define R_XMT_PTR               0x208
#define R_XMT_STATUS            0x20C
#define R_RCV_CONTROL           0x210
#define R_RCV_ADDR              0x214
#define R_RCV_PTR               0x218
#define R_RCV_STATUS            0x21C

/* EMAC Registers */

#define R_RCV_CONFIG            0x400
#define R_RCV_MAX_LENGTH        0x404
#define R_XMT_MAX_LENGTH        0x408
#define R_MII_STATUS_CONTROL    0x410
#define R_MII_DATA              0x414
#define R_ENET_INT_MASK         0x418
#define R_ENET_INT_STATUS       0x41C
#define R_CAM_DATA_L            0x420
#define R_CAM_DATA_H            0x424
#define R_CAM_CONTROL           0x428
#define R_ENET_CONTROL          0x42C
#define R_XMT_CONTROL1          0x430
#define R_XMT_WATERMARK         0x434
#define R_MIB_CONTROL           0x438
#define R_MIB_COUNTERS          0x500        /* Block base address */

/* Core Configuration Space Registers (Section 2, Table 3) */

#define R_SBIMSTATE             0xF90
#define R_SBINTVEC              0xF94
#define R_SBTMSTATELOW          0xF98
#define R_SBTMSTATEHI           0xF9C
#define R_SBBWA0                0xFA0
#define R_SBIMCONFIGLOW         0xFA8
#define R_SBIMCONFIGHIGH        0xFAC
#define R_SBADMATCH0            0xFB0
#define R_SBTMCONFIGLOW         0xFB8
#define R_SBTMCONFIGHIGH        0xFBC
#define R_SBBCONFIG             0xFC0
#define R_SBBSTATE              0xFC8
#define R_SBACTCNFG             0xFD8
#define R_SBFLAGST              0xFE8
#define R_SBIDLOW               0xFF8
#define R_SBIDHIGH              0xFFC


/* Ethernet PHY MII (Section 5, Table 103) */

#define R_BMCR                  0x00
#define R_BMSR                  0x01
#define R_PHYIDR1               0x02
#define R_PHYIDR2               0x03
#define R_ANAR                  0x04
#define R_ANLPAR                0x05
#define R_ANER                  0x06
#define R_NEXT_PAGE             0x07
#define R_LP_NEXT_PAGE          0x08

#define R_100X_AUX_CONTROL      0x10
#define R_100X_AUX_STATUS       0x11
#define R_100X_RCV_ERROR_CTR    0x12
#define R_100X_CARR_SENSE_CTR   0x13
#define R_100X_DISCONNECT_CRT   0x14
#define R_AUX_CTL_STATUS        0x18
#define R_AUX_STATUS_SUMMARY    0x19
#define R_INTERRUPT             0x1A
#define R_AUX_MODE_2            0x1B
#define R_10BT_T_ERR_GEN_STAT   0x1C
#define R_AUX_MODE              0x1D
#define R_AUX_MULTI_PHY         0x1E


/* Codec Core (Section 4) - BCM4402 only, not supported. */


/* PCI Core (Section 6) */

/* Extended PCI Configuration Space */

#define R_PCI_BAR0_WINDOW       0x080
#define R_PCI_BAR1_WINDOW       0x084
#define R_BAR1_BURST_CONTROL    0x08C
#define R_PCI_INT_STATUS        0x090
#define R_PCI_INT_MASK          0x094
#define R_PCI_TO_SB_MAILBOX     0x098

#define R_BACKPLANE_ADDRESS     0x0A0
#define R_BACKPLANE_DATA        0x0A4

/* PCI Enumeration Space */

#define R_SB_TO_PCI_MAILBOX     0x028
#define R_BROADCAST_ADDRESS     0x050
#define R_BROADCAST_DATA        0x054
#define R_GPIO_INPUT            0x060
#define R_GPIO_OUTPUT           0x064
#define R_GPIO_OUT_EN           0x068
#define R_GPIO_CONTROL          0x06C

#define R_SB_TO_PCI_TRANSLATION0 0x100
#define R_SB_TO_PCI_TRANSLATION1 0x104
#define R_SB_TO_PCI_TRANSLATION2 0x108   /* Not documented */


/* Core Configuration Space Registers. */

/* These registers are found at the top of the 4K window for each core. */
 
/* SBIS: System Backplace Initiator State Register (0xF90, R/W) */

#define S_SBIS_PC               0                       /* PipeCount */
#define M_SBIS_PC               _DD_MAKEMASK(4,S_SBIS_PC)
#define V_SBIS_PC(x)            _DD_MAKEVALUE(x,S_SBIS_PC)
#define G_SBIS_PC(x)            _DD_GETVALUE(x,S_SBIS_PC,M_SBIS_PC)

#define S_SBIS_PL               4                       /* Policy */
#define M_SBIS_PL               _DD_MAKEMASK(2,S_SBIS_PL)
#define V_SBIS_PL(x)            _DD_MAKEVALUE(x,S_SBIS_PL)
#define G_SBIS_PL(x)            _DD_GETVALUE(x,S_SBIS_PL,M_SBIS_PL)

#define M_SBIS_IE               _DD_MAKEMASK1(17)       /* InbandError */
#define M_SBIS_TO               _DD_MAKEMASK1(18)       /* TimeOut */

/* SBINT: System Backplane Interrupt Vector Register (0xF94, R/W) */

#define S_SBINT_MK               0                      /* Mask */
#define M_SBINT_MK               _DD_MAKEMASK(3,S_SBINT_MK)
#define V_SBINT_MK(x)            _DD_MAKEVALUE(x,S_SBINT_MK)
#define G_SBINT_MK(x)            _DD_GETVALUE(x,S_SBINT_MK,M_SBINT_MK)
#define K_SBINT_PCI              (1<<0)
#define K_SBINT_ENET_MAC         (1<<1)
#define K_SBINT_CODEC            (1<<2)

/* SBTS(LO): System Backplane Target State Low Register (0xF98, R/W) */

#define M_SBTS_RS               _DD_MAKEMASK1(0)        /* Reset */
#define M_SBTS_RJ               _DD_MAKEMASK1(1)        /* Reject */
#define M_SBTS_CE               _DD_MAKEMASK1(16)       /* ClockEnable */
#define M_SBTS_FC               _DD_MAKEMASK1(17)       /* ForceGatedClocks */

#define S_SBTS_FL               18                      /* Flags */
#define M_SBTS_FL               _DD_MAKEMASK(12,S_SBTS_FL)
#define V_SBTS_FL(x)            _DD_MAKEVALUE(x,S_SBTS_FL)
#define G_SBTS_FL(x)            _DD_GETVALUE(x,S_SBTS_FL,M_SBTS_FL)

#define M_SBTS_PE               _DD_MAKEMASK1(30)       /* PMEEnable */
#define M_SBTS_BE               _DD_MAKEMASK1(31)       /* BISTEnable */

/* SBTS(HI): System Backplane Target State High Register (0xF9C, R/W) */

#define M_SBTS_SE               _DD_MAKEMASK1(0)        /* SError */
#define M_SBTS_IN               _DD_MAKEMASK1(1)        /* Interrupt */
#define M_SBTS_BY               _DD_MAKEMASK1(2)        /* Busy */
#define M_SBTS_GC               _DD_MAKEMASK1(29)       /* GatedClkRequest */
#define M_SBTS_BF               _DD_MAKEMASK1(30)       /* BISTFail */
#define M_SBTS_BD               _DD_MAKEMASK1(31)       /* BISTDone */

/* SBFS: System Backplane Flag Status Register (0xFE8, RO) */

#define M_SBFS_MI               _DD_MAKEMASK1(7)        /* MII pins disabled */

/* SBID(LO): System Backplane Identification Low Register (0xFF8, RO) */

#define S_SBID_CS               0                       /* ConfigSpace */
#define M_SBID_CS               _DD_MAKEMASK(3,S_SBID_CS)
#define V_SBID_CS(x)            _DD_MAKEVALUE(x,S_SBID_CS)
#define G_SBID_CS(x)            _DD_GETVALUE(x,S_SBID_CS,M_SBID_CS)

#define S_SBID_AR               3                       /* AddressRanges */
#define M_SBID_AR               _DD_MAKEMASK(3,S_SBID_AR)
#define V_SBID_AR(x)            _DD_MAKEVALUE(x,S_SBID_AR)
#define G_SBID_AR(x)            _DD_GETVALUE(x,S_SBID_AR,M_SBID_AR)

#define M_SBID_SC               _DD_MAKEMASK1(6)        /* Synch */
#define M_SBID_IT               _DD_MAKEMASK1(7)        /* Initiator */

#define S_SBID_MN               8                       /* MinLatency */
#define M_SBID_MN               _DD_MAKEMASK(4,S_SBID_MN)
#define V_SBID_MN(x)            _DD_MAKEVALUE(x,S_SBID_MN)
#define G_SBID_MN(x)            _DD_GETVALUE(x,S_SBID_MN,M_SBID_MN)

#define S_SBID_MX               12                      /* MaxLatency */
#define M_SBID_MX               _DD_MAKEMASK(4,S_SBID_MX)
#define V_SBID_MX(x)            _DD_MAKEVALUE(x,S_SBID_MX)
#define G_SBID_MX(x)            _DD_GETVALUE(x,S_SBID_MX,M_SBID_MX)

#define M_SBID_FI               _DD_MAKEMASK1(16)       /* FirstInitiator */
#define M_SBID_NT               _DD_MAKEMASK1(17)       /* NoTarget */

#define S_SBID_CC               18                      /* CycleCounterWidth */
#define M_SBID_CC               _DD_MAKEMASK(2,S_SBID_CC)
#define V_SBID_CC(x)            _DD_MAKEVALUE(x,S_SBID_CC)
#define G_SBID_CC(x)            _DD_GETVALUE(x,S_SBID_CC,M_SBID_CC)

#define S_SBID_TP               20                      /* TargetPorts */
#define M_SBID_TP               _DD_MAKEMASK(4,S_SBID_TP)
#define V_SBID_TP(x)            _DD_MAKEVALUE(x,S_SBID_TP)
#define G_SBID_TP(x)            _DD_GETVALUE(x,S_SBID_TP,M_SBID_TP)

#define S_SBID_IP               24                      /* InitPorts */
#define M_SBID_IP               _DD_MAKEMASK(4,S_SBID_IP)
#define V_SBID_IP(x)            _DD_MAKEVALUE(x,S_SBID_IP)
#define G_SBID_IP(x)            _DD_GETVALUE(x,S_SBID_IP,M_SBID_IP)

/* SBID(HI): System Backplane Identification High Register (0xFFC, RO) */

#define S_SBID_RV               0                       /* Revision */
#define M_SBID_RV               _DD_MAKEMASK(4,S_SBID_RV)
#define V_SBID_RV(x)            _DD_MAKEVALUE(x,S_SBID_RV)
#define G_SBID_RV(x)            _DD_GETVALUE(x,S_SBID_RV,M_SBID_RV)

#define S_SBID_CR               4                       /* Core */
#define M_SBID_CR               _DD_MAKEMASK(12,S_SBID_CR)
#define V_SBID_CR(x)            _DD_MAKEVALUE(x,S_SBID_CR)
#define G_SBID_CR(x)            _DD_GETVALUE(x,S_SBID_CR,M_SBID_CR)
#define K_CR_PCI                0x804
#define K_CR_ENET_MAC           0x806
#define K_CR_CODEC              0x807

#define S_SBID_VN               16                      /* Vendor */
#define M_SBID_VN               _DD_MAKEMASK(16,S_SBID_VN)
#define V_SBID_VN(x)            _DD_MAKEVALUE(x,S_SBID_VN)
#define G_SBID_VN(x)            _DD_GETVALUE(x,S_SBID_VN,M_SBID_VN)
#define K_VN_BROADCOM           0x4243


/* Ethernet MAC Registers */

/* DVCTL: Device Control Register (0x000, R/W) */

#define M_DVCTL_PM              _DD_MAKEMASK1(7)        /* PatMatchEn */
#define M_DVCTL_IP              _DD_MAKEMASK1(10)       /* InternalEPHY */
#define M_DVCTL_ER              _DD_MAKEMASK1(15)       /* EPHYReset */
#define M_DVCTL_MP              _DD_MAKEMASK1(16)       /* MIIPHYModeEn */
#define M_DVCTL_CO              _DD_MAKEMASK1(17)       /* ClkOutputEn */

#define S_DVCTL_PA              18                      /* PHYAddrReg */
#define M_DVCTL_PA              _DD_MAKEMASK(5,S_DVCTL_PA)
#define V_DVCTL_PA(x)           _DD_MAKEVALUE(x,S_DVCTL_PA)
#define G_DVCTL_PA(x)           _DD_GETVALUE(x,S_DVCTL_PA,M_DVCTL_PA)

/* BIST: Built-In Self Test Register (0x00C, RO) */

#define S_BIST_BS               0                       /* BISTStatus */
#define M_BIST_BS               _DD_MAKEMASK(2,S_BIST_BS)
#define V_BIST_BS(x)            _DD_MAKEVALUE(x,S_BIST_BS)
#define G_BIST_BS(x)            _DD_GETVALUE(x,S_BIST_BS,M_BIST_BS)

/* WKUP: Wakeup Length Register (0x010, R/W) */

#define S_WKUP_P0               0                       /* Pattern0 */
#define M_WKUP_P0               _DD_MAKEMASK(7,S_WKUP_P0)
#define V_WKUP_P0(x)            _DD_MAKEVALUE(x,S_WKUP_P0)
#define G_WKUP_P0(x)            _DD_GETVALUE(x,S_WKUP_P0,M_WKUP_P0)

#define M_WKUP_D0               _DD_MAKEMASK1(7)        /* Disable0 */

#define S_WKUP_P1               0                       /* Pattern1 */
#define M_WKUP_P1               _DD_MAKEMASK(7,S_WKUP_P1)
#define V_WKUP_P1(x)            _DD_MAKEVALUE(x,S_WKUP_P1)
#define G_WKUP_P1(x)            _DD_GETVALUE(x,S_WKUP_P1,M_WKUP_P1)

#define M_WKUP_D1               _DD_MAKEMASK1(15)       /* Disable1 */

#define S_WKUP_P2               0                       /* Pattern2 */
#define M_WKUP_P2               _DD_MAKEMASK(7,S_WKUP_P2)
#define V_WKUP_P2(x)            _DD_MAKEVALUE(x,S_WKUP_P2)
#define G_WKUP_P2(x)            _DD_GETVALUE(x,S_WKUP_P2,M_WKUP_P2)

#define M_WKUP_D2               _DD_MAKEMASK1(23)       /* Disable2 */

#define S_WKUP_P3               0                       /* Pattern3 */
#define M_WKUP_P3               _DD_MAKEMASK(7,S_WKUP_P3)
#define V_WKUP_P3(x)            _DD_MAKEVALUE(x,S_WKUP_P3)
#define G_WKUP_P3(x)            _DD_GETVALUE(x,S_WKUP_P3,M_WKUP_P3)

#define M_WKUP_D3               _DD_MAKEMASK1(31)       /* Disable3 */

/* ISR: Interrupt Status Register (0x020, R/W) */
/* IMR: Interrupt Mask Register   (0x024, R/W) */

#define M_INT_PM                _DD_MAKEMASK1(6)        /* PME */
#define M_INT_TO                _DD_MAKEMASK1(7)        /* TimeOut */
#define M_INT_DE                _DD_MAKEMASK1(10)       /* DescErr */
#define M_INT_DA                _DD_MAKEMASK1(11)       /* DataErr */
#define M_INT_DP                _DD_MAKEMASK1(12)       /* DescProtoErr */
#define M_INT_RU                _DD_MAKEMASK1(13)       /* RcvDescUf */
#define M_INT_RO                _DD_MAKEMASK1(14)       /* RcvFIFOOf */
#define M_INT_XU                _DD_MAKEMASK1(15)       /* XmtFIFOUf */
#define M_INT_RI                _DD_MAKEMASK1(16)       /* RcvInt */
#define M_INT_XI                _DD_MAKEMASK1(24)       /* XmtInt */
#define M_INT_EI                _DD_MAKEMASK1(26)       /* EMACInterrupt */
#define M_INT_IW                _DD_MAKEMASK1(27)       /* IntMIIWrite */
#define M_INT_IR                _DD_MAKEMASK1(28)       /* IntMIIRead */

/* EMCTL: Ethernet MAC Control Register (0x0A8, R/W) */

#define M_EMCTL_CC              _DD_MAKEMASK1(0)        /* CRCCheckXmt */
#define M_EMCTL_EP              _DD_MAKEMASK1(2)        /* EPHYPowerDown */
#define M_EMCTL_ED              _DD_MAKEMASK1(3)        /* EPHYEnergyDetect */

#define S_EMCTL_LC              5                       /* LEDControl */
#define M_EMCTL_LC              _DD_MAKEMASK(3,S_EMCTL_LC)
#define V_EMCTL_LC(x)           _DD_MAKEVALUE(x,S_EMCTL_LC)
#define G_EMCTL_LC(x)           _DD_GETVALUE(x,S_EMCTL_LC,M_EMCTL_LC)

/* EMFLOW: Ethernet MAC Flow Control Register (0x0AC, R/W) */

#define S_EMFLOW_RF             0                       /* RcvFlowControl */
#define M_EMFLOW_RF             _DD_MAKEMASK(8,S_EMFLOW_RF)
#define V_EMFLOW_RF(x)          _DD_MAKEVALUE(x,S_EMFLOW_RF)
#define G_EMFLOW_RF(x)          _DD_GETVALUE(x,S_EMFLOWL_RF,M_EFLOW_RF)

#define M_EMFLOW_PE             _DD_MAKEMASK1(15)       /* PauseEn */


/* INTLZY: Interrupt Receive Lazy Timeout Register (0x100, R/W) */

#define S_INTLZY_TO             0                       /* TimeOut */
#define M_INTLZY_TO             _DD_MAKEMASK(24,S_INTLZY_TO)
#define V_INTLZY_TO(x)          _DD_MAKEVALUE(x,S_INTLZY_TO)
#define G_INTLZY_TO(x)          _DD_GETVALUE(x,S_INTLZY_TO,M_INTLZY_TO)

#define S_INTLZY_FC             24                      /* FrameCount */
#define M_INTLZY_FC             _DD_MAKEMASK(8,S_INTLZY_FC)
#define V_INTLZY_FC(x)          _DD_MAKEVALUE(x,S_INTLZY_FC)
#define G_INTLZY_FC(x)          _DD_GETVALUE(x,S_INTLZY_FC,M_INTLZY_FC)


/* RCFG: Receiver Configuration Register (0x400, R/W) */

#define M_RCFG_DB               _DD_MAKEMASK1(0)        /* DisB */
#define M_RCFG_AM               _DD_MAKEMASK1(1)        /* AccMult */
#define M_RCFG_RD               _DD_MAKEMASK1(2)        /* RcvDisableTx */
#define M_RCFG_PR               _DD_MAKEMASK1(3)        /* Prom */
#define M_RCFG_LB               _DD_MAKEMASK1(4)        /* Lpbk */
#define M_RCFG_EF               _DD_MAKEMASK1(5)        /* EnFlow */
#define M_RCFG_UF               _DD_MAKEMASK1(6)        /* UniFlow */
#define M_RCFG_RF               _DD_MAKEMASK1(7)        /* RejectFilter */


/* MIICTL: MII Status/Control Register (0x410, R/W) */

#define S_MIICTL_MD             0                       /* MDC */
#define M_MIICTL_MD             _DD_MAKEMASK(7,S_MIICTL_MD)
#define V_MIICTL_MD(x)          _DD_MAKEVALUE(x,S_MIICTL_MD)
#define G_MIICTL_MD(x)          _DD_GETVALUE(x,S_MIICTL_MD,M_MIICTL_MD)

#define M_MIICTL_PR             _DD_MAKEMASK1(7)        /* PreEn */

/* MIIDATA: MII Data Register (0x414, R/W) */

#define S_MIIDATA_D             0                       /* Data */
#define M_MIIDATA_D             _DD_MAKEMASK(16,S_MIIDATA_D)
#define V_MIIDATA_D(x)          _DD_MAKEVALUE(x,S_MIIDATA_D)
#define G_MIIDATA_D(x)          _DD_GETVALUE(x,S_MIIDATA_D,M_MIIDATA_D)

#define S_MIIDATA_TA            16                      /* Turnaround */
#define M_MIIDATA_TA            _DD_MAKEMASK(2,S_MIIDATA_TA)
#define V_MIIDATA_TA(x)         _DD_MAKEVALUE(x,S_MIIDATA_TA)
#define G_MIIDATA_TA(x)         _DD_GETVALUE(x,S_MIIDATA_TA,M_MIIDATA_TA)
#define K_TA_VALID              0x2

#define S_MIIDATA_RA            18                      /* RegAddr */
#define M_MIIDATA_RA            _DD_MAKEMASK(5,S_MIIDATA_RA)
#define V_MIIDATA_RA(x)         _DD_MAKEVALUE(x,S_MIIDATA_RA)
#define G_MIIDATA_RA(x)         _DD_GETVALUE(x,S_MIIDATA_RA,M_MIIDATA_RA)

#define S_MIIDATA_PM            23                      /* PhysMedia */
#define M_MIIDATA_PM            _DD_MAKEMASK(5,S_MIIDATA_PM)
#define V_MIIDATA_PM(x)         _DD_MAKEVALUE(x,S_MIIDATA_PM)
#define G_MIIDATA_PM(x)         _DD_GETVALUE(x,S_MIIDATA_PM,M_MIIDATA_PM)

#define S_MIIDATA_OP            28                      /* Opcode */
#define M_MIIDATA_OP            _DD_MAKEMASK(2,S_MIIDATA_OP)
#define V_MIIDATA_OP(x)         _DD_MAKEVALUE(x,S_MIIDATA_OP)
#define G_MIIDATA_OP(x)         _DD_GETVALUE(x,S_MIIDATA_OP,M_MIIDATA_OP)
#define K_MII_OP_WRITE          0x1
#define K_MII_OP_READ           0x2

#define S_MIIDATA_SB            30                      /* StartBits */
#define M_MIIDATA_SB            _DD_MAKEMASK(2,S_MIIDATA_SB)
#define V_MIIDATA_SB(x)         _DD_MAKEVALUE(x,S_MIIDATA_SB)
#define G_MIIDATA_SB(x)         _DD_GETVALUE(x,S_MIIDATA_SB,M_MIIDATA_SB)
#define K_MII_START             0x1

/* EIMR: Ethernet Interrupt Mask Register (0x418, R/W) */
/* EISR: Ethernet Interrupt Status Register (0x41C, R/W) */

#define M_EINT_MI               _DD_MAKEMASK1(0)        /* MIIInt */
#define M_EINT_MB               _DD_MAKEMASK1(1)        /* MIBInt */
#define M_EINT_FM               _DD_MAKEMASK1(2)        /* FlowInt */

/* CAM: CAM Data Low Register (0x420, R/W) */

#define S_CAM_CD_L              0                       /* CAMDataL */
#define M_CAM_CD_L              _DD_MAKEMASK(32,S_CAM_CD_L)
#define V_CAM_CD_L(x)           _DD_MAKEVALUE(x,S_CAM_CD_L)
#define G_CAM_CD_L(x)           _DD_GETVALUE(x,S_CAM_CD_L,M_CAM_CD_L)

/* CAM: CAM Data High Register (0x424, R/W) */

#define S_CAM_CD_H              0                       /* CAMDataH */
#define M_CAM_CD_H              _DD_MAKEMASK(16,S_CAM_CD_H)
#define V_CAM_CD_H(x)           _DD_MAKEVALUE(x,S_CAM_CD_H)
#define G_CAM_CD_H(x)           _DD_GETVALUE(x,S_CAM_CD_H,M_CAM_CD_H)

#define M_CAM_VB                _DD_MAKEMASK1(16)       /* ValidBit */

/* CAMCTL: CAM Control Register (0x428, R/W) */

#define M_CAMCTL_CE             _DD_MAKEMASK1(0)        /* CAMEnable */
#define M_CAMCTL_MS             _DD_MAKEMASK1(1)        /* MaskSelect */
#define M_CAMCTL_CR             _DD_MAKEMASK1(2)        /* CAMRead */
#define M_CAMCTL_CW             _DD_MAKEMASK1(3)        /* CAMWrite */

#define S_CAMCTL_IX             16                      /* Index */
#define M_CAMCTL_IX             _DD_MAKEMASK(6,S_CAMCTL_IX)
#define V_CAMCTL_IX(x)          _DD_MAKEVALUE(x,S_CAMCTL_IX)
#define G_CAMCTL_IX(x)          _DD_GETVALUE(x,S_CAMCTL_IX,M_CAMCTL_IX)

#define M_CAMCTL_CB             _DD_MAKEMASK1(31)       /* CAMBusy */

/* ECTL: Ethernet Control Register (0x42C, R/W) */

#define M_ECTL_EE               _DD_MAKEMASK1(0)        /* EMACEnable */
#define M_ECTL_ED               _DD_MAKEMASK1(1)        /* EMACDisable */
#define M_ECTL_ES               _DD_MAKEMASK1(2)        /* EMACSoftReset */
#define M_ECTL_EP               _DD_MAKEMASK1(3)        /* ExtPHYSelect */

/* TCTL: Transmit Control Register (0x430, R/W) */

#define M_TCTL_FD               _DD_MAKEMASK1(0)        /* FullDuplex */
#define M_TCTL_FM               _DD_MAKEMASK1(1)        /* FlowMode */
#define M_TCTL_SB               _DD_MAKEMASK1(2)        /* SingleBackoffEn */
#define M_TCTL_SS               _DD_MAKEMASK1(3)        /* SmSlotTime */

/* MIBCTL: MIB Control Register (0x438, R/W) */

#define M_MIBCTL_RO              _DD_MAKEMASK1(0)       /* RO */


/* DMA Control Registers */

/* XCTL: Transmit Channel Control Register (0x200, R/W) */

#define M_XCTL_XE               _DD_MAKEMASK1(0)        /* XmtEn */
#define M_XCTL_SE               _DD_MAKEMASK1(1)        /* SuspEn */
#define M_XCTL_LE               _DD_MAKEMASK1(2)        /* LoopbackEn */
#define M_XCTL_FP               _DD_MAKEMASK1(3)        /* FairPriority */

/* XADDR: Transmit Descriptor Table Address Register (0x204, R/W) */

#define S_XADDR_BA              12                      /* BaseAddr */
#define M_XADDR_BA              _DD_MAKEMASK(20,S_XADDR_BA)
#define V_XADDR_BA(x)           _DD_MAKEVALUE(x,S_XADDR_BA)
#define G_XADDR_BA(x)           _DD_GETVALUE(x,S_XADDR_BA,M_XADDR_BA)

/* XPTR: Transmit Descriptor Table Pointer Register (0x208, R/W) */

#define S_XPTR_LD              0                        /* LastDscr */
#define M_XPTR_LD              _DD_MAKEMASK(12,S_XPTR_LD)
#define V_XPTR_LD(x)           _DD_MAKEVALUE(x,S_XPTR_LD)
#define G_XPTR_LD(x)           _DD_GETVALUE(x,S_XPTR_LD,M_XPTR_LD)

/* XSTAT: Transmit Channel Status Register (0x20C, RO) */

#define S_XSTAT_CD             0                        /* CurrDscr */
#define M_XSTAT_CD             _DD_MAKEMASK(12,S_XSTAT_CD)
#define V_XSTAT_CD(x)          _DD_MAKEVALUE(x,S_XSTAT_CD)
#define G_XSTAT_CD(x)          _DD_GETVALUE(x,S_XSTAT_CD,M_XSTAT_CD)

#define S_XSTAT_XS             12                       /* XmtState */
#define M_XSTAT_XS             _DD_MAKEMASK(4,S_XSTAT_XS)
#define V_XSTAT_XS(x)          _DD_MAKEVALUE(x,S_XSTAT_XS)
#define G_XSTAT_XS(x)          _DD_GETVALUE(x,S_XSTAT_XS,M_XSTAT_XS)
#define K_XS_DISABLED          0x0
#define K_XS_ACTIVE            0x1
#define K_XS_IDLE_WAIT         0x2
#define K_XS_STOPPED           0x3
#define K_XS_SUSPEND_PENDING   0x4

#define S_XSTAT_XE             16                       /* XmtErr */
#define M_XSTAT_XE             _DD_MAKEMASK(4,S_XSTAT_XE)
#define V_XSTAT_XE(x)          _DD_MAKEVALUE(x,S_XSTAT_XE)
#define G_XSTAT_XE(x)          _DD_GETVALUE(x,S_XSTAT_XE,M_XSTAT_XE)
#define K_XE_NONE              0x0
#define K_XE_DSCR_PROTOCOL     0x1
#define K_XE_FIFO_UNDERRUN     0x2
#define K_XE_DATA_TRANSFER     0x3
#define K_XE_DSCR_READ         0x4

/* RCTL: Receive Channel Control Register (0x210, R/W) */

#define M_RCTL_RE               _DD_MAKEMASK1(0)        /* RcvEn */

#define S_RCTL_RO               1                       /* RcvOffset */
#define M_RCTL_RO               _DD_MAKEMASK(7,S_RCTL_RO)
#define V_RCTL_RO(x)            _DD_MAKEVALUE(x,S_RCTL_RO)
#define G_RCTL_RO(x)            _DD_GETVALUE(x,S_RCTL_RO,M_RCTL_RO)

#define M_RCTL_FM               _DD_MAKEMASK1(8)        /* FIFOMode */

/* RADDR: Receive Descriptor Table Address Register (0x214, R/W) */

#define S_RADDR_BA              12                      /* BaseAddr */
#define M_RADDR_BA              _DD_MAKEMASK(20,S_RADDR_BA)
#define V_RADDR_BA(x)           _DD_MAKEVALUE(x,S_RADDR_BA)
#define G_RADDR_BA(x)           _DD_GETVALUE(x,S_RADDR_BA,M_RADDR_BA)

/* RPTR: Receive Descriptor Table Pointer Register (0x218, R/W) */

#define S_RPTR_LD              0                        /* LastDscr */
#define M_RPTR_LD              _DD_MAKEMASK(12,S_RPTR_LD)
#define V_RPTR_LD(x)           _DD_MAKEVALUE(x,S_RPTR_LD)
#define G_RPTR_LD(x)           _DD_GETVALUE(x,S_RPTR_LD,M_RPTR_LD)

/* RSTAT: Receive Channel Status Register (0x21C, RO) */

#define S_RSTAT_CD             0                        /* CurrDscr */
#define M_RSTAT_CD             _DD_MAKEMASK(12,S_RSTAT_CD)
#define V_RSTAT_CD(x)          _DD_MAKEVALUE(x,S_RSTAT_CD)
#define G_RSTAT_CD(x)          _DD_GETVALUE(x,S_RSTAT_CD,M_RSTAT_CD)

#define S_RSTAT_RS             12                       /* RcvState */
#define M_RSTAT_RS             _DD_MAKEMASK(4,S_RSTAT_RS)
#define V_RSTAT_RS(x)          _DD_MAKEVALUE(x,S_RSTAT_RS)
#define G_RSTAT_RS(x)          _DD_GETVALUE(x,S_RSTAT_RS,M_RSTAT_RS)
#define K_RS_DISABLED          0x0
#define K_RS_ACTIVE            0x1
#define K_RS_IDLE_WAIT         0x2
#define K_RS_STOPPED           0x3

#define S_RSTAT_RE             16                       /* RcvErr */
#define M_RSTAT_RE             _DD_MAKEMASK(4,S_RSTAT_RE)
#define V_RSTAT_RE(x)          _DD_MAKEVALUE(x,S_RSTAT_RE)
#define G_RSTAT_RE(x)          _DD_GETVALUE(x,S_RSTAT_RE,M_RSTAT_RE)
#define K_RE_NONE              0x0
#define K_RE_DSCR_PROTOCOL     0x1
#define K_RE_FIFO_OVERFLOW     0x2
#define K_RE_DATA_TRANSFER     0x3
#define K_RE_DSCR_READ         0x4


/* DMA Descriptor Structure (Table 66) */

/* Word 0: Flags and Count */

#define S_DSCR0_BC             0                        /* BufCount */
#define M_DSCR0_BC             _DD_MAKEMASK(13,S_DSCR0_BC)
#define V_DSCR0_BC(x)          _DD_MAKEVALUE(x,S_DSCR0_BC)
#define G_DSCR0_BC(x)          _DD_GETVALUE(x,S_DSCR0_BC,M_DSCR0_BC)

#define S_DSCR0_FL             20                       /* Flags */
#define M_DSCR0_FL             _DD_MAKEMASK(8,S_DSCR0_FL)
#define V_DSCR0_FL(x)          _DD_MAKEVALUE(x,S_DSCR0_FL)
#define G_DSCR0_FL(x)          _DD_GETVALUE(x,S_DSCR0_FL,M_DSCR0_FL)

#define M_DSCR0_ET             _DD_MAKEMASK1(28)        /* EOT */
#define M_DSCR0_IC             _DD_MAKEMASK1(29)        /* IOC */
#define M_DSCR0_EF             _DD_MAKEMASK1(30)        /* EOF */
#define M_DSCR0_SF             _DD_MAKEMASK1(31)        /* SOF */

/* Word 1: Data Buffer Pointer */

#define S_DSCR1_DB             0                        /* DataBufPtr */
#define M_DSCR1_DB             _DD_MAKEMASK(32, S_DSCR1_DB)
#define V_DSCR1_DB(x)          _DD_MAKEVALUE(x,S_DSCR1_DB)
#define G_DSCR1_DB(x)          _DD_GETVALUE(x,S_DSCR1_DB,M_DSCR1_DB)


/* DMA Receive Headers (Table 67) */

#define S_RCVHDR0_CD           0                        /* FrameLen (!) */
#define M_RCVHDR0_CD           _DD_MAKEMASK(16,S_RCVHDR0_CD)
#define V_RCVHDR0_CD(x)        _DD_MAKEVALUE(x,S_RCVHDR0_CD)
#define G_RCVHDR0_CD(x)        _DD_GETVALUE(x,S_RCVHDR0_CD,M_RCVHDR0_CD)

#define S_RCVHDR0_DC           24                       /* DescrCnt */
#define M_RCVHDR0_DC           _DD_MAKEMASK(4,S_RCVHDR0_DC)
#define V_RCVHDR0_DC(x)        _DD_MAKEVALUE(x,S_RCVHDR0_DC)
#define G_RCVHDR0_DC(x)        _DD_GETVALUE(x,S_RCVHDR0_DC,M_RCVHDR0_DC)

/* The following flags are documented for the 4710 core but not for the 4401 */
#define M_RCVHDR0_L            _DD_MAKEMASK1(27)         /* Last */
#define M_RCVHDR0_F            _DD_MAKEMASK1(26)         /* First */
#define M_RCVHDR0_W            _DD_MAKEMASK1(25)         /* Wrap */
#define M_RCVHDR0_MISS         _DD_MAKEMASK1(23)         /* Miss */
#define M_RCVHDR0_BRDCAST      _DD_MAKEMASK1(22)         /* Broadcast */
#define M_RCVHDR0_MULT         _DD_MAKEMASK1(21)         /* Multicast */
#define M_RCVHDR0_LG           _DD_MAKEMASK1(20)         /* Large */
#define M_RCVHDR0_NO           _DD_MAKEMASK1(19)         /* NonOctet Aligned */
#define M_RCVHDR0_RXER         _DD_MAKEMASK1(18)         /* Symbol Error */
#define M_RCVHDR0_CRC          _DD_MAKEMASK1(17)         /* CRC */
#define M_RCVHDR0_OV           _DD_MAKEMASK1(16)         /* Overflow */
#define M_RCVHDR0_ERRORS       (M_RCVHDR0_NO | M_RCVHDR0_RXER | M_RCVHDR0_CRC \
                                | M_RCVHDR0_OV)


/* Offsets of MIB counters in Statistics Block (Table 161)
   Registers are 16 bits except as noted. */

#define TX_GD_OCTETS            0x500        /* 32 bits */
#define TX_GD_PKTS              0x504
#define TX_ALL_OCTETS           0x508        /* 32 bits */
#define TX_ALL_PKTS             0x50C
#define TX_BRDCAST              0x510
#define TX_MULT                 0x514
#define TX_64                   0x518
#define TX_65_127               0x51C
#define TX_128_255              0x520
#define TX_256_511              0x524
#define TX_512_1023             0x528
#define TX_1024_MAX             0x52C
#define TX_JAB                  0x530
#define TX_OVR                  0x534
#define TX_FRAG                 0x538
#define TX_UNDERRUN             0x53C
#define TX_COL                  0x540
#define TX_1_COL                0x544
#define TX_M_COL                0x548
#define TX_EX_COL               0x54C
#define TX_LATE                 0x550
#define TX_DEF                  0x554
#define TX_CRS                  0x558
#define TX_PAUS                 0x55C

#define RX_GD_OCTETS            0x580        /* 32 bits */
#define RX_GD_PKTS              0x584
#define RX_ALL_OCTETS           0x588        /* 32 bits */
#define RX_ALL_PKTS             0x58C
#define RX_BRDCAST              0x590
#define RX_MULT                 0x594
#define RX_64                   0x598
#define RX_65_127               0x59C
#define RX_128_255              0x5A0
#define RX_256_511              0x5A4
#define RX_512_1023             0x5A8
#define RX_1024_MAX             0x5AC
#define RX_JAB                  0x5B0
#define RX_OVR                  0x5B4
#define RX_FRAG                 0x5B8
#define RX_DROP                 0x5BC
#define RX_CRC_ALIGN            0x5C0
#define RX_UND                  0x5C4
#define RX_CRC                  0x5C8
#define RX_ALIGN                0x5CC
#define RX_SYM                  0x5D0
#define RX_PAUSE                0x5D4
#define RX_CNTRL                0x5D8


/* In the default mapping, the top 4K of the BAR0 window maps into the
   SPROM.  Note that access latency is high and the corresponding code
   must be prepared to avoid or deal with bus errors caused by PCI
   timeouts. */

#define SPROM_BASE             0x1000

#define SPROM_MAC_ADDR         0x4E         /* 6 bytes */
#define SPROM_PHY_ADDR         90           /* 2 bytes, bits [4:0] (undoc) */


/* Ethernet PHY Registers (see also mii.h) */

/* PHYINT: Interrupt Register (0x1A, R/W) */

#define M_PHYINT_IS             _DD_MAKEMASK1(0)        /* InterruptStatus */
#define M_PHYINT_LC             _DD_MAKEMASK1(1)        /* LinkChangeInt */
#define M_PHYINT_SP             _DD_MAKEMASK1(2)        /* SpeedChangeInt */
#define M_PHYINT_DC             _DD_MAKEMASK1(3)        /* DuplexChangeInt */
#define M_PHYINT_MI             _DD_MAKEMASK1(8)        /* MasterIntMask */
#define M_PHYINT_LI             _DD_MAKEMASK1(9)        /* LinkIntMask */
#define M_PHYINT_SI             _DD_MAKEMASK1(10)       /* SpeedIntMask */
#define M_PHYINT_FD             _DD_MAKEMASK1(11)       /* FullDuplexIntMask */
#define M_PHYINT_IE             _DD_MAKEMASK1(14)       /* InterruptEnable */

/* PHYAUX2: Auxiliary Mode 2 Register (0x1B, R/W) */

#define M_PHYAUX2_QP            _DD_MAKEMASK1(1)        /* QualParallelDet */
#define M_PHYAUX2_AL            _DD_MAKEMASK1(2)        /* ActivityLinkLED */
#define M_PHYAUX2_LF            _DD_MAKEMASK1(5)        /* ActivityLEDForce */
#define M_PHYAUX2_TM            _DD_MAKEMASK1(6)        /* TrafficMeterLED */
#define M_PHYAUX2_BE            _DD_MAKEMASK1(7)        /* Block10BTEcho */
#define M_PHYAUX2_HF            _DD_MAKEMASK1(9)        /* HSTRFIFO */
#define M_PHYAUX2_TR            _DD_MAKEMASK1(10)       /* TokenRing */
#define M_PHYAUX2_DC            _DD_MAKEMASK1(11)       /* 10BTDribbleCorr */


/* PCI Core Registers */

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

#define S_SBXLAT_UA             26                      /* UpperAddress */
#define M_SBXLAT_UA             _DD_MAKEMASK(6,S_SBXLAT_UA)
#define V_SBXLAT_UA(x)          _DD_MAKEVALUE(x,S_SBXLAT_UA)
#define G_SBXLAT_UA(x)          _DD_GETVALUE(x,S_SBXLAT_UA,M_SBXLAT_UA)

#endif /* _BCM4400_H_ */
