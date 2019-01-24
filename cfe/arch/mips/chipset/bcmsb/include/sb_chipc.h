/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Silicon Backplane definitions       File: sb_chipc.h
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

#ifndef _SBCHIPC_H_
#define _SBCHIPC_H_

/*
 * Register and bit definitions for the Chip Common control
 * registers (OCP ID 0x800).
 */


/* Chip Common Core (Section 9) */

/* Control and Interrupt Registers (Section 9.2.1) */
#define R_CHIPID                0x000
#define R_CORECAPABILITIES      0x004
#define R_CORECONTROL           0x008
#define R_INTSTATUS             0x020
#define R_INTMASK               0x024
#define R_SFLASHCONTROL         0x040
#define R_SFLASHADDRESS         0x044
#define R_SFLASHDATA            0x048
#define R_BROADCASTADDRESS      0x050
#define R_BROADCASTDATA         0x054

/* GPIO Registers (Section 9.2.2) */
#define R_GPIOINPUT             0x060
#define R_GPIOOUTPUT            0x064
#define R_GPIOOUTEN             0x068
#define R_GPIOCONTROL           0x06C
#define R_GPIOINTPOLARITY       0x070
#define R_GPIOINTMASK           0x074
#define R_WATCHDOGCOUNTER       0x080
/* Backward compatibility aliases */
#define R_WATCHDOGCNTR          R_WATCHDOGCOUNTER

/* Clock Control Registers (Section 9.2.3) */
#define R_CLOCKCONTROLN         0x090
#define R_CLOCKCONTROLM0        0x094
#define R_CLOCKCONTROLM1        0x098
#define R_CLOCKCONTROLM2        0x09C
#define R_CLOCKCONTROLM3        0x0A0
#define R_UARTCLOCKDIV          0x0A4
/* Backward compatibility aliases */
#define R_CLOCKCONTROLSB        R_CLOCKCONTROLM0
#define R_CLOCKCONTROLPCI       R_CLOCKCONTROLM1
#define R_CLOCKCONTROLMII       R_CLOCKCONTROLM2
#define R_CLOCKCONTROLCPU       R_CLOCKCONTROLM3

/* External Interface Bus Control Registers (Section 9.2.4) */
#define R_CS01CONFIG            0x100
#define R_CS01MEMWAITCNT        0x104
#define R_CS01ATTRWAITCNT       0x108
#define R_CS01IOWAITCNT         0x10C
#define R_CS23CONFIG            0x110
#define R_CS23MEMWAITCNT        0x114
#define R_CS23ATTRWAITCNT       0x118
#define R_CS23IOWAITCNT         0x11C
#define R_CS4CONFIG             0x120
#define R_CS4WAITCNT            0x124
#define R_PARALLELFLASHCONFIG   0x128
#define R_PARALLELFLASHWAITCNT  0x12C

/* UART Registers (Section 9.2.5) - see ns16550.h */

/* System Backplane Configuration Register (Section 9.2.6) */
#define R_FLAGSTATUS            0xFE8

/* JTAG Backplane Access Registers (Section 9.2.7) */
#define R_JTAG_ADDRESS_WR       0x30
#define R_JTAG_ADDRESS_RD       0x31
#define R_JTAG_DATA_WR          0x32
#define R_JTAG_DATA_RD          0x33
#define R_JTAG_CONTROL_WR       0x34
#define R_JTAG_CONTROL_RD       0x35


/* CHIPID: ChipID Register (0x000, RO) */

#define S_CHIPID_CI             0                       /* ChipID */
#define M_CHIPID_CI             _DD_MAKEMASK(16,S_CHIPID_CI)
#define V_CHIPID_CI(x)          _DD_MAKEVALUE(x,S_CHIPID_CI)
#define G_CHIPID_CI(x)          _DD_GETVALUE(x,S_CHIPID_CI,M_CHIPID_CI)

#define S_CHIPID_RI             16                      /* RevisionID */
#define M_CHIPID_RI             _DD_MAKEMASK(4,S_CHIPID_RI)
#define V_CHIPID_RI(x)          _DD_MAKEVALUE(x,S_CHIPID_RI)
#define G_CHIPID_RI(x)          _DD_GETVALUE(x,S_CHIPID_RI,M_CHIPID_RI)

#define S_CHIPID_PO             20                      /* PackageOption */
#define M_CHIPID_PO             _DD_MAKEMASK(4,S_CHIPID_PO)
#define V_CHIPID_PO(x)          _DD_MAKEVALUE(x,S_CHIPID_PO)
#define G_CHIPID_PO(x)          _DD_GETVALUE(x,S_CHIPID_PO,M_CHIPID_PO)

/* CORECAP: Core Capabilities Register (0x004, RO) */

#define S_CORECAP_UP            0                       /* UARTsPresent */
#define M_CORECAP_UP            _DD_MAKEMASK(2,S_CORECAP_UP)
#define V_CORECAP_UP(x)         _DD_MAKEVALUE(x,S_CORECAP_UP)
#define G_CORECAP_UP(x)         _DD_GETVALUE(x,S_CORECAP_UP,M_CORECAP_UP)

#define M_CORECAP_EM            _DD_MAKEMASK1(2)        /* EndianMode */

#define S_CORECAP_CS            3                       /* UARTClkSel */
#define M_CORECAP_CS            _DD_MAKEMASK(2,S_CORECAP_CS)
#define V_CORECAP_CS(x)         _DD_MAKEVALUE(x,S_CORECAP_CS)
#define G_CORECAP_CS(x)         _DD_GETVALUE(x,S_CORECAP_CS,M_CORECAP_CS)
#define K_CS_EXTERNAL           0x0
#define K_CS_INTERNAL           0x1

#define M_CORECAP_UG            _DD_MAKEMASK1(5)        /* UARTGPIOs */
#define M_CORECAP_EB            _DD_MAKEMASK1(6)        /* ExtBusPresent */

#define S_CORECAP_FT            8                       /* FlashType */
#define M_CORECAP_FT            _DD_MAKEMASK(3,S_CORECAP_FT)
#define V_CORECAP_FT(x)         _DD_MAKEVALUE(x,S_CORECAP_FT)
#define G_CORECAP_FT(x)         _DD_GETVALUE(x,S_CORECAP_FT,M_CORECAP_FT)
#define K_FT_NONE               0x0
#define K_FT_STM_SERIAL         0x1
#define K_FT_ATMEL_SERIAL       0x2
#define K_FT_PARALLEL           0x7

#define S_CORECAP_PL            16                      /* PLLCtlPresent */
#define M_CORECAP_PL            _DD_MAKEMASK(2,S_CORECAP_PL)
#define V_CORECAP_PL(x)         _DD_MAKEVALUE(x,S_CORECAP_PL)
#define G_CORECAP_PL(x)         _DD_GETVALUE(x,S_CORECAP_PL,M_CORECAP_PL)
#define K_PLL_NONE              0x0
#define K_PLL_TYPE1             0x1
#define K_PLL_TYPE2             0x2
#define K_PLL_TYPE3             0x3
/* Backward compatibility aliases */
#define K_PL_NONE               0x0
#define K_PL_4710               0x1
#define K_PL_4704               0x2
#define K_PL_5365               0x3

/* CORECTL: Core Control Register (0x008, R/W) */

#define M_CORECTL_CO            _DD_MAKEMASK1(0)        /* UARTClkOverride */
#define M_CORECTL_SE            _DD_MAKEMASK1(1)        /* SyncClkOutEn */

/* INTSTAT: Int Status Register (0x020, R/W) */

#define M_INTSTAT_GI            _DD_MAKEMASK1(0)        /* GPIOInt */
#define M_INTSTAT_EI            _DD_MAKEMASK1(1)        /* ExtInt */
#define M_INTSTAT_WD            _DD_MAKEMASK1(2)        /* WDReset */

/* INTMASK: Int Mask Register (0x024, R/W) */

#define M_INTMASK_EI            _DD_MAKEMASK1(1)        /* ExtInt */

/* SFLASHCTL: SFlash Control Register (0x040, R/W) */

#define S_SFLASHCTL_OC          0                       /* Opcode */
#define M_SFLASHCTL_OC          _DD_MAKEMASK(8,S_SFLASHCTL_OC)
#define V_SFLASHCTL_OC(x)       _DD_MAKEVALUE(x,S_SFLASHCTL_OC)
#define G_SFLASHCTL_OC(x)       _DD_GETVALUE(x,S_SFLASHCTL_OC,M_SFLASHCTL_OC)

#define S_SFLASHCTL_AC          8                       /* Action */
#define M_SFLASHCTL_AC          _DD_MAKEMASK(8,S_SFLASHCTL_AC)
#define V_SFLASHCTL_AC(x)       _DD_MAKEVALUE(x,S_SFLASHCTL_AC)
#define G_SFLASHCTL_AC(x)       _DD_GETVALUE(x,S_SFLASHCTL_AC,M_SFLASHCTL_AC)

#define M_SFLASHCTL_AB          _DD_MAKEMASK1(32)       /* Start/Busy */

/* SFLASHADDR: SFlash Address Register (0x44, R/W) */

/* SFLASHDATA: SFlash Data Register (0x48, R/W) */

/* BCASTADDR: Broadcast Address Register (0x050, R/W) */

/* BCASTDATA: Broadcast Data Register (0x054, R/W) */


/* ... GPIO ... */

/* WDOG: Watchdog Counter Register (0x080, R/W) */


/* CCN: Clock Control N Register (0x090, R/W, buffered) */

#define S_CCN_N1                0                       /* N1Control */
#define M_CCN_N1                _DD_MAKEMASK(6,S_CCN_N1)
#define V_CCN_N1(x)             _DD_MAKEVALUE(x,S_CCN_N1)
#define G_CCN_N1(x)             _DD_GETVALUE(x,S_CCN_N1,M_CCN_N1)

#define S_CCN_N2                8                       /* N2Control */
#define M_CCN_N2                _DD_MAKEMASK(5,S_CCN_N2)
#define V_CCN_N2(x)             _DD_MAKEVALUE(x,S_CCN_N2)
#define G_CCN_N2(x)             _DD_GETVALUE(x,S_CCN_N2,M_CCN_N2)

/* CCM0: Clock Control M0 Register (0x094, R/W, buffered) */
/* CCM1: Clock Control M1 Register (0x098, R/W, buffered) */
/* CCM2: Clock Control M2 Register (0x09C, R/W, buffered) */
/* CCM3: Clock Control M3 Register (0x0A0, R/W, buffered) */

#define S_CCM_M1                0                       /* M1Control */
#define M_CCM_M1                _DD_MAKEMASK(6,S_CCM_M1)
#define V_CCM_M1(x)             _DD_MAKEVALUE(x,S_CCM_M1)
#define G_CCM_M1(x)             _DD_GETVALUE(x,S_CCM_M1,M_CCM_M1)

#define S_CCM_M2                8                       /* M2Control */
#define M_CCM_M2                _DD_MAKEMASK(5,S_CCM_M2)
#define V_CCM_M2(x)             _DD_MAKEVALUE(x,S_CCM_M2)
#define G_CCM_M2(x)             _DD_GETVALUE(x,S_CCM_M2,M_CCM_M2)

#define S_CCM_M3                16                      /* M3Control */
#define M_CCM_M3                _DD_MAKEMASK(6,S_CCM_M3)
#define V_CCM_M3(x)             _DD_MAKEVALUE(x,S_CCM_M3)
#define G_CCM_M3(x)             _DD_GETVALUE(x,S_CCM_M3,M_CCM_M3)

#define S_CCM_MM                24                      /* MuxControl */
#define M_CCM_MM                _DD_MAKEMASK(5,S_CCM_MM)
#define V_CCM_MM(x)             _DD_MAKEVALUE(x,S_CCM_MM)
#define G_CCM_MM(x)             _DD_GETVALUE(x,S_CCM_MM,M_CCM_MM)
/* Backward compatibility aliases */
#define S_CCM_MC                S_CCM_MM
#define M_CCM_MC                M_CCM_MM
#define V_CCM_MC                V_CCM_MM
#define G_CCM_MC                G_CCM_MM

/* UARTDIV: UART Clock Div Register (0xA4, R/W) */

#define S_UARTDIV_CD            0                       /* ClkDiv */
#define M_UARTDIV_CD            _DD_MAKEMASK(6,S_UARTDIV_CD)
#define V_UARTDIV_CD(x)         _DD_MAKEVALUE(x,S_UARTDIV_CD)
#define G_UARTDIV_CD(x)         _DD_GETVALUE(x,S_UARTDIV_CD,M_UARTDIV_CD)

/* CSO1CONFIG: CS01 Configuration Register (0x100, R/W) */
/* CSO1CONFIG: CS23 Configuration Register (0x110, R/W) */
/* CS4CONFIG:  CS4  Configuration Register (0x120, R/W) */

#define M_CS_EN                 _DD_MAKEMASK1(0)        /* Enable */

#define S_CS_EM                 1                       /* ExtIfMode */
#define M_CS_EM                 _DD_MAKEMASK(3,S_CS_EM)
#define V_CS_EM(x)              _DD_MAKEVALUE(x,S_CS_EM)
#define G_CS_EM(x)              _DD_GETVALUE(x,S_CS_EM,M_CS_EM)

#define M_CS_DS                 _DD_MAKEMASK1(4)        /* DestSize */

#define S_CS_CD                 5                       /* ClkDivider */
#define M_CS_CD                 _DD_MAKEMASK(2,S_CS_CD)
#define V_CS_CD(x)              _DD_MAKEVALUE(x,S_CS_CD)
#define G_CS_CD(x)              _DD_GETVALUE(x,S_CS_CD,M_CS_CD)

#define M_CS_CE                 _DD_MAKEMASK1(7)        /* ClkEnable */
#define M_CS_SB                 _DD_MAKEMASK1(8)        /* Size/ByteStrobe */

/* CSO1MEMWAITCNT:  CS01 Memory Wait Count Register    (0x104, R/W) */
/* CS23MEMWAITCNT:  CS23 Memory Wait Count Register    (0x114, R/W) */
/* CS4MEMWAITCNT:   CS4  Memory Wait Count Register    (0x124, R/W) */

/* CS01ATTRWAITCNT: CS01 Attribute Wait Count Register (0x108, R/W) */
/* CS23ATTRWAITCNT: CS23 Attribute Wait Count Register (0x118, R/W) */
/* CS4ATTRWAITCNT:  CS4  Attribute Wait Count Register (0x128, R/W) */

/* CS01IOWAITCNT:   CS01 I/O Wait Count Register       (0x10C, R/W) */
/* CS23IOWAITCNT:   CS23 I/O Wait Count Register       (0x11C, R/W) */
/* CS4IOWAITCNT:    CS4  I/O Wait Count Register       (0x12C, R/W) */

#define S_CS_W0                 0                       /* WaitCount0 */
#define M_CS_W0                 _DD_MAKEMASK(6,S_CS_W0)
#define V_CS_W0(x)              _DD_MAKEVALUE(x,S_CS_W0)
#define G_CS_W0(x)              _DD_GETVALUE(x,S_CS_W0,M_CS_W0)

#define S_CS_W1                 8                      /* WaitCount1 */
#define M_CS_W1                 _DD_MAKEMASK(5,S_CS_W1)
#define V_CS_W1(x)              _DD_MAKEVALUE(x,S_CS_W1)
#define G_CS_W1(x)              _DD_GETVALUE(x,S_CS_W1,M_CS_W1)

#define S_CS_W2                 16                      /* WaitCount2 */
#define M_CS_W2                 _DD_MAKEMASK(5,S_CS_W2)
#define V_CS_W2(x)              _DD_MAKEVALUE(x,S_CS_W2)
#define G_CS_W2(x)              _DD_GETVALUE(x,S_CS_W2,M_CS_W2)

#define S_CS_W3                 24                       /* WaitCount0 */
#define M_CS_W3                 _DD_MAKEMASK(5,S_CS_W3)
#define V_CS_W3(x)              _DD_MAKEVALUE(x,S_CS_W3)
#define G_CS_W3(x)              _DD_GETVALUE(x,S_CS_W3,M_CS_W3)

#endif /* _SBCHIPC_H_ */
