/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Silicon Backplane external interface        File: sb_extif.h
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

#ifndef _SBEXTIF_H_
#define _SBEXTIF_H_

/*
 * Register and bit definitions for the External Interface control
 * registers (OCP ID 0x811)
 */


/* External Interface Core (Section 11) */

#define R_CORECNTL              0x000
#define R_EXTSTATUS             0x004
/* PCMCIA Control Registers */
#define R_PCMCIACONFIG          0x010
#define R_PCMCIAMEMWAITCNT      0x014
#define R_PCMCIAATTRWAITCNT     0x018
#define R_PCMCIAIOWAITCNT       0x01C

/* Programmable Interface Control Registers */
#define R_PROGINTCONFIG         0x020
#define R_PROGWAITCNT           0x024

/* Flash Control Registers */
#define R_FLASHCONFIG           0x028
#define R_FLASHWAITCNT          0x02C

#define R_WATCHDOGCNTR          0x040

/* Clock Control Registers (Section 11.1) */
#define R_CLOCKCONTROLN         0x044
#define R_CLOCKCONTROLSB        0x048
#define R_CLOCKCONTROLPCI       0x04C
#define R_CLOCKCONTROLMII       0x050

/* GPIO Registers (Section 11.2) */
#define R_GPIOINPUT             0x060
#define R_GPIOOUTPUT0           0x064
#define R_GPIOOUTPUT1           0x06C
#define R_GPIOOUTPUT2           0x074
#define R_GPIOOUTPUT3           0x07C
#define R_GPIOOUTPUT4           0x084
#define R_GPIOOUTEN0            0x068
#define R_GPIOOUTEN1            0x070
#define R_GPIOOUTEN2            0x078
#define R_GPIOOUTEN3            0x080
#define R_GPIOOUTEN4            0x088
#define R_EJTAGOUTEN            0x090
#define R_GPIOINTPOLARITY       0x094
#define R_GPIOINTMASK           0x098


/* CORECTL: Core Control Register (0x000, R/W) */

#define M_CORECTL_UE            _DD_MAKEMASK1(0)        /* UartEnable */

/* EXTSTAT: External Status Register (0x004, RO) */

#define M_EXTSTAT_EM            _DD_MAKEMASK1(0)        /* EndianMode */
#define M_EXTSTAT_EI            _DD_MAKEMASK1(1)        /* ExtInt */
#define M_EXTSTAT_GI            _DD_MAKEMASK1(2)        /* GPIOInt */


/* PCMCIACFG: PCMCIA Configuration Register (0x010, R/W) */
/* PROGCFG:   Programmable Interface Configuration Register (0x020, R/W) */
/* FLASHCFG:  Flash Configuration Register (0x028, R/W) */

#define M_IFCFG_EN              _DD_MAKEMASK1(0)        /* Enable */

#define S_IFCFG_EM              1                       /* ExtIfMode */
#define M_IFCFG_EM              _DD_MAKEMASK(3,S_IFCFG_EM)
#define V_IFCFG_EM(x)           _DD_MAKEVALUE(x,S_IFCFG_EM)
#define G_IFCFG_EM(x)           _DD_GETVALUE(x,S_IFCFG_EM,M_IFCFG_EM)
#define K_EM_ASYNC              0
#define K_EM_SYNC               1
#define K_EM_PCMCIA             2

#define M_IFCFG_DS              _DD_MAKEMASK1(4)        /* DestSize */
#define M_IFCFG_BS              _DD_MAKEMASK1(5)        /* ByteSwap */

#define S_IFCFG_CD              6                       /* ClockDivider */
#define M_IFCFG_CD              _DD_MAKCDASK(2,S_IFCFG_CD)
#define V_IFCFG_CD(x)           _DD_MAKEVALUE(x,S_IFCFG_CD)
#define G_IFCFG_CD(x)           _DD_GETVALUE(x,S_IFCFG_CD,M_IFCFG_CD)

#define M_IFCFG_CE              _DD_MAKEMASK1(8)        /* ClockEnable */
#define M_IFCFG_SB              _DD_MAKEMASK1(9)        /* Size/ByteStrobe */


/* WDOG: Watchdog Counter Register (0x040, R/W) */


/* CCN: Clock Control N Register (0x044, R/W, buffered) */

#define S_CCN_N1                0                       /* N1Control */
#define M_CCN_N1                _DD_MAKEMASK(6,S_CCN_N1)
#define V_CCN_N1(x)             _DD_MAKEVALUE(x,S_CCN_N1)
#define G_CCN_N1(x)             _DD_GETVALUE(x,S_CCN_N1,M_CCN_N1)

#define S_CCN_N2                8                       /* N2Control */
#define M_CCN_N2                _DD_MAKEMASK(5,S_CCN_N2)
#define V_CCN_N2(x)             _DD_MAKEVALUE(x,S_CCN_N2)
#define G_CCN_N2(x)             _DD_GETVALUE(x,S_CCN_N2,M_CCN_N2)

/* CCSB:  Clock Control SB Register (0x048, R/W, buffered) */
/* CCPCI: Clock Control PCI Register (0x04C, R/W, buffered) */
/* CCMII: Clock Control MII Register (0x050, R/W, buffered) */

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

#define S_CCM_MC                24                       /* MuxControl */
#define M_CCM_MC                _DD_MAKEMASK(5,S_CCM_MC)
#define V_CCM_MC(x)             _DD_MAKEVALUE(x,S_CCM_MC)
#define G_CCM_MC(x)             _DD_GETVALUE(x,S_CCM_MC,M_CCM_MC)


#endif /* _SBEXTIF_H_ */
