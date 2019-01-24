/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Silicon Backplane definitions       	File: sb_mips.h
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

#ifndef _SBMIPS_H_
#define _SBMIPS_H_

/*
 * Register and bit definitions for the CPU control registers of the
 * MIPS (OCP ID 0x805) and MIPS33 (OCP ID 0x813) cores.
 */


/* MIPS Core 0x805 (Ref 1, Section 4) */

#define R_CORE_CONTROL          0x000
#define R_BIST_STATUS           0x004

/* CORECTL: Core Control Register (0x000, R/W) */

#define M_CORECTL_FR            _DD_MAKEMASK1(0)        /* ForceReset */
#define M_CORECTL_DF            _DD_MAKEMASK1(1)        /* DisableFlashExceptions */

/* BIST: BIST Status Register (0x00C, RO) */

#define S_BIST_BS               0                       /* BistStatus */
#define M_BIST_BS               _DD_MAKEMASK(8,S_BIST_BS)
#define V_BIST_BS(x)            _DD_MAKEVALUE(x,S_BIST_BS)
#define G_BIST_BS(x)            _DD_GETVALUE(x,S_BIST_BS,M_BIST_BS)


/* MIPS33 Core 0x813 Extensions (Ref 2, Section 10). */

#define R_BIST_CONTROL          0x008
#define R_INT_STATUS            0x020
#define R_INT_MASK              0x024
#define R_TIMER0                0x028
#define R_TEST_MUX_SELECT       0x030
#define R_TEST_MUX_ENABLE       0x034
#define R_EJTAG_PIO_EN          0x02c

/* BISTCTL: BIST Control Register (0x008, R/W) */

#define M_BISTCTL_BD            _DD_MAKEMASK1(0)        /* BISTDump */
#define M_BISTCTL_BG            _DD_MAKEMASK1(1)        /* BISTDebug */
#define M_BISTCTL_BH            _DD_MAKEMASK1(2)        /* BISTHold */

/* ISR: Interrupt Status Register (0x020, R/W) */
/* IMR: Interrupt Mask Register (0x024, R/W) */

#define M_INT_T0                _DD_MAKEMASK1(0)        /* Timer0 */

/* TIMER0: Timer0 Register (0x030, R/W) */

/* TMUXSEL: Test Multiplexer Select Register (0x028, R/W) */

#define S_TMUXSEL_SL            0                       /* SelLow */
#define M_TMUXSEL_SL            _DD_MAKEMASK(8,S_TMUXSEL_SL)
#define V_TMUXSEL_SL(x)         _DD_MAKEVALUE(x,S_TMUXSEL_SL)
#define G_TMUXSEL_SL(x)         _DD_GETVALUE(x,S_TMUXSEL_SL,M_TMUXSEL_SL)

#define S_TMUXSEL_SH            0                       /* SelHigh */
#define M_TMUXSEL_SH            _DD_MAKEMASK(8,S_TMUXSEL_SH)
#define V_TMUXSEL_SH(x)         _DD_MAKEVALUE(x,S_TMUXSEL_SH)
#define G_TMUXSEL_SH(x)         _DD_GETVALUE(x,S_TMUXSEL_SH,M_TMUXSEL_SH)

/* TMUXEN: Test Multiplexer Select Enable Register (0x034, R/W) */

/* EJTAGPIOEN: EJTAG GPIO Enable Register (0x02C, R/W) */

#endif /* _SBMIPS_H_ */
