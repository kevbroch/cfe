/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  MPC824x Registers			File: mpc824x_epic.h
    *  
    *  Motorola MPC824x Embedded Programmable Interrupt Controller
    *  (EPIC) registers and values
    *  
    *  Author:  Ed Satterthwaite
    *  
    *********************************************************************  
    *
    *  Copyright 2004
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

/*
 * All register offsets are relative to the value of EUMBBAR.
 * All are 32-bit registers and must be accessed as such.
 */

#define EPIC_FRR		0x41000
#define EPIC_GCR		0x41020
#define EPIC_EICR		0x41030
#define EPIC_EVI		0x41080
#define EPIC_PI			0x41090
#define EPIC_SVR		0x410E0
#define EPIC_TFRR		0x410F0

#define EPIC_GTCCR0		0x41100
#define EPIC_GTBCR0		0x41110
#define EPIC_GTVPR0		0x41120
#define EPIC_GTDR0		0x41130
#define EPIC_GTCCR1		0x41140
#define EPIC_GTBCR1		0x41150
#define EPIC_GTVPR1		0x41160
#define EPIC_GTDR1		0x41170
#define EPIC_GTCCR2		0x41180
#define EPIC_GTBCR2		0x41190
#define EPIC_GTVPR2		0x411A0
#define EPIC_GTDR2		0x411B0
#define EPIC_GTCCR3		0x411C0
#define EPIC_GTBCR3		0x411D0
#define EPIC_GTVPR3		0x411E0
#define EPIC_GTDR3		0x411F0


#define EPIC_IVPR0		0x50200		/* IRQn */
#define EPIC_IDR0		0x50210
#define EPIC_IVPR1		0x50220
#define EPIC_IDR1		0x50230
#define EPIC_IVPR2		0x50240
#define EPIC_IDR2		0x50250
#define EPIC_IVPR3		0x50260
#define EPIC_IDR3		0x50270
#define EPIC_IVPR4		0x50280
#define EPIC_IDR4		0x50290

#define EPIC_SVPR0		0x50200		/* serial interrupt n */
#define EPIC_SDR0		0x50210
#define EPIC_SVPR1		0x50220
#define EPIC_SDR1		0x50230
#define EPIC_SVPR2		0x50240
#define EPIC_SDR2		0x50250
#define EPIC_SVPR3		0x50260
#define EPIC_SDR3		0x50270
#define EPIC_SVPR4		0x50280
#define EPIC_SDR4		0x50290
#define EPIC_SVPR5		0x502A0
#define EPIC_SDR5		0x502B0
#define EPIC_SVPR6		0x502C0
#define EPIC_SDR6		0x502D0
#define EPIC_SVPR7		0x502E0
#define EPIC_SDR7		0x502F0
#define EPIC_SVPR8		0x50300
#define EPIC_SDR8		0x50310
#define EPIC_SVPR9		0x50320
#define EPIC_SDR9		0x50330
#define EPIC_SVPR10		0x50340
#define EPIC_SDR10		0x50350
#define EPIC_SVPR11		0x50360
#define EPIC_SDR11		0x50370
#define EPIC_SVPR12		0x50380
#define EPIC_SDR12		0x50390
#define EPIC_SVPR13		0x503A0
#define EPIC_SDR13		0x503B0
#define EPIC_SVPR14		0x503C0
#define EPIC_SDR14		0x503D0
#define EPIC_SVPR15		0x503E0
#define EPIC_SDR15		0x503F0

#define EPIC_IIVPR0		0x51020		/* i2c */
#define EPIC_IIDR0		0x51030
#define EPIC_IIVPR1		0x51040		/* DMA ch 0 */
#define EPIC_IIDR1		0x51050
#define EPIC_IIVPR2		0x51060		/* DMA ch 1 */
#define EPIC_IIDR2		0x51070
#define EPIC_IIVPR3		0x510C0		/* Message Unit */
#define EPIC_IIDR3		0x510D0


#define EPIC_PCTPR		0x60080
#define EPIC_IACK		0x600A0
#define EPIC_EOI		0x600B0


/*
 * Feature Reporting Register
 */

#define S_FRR_VID		0
#define M_FRR_VID		_MMR_MAKEMASK(8,S_FRR_VID)
#define V_FRR_VID(x)		_MMR_MAKEVALUE(x,S_FRR_VID)
#define G_FRR_VID(x)		_MMR_GETVALUE(x,S_FRR_VID,M_FRR_VID)

#define S_FRR_NCPU		8
#define M_FRR_NCPU		_MMR_MAKEMASK(5,S_FRR_NCPU)
#define V_FRR_NCPU(x)		_MMR_MAKEVALUE(x,S_FRR_NCPU)
#define G_FRR_NCPU(x)		_MMR_GETVALUE(x,S_FRR_NCPU,M_FRR_NCPU)

#define S_FRR_NIRQ		16
#define M_FRR_NIRQ		_MMR_MAKEMASK(11,S_FRR_NIRQ)
#define V_FRR_NIRQ(x)		_MMR_MAKEVALUE(x,S_FRR_NIRQ)
#define G_FRR_NIRQ(x)		_MMR_GETVALUE(x,S_FRR_NIRQ,M_FRR_NIRQ)

/*
 * Global Configuration Register
 */

#define M_GCR_M			_MMR_MAKEMASK1(29)
#define M_CCR_R			_MMR_MAKEMASK1(31)

/*
 * EPIC Interrupt Configuration Register
 */

#define M_EICR_SIE		_MMR_MAKEMASK1(27)

#define S_EICR_R		28
#define M_EICR_R		_MMR_MAKEMASK(3,S_EICR_R)
#define V_EICR_R(x)		_MMR_MAKEVALUE(x,S_EICR_R)
#define G_EICR_R(x)		_MMR_GETVALUE(x,S_EICR_R,M_EICR_R)

/*
 * EPIC Vendor Identification Register
 */

#define S_EVI_VENDOR_ID		0
#define M_EVI_VENDOR_ID		_MMR_MAKEMASK(8,S_EVI_VENDOR_ID)
#define V_EVI_VENDOR_ID(x)	_MMR_MAKEVALUE(x,S_EVI_VENDOR_ID)
#define G_EVI_VENDOR_ID(x)	_MMR_GETVALUE(x,S_EVI_VENDOR_ID,M_EVI_VENDOR_ID)

#define S_EVI_DEVICE_ID		8
#define M_EVI_DEVICE_ID		_MMR_MAKEMASK(8,S_EVI_DEVICE_ID)
#define V_EVI_DEVICE_ID(x)	_MMR_MAKEVALUE(x,S_EVI_DEVICE_ID)
#define G_EVI_DEVICE_ID(x)	_MMR_GETVALUE(x,S_EVI_DEVICE_ID,M_EVI_DEVICE_ID)

#define S_EVI_STEP		16
#define M_EVI_STEP		_MMR_MAKEMASK(8,S_EVI_STEP)
#define V_EVI_STEP(x)		_MMR_MAKEVALUE(x,S_EVI_STEP)
#define G_EVI_STEP(x)		_MMR_GETVALUE(x,S_EVI_STEP,M_EVI_STEP)

/*
 * Processor Initialization Register
 */

#define M_PI_P0			_MMR_MAKEMASK1(0)

/*
 * Spurious Vector Register
 */

#define S_SVR_VECTOR		0
#define M_SVR_VECTOR		_MMR_MAKEMASK(8,S_SVR_VECTOR)
#define V_SVR_VECTOR(x)		_MMR_MAKEVALUE(x,S_SVR_VECTOR)
#define G_SVR_VECTOR(x)		_MMR_GETVALUE(x,S_SVR_VECTOR,M_SVR_VECTOR)

/*
 * Timer Frequency Reporting Register
 */

#define S_TFRR_TIMER_FREQ	0
#define M_TFRR_TIMER_FREQ	_MMR_MAKEMASK(32,S_TFRR_TIMER_FREQ)
#define V_TFRR_TIMER_FREQ(x)	_MMR_MAKEVALUE(x,S_TFRR_TIMER_FREQ)
#define G_TFRR_TIMER_FREQ(x)	_MMR_GETVALUE(x,S_TFRR_TIMER_FREQ,M_TFRR_TIMER_FREQ)

/*
 * Global Time Current Count Registers
 */

#define S_GTCCR_COUNT		0
#define M_GTCCR_COUNT		_MMR_MAKEMASK(31,S_GTCCR_COUNT)
#define V_GTCCR_COUNT(x)	_MMR_MAKEVALUE(x,S_GTCCR_COUNT)
#define G_GTCCR_COUNT(x)	_MMR_GETVALUE(x,S_GTCCR_COUNT,M_GTCCR_COUNT)

#define M_GTCCR_T		_MMR_MAKEMASK1(31)

/*
 * Global Timer Base Count Registers
 */

#define S_GTBCR_BASE_COUNT	0
#define M_GTBCR_BASE_COUNT	_MMR_MAKEMASK(31,S_GTBCR_BASE_COUNT)
#define V_GTBCR_BASE_COUNT(x)	_MMR_MAKEVALUE(x,S_GTBCR_BASE_COUNT)
#define G_GTBCR_BASE_COUNT(x)	_MMR_GETVALUE(x,S_GTBCR_BASE_COUNT,M_GTBCR_BASE_COUNT)

#define M_GTBCR_CI		_MMR_MAKEMASK1(31)

/*
 * Global Timer Vector/Priority Registers
 */

#define S_GTVPR_VECTOR		0
#define M_GTVPR_VECTOR		_MMR_MAKEMASK(8,S_GTVPR_VECTOR)
#define V_GTVPR_VECTOR(x)	_MMR_MAKEVALUE(x,S_GTVPR_VECTOR)
#define G_GTVPR_VECTOR(x)	_MMR_GETVALUE(x,S_GTVPR_VECTOR,M_GTVPR_VECTOR)

#define S_GTVPR_PRIORITY	16
#define M_GTVPR_PRIORITY	_MMR_MAKEMASK(4,S_GTVPR_PRIORITY)
#define V_GTVPR_PRIORITY(x)	_MMR_MAKEVALUE(x,S_GTVPR_PRIORITY)
#define G_GTVPR_PRIORITY(x)	_MMR_GETVALUE(x,S_GTVPR_PRIORITY,M_GTVPR_PRIORITY)

#define M_GTVPR_A		_MMR_MAKEMASK1(30)
#define M_GTVPR_M		_MMR_MAKEMASK1(31)

/*
 * Global Timer Destination Registers
 */

#define M_GTDR_P0		_MMR_MAKEMASK1(0)

/*
 * Direct Interrupt Vector/Priority Registers
 * Serial Interrupt Vector/Priority Registers
 * Internal Interrupt Vector/Priority Registers
 */

#define S_VPR_VECTOR		0
#define M_VPR_VECTOR		_MMR_MAKEMASK(8,S_VPR_VECTOR)
#define V_VPR_VECTOR(x)		_MMR_MAKEVALUE(x,S_VPR_VECTOR)
#define G_VPR_VECTOR(x)		_MMR_GETVALUE(x,S_VPR_VECTOR,M_VPR_VECTOR)

#define S_VPR_PRIORITY		16
#define M_VPR_PRIORITY		_MMR_MAKEMASK(4,S_VPR_PRIORITY)
#define V_VPR_PRIORITY(x)	_MMR_MAKEVALUE(x,S_VPR_PRIORITY)
#define G_VPR_PRIORITY(x)	_MMR_GETVALUE(x,S_VPR_PRIORITY,M_VPR_PRIORITY)

#define M_VPR_S			_MMR_MAKEMASK1(22)
#define M_VPR_P			_MMR_MAKEMASK1(23)
#define M_VPR_A			_MMR_MAKEMASK1(30)
#define M_VPR_M			_MMR_MAKEMASK1(31)

/*
 * Direct Interrupt Destination Registers
 * Serial Interrupt Destination Registers
 * Internal Interrupt Destination Registers
 */

#define M_DR_P0			_MMR_MAKEMASK1(0)

/*
 * Processor Current Task Priority Register
 */

#define S_PCTPR_TASKP		0
#define M_PCTPR_TASKP		_MMR_MAKEMASK(4,S_PCTPR_TASKP)
#define V_PCTPR_TASKP(x)	_MMR_MAKEVALUE(x,S_PCTPR_TASKP)
#define G_PCTPR_TASKP(x)	_MMR_GETVALUE(x,S_PCTPR_TASKP,M_PCTPR_TASKP)


/*
 * Processor Interrupt Acknowledge Register
 */

#define S_IACK_VECTOR		0
#define M_IACK_VECTOR		_MMR_MAKEMASK(8,S_IACK_VECTOR)
#define V_IACK_VECTOR(x)	_MMR_MAKEVALUE(x,S_IACK_VECTOR)
#define G_IACK_VECTOR(x)	_MMR_GETVALUE(x,S_IACK_VECTOR,M_IACK_VECTOR)

/*
 * Processor End-of-Interrupt Register
 */

#define S_EOI_CODE		0
#define M_EOI_CODE		_MMR_MAKEMASK(4,S_EOI_CODE)
#define V_EOI_CODE(x)		_MMR_MAKEVALUE(x,S_EOI_CODE)
#define G_EOI_CODE(x)		_MMR_GETVALUE(x,S_EOI_CODE,M_EOI_CODE)
