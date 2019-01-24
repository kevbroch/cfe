/*  *********************************************************************
    *  PowerPC Board Support Package
    *  
    *  PPC CPU definitions			File: ppcdefs.h
    * 
    *  This module contains constants and macros specific to the
    *  PowerPC core.
    *  
    *  Author:  Mitch Lichtenberg
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


/*  *********************************************************************
    *  Bitfield macros
    *
    *  Two variants of these macros, to make it easier to 
    *  import the bit fields from existing documentation.
    *
    *    _MML_xxx   makes bitmasks numbered from the left (as you
    *               might find in the core documentation
    *               bit zero is the MOST signficant bit.
    *		    When defining multiple-bit fields,
    *		    we still reference the first bit as the
    *               least significant bit.  So, if you number
    *		    your bits from left to right, and want
    *               to make a mask with the top 5 bits in the
    *               register (leftmost bit), then the bit field
    *               starts at bit 4 (bits 0..4).
    *
    *    _MMR_xxx   makes bitmasks numbered from the right, with
    *               bit zero as the LEAST significant bit.  This is
    *               more what we're used to.
    ********************************************************************* */

/*
 * Make a mask for 1 bit at position 'n'
 */

#define _MMR_MAKEMASK1(n) (1 << (n))
#define _MML_MAKEMASK1(n) (1 << (31-(n)))

/*
 * Make a mask for 'v' bits at position 'n'
 * bit position 'n' is the _least significant_ bit of a multi-bit field
 */

#define _MMR_MAKEMASK(v,n) (((1<<(v))-1) << (n))
#define _MML_MAKEMASK(v,n) (((1<<(v))-1) << (31-(n)))

/*
 * Make a value at 'v' at bit position 'n'.
 * bit position 'n' is the _least significant_ bit of a multi-bit field
 */

#define _MMR_MAKEVALUE(v,n) ((v) << (n))
#define _MML_MAKEVALUE(v,n) ((v) << (31-(n)))

/*
 * Retrieve a value from 'v' at bit position 'n' with 'm' mask bits
 * bit position 'n' is the _least significant_ bit of a multi-bit field
 */

#define _MMR_GETVALUE(v,n,m) (((v) & (m)) >> (n))
#define _MML_GETVALUE(v,n,m) (((v) & (m)) >> (31-(n)))


/*  *********************************************************************
    *  General Registers
    ********************************************************************* */

#define r0 0
#define r1 1
#define r2 2
#define r3 3
#define r4 4
#define r5 5
#define r6 6
#define r7 7
#define r8 8
#define r9 9
#define r10 10
#define r11 11
#define r12 12
#define r13 13
#define r14 14
#define r15 15
#define r16 16
#define r17 17
#define r18 18
#define r19 19
#define r20 20
#define r21 21
#define r22 22
#define r23 23
#define r24 24
#define r25 25
#define r26 26
#define r27 27
#define r28 28
#define r29 29
#define r30 30
#define r31 31

/*  *********************************************************************
    *  Special Processor Registers
    ********************************************************************* */

#define SPR_XER		1
#define SPR_LR		8
#define SPR_CTR		9

#define SPR_ICR		148

#define SPR_TBL		268
#define SPR_TBU		269

#define SPR_HID0	1008
#define SPR_HID1	1009
#define SPR_HID2	1011

#define SPR_MBAR	311
#define SPR_SVR		286
#define SPR_PVR		287 

#define SPR_IBAT0U	528
#define SPR_IBAT0L	529
#define SPR_IBAT1U	530
#define SPR_IBAT1L	531
#define SPR_IBAT2U	532
#define SPR_IBAT2L	533
#define SPR_IBAT3U	534
#define SPR_IBAT3L	535

#define SPR_DBAT0U	536
#define SPR_DBAT0L	537
#define SPR_DBAT1U	538
#define SPR_DBAT1L	539
#define SPR_DBAT2U	540
#define SPR_DBAT2L	541
#define SPR_DBAT3U	542
#define SPR_DBAT3L	543

#define SPR_IBAT4U	560
#define SPR_IBAT4L	561
#define SPR_IBAT5U	562
#define SPR_IBAT5L	563
#define SPR_IBAT6U	564
#define SPR_IBAT6L	565
#define SPR_IBAT7U	566
#define SPR_IBAT7L	567

#define SPR_DBAT4U	568
#define SPR_DBAT4L	569
#define SPR_DBAT5U	570
#define SPR_DBAT5L	571
#define SPR_DBAT6U	572
#define SPR_DBAT6L	573
#define SPR_DBAT7U	574
#define SPR_DBAT7L	575

#define SPR_DMISS	976
#define SPR_DCMP	977
#define SPR_HASH1	978
#define SPR_HASH2	979
#define SPR_IMISS	980
#define SPR_ICMP	981
#define SPR_RPA		982

#define SPR_SDR1	25

#define SPR_SPRG0	272
#define SPR_SPRG1	273
#define SPR_SPRG2	274
#define SPR_SPRG3	275
#define SPR_SPRG4	276
#define SPR_SPRG5	277
#define SPR_SPRG6	278
#define SPR_SPRG7	279

#define SPR_DSISR	18
#define SPR_SRR0	26
#define SPR_SRR1	27
#define SPR_DAR		19
#define SPR_DEC		22
#define SPR_EAR		282
#define SPR_WTBL	284
#define SPR_WTBU	285
#define SPR_WTBH	SPR_WTBU

#define SPR_CSRR0	58
#define SPR_CSRR1	59

#define SPR_IABR	1010
#define SPR_IABR2	1018
#define SPR_DABR	1013
#define SPR_DABR2	317

#define SPR_IBCR	309
#define SPR_DBCR	310

/*  *********************************************************************
    *  Processor Version Register
    ********************************************************************* */

/*
 * It's a little strange looking, but for clarity we define
 * the bit positions as "posn + width - 1"
 * where 'posn' is the left side of the bit field from the core manual,
 * 'width' is the width of the field from the core manual.  We 
 * subtract 1 since we number our bits from zero.
 *
 * For example, a 1-bit field in position 0 will be 0+1-1 = 0,
 * just what you expect.
 * A 3 bit field at position 0 will be 0+3-1 = 2, meaning the
 * low-order bit of the 3-bit field is in position 2, the 3rd
 * bit from the left.
 */

#define S_PVR_CID	(0+4-1)
#define M_PVR_CID	_MML_MAKEMASK(4,S_PVR_CID)
#define V_PVR_CID(x)	_MML_MAKEVALUE(x,S_PVR_CID)
#define G_PVR_CID(x)	_MML_GETVALUE(x,S_PVR_CID,M_PVR_CID)

#define S_PVR_PT	(6+4-1)
#define M_PVR_PT	_MML_MAKEMASK(4,S_PVR_PT)
#define V_PVR_PT(x)	_MML_MAKEVALUE(x,S_PVR_PT)
#define G_PVR_PT(x)	_MML_GETVALUE(x,S_PVR_PT,M_PVR_PT)

#define S_PVR_PID	(10+6-1)
#define M_PVR_PID	_MML_MAKEMASK(6,S_PVR_PID)
#define V_PVR_PID(x)	_MML_MAKEVALUE(x,S_PVR_PID)
#define G_PVR_PID(x)	_MML_GETVALUE(x,S_PVR_PID,M_PVR_PID)

#define S_PVR_PROC	(16+4-1)
#define M_PVR_PROC	_MML_MAKEMASK(4,S_PVR_PROC)
#define V_PVR_PROC(x)	_MML_MAKEVALUE(x,S_PVR_PROC)
#define G_PVR_PROC(x)	_MML_GETVALUE(x,S_PVR_PROC,M_PVR_PROC)

#define S_PVR_MFG	(20+4-1)
#define M_PVR_MFG	_MML_MAKEMASK(4,S_PVR_MFG)
#define V_PVR_MFG(x)	_MML_MAKEVALUE(x,S_PVR_MFG)
#define G_PVR_MFG(x)	_MML_GETVALUE(x,S_PVR_MFG,M_PVR_MFG)

#define S_PVR_MJREV	(24+4-1)
#define M_PVR_MJREV	_MML_MAKEMASK(4,S_PVR_MJREV)
#define V_PVR_MJREV(x)	_MML_MAKEVALUE(x,S_PVR_MJREV)
#define G_PVR_MJREV(x)	_MML_GETVALUE(x,S_PVR_MJREV,M_PVR_MJREV)

#define S_PVR_MNREV	(28+4-1)
#define M_PVR_MNREV	_MML_MAKEMASK(4,S_PVR_MNREV)
#define V_PVR_MNREV(x)	_MML_MAKEVALUE(x,S_PVR_MNREV)
#define G_PVR_MNREV(x)	_MML_GETVALUE(x,S_PVR_MNREV,M_PVR_MNREV)

/*  *********************************************************************
    *  Machine State Register
    ********************************************************************* */

#define M_MSR_POW	_MML_MAKEMASK1(13)
#define M_MSR_TGPR	_MML_MAKEMASK1(14)
#define M_MSR_ILE	_MML_MAKEMASK1(15)
#define M_MSR_EE	_MML_MAKEMASK1(16)
#define M_MSR_PR	_MML_MAKEMASK1(17)
#define M_MSR_FP	_MML_MAKEMASK1(18)
#define M_MSR_ME	_MML_MAKEMASK1(19)
#define M_MSR_FE0	_MML_MAKEMASK1(20)
#define M_MSR_SE        _MML_MAKEMASK1(21)
#define M_MSR_BE        _MML_MAKEMASK1(22)
#define M_MSR_FE1       _MML_MAKEMASK1(23)
#define M_MSR_CE        _MML_MAKEMASK1(24)
#define M_MSR_IP        _MML_MAKEMASK1(25)
#define M_MSR_IR        _MML_MAKEMASK1(26)
#define M_MSR_DR        _MML_MAKEMASK1(27)
#define M_MSR_RI        _MML_MAKEMASK1(30)
#define M_MSR_LE        _MML_MAKEMASK1(31)

/*  *********************************************************************
    *  Hardware Implementation Register (HID0)
    ********************************************************************* */

#define M_HID0_EMCP	_MML_MAKEMASK1(0)
#define M_HID0_EBA	_MML_MAKEMASK1(2)
#define M_HID0_EBD	_MML_MAKEMASK1(3)
#define M_HID0_SBCLK	_MML_MAKEMASK1(4)
#define M_HID0_ECLK     _MML_MAKEMASK1(6)
#define M_HID0_PAR	_MML_MAKEMASK1(7)
#define M_HID0_DOZE	_MML_MAKEMASK1(8)
#define M_HID0_NAP	_MML_MAKEMASK1(9)
#define M_HID0_SLEEP	_MML_MAKEMASK1(10)
#define M_HID0_DPM	_MML_MAKEMASK1(11)

#define M_HID0_ICE	_MML_MAKEMASK1(16)
#define M_HID0_DCE	_MML_MAKEMASK1(17)
#define M_HID0_ILOCK	_MML_MAKEMASK1(18)
#define M_HID0_DLOCK	_MML_MAKEMASK1(19)
#define M_HID0_ICFI	_MML_MAKEMASK1(20)
#define M_HID0_DCFI	_MML_MAKEMASK1(21)

#define M_HID0_IFEM	_MML_MAKEMASK1(24)

#define M_HID0_FBIOB	_MML_MAKEMASK1(27)
#define M_HID0_ABE	_MML_MAKEMASK1(28)

#define M_HID0_NOOPTI	_MML_MAKEMASK1(31)

/*  *********************************************************************
    *  Hardware Implementation Register (HID1)
    ********************************************************************* */

#define M_HID1_PC0	_MML_MAKEMASK1(0)
#define M_HID1_PC1	_MML_MAKEMASK1(1)
#define M_HID1_PC2	_MML_MAKEMASK1(2)
#define M_HID1_PC3	_MML_MAKEMASK1(3)
#define M_HID1_PC4	_MML_MAKEMASK1(4)

#define S_HID1_PLLCFG	(0+5-1)
#define M_HID1_PLLCFG	_MML_MAKEMASK(5,S_HID1_PLLCFG)
#define V_HID1_PLLCFG(x) _MML_MAKEVALUE(x,S_HID1_PLLCFG)
#define G_HID1_PLLCFG(x) _MML_GETVALUE(x,S_HID1_PLLCFG,M_HID1_PLLCFG)

/*  *********************************************************************
    *  Hardware Implementation Register (HID2)
    ********************************************************************* */

#define M_HID2_LET	_MML_MAKEMASK1(4)
#define M_HID2_HBE	_MML_MAKEMASK1(13)
#define M_HID2_SFP	_MML_MAKEMASK1(15)

#define S_HID2_IWLCK	(16+3-1)
#define M_HID2_IWLCK	_MML_MAKEMASK(3,S_HID2_IWLCK)
#define V_HID2_IWLCK(x)	_MML_MAKEVALUE(x,S_HID2_IWLCK)
#define G_HID2_IWLCK(x)	_MML_GETVALUE(x,S_HID2_IWLCK,M_HID2_IWLCK)

#define S_HID2_DWLCK	(24+3-1)
#define M_HID2_DWLCK	_MML_MAKEMASK(3,S_HID2_DWLCK)
#define V_HID2_DWLCK(x)	_MML_MAKEVALUE(x,S_HID2_DWLCK)
#define G_HID2_DWLCK(x)	_MML_GETVALUE(x,S_HID2_DWLCK,M_HID2_DWLCK)

/*  *********************************************************************
    *  BAT registers
    ********************************************************************* */

#define S_BATU_BEPI	(0+15-1)
#define M_BATU_BEPI	_MML_MAKEMASK(15,S_BATU_BEPI)
#define V_BATU_BEPI(x)	_MML_MAKEVALUE(x,S_BATU_BEPI)
#define G_BATU_BEPI(x)	_MML_GETVALUE(x,S_BATU_BEPI,M_BATU_BEPI)

#define S_BATU_BL	(19+11-1)
#define M_BATU_BL	_MML_MAKEMASK(11,S_BATU_BL)
#define V_BATU_BL(x)	_MML_MAKEVALUE(x,S_BATU_BL)
#define G_BATU_BL(x)	_MML_GETVALUE(x,S_BATU_BL,M_BATU_BL)

#define K_BATU_BL_128K	0x000
#define K_BATU_BL_256K	0x001
#define K_BATU_BL_512K	0x003
#define K_BATU_BL_1M	0x007
#define K_BATU_BL_2M	0x00F
#define K_BATU_BL_4M	0x01F
#define K_BATU_BL_8M	0x03F
#define K_BATU_BL_16M	0x07F
#define K_BATU_BL_32M	0x0FF
#define K_BATU_BL_64M	0x1FF
#define K_BATU_BL_128M	0x3FF
#define K_BATU_BL_256M	0x7FF

#define M_BATU_VS	_MML_MAKEMASK1(30)
#define M_BATU_VP	_MML_MAKEMASK1(31)


#define S_BATL_BRPN	(0+15-1)
#define M_BATL_BRPN	_MML_MAKEMASK(15,S_BATL_BRPN)
#define V_BATL_BRPN(x)	_MML_MAKEVALUE(x,S_BATL_BRPN)
#define G_BATL_BRPN(x)	_MML_GETVALUE(x,S_BATL_BRPN,M_BATL_BRPN)

#define S_BATL_WIMG	(25+4-1)
#define M_BATL_WIMG	_MML_MAKEMASK(4,S_BATL_WIMG)
#define V_BATL_WIMG(x)	_MML_MAKEVALUE(x,S_BATL_WIMG)
#define G_BATL_WIMG(x)	_MML_GETVALUE(x,S_BATL_WIMG,M_BATL_WIMG)

#define M_BATL_WRITETHROUGH _MML_MAKEMASK1(25)
#define M_BATL_INHIBIT      _MML_MAKEMASK1(26)
#define M_BATL_MEMCOHERENCE _MML_MAKEMASK1(27)
#define M_BATL_GUARDED      _MML_MAKEMASK1(28)

#define S_BATL_PP	(30+2-1)
#define M_BATL_PP	_MML_MAKEMASK(2,S_BATL_PP)
#define V_BATL_PP(x)	_MML_MAKEVALUE(x,S_BATL_PP)
#define G_BATL_PP(x)	_MML_GETVALUE(x,S_BATL_PP,M_BATL_PP)

#define K_BATL_PP_NOACC	 0
#define K_BATL_PP_RDONLY 1
#define K_BATL_PP_RDWR	 2
#define K_BATL_PP_3	 3


/*  *********************************************************************
    *  Exception offsets
    ********************************************************************* */

#define EXC_OFF_RESET	0x100
#define EXC_OFF_MCHECK	0x200
#define EXC_OFF_DSI	0x300
#define EXC_OFF_ISI	0x400
#define EXC_OFF_INT	0x500
#define EXC_OFF_ALIGN	0x600
#define EXC_OFF_PROGRAM	0x700
#define EXC_OFF_FPUNVL	0x800
#define EXC_OFF_DECR	0x900
#define EXC_OFF_CINT	0xA00
#define EXC_OFF_SYSCALL	0xC00
#define EXC_OFF_TRACE	0xD00
#define EXC_OFF_RSVD	0xE00
#define EXC_OFF_ITLBMISS 0x1000
#define EXC_OFF_DTLBMISS_LD 0x1100
#define EXC_OFF_DTLBMISS_ST 0x1200
#define EXC_OFF_IBREAK	0x1300
#define EXC_OFF_SMI	0x1400


/*  *********************************************************************
    *  Macros for common instruction sequences
    ********************************************************************* */


/*  *********************************************************************
    *  Macros for generating assembly language routines
    ********************************************************************* */

#if defined(__ASSEMBLER__)

/* global leaf function (does not call other functions) */
#define LEAF(name)		\
  	.align 2;               \
        .globl name;		\
        .type name,@function;   \
name:


/* end of a global function */
#define END(name) \
  	.size	name,.-name; 

/* define & export a symbol */
#define EXPORT(name)		\
  	.globl name;		\
name:

/* import a symbol */
#define	IMPORT(name, size)	\
	.extern	name,size

/* define a zero-fill common block (BSS if not overridden) with a global name */
#define COMM(name,size)		\
	.comm	name,size

/* define a zero-fill common block (BSS if not overridden) with a local name */
#define LCOMM(name,size)		\
  	.lcomm	name,size

#endif
