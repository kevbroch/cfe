/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Silicon Backplane definitions       	File: sb_bp.c
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

#ifndef _SBBP_H_
#define _SBBP_H_

#define _DD_MAKEMASK1(n) (1 << (n))
#define _DD_MAKEMASK(v,n) ((((1)<<(v))-1) << (n))
#define _DD_MAKEVALUE(v,n) ((v) << (n))
#define _DD_GETVALUE(v,n,m) (((v) & (m)) >> (n))

/*
 * Definitions for the address map and core configuration space
 * registers of Broadcom SiliconBackplane products.
 *
 * These products contain multiple cores that communicate via a
 * system backplane ("Silicon Backplane") internal to the chip.
 *
 * Each core occupies 4K (0x1000) bytes of "enumeration" space.
 * Enumeration spaces are contiguous and begin at 0x18000000, but both
 * the selection and ordering of cores is chip-specific.
 *
 * Each enumeration space includes a configuration region at offset
 * 0xF00 consisting of registers to identify the core and map its
 * resources.  Some cores, including the MAC, have a standard DMA
 * engine; if so, its registers appear at offset 0x200 in enumeration
 * space.
 */

/* Vendor and Core identifiers */

#define K_VN_BROADCOM           0x4243

#define K_CR_CHIP_COMMON        0x800
#define K_CR_ILINE20            0x801
#define K_CR_SDRAM              0x803
#define K_CR_PCI                0x804
#define K_CR_MIPS               0x805
#define K_CR_ENET               0x806
#define K_CR_CODEC              0x807
#define K_CR_USB                0x808
#define K_CR_ILINE100           0x80A
#define K_CR_IPSEC              0x80B
#define K_CR_PCMCIA             0x80D
#define K_CR_MEMC               0x80F
#define K_CR_EXT                0x811
#define K_CR_MAC11              0x812
#define K_CR_MIPS33             0x816


/* Each core gets a 4K window for registers. */

#define SB_CORE_SIZE   	        0x1000

/* Core Configuration Space Registers (Section 2, Table 3) */

/* These registers are found at the top of the 4K window for each core. */
 
#define R_SBIPSFLAG             0xF08
#define R_SBTPSFLAG             0xF18
#define R_SBTMERRLOGA           0xF48
#define R_SBTMERRLOG            0xF50
#define R_SBADMATCH3            0xF60
#define R_SBADMATCH2            0xF68
#define R_SBADMATCH1            0xF70
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


/* SBISF: System Backplane Initiator Slave Flag Control Register (0xF08, R/W) */

#define S_SBISF_F1              0                       /* IntFlag1  */
#define M_SBISF_F1              _DD_MAKEMASK(6,S_SBISF_F1)
#define V_SBISF_F1(x)           _DD_MAKEVALUE(x,S_SBISF_F1)
#define G_SBISF_F1(x)           _DD_GETVALUE(x,S_SBISF_F1,M_SBISF_F1)

#define S_SBISF_F2              8                       /* IntFlag2  */
#define M_SBISF_F2              _DD_MAKEMASK(6,S_SBISF_F2)
#define V_SBISF_F2(x)           _DD_MAKEVALUE(x,S_SBISF_F2)
#define G_SBISF_F2(x)           _DD_GETVALUE(x,S_SBISF_F2,M_SBISF_F2)

#define S_SBISF_F3              16                      /* IntFlag3  */
#define M_SBISF_F3              _DD_MAKEMASK(6,S_SBISF_F3)
#define V_SBISF_F3(x)           _DD_MAKEVALUE(x,S_SBISF_F3)
#define G_SBISF_F3(x)           _DD_GETVALUE(x,S_SBISF_F3,M_SBISF_F3)

#define S_SBISF_F4              24                      /* IntFlag4  */
#define M_SBISF_F4              _DD_MAKEMASK(6,S_SBISF_F4)
#define V_SBISF_F4(x)           _DD_MAKEVALUE(x,S_SBISF_F4)
#define G_SBISF_F4(x)           _DD_GETVALUE(x,S_SBISF_F4,M_SBISF_F4)

/* SBTSF: System Backplane Target Slave Flag Control Register (0xF18, RO) */

#define S_SBTSF_FN              0                       /* SBPTSFlagNum0  */
#define M_SBTSF_FN              _DD_MAKEMASK(6,S_SBTSF_FN)
#define V_SBTSF_FN(x)           _DD_MAKEVALUE(x,S_SBTSF_FN)
#define G_SBTSF_FN(x)           _DD_GETVALUE(x,S_SBTSF_FN,M_SBTSF_FN)

#define M_SBTSF_FE              _DD_MAKEMASK1(6)        /* SBTPSFlag0En0 */

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
#define M_SBINT_MK               _DD_MAKEMASK(7,S_SBINT_MK)
#define V_SBINT_MK(x)            _DD_MAKEVALUE(x,S_SBINT_MK)
#define G_SBINT_MK(x)            _DD_GETVALUE(x,S_SBINT_MK,M_SBINT_MK)
/* XXX For the BCM4401 */
#define K_SBINT_ENET_MAC         (1<<1)

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

/* USB core only */
#define M_SBTS_UH               _DD_MAKEMASK1(29)       /* USBHost */

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

#define S_SBID_VN               16                      /* Vendor */
#define M_SBID_VN               _DD_MAKEMASK(16,S_SBID_VN)
#define V_SBID_VN(x)            _DD_MAKEVALUE(x,S_SBID_VN)
#define G_SBID_VN(x)            _DD_GETVALUE(x,S_SBID_VN,M_SBID_VN)

#endif /* _SBBP_H_ */
