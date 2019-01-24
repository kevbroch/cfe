/*  *********************************************************************
    *  BCM1280/BCM1480 Board Support Package
    *  
    *  DRAM Startup Module  		      File: bcm1480_draminit.c
    *  
    *  This module contains code to initialize and start the DRAM
    *  controller on the BCM1255/BCM1280/BCM1455/BCM1480.
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2004,2005
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
 * This code can be linked into non-CFE, non-BCM1480 things like
 * SOCVIEW, a JTAG tool.  In that case it's not even running on a
 * 1400, but we can borrow the code to generate timing values for us.
 * 
 * The _MCSTANDALONE_ ifdef is normally turned *off* for firmware use,
 * but programs like "memconfig" (CFE host tool) or SOCVIEW use it
 * to allow us to run the memory initialization outside a 1400.  */

#ifdef _MCSTANDALONE_
#include <stdio.h>
#include <string.h>
#else
#include "sbmips.h"
#endif

#include "bcm1480_regs.h"
#include "bcm1480_mc.h"
#include "bcm1480_scd.h"
#include "sb1250_smbus.h"

/*
 * Uncomment to use data mover to zero memory 
 * Note: this is not a good idea in Pass1, since we'll
 * be running cacheable noncoherent at this point in the
 * CFE init sequence.
 */
/* #define _DMZERO_ */

#ifdef _DMZERO_
#include "sb1250_dma.h"
#endif

/*  *********************************************************************
    *  Magic Constants
    ********************************************************************* */

/* 
 * This constant represents the "round trip" time of your board.
 * Measured from the pins on the BCM1480, it is the time from the
 * rising edge of the MCLK pin to the rising edge of the DQS coming
 * back from the memory.  (Note: 1250 values used for now.)
 *
 * It is used in the calculation of which cycle responses are expected
 * from the memory for a given request.  The units are in tenths of
 * nanoseconds.
 */

#define DEFAULT_MEMORY_ROUNDTRIP_TIME	25		/* 2.5ns (default) */
#define DEFAULT_MEMORY_ROUNDTRIP_TIME_FCRAM	20	/* 2.0ns for FCRAM */

#define BCM1480_ROUNDTRIP_DELAY_CHIP	26		/* 2.6 ns fixed for bcm1480 */

#define DLL_SCALE_NUMERATOR	30		/* 30/400 = 0.075 */
#define DLL_SCALE_DENOMINATOR	400
#define DLL_OFFSET		63		/* 63/400 = 0.1575 */


/*
 * The constants below were created by careful measurement of 
 * BCM1250 parts.  The units are in tenths of nanoseconds
 * to be compatible with the rest of the calculations in bcm1480_auto_timing.
 */

#define BCM1480_MIN_R2W_TIME		30	/* 3.0 ns */
#define BCM1480_MIN_DQS_MARGIN		25
#define BCM1480_WINDOW_OPEN_OFFSET	18
#define BCM1480_CLOSE_01_OFFSET		34
#define BCM1480_CLOSE_02_OFFSET		22
#define BCM1480_CLOSE_12_OFFSET		24


#define BURSTLEN			4		/* always 4 per burst */

/*  *********************************************************************
    *  Basic types
    ********************************************************************* */

#ifdef _CFE_
#include "lib_types.h"
#else
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
#endif

/*
 * For SOCVIEW and non-CFE, non-MIPS stuff, make sure the "port"
 * data type is 64 bits.  Otherwise we take our cue from 'long'
 * which will be pointer-sized.
 */

#if defined(_MCSTANDALONE_)
typedef long long sbport_t;
#else
typedef long sbport_t;
#endif

#ifdef _CFE_
#include "bsp_config.h"
#endif

#define TRUE 1
#define FALSE 0

/*  *********************************************************************
    *  Configuration
    ********************************************************************* */

/*
 * This module needs to be compiled with mips64 to ensure that 64-bit
 * values are in 64-bit registers and that reads/writes of 64-bit numbers
 * are done with the ld/sd instructions.  
 */
#if !defined(__mips64) && !defined(_MCSTANDALONE_)
#error "This module MUST be compiled with __mips64.  See the comments for details."
#endif

/*
 * Configure some stuff here if not running under the firmware.
 */

#ifndef _CFE_
#define CFG_DRAM_ECC		0
#define CFG_DRAM_SMBUS_CHANNEL	0
#define CFG_DRAM_SMBUS_BASE	0x54
#define CFG_DRAM_BLOCK_SIZE	32
#endif


/*
 * Clock, data, and address class for memory drive config register. 
 * Set all three to 1 as default.
 */

#define V_MC_DRVCONFIG_CLASS_DEFAULT         	M_BCM1480_MC_CLK_CLASS | \
                                                M_BCM1480_MC_DATA_CLASS | \
                                                M_BCM1480_MC_ADDR_CLASS

/*
 * Address, DQI, and DQO coarse/fine adjustments.
 */

#define V_MC_DLLCONFIG_ADJ_DEFAULT_PASS2	V_BCM1480_MC_ADDR_COARSE_ADJ(0x0) | \
                                                V_BCM1480_MC_ADDR_FREQ_RANGE(0) | \
                                                V_BCM1480_MC_DQI_COARSE_ADJ(0) | \
                                                V_BCM1480_MC_DQI_FREQ_RANGE(0) | \
                                                V_BCM1480_MC_DQO_COARSE_ADJ(0) | \
                                                V_BCM1480_MC_DQO_FREQ_RANGE(0) | \
                                                V_BCM1480_MC_DLL_DEFAULT_DEFAULT | \
                                                V_BCM1480_MC_DLL_FREQ_RANGE(0)

#define V_MC_DLLCONFIG_ADJ_DEFAULT		V_BCM1480_MC_ADDR_COARSE_ADJ(0) | \
                                                V_BCM1480_MC_ADDR_FINE_ADJ(0x8) | \
                                                V_BCM1480_MC_DQI_COARSE_ADJ(0) | \
                                                V_BCM1480_MC_DQI_FINE_ADJ(0x8) | \
                                                V_BCM1480_MC_DQO_COARSE_ADJ(0) | \
                                                V_BCM1480_MC_DQO_FINE_ADJ(0x8) | \
                                                V_BCM1480_MC_DLL_DEFAULT_DEFAULT | \
                                                V_BCM1480_MC_DLL_STEP_SIZE(0x8)

#define V_MC_ODTCONFIG_DEFAULT			M_BCM1480_MC_RD_ODT0_CS0 | \
		                                M_BCM1480_MC_WR_ODT0_CS0 | \
                                                M_BCM1480_MC_RD_ODT2_CS2 | \
                                                M_BCM1480_MC_WR_ODT2_CS2 | \
                                                M_BCM1480_MC_RD_ODT4_CS4 | \
                                                M_BCM1480_MC_WR_ODT4_CS4 | \
                                                M_BCM1480_MC_RD_ODT6_CS6 | \
                                                M_BCM1480_MC_WR_ODT6_CS6 | \
                                                M_BCM1480_MC_CS_ODD_ODT_EN
/*
 * These belong in some BCM1480-specific file I'm sure.
 */

#define MC_32BIT_CHANNELS	4     	/* There are 4 32-bit channels */
#define MC_32BIT_CHIPSELS	4      	/* Each channel has 4 chip selects */

#define MC_64BIT_CHANNELS	2	/* There are 2 64-bit channels */	
#define MC_64BIT_CHIPSELS	8	/* Each channel has 8 chip selects */ 

#define MC_MAX_CHANNELS		4	
#define MC_MAX_CHIPSELS		8

#define MC_FIRSTCHANNEL 0

/*  *********************************************************************
    *  Reference Clock
    ********************************************************************* */

#ifdef _MAGICWID_
  /* 
   * You really don't want to know about this.  During testing, we futz
   * with the 100mhz clock and store the actual speed of the clock
   * in the PromICE so we can make the calculations work out correctly
   * (and automatically)
   */
  #define BCM1480_REFCLK (*((uint64_t *) PHYS_TO_K1(0x1FC00018)))
  #undef K_SMB_FREQ_100KHZ
  #define K_SMB_FREQ_100KHZ ((BCM1480_REFCLK*10)/8)
#else
  /*
   * If non-CFE, non-MIPS, make the refclk an input parameter.
   */
  #if defined(_MCSTANDALONE_)
    int bcm1480_refclk = 100;
    int dram_cas_latency;
    int dram_tMemClk;
    #define BCM1480_REFCLK bcm1480_refclk
  #endif
#endif

/*
 * Define our reference clock.  The default is 100MHz unless
 * overridden.  You can override this in your bsp_config.h file.
 */

#ifdef BCM1480_REFCLK_HZ
   #define BCM1480_REFCLK ((BCM1480_REFCLK_HZ)/1000000)
#endif

#ifndef BCM1480_REFCLK
  #define BCM1480_REFCLK	100		/* speed of refclk, in Mhz */
#endif

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

/*
 * For the general case, reads/writes to MC CSRs are just pointer
 * references.  In SOCVIEW and other non-CFE, non-MIPS programs, we hook the
 * read/write calls to let us supply the data from somewhere else.
 */

#if defined(_MCSTANDALONE_)
  #define PHYS_TO_K1(x) (x)
  #define WRITECSR(csr,val) sbwritecsr(csr,val)
  #define READCSR(csr)      sbreadcsr(csr)
  extern void sbwritecsr(uint64_t,uint64_t);
  extern uint64_t sbreadcsr(uint64_t);
#else	/* normal case */
  #define WRITECSR(csr,val) *((volatile uint64_t *) (csr)) = (val)
  #define READCSR(csr) (*((volatile uint64_t *) (csr)))
#endif

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

/*  *********************************************************************
    *  JEDEC values 
    ********************************************************************* */

#define JEDEC_SDRAM_MRVAL_CAS15_BL8 	0x53	/* 8-byte bursts, sequential, CAS 1.5 */
#define JEDEC_SDRAM_MRVAL_CAS2_BL8	0x23	/* 8-byte bursts, sequential, CAS 2 */
#define JEDEC_SDRAM_MRVAL_CAS25_BL8	0x63	/* 8-byte bursts, sequential, CAS 2.5 */
#define JEDEC_SDRAM_MRVAL_CAS3_BL8	0x33	/* 8-byte bursts, sequential, CAS 3 */
#define JEDEC_SDRAM_MRVAL_CAS35_BL8	0x73    /* 8-byte bursts, sequential, CAS 3.5 */
#define JEDEC_SDRAM_MRVAL_CAS4_BL8	0x43	/* 8-byte bursts, sequential, CAS 4.0 */
#define JEDEC_SDRAM_MRVAL_DDR2_CAS5_BL8	0x53	/* 8-byte bursts, sequential, CAS 5.0 */
#define JEDEC_SDRAM_MRVAL_DDR2_CAS6_BL8	0x63	/* 8-byte bursts, sequential, CAS 6.0 */

#define JEDEC_SDRAM_MRVAL_CAS15 0x52	/* 4-byte bursts, sequential, CAS 1.5 */
#define JEDEC_SDRAM_MRVAL_CAS2	0x22	/* 4-byte bursts, sequential, CAS 2 */
#define JEDEC_SDRAM_MRVAL_CAS25	0x62	/* 4-byte bursts, sequential, CAS 2.5 */
#define JEDEC_SDRAM_MRVAL_CAS3	0x32	/* 4-byte bursts, sequential, CAS 3 */
#define JEDEC_SDRAM_MRVAL_CAS35 0x72    /* 4-byte bursts, sequential, CAS 3.5 */
#define JEDEC_SDRAM_MRVAL_CAS4	0x42	/* 4-byte bursts, sequential, CAS 4.0 */
#define JEDEC_SDRAM_MRVAL_DDR2_CAS5	0x52	/* 4-byte bursts, sequential, CAS 5.0 */
#define JEDEC_SDRAM_MRVAL_DDR2_CAS6	0x62	/* 4-byte bursts, sequential, CAS 6.0 */

#define JEDEC_SDRAM_MRVAL_RESETDLL 0x100

#define JEDEC_SDRAM_MRVAL_WR_2  0x200
#define JEDEC_SDRAM_MRVAL_WR_3  0x400
#define JEDEC_SDRAM_MRVAL_WR_4  0x600
#define JEDEC_SDRAM_MRVAL_WR_5  0x800
#define JEDEC_SDRAM_MRVAL_WR_6  0xA00

#define JEDEC_SDRAM_MRVAL_AL_0  0x000
#define JEDEC_SDRAM_MRVAL_AL_1  0x008
#define JEDEC_SDRAM_MRVAL_AL_2  0x010
#define JEDEC_SDRAM_MRVAL_AL_3  0x018
#define JEDEC_SDRAM_MRVAL_AL_4  0x020
#define JEDEC_SDRAM_MRVAL_AL_5  0x028
#define JEDEC_SDRAM_MRVAL_AL_6  0x030
#define JEDEC_SDRAM_MRVAL_AL_7  0x038

#define JEDEC_SDRAM_EMRVAL	0x000
#define JEDEC_SDRAM_EMRVAL_OCD_DEFAULT	0x380
#define JEDEC_SDRAM_EMRVAL_DQS_DISABLE	0x400
#define JEDEC_SDRAM_EMRVAL_RDQS_ENABLE 	0x800
#define JEDEC_SDRAM_EMRVAL_RTT_50	0x044
#define JEDEC_SDRAM_EMRVAL_RTT_75	0x004
#define JEDEC_SDRAM_EMRVAL_RTT_150	0x040
#define JEDEC_SDRAM_EMRVAL_DS_REDUCED	0x002

#define FCRAM_MRVAL		0x32
#define FCRAM_EMRVAL		0

#define SGRAM_MRVAL		0x32	/* 4-byte bursts, sequential, CAS 3 */
#define SGRAM_MRVAL_RESETDLL	0x400
#define SGRAM_EMRVAL		0x02

/* 
 * DECTO10THS(x) - this converts a BCD-style number found in 
 * JEDEC SPDs to a regular number.  So, 0x75 might mean "7.5ns"
 * and we convert this into tenths (75 decimal).  Many of the
 * calculations for the timing are done in terms of tenths of nanoseconds
 */

#define DECTO10THS(x) ((((x) >> 4)*10)+((x) & 0x0F))

/*  *********************************************************************
    *  Memory region sizes (BCM1480-specific)
    ********************************************************************* */

#define REGION0_LOC	0x0000
#define REGION0_SIZE	256

#define REGION1_LOC	0x0800
#define REGION1_SIZE	512

#define REGION2_LOC	0x0C00
#define REGION2_SIZE	256

#define REGION3_LOC	0x1400
#define REGION3_SIZE	(59*1024)

/*  *********************************************************************
    *  Global Data structure
    * 
    *  This is a hideous hack.  We're going to actually use "memory"
    *  before it is configured.  The L1 DCache will be clean before
    *  we get here, so we'll just locate this structure in memory 
    *  (at 0, for example) and "hope" we don't need to evict anything.
    *  If we keep the data below 256 cache lines, we'll only use one way
    *  of each cache line.  That's 8K, more than enough. 
    *
    *  This data structure needs to be used both for our data and the
    *  "C" stack, so be careful when you edit it!
    ********************************************************************* */

typedef struct csdata_s {	 /* Geometry info from table or SMbus */
    uint8_t rows;		 /*  0:  */
    uint8_t cols;		 /*  1:  */
    uint8_t banks;		 /*  2:  */
    uint8_t flags;		 /*  3:  */
   
    uint8_t spd_dramtype;	 /*  4: SPD[2] */
    uint8_t spd_tCK_25;		 /*  5: SPD[9]  tCK @ CAS 2.5 */
    uint8_t spd_tCK_20;		 /*  6: SPD[23] tCK @ CAS 2.0 */
    uint8_t spd_tCK_10;		 /*  7: SPD[25] tCK @ CAS 1.0 */
    uint8_t spd_rfsh;		 /*  8: SPD[12] Refresh Rate */
    uint8_t spd_caslatency;	 /*  9: SPD[18] CAS Latencies Supported */
    uint8_t spd_attributes;	 /* 10: SPD[21] Attributes */
    uint8_t spd_tRAS;		 /* 11: SPD[30] */
    uint8_t spd_tRP;		 /* 12: SPD[27] */
    uint8_t spd_tRRD;		 /* 13: SPD[28] */
    uint8_t spd_tRCD;		 /* 14: SPD[29] */
    uint8_t spd_tRFC;		 /* 15: SPD[42] */
    uint8_t spd_tRC;		 /* 16: SPD[41] */
    uint8_t spd_tRTP;		 /* 17: SPD[38] */
    uint8_t spd_dimmtype;	 /* 18: SPD[20] DDR2 only DIMM type */

} csdata_t;			 /* total size: 19 bytes */

#define CS_PRESENT 1			/* chipsel is present (in use) */
#define CS_AUTO_TIMING 2		/* chipsel has timing information */

#define CS_CASLAT_10	0x20		/* upper four bits are the CAS latency */
#define CS_CASLAT_15	0x30		/* we selected.  bits 7..5 are the */
#define CS_CASLAT_20	0x40		/* whole number and bit 4 is the */
#define CS_CASLAT_25	0x50		/* fraction. */
#define CS_CASLAT_30	0x60
#define CS_CASLAT_40	0x80
#define CS_CASLAT_50	0xA0
#define CS_CASLAT_60	0xC0
#define CS_CASLAT_MASK	0xF0
#define CS_CASLAT_SHIFT 4

typedef struct mcdata_s {	 /* Information per MC channel */
    uint32_t cfgcsint; 		 /* 0:  try to interleave this many CS bits */
    uint32_t csint;		 /* 4:  # of chip select interleave bits */
    uint32_t highest_cs_present; /* 8:  Highest chip select being used. */
    uint16_t mintmemclk;	 /* 12: minimum tMemClk */
    uint16_t roundtrip;		 /* 14: Round trip time from CLK to returned DQS at BCM1250 pin */    
    uint16_t dramtype;		 /* 16: DRAM Type */
    uint16_t clk_ratio;		 /* 18: clock ratio for delay loops */
    uint32_t pagepolicy;	 /* 20: Page policy */
    uint32_t chantype;		 /* 24: Channel type 32-bit or 64-bit */
    uint32_t chanintlvint;	 /* 28: # of channel interleave bits */
    uint32_t flags;		 /* 32: ECC enabled */
    uint16_t tCK;		 /* 36: tCK for manual timing */
    uint16_t rfsh;		 /* 38: refresh rate for manual timing */
    uint16_t tWR;		 /* 40: Write recovery. DDR2 Mode Register field */
    uint64_t dllcfg;		 /* 42: default dll config */
    uint64_t drvcfg;		 /* 50: default drive config */
    uint64_t odtcfg;		 /* 58: */
    uint64_t mantiming;		 /* 66: manual timing */
    uint64_t modebits;		 /* 74: mode bits (for ganged channels) */
    uint16_t odt_mc;		 /* 76: ODT setting on MC side */
    uint16_t odt_dram;		 /* 78: ODt setting on DRAM side */
    uint16_t tAL;		 /* 80: Additive Latency. DDR2 EMR field */
    uint16_t tRAP;		 /* 82: */
    uint64_t odtcfg2;		 /* 90: */	
    uint16_t odt2_mc;		 /* 92: */
    uint16_t odt2_dram;		 /* 94: */
    uint16_t numdimms;		 /* 96: Number of DIMMs (if any) for this MC */
    uint8_t addrc_reg;		 /* 97: */
    uint8_t addrc_unbuf;         /* 98: */
    uint16_t dllcfg_override;	 /* 100: */
    csdata_t csdata[MC_MAX_CHIPSELS]; /* 100: per-chipsel data (8 * 19) */
} mcdata_t;			 /* total size: 236 bytes */

typedef struct initdata_s {
    uint64_t dscr[4];		 /* 0:  DMover descriptor (one cache line) */
    uint32_t flags;		 /* 32: various flags */
    uint32_t inuse;		 /* 36: indicates MC is in use */
    uint16_t cfg_chanintlv_type; /* 40: Try to interleave channels */
    uint64_t ttlbytes;		 /* 48: total bytes */
    mcdata_t mc[MC_MAX_CHANNELS];/* 56: per-channel data (4 * 218) */
} initdata_t;			 /* total size: 900 bytes */

#define M_MCINIT_TRYPINTLV 1		/* Try to do port interleaving */
#define M_MCINIT_PINTLV	   2		/* Actually do port interleaving */

/* Work area: initdata structure plus enough working stack to run the
   DRAM init routine.  We round the initdata structure up to a 1K boundary,
   and throw in an extra 1K for stack space.

   This **MUST** evaluate to a compile-time constant.  */
#define WORK_AREA_SIZE (((sizeof(initdata_t) + 1023) / 1024) + 1) * 1024

/*  *********************************************************************
    *  Configuration data structure
    ********************************************************************* */

#include "bcm1480_draminit.h"
#include "jedec.h"

/*  *********************************************************************
    *  Initialized data
    *  
    *  WARNING WARNING WARNING!
    *  
    *  This module is *very magical*!   We are using the cache as
    *  SRAM, and we're running as relocatable code *before* the code
    *  is relocated and *before* the GP register is set up.
    *  
    *  Therefore, there should be NO data declared in the data
    *  segment - all data must be allocated in the .text segment
    *  and references to this data must be calculated by an inline
    *  assembly stub.  
    *  
    *  If you grep the disassembly of this file, you should not see
    *  ANY references to the GP register.
    ********************************************************************* */


#ifdef _MCSTANDALONE_NOISY_
static char *bcm1480_rectypes[] = {"MCR_GLOBALS","MCR_CHCFG","MCR_TIMING",
				   "MCR_DLLCFG","MCR_GEOM","MCR_SPD","MCR_MANTIMING","MCR_TIMING2","MCR_ODTCFG","MCR_ODTCFG2",
				   "MCR_ADDRCOARSE"};
#endif

/*  *********************************************************************
    *  Module Description
    *  
    *  This module attempts to initialize the DRAM controller on the
    *  BCM1480. There are four channels (0-3), each providing a 32-bit
    *  data path. Pairs of channels can be ganged together to form 
    *  up to two 64-bit channels (0-1). When ganged together, channels
    *  0 & 2 become channel 0 and channels 1 & 3 become channel 1. 
    *
    *  Each 32-bit channel can support four chip selects, or two double-
    *  sided DDR SDRAM DIMMs. 
    *
    *  Each 64-bit channel can support eight chip selects, or four 
    *  double-sided DDR SDRAM DIMMs.
    *   
    *  The controller can support up eight DIMMs.
    *
    *  The steps to initialize the DRAM controller are:
    *  
    *      * Read the SPD, verify DDR SDRAMs or FCRAMs
    *      * Obtain #rows, #cols, #banks, and module size
    *      * Calculate row, column, and bank masks
    *      * Calculate chip selects
    *      * Calculate timing register.  Note that we assume that
    *        all banks will use the same timing.
    *      * Repeat for each DRAM.
    * 
    *  DRAM Controller registers are programmed in the following order:
    *  
    *  	   MC_TEST_DATA, MC_TEST_ECC
    *
    *  	   MC_CSXX_BA, MC_CSXX_COL, MC_CSXX_ROW
    *      (repeated for each chip select)
    *
    *  	   MC_CS_START, MC_CS_END
    *
    *	   MC_CONFIG (for CS interleaving)
    *	   MC_GLB_INTLV (for channel interleaving)
    *
    *  	   MC_CLOCK_CFG
    *  	   MC_TIMING
    *      (delay)
    *  	   MC_DRAMMODE
    *      (delay after each mode setting ??)
    *  
    *  Once the registers are initialized, the DRAM is activated by
    *  sending it the following sequence of commands:
    *  
    *       PRE (precharge)
    *       EMRS (extended mode register set)
    *       MRS (mode register set)
    *       PRE (precharge) 
    *       AR (auto-refresh)
    *       AR (auto-refresh again)
    *       MRS (mode register set)
    *  
    *  then wait 200 memory clock cycles without accessing DRAM.
    *  
    *  Following initialization, the ECC bits must be cleared.  This
    *  can be accomplished by disabling ECC checking on all memory
    *  controllers, and then zeroing all memory via the mapping
    *  in xkseg.
    ********************************************************************* */

/*  *********************************************************************
    *
    * Address Bit Assignment Algorithm:
    * 
    * Good performance can be achieved by taking the following steps
    * when assigning address bits to the row, column, and interleave
    * masks.  You will need to know the following:
    * 
    *    - The number of rows, columns, and banks on the memory devices
    *    - The block size (larger tends to be better for sequential
    *      access)
    *    - Whether you will interleave chip-selects
    *    - Whether you will be using both memory controllers and want
    *      to interleave between them
    * 
    * By choosing the masks carefully you can maximize the number of
    * open SDRAM banks and reduce access times for nearby and sequential
    * accesses.
    * 
    * The diagram below depicts a physical address and the order
    * that the bits should be placed into the masks.  Start with the 
    * least significant bit and assign bits to the row, column, bank,
    * and interleave registers in the following order:
    * 
    *         <------------Physical Address--------------->
    * Bits:	RRRRRRR..R  CCCCCCCC..C  SS  BB  PP  CCc00
    * Step:	    6           5         4   3   2    1
    * 
    * Where:
    *     R = Row Address Bit     (MC_CSXX_ROW register)
    *     C = Column Address Bit  (MC_CSXX_COL register)
    *     S = Chip Select         (MC_CONFIG register)    
    *                             (when interleaving via chip selects)
    *     B = Bank Bit            (MC_CSXX_BA register)
    *     P = Channel Select Bit  (MC_GLB_INTLV register)  
    *                             (when interleaving memory channels)
    *	  c = Column Address Bit  (MC_CSXX_COL register for 32-bit channels)
    *		       		  (0 for 64-bit channels)
    *     0 = must be zero
    * 
    * When an address bit is "assigned" it is set in one of the masks
    * in the MC_CSXX_ROW, MC_CSXX_COL, MC_CSXX_BA, MC_CONFIG, or MC_GLB_INTLV  
    * registers.
    * 
    * 
    * 1. The least significant 5 bits specify the byte within a cache line,
    *	 and always zero in physical addresses presented to the memory
    *	 controller. In 32-bit channels, the controller uses the top 3 bits
    *	 as column bits to sequence the 8 chunks of the cache line, and 
    *	 the bottom 2 bits are ignored. For ganged 64-bit channels the top
    *	 2 bits are used to sequence the four chunks, and the bottom 3 bits
    *	 are ignored. The appropriate number of 2 or 3 column bits must be
    *	 subtracted from the total number in the device to give the number
    *	 of column bits.
    *
    * 2. These one or two bits are used for interleaving the channels and
    *	 are specified in the MC_GLB_INTLV register.
    * 
    * 3. These bits select the internal bank within a memory device. Most
    * 	 devices have four banks, so 2 bits set in the MC_CSXX_BA register.
    *
    * 4. These bits select the interleaving among physical devices on a
    *	 channel via chip selects in the MC_CONFIG register. If chip select
    *	 interleaving is not used on the channel, no bits are specified here.
    *
    * 5. The remaining column bits not used in field 1 are set in the
    *	 MC_CSXX_COL registers.
    *
    * 6. The row bits of the device are specified in the MC_CSXX_ROW
    *	 registers.
    ********************************************************************* */



 /**********************************************************************
  * bcm1480_decto100ths(unsigned int value) 
  *
  * Converts a BCD-style number to a regular number.  
  * So, 0x75 might mean "7.50ns" and we convert this into 100ths 
  * (750 decimal).
  *
  ********************************************************************** */

static unsigned int bcm1480_decto100ths(uint8_t value)
{
    unsigned int low_nib, high_nib;

    high_nib = (value >> 4)*100;
    switch (value & 0x0F) {
	case 0xA: low_nib = 25; break;
	case 0xB: low_nib = 33; break;
	case 0xC: low_nib = 66; break;
	case 0xD: low_nib = 75; break;
	default: low_nib = (value & 0x0F)*10; break;
	}

    return high_nib + low_nib;
}

/*  *********************************************************************
    *  bcm1480_find_timingcs(mc)
    *  
    *  For a given memory controller, choose the chip select whose
    *  timing values will be used to base the TIMING and MCLOCK_CFG
    *  registers on.  
    *  
    *  Input parameters: 
    *  	   mc - memory controller
    *  	   
    *  Return value:
    *  	   chip select index, or -1 if no active chip selects.
    ********************************************************************* */


static int bcm1480_find_timingcs(mcdata_t *mc)
{
    int idx,slowest_idx;
    unsigned int slowest,memspeed;

    slowest_idx=-1; slowest=0;
    /* Get first present CS */
    for (idx = 0; idx < MC_MAX_CHIPSELS; idx++) {
	if (mc->csdata[idx].flags & CS_PRESENT) {
	    slowest = bcm1480_decto100ths(mc->csdata[idx].spd_tCK_25);
	    slowest_idx = idx;
	    break;
	}
    }

    /* Now find slowest CS */
    for (idx = 0; idx < MC_MAX_CHIPSELS; idx++) {
	if (mc->csdata[idx].flags & CS_PRESENT) {
	    memspeed = bcm1480_decto100ths(mc->csdata[idx].spd_tCK_25);
	    if (memspeed > slowest) {
		slowest = bcm1480_decto100ths(mc->csdata[idx].spd_tCK_25);
		slowest_idx = idx;
		}
	    }
	}

    return slowest_idx;
}

/*  *********************************************************************
    *  bcm1480_auto_timing(mcidx,tdata)
    *  
    *  Program the memory controller's timing registers based on the
    *  timing information stored with the chip select data.  For DIMMs
    *  this information comes from the SPDs, otherwise it was entered
    *  from the datasheets into the tables in the init modules.
    *  
    *  Input parameters: 
    *  	   mcidx - memory controller index (0 or 1)
    *  	   tdata - a chip select data (csdata_t)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void bcm1480_auto_timing(int mcidx,mcdata_t *mc,csdata_t *tdata)
{
    uint64_t sysrev;
    unsigned int plldiv;

    unsigned int res;

    unsigned int clk_ratio;
    unsigned int refrate;
    unsigned int ref_freq;
    unsigned int caslatency;

    unsigned int spd_tCK_25;
    unsigned int spd_tCK_20;
    unsigned int spd_tCK_10;
    unsigned int tCpuClk;
    unsigned int tMemClk;
    unsigned int fMemClk;

    unsigned int w2rIdle,r2wIdle,r2rIdle,w2wIdle;
    unsigned int tCL,tCrDh,tFIFO;
    unsigned int tCwD;
    unsigned int tRAS;
    unsigned int tWR,tWTR;
    unsigned int tRP,tRRD,tRCD,tRC,tRCw,tRCr,tRFC,tRTP,tAL,tRAP;

    uint64_t timing1,timing2,mclkcfg,drvcfg,dllcfg;
    sbport_t base;

    unsigned int dqsArrival,AddrDllShft,dll_step,dll_skew,window_transition_adj,freq_range,do_dqoshift;

    sysrev = READCSR(PHYS_TO_K1(A_SCD_SYSTEM_REVISION));

    /*
     * We need our cpu clock for all sorts of things.
     */
#if defined(_FUNCSIM_)
    plldiv = 16;		/* 800MHz CPU for functional simulation */
#else
    plldiv = G_BCM1480_SYS_PLL_DIV(READCSR(PHYS_TO_K1(A_SCD_SYSTEM_CFG)));
#endif
    if (plldiv == 0) {
	/* XXX: should be common macro, also defaulted by boards' *_devs.c.  */
	plldiv = 6;
	}

    /*
     * Compute tCpuClk, in picoseconds to avoid rounding errors.
     *
     * Calculation:
     *     tCpuClk = 1/fCpuClk
     *             = 1/(100MHz * plldiv/2) 
     *             = 2/(100MHz*plldiv)
     *             = 2/(100*plldiv) us 
     *		   = 20/plldiv ns 
     *             = 2000000/plldiv 10ths of ns
     *
     * If BCM1480_REFCLK is in MHz, then:
     *           2/(BCM1480_REFCLK*plldiv) us 
     *         = 2000/(BCM1480_REFCLK*plldiv) ns 
     *         = 2000000/(BCM1480_REFCLK*plldiv) ps
     *
     * However, we want to round the result to the nearest integer,
     * so we double the numerator (to 4000000) to get one more bit
     * of precision in the quotient, then add one and scale it back down
     */

    /* tCpuClk is in picoseconds */
    tCpuClk = ((4000000/(BCM1480_REFCLK*plldiv))+1)/2;
    
    spd_tCK_25 = bcm1480_decto100ths(tdata->spd_tCK_25);
    spd_tCK_20 = bcm1480_decto100ths(tdata->spd_tCK_20);
    spd_tCK_10 = bcm1480_decto100ths(tdata->spd_tCK_10);

    /* 
     * Compute the target tMemClk, in units of 100ths of nanoseconds
     * to be similar to the JEDEC SPD values.  This will be
     *
     *     MAX(MIN_tMEMCLK,spd_tCK_25) 
     */

    tMemClk = spd_tCK_25;
    if (mc->mintmemclk > tMemClk) tMemClk = mc->mintmemclk;

    /* 
     * Now compute our clock ratio (the amount we'll divide tCpuClk by
     * to get as close as possible to tMemClk without exceeding it.
     *
     * It's (tMemClk*10) here because tCpuClk is in picoseconds 
     *
     * The ratio needs to be relative to the ZBBus, which runs at
     * 1/2 the core clock.
     *
     * The clock ratios are expressed in quarters (denominator=4)
     * so multiply the numerator by 4 before doing the divide.
     * Therefore, the low 2 bits fo the clk_ratio result will be
     * the fractional part.
     */

    clk_ratio = ((4*((tMemClk*10) + tCpuClk - 2)) / (2*tCpuClk)) - 1;
#ifdef _MCSTANDALONE_NOISY_
    printf("DRAM: Would like to use clk_ratio %d\n",clk_ratio);
    printf("DRAM: memory's tMemClk is %d\n",tMemClk);
#endif
    if (clk_ratio < 4) clk_ratio = 4;
    if (clk_ratio > 24) clk_ratio = 24;


#if _BCM1480_PASS1_WORKAROUNDS_
    /* 
     * 1480 S0 erratum SOC-81:  Keep clk_ratio an integer multiple.
     * Fixed in all later revisions.
     */

#if _BCM1480_S0_WORKAROUNDS_
    /* Do not do rev check.  If defined, always do S0 workarounds. */
    if (1)  { /* G_SYS_REVISION(sysrev) <= K_SYS_REVISION_BCM1480_S0) */
	clk_ratio = ((clk_ratio+3) & ~3);	/* make integer multiple */
	if (clk_ratio > 8) clk_ratio = 8;	/* but don't exceed 2 */

	/*
	 * XXX Actually, for now, force memory speed to be CPU / 4
	 * for CPU clocks > 400MHz, since there may be speed/temp
	 * issues on the MC pins in S0.  Note that 450, 500MHz may
	 * run DDR-2 parts out of spec (too slow) because of this!
	 * (This too is fixed in all later revs.)
	 */
	if (plldiv > 8) clk_ratio = 8;
	}
#endif
#endif

    /* 
     * Now, recompute tMemClk using the new clk_ratio.  This gives us
     * the actual tMemClk that the memory controller will generate
     *
     * Calculation:
     *      fMemClk = BCM1480_REFCLK * plldiv /  clk_ratio Mhz
     *
     *      tMemClk = 1/fMemClk us
     *              = clk_ratio / (BCM1480_REFCLK * plldiv) us
     *              = 10000 * clk_ratio / (BCM1480_REFCLK * plldiv) 0.1ns
     *
     * The resulting tMemClk is in 100ths of nanoseconds so we
     * can compare it with the SPD values.  The x100000 converts
     * us to 0.01ns
     */

new_ratio:
    tMemClk = (100000 * clk_ratio)/(BCM1480_REFCLK * plldiv);

    /* Calculate the refresh rate */

    switch (tdata->spd_rfsh & JEDEC_RFSH_MASK) {
	case JEDEC_RFSH_64khz:	ref_freq = 64;  break;
	case JEDEC_RFSH_256khz:	ref_freq = 256; break;
	case JEDEC_RFSH_128khz:	ref_freq = 128; break;
	case JEDEC_RFSH_32khz:	ref_freq = 32;  break;
	case JEDEC_RFSH_8khz:	ref_freq = 8;   break;
	default: 		ref_freq = 8;   break;
	}


    /*
     * Compute the target refresh value, in Khz/32.  We know
     * the rate that the DIMMs expect (in Khz, above).  So we need
     * to calculate what the MemClk is divided by to get that value.
     * There is an internal divide-by-32 in the 1400 in the refresh
     * generation.
     * 
     * fMemClk (in KHz) calculated as follows:
     *
     *    core_clock  = plldiv * reference * 1000 / 2;
     *    zbbus_clock = core_clock / 2;
     *    mem_clock   = (zbbus_clock * 4) / clk_ratio
     *
     * The refresh counter ticks once every fMemClk cycles, and
     * we want 'ref_freq' number of cycles to happen per second.
     * The refresh counter ticks once every 32 MemClks.
     *
     * Therefore, we can issue refresh pulses at a rate of
     * fMemClk/32 if we wanted to, but to get the correct
     * counter value all we need to do is:
     *
     *    refrate = (fMemClk/32)  /  ref_freq  (in Khz)
     */

    fMemClk = ((plldiv * BCM1480_REFCLK * 1000 / 2 / 2) * 4 / clk_ratio);
    refrate = fMemClk / 32 / ref_freq;

    /* Don't let the refresh rate go beyond the field width */
    if (refrate > 255) refrate = 255;

    /*
     * Calculate CAS Latency in half cycles.  The low bit indicates
     * half a cycle, so 2 (0010) = 1 cycle and 3 (0011) = 1.5 cycles
     */

    res = tdata->spd_caslatency;
    if (mc->dramtype == JEDEC_DDR2) {
	if (res & JEDEC_CASLAT_DDR2_6) caslatency = (6 << 1);		/* 6.0 */
       	else if (res & JEDEC_CASLAT_DDR2_5) caslatency = (5 << 1);	/* 5.0 */
	else if (res & JEDEC_CASLAT_DDR2_4) caslatency = (4 << 1);	/* 4.0 */
	else if (res & JEDEC_CASLAT_DDR2_3) caslatency = (3 << 1);	/* 3.0 */
	else caslatency = (2 << 1);					/* 2.0 */
	
	if ((spd_tCK_10 != 0) && (spd_tCK_10 <= tMemClk)) {
	    caslatency -= (2 << 1);				/* subtract 2 for ddr2 */
	    }
	else if ((spd_tCK_20 != 0) && (spd_tCK_20 <= tMemClk)) {
	    caslatency -= (1 << 1);				/* subtract 1 for ddr2 */
	    }
	}
    else {
	if (res & JEDEC_CASLAT_40) caslatency = (4 << 1);		/* 4.0 */
	else if (res & JEDEC_CASLAT_35) caslatency = (3 << 1) + 1;	/* 3.5 */
	else if (res & JEDEC_CASLAT_30) caslatency = (3 << 1);		/* 3.0 */
	else if (res & JEDEC_CASLAT_25) caslatency = (2 << 1) + 1;	/* 2.5 */
	else if (res & JEDEC_CASLAT_20) caslatency = (2 << 1);		/* 2.0 */
	else if (res & JEDEC_CASLAT_15) caslatency = (1 << 1) + 1;	/* 1.5 */
	else caslatency = (1 << 1);					/* 1.0 */

	if ((spd_tCK_10 != 0) && (spd_tCK_10 <= tMemClk)) {
	    caslatency -= (1 << 1);				/* subtract 1.0 */
	    }
	else if ((spd_tCK_20 != 0) && (spd_tCK_20 <= tMemClk)) {
	    caslatency -= 1;				/* subtract 0.5 */
	    }
	}

    /*
     * Store the CAS latency in the chip select info
     */
    
    tdata->flags &= ~CS_CASLAT_MASK;
    tdata->flags |= (((caslatency << CS_CASLAT_SHIFT)) & CS_CASLAT_MASK);
#ifdef _MCSTANDALONE_
    dram_cas_latency = caslatency;	
    dram_tMemClk = tMemClk;
#endif

    /*
     * Now, on to the timing parameters.
     */

    w2rIdle = 1;	/* Needs to be set on all parts.  XXX this is a 1250 bug, what to do on 1480? */
    r2rIdle = 0;

    /*
     * Get address coarse before we need to use it in data capture window calc.
     */
    mc->dllcfg &= ~M_BCM1480_MC_ADDR_COARSE_ADJ;
    if ((tdata->spd_dimmtype & JEDEC_DIMMTYPE_RDIMM) ||
	(tdata->spd_dimmtype & JEDEC_DIMMTYPE_MRDIMM) ||
	(mc->flags & MCFLG_FORCEREG)) { 
	mc->dllcfg |= V_BCM1480_MC_ADDR_COARSE_ADJ(mc->addrc_reg);
	}
    else {
	mc->dllcfg |= V_BCM1480_MC_ADDR_COARSE_ADJ(mc->addrc_unbuf);
	}

    /*
     * dll step according to B0 UM table 84, Pg 141
     */

    /* 
     * defines dll step in pico seconds
     * adjusting DLL is subject to process/Voltage/Temperature variations
     * dll also have saturation point before hitting the far end of the range
     */
#define DLL_STEP_250_400 51
#define DLL_STEP_200_249 58
#define DLL_STEP_160_199 67
#define DLL_STEP_100_159 75

    /*
     * Newer "Window" calculations done in ns rather than tMemClks
     */

    /*
      calculate time due to mclk shift by addr-coarse dll setting 
     */

    dllcfg  = mc->dllcfg;
    AddrDllShft = dllcfg & 0x1f ;
    if (dllcfg & 0x20 ) { /* is negative shift */
      AddrDllShft = - AddrDllShft ; }
    /*
     * compute range of dll operation
     * not sure if dll range is already set at this point 
     * we should never need to adjust DQSi DLL 
     */

    if ( tMemClk < 401 ) { 
	dll_step = DLL_STEP_250_400;
	freq_range = DLL_FREQ_RANGE_250_400;
	}
    else if ( tMemClk < 501 ) { 
	dll_step = DLL_STEP_200_249;
	freq_range = DLL_FREQ_RANGE_200_249;
	}
    else if ( tMemClk < 626 ) { 
	dll_step = DLL_STEP_160_199;
	freq_range = DLL_FREQ_RANGE_160_199;
	}
    else { 
	dll_step = DLL_STEP_100_159;
	freq_range = DLL_FREQ_RANGE_100_159;
	}

    /* dll_step is in ps and dqsArrival is in 100th of ns */

    dll_skew = (dll_step * AddrDllShft) / 10 ;    
    do_dqoshift = (dll_skew > (tMemClk/4))? 1 : 0 ;

    /*
     *  round trip delays are in tenth of ns and tMemClk is in hundredth of ns
     *  we want dsq arrival value in 10s
     */
    dqsArrival = 10* (BCM1480_ROUNDTRIP_DELAY_CHIP + mc->roundtrip) + dll_skew ;    

    /*
     * redefine boundaries that tFIFO and tCRDh must change
     * according to lab measurements
     * these numbers keep the transition point right in the middle of 
     * overlap between two adjacent windows
     *
     */

    //    window_transition_adj = 100 * 4 / tMemClk ;  
    window_transition_adj = 35  ;  // from rsm's measurements

    /* 
     *if (dqsArrival < (5-window_transition_adj)) { tFIFO=0; tCrDh=1; r2wIdle=0;}
     *else if (dqsArrival < 10) { tFIFO=1; tCrDh=0; r2wIdle=1;}
     *else if (dqsArrival < (15-window_transition_adj)) { tFIFO=1; tCrDh=1; r2wIdle=1;}
     *else if (dqsArrival < 20) { tFIFO=2; tCrDh=0; r2wIdle=2;}
     *else if (dqsArrival < (25-window_transition_adj)) { tFIFO=2; tCrDh=1; r2wIdle=2;}
     *else  { tFIFO=3; tCrDh=0; r2wIdle=3;}
    */
    
    if (dqsArrival < ((tMemClk/2)-window_transition_adj)) { tFIFO=0; tCrDh=1; r2wIdle=0;}
    else if (dqsArrival < tMemClk) { tFIFO=1; tCrDh=0; r2wIdle=1;}
    else if (dqsArrival < ((tMemClk+(tMemClk/2))-window_transition_adj)) { tFIFO=1; tCrDh=1; r2wIdle=1;}
    else if (dqsArrival < (2 * tMemClk)) { tFIFO=2; tCrDh=0; r2wIdle=2;}
    else if (dqsArrival < ((2 * tMemClk + (tMemClk/2))-window_transition_adj)) { tFIFO=2; tCrDh=1; r2wIdle=2;}
    else  { tFIFO=3; tCrDh=0; r2wIdle=3;}

    if ((mc->odt_mc != ODT_OFF) && (r2wIdle < 3)) r2wIdle++;

    /* ======================================================================== */

    /* Recompute tMemClk as a fixed-point 6.2 value */

    tMemClk = (4000 *  clk_ratio) / (BCM1480_REFCLK * plldiv);

    /* ======================================================================== */

#ifdef _VERILOG_
    tFIFO = 0;
#endif

    /*
     * With the actual tMemClk in hand, calculate tRAS, tRC, tRP, tRRD, and tRCD 
     */

    tRAS = ( ((unsigned int)(tdata->spd_tRAS))*4 + tMemClk-1) / tMemClk;
    tRC =  ( ((unsigned int)(tdata->spd_tRC))*4  + tMemClk-1) / tMemClk;
    tRP =  ( ((unsigned int)(tdata->spd_tRP))    + tMemClk-1) / tMemClk;
    tRRD = ( ((unsigned int)(tdata->spd_tRRD))   + tMemClk-1) / tMemClk;
    tRCD = ( ((unsigned int)(tdata->spd_tRCD))   + tMemClk-1) / tMemClk;
   
    /* tWR is the write recovery time, a constant of 15ns for DDR DIMMs. */

    tWR  = ( ((unsigned int) 15)*4               + tMemClk-1) / tMemClk;

    /* Get initial value for tCL */
    tCL = (caslatency >> 1);

    /*
     * Check for registered DIMMs, or if we are "forcing" registered
     * DIMMs, as might be the case of regular unregistered DIMMs
     * behind an external register.
     */
    switch (mc->dramtype) {
	case FCRAM:
	    /* For FCRAMs, tCwD is always caslatency - 1  */
	    tCwD = (caslatency >> 1) - 1; 
	    tRCD = 1;		/* always 1 for FCRAM */
	    tRP = 1;		/* always 1 for FCRAM */
	    break;
	case JEDEC_DDR2:
	    tCwD = (caslatency >> 1) - 1;
	    if ((tdata->spd_dimmtype & JEDEC_DIMMTYPE_RDIMM) ||
	        (tdata->spd_dimmtype & JEDEC_DIMMTYPE_MRDIMM) ||
		(mc->flags & MCFLG_FORCEREG)) { 
		tCwD++;
		tCL++;
        	}
	    if (tRRD < 2) tRRD = 2;	/* 2 clock minimum at any tCK */
	    if (tdata->banks >= 3) tRRD++; /* Add an extra cycle if we have 8 or more banks */
	    break;
	default:
	    /* Otherwise calculate based on registered attribute */
	    if ((tdata->spd_attributes & JEDEC_ATTRIB_REG) ||
		(mc->flags & MCFLG_FORCEREG)) {  	/* registered DIMM */
		tCwD = 2; 
		tCL++;
		}
	    else {			/* standard unbuffered DIMM */
		tCwD = 1;
		}
	    break;
	}

    /*
     * tWTR should be 1 tick unless we're actually using
     * CAS Latency 1.5 (unlikely) or memory runs faster than
     * 166MHz (tCK = 6.0ns or less)
     * 
     * CAS Latency is stored in "halves", so 3 means "1.5" 
     */

    tWTR = 1;
    if ((caslatency == 3) || (spd_tCK_25 <= 60)) tWTR = 2;

    /*
     * Okay, using this info, figure out tRCw,tRCr.
     */

    tRCw = max(tRC, tRP + max(tRAS, tRCD + tCwD + BURSTLEN/2 + tWR));
    tRCr = max(tRC, tRP + max(tRAS, tRCD + BURSTLEN/2));

    /*
     * Calculate tRFC if the SPD did not specify it.  Use the DIMM's
     * actual rated speed, spd_tCK_25.  Remember that spd_tCK_25 is in
     * tenths of nanoseconds, and tMemClk is in fixed-6.2 format,
     * but the SPD value itself is in nanoseconds (no tenths).
     *
     * Use the value from the first expression below that matches:
     *
     *    100Mhz or less  [10.0ns or more]  -- tRFC = 80ns
     *    133Mhz or less  [7.5ns or more]   -- tRFC = 75ns
     *    166Mhz or less  [6.0ns or more]   -- tRFC = 72ns
     *    All others:                       -- tRFC = 70ns
     *
     * Special case for gigabit parts: always use 120ns [see JEDEC spec]
     * 
     * Note: the calculation may cause tRFC to overflow the 4-bit field
      * that we have allocated for it.  If that happens, reduce memory
     * speed and try again.  Hopefully we won't go into a loop.
     */

    if (tdata->spd_tRFC == 0) {
	unsigned int calcRFC;	/* in nanoseconds */

	if (tdata->rows >= 14) {	/* Gigabit parts have >= 14 rows */
	    calcRFC = 120;
	    } 
	else {
	    if (spd_tCK_25 >= 1000)     calcRFC = 80;	/* 100MHz */
	    else if (spd_tCK_25 >= 750) calcRFC = 75;	/* 133MHz */
	    else if (spd_tCK_25 >= 600) calcRFC = 72;	/* 166MHz */
	    else calcRFC = 70;				/* Others */
	    }

	tRFC = (calcRFC*4 + tMemClk-1) / tMemClk;

	/* 
	 * if tRFC is greater than 48,
	 * then we need to slow the memory down.
	 */

	if (tRFC > 48) {
#ifdef _MCSTANDALONE_NOISY_
	    printf("DRAM: tRFC too big, reducing memory speed\n");
#endif
	    clk_ratio++;
	    goto new_ratio;			/* yikes! */
	    }
	}
    else {
	tRFC = ( ((unsigned int) tdata->spd_tRFC)*4 + tMemClk-1) / tMemClk;
	}

    if(mc->dramtype == JEDEC_DDR2) {

//	tFIFO=2; tCrDh=1; r2wIdle=2;                                                                                        

	r2rIdle=1; /* Must be set for DDR2. */

	mc->tWR = tWR; /* DDR2 part requires tWR. See bcm1480_jedec_ddr2_initcmds */

	mc->drvcfg |= M_BCM1480_MC_DQS_DIFF;

	/* timing2 register stuff */

	tAL = 0;
	if ( (mc->tAL >= (11 - tCL)) || (mc->tAL >= (tRCD - 1)) ) 
	    mc->tAL =  min( ((11 - tCL) - 1), ((tRCD - 1) - 1) );
	if (mc->tAL > 5) mc->tAL = 5;
	tAL = mc->tAL;

	/* min 2 cycles */
	tRTP = ( ((unsigned int)(tdata->spd_tRTP))   + tMemClk-1) / tMemClk;
	if (tRTP < 2) tRTP = 2;

	w2wIdle = 1;  /* 1 for ODT 0 for non-ODT */

	tRAP = mc->tRAP; /* DDR2 set to 0, DDR1 set to 1 */

	}
    else {
	tAL = 0; /*Default*/
	tRTP = 2; /*Default*/
	w2wIdle = 0; /*Default*/
	tRAP = 1;
	}

    /*
     * Finally, put it all together in the timing register.
     */
    timing1 = V_BCM1480_MC_tRCD(tRCD) |
	V_BCM1480_MC_tCL(tCL) |
	(tCrDh ? M_BCM1480_MC_tCrDh : 0) |
	V_BCM1480_MC_tWR(tWR) |
	V_BCM1480_MC_tCwD(tCwD) |
	V_BCM1480_MC_tRP(tRP) |
	V_BCM1480_MC_tRRD(tRRD) |
	V_BCM1480_MC_tRCw(tRCw) |
	V_BCM1480_MC_tRCr(tRCr) |
	V_BCM1480_MC_tRFC(tRFC) |
	V_BCM1480_MC_tFIFO(tFIFO) |
	V_BCM1480_MC_tR2W(r2wIdle) |
	V_BCM1480_MC_tW2R(w2rIdle) |
	(r2rIdle ? M_BCM1480_MC_tR2R : 0);

    base = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx));

    WRITECSR(base+R_BCM1480_MC_TIMING1,timing1);

    timing2 = V_BCM1480_MC_tAL(tAL) | 
	V_BCM1480_MC_tRTP(tRTP) |	
	V_BCM1480_MC_tW2W(w2wIdle) |
	V_BCM1480_MC_tRAP(tRAP);

    if (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0)
	WRITECSR(base+R_BCM1480_MC_TIMING2,timing2);    

    /*
     * Memory DLL config and memory drive config registers.
     */
    if (mc->numdimms <= 1) {
	if (mc->odt_mc == ODT_150) {
	    mc->drvcfg |= M_BCM1480_MC_DQ_ODT_150;
	    mc->drvcfg |= M_BCM1480_MC_DQS_ODT_150;
	    }
	else if (mc->odt_mc == ODT_75) {
	    mc->drvcfg |= M_BCM1480_MC_DQ_ODT_75;
	    mc->drvcfg |= M_BCM1480_MC_DQS_ODT_75;
	    }
	}
    else { /* 2 or more DIMMs */
	if (mc->odt2_mc == ODT_150) {
	    mc->drvcfg |= M_BCM1480_MC_DQ_ODT_150;
	    mc->drvcfg |= M_BCM1480_MC_DQS_ODT_150;
	    }
	else if (mc->odt2_mc == ODT_75) {
	    mc->drvcfg |= M_BCM1480_MC_DQ_ODT_75;
	    mc->drvcfg |= M_BCM1480_MC_DQS_ODT_75;
	    }
	}
	
    if (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0) {
	if ((mc->flags & MCFLG_DQO_SHIFT) && do_dqoshift)
	    mc->dllcfg |= M_BCM1480_MC_DQO_SHIFT;
	if (mc->dllcfg_override == FALSE) {
	    mc->dllcfg &= ~(M_BCM1480_MC_ADDR_FREQ_RANGE |M_BCM1480_MC_DQI_FREQ_RANGE |
			    M_BCM1480_MC_DQO_FINE_ADJ | M_BCM1480_MC_DLL_FREQ_RANGE);
	    mc->dllcfg |= V_BCM1480_MC_ADDR_FREQ_RANGE(freq_range) |
			   V_BCM1480_MC_DQI_FREQ_RANGE(freq_range) |
			   V_BCM1480_MC_DQO_FREQ_RANGE(freq_range) |
			   V_BCM1480_MC_DLL_FREQ_RANGE(freq_range);
	    }
	}
	
    WRITECSR(base+R_BCM1480_MC_DLL_CFG,mc->dllcfg);
    WRITECSR(base+R_BCM1480_MC_DRIVE_CFG,mc->drvcfg);

    /*
     * Memory ODT config register
     */
    if ((G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0) && (mc->dramtype == JEDEC_DDR2)) {
	if (mc->numdimms <= 1)
	    WRITECSR(base+R_BCM1480_MC_ODT,mc->odtcfg);
	else
	    WRITECSR(base+R_BCM1480_MC_ODT,mc->odtcfg2);
	}	    

    /*
     * Chan 1 drive config bits (31:0 only) are used for chan 1,2,3.
     */
    if (mcidx > MC_CHAN0) {
	/* Need to mask out only 31:0 bits from current channel */ 
	drvcfg = READCSR(PHYS_TO_K1(A_BCM1480_MC_BASE(MC_CHAN1))+R_BCM1480_MC_DRIVE_CFG) 
	    & ~(_SB_MAKEMASK(32,0));
	drvcfg |= (mc->drvcfg & _SB_MAKEMASK(32,0));
	WRITECSR(PHYS_TO_K1(A_BCM1480_MC_BASE(MC_CHAN1))+R_BCM1480_MC_DRIVE_CFG,drvcfg);
	}

#if _BCM1480_PASS1_WORKAROUNDS_
    /*
     * Start ZCLK-MCLK ratio at 24 (6:1) and increase gradually so master DLL locks
     * on right harmonic of the clock.
     */
    int i;
    for (i=24;i > clk_ratio;i--) {
	mclkcfg = V_BCM1480_MC_CLK_RATIO(i) |
	    V_BCM1480_MC_REF_RATE(refrate);
	WRITECSR(base+R_BCM1480_MC_MCLK_CFG,mclkcfg);
	}
#endif

    /*
     * Set the clk_ratio and refresh rate in memory clock config register. Disable AR for now.
     */
    mclkcfg = V_BCM1480_MC_CLK_RATIO(clk_ratio) |
	V_BCM1480_MC_REF_RATE(refrate);

    /*
     * Only disable if it's B0 or higher.
     */
    if (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0)
	mclkcfg |= M_BCM1480_MC_AUTO_REF_DIS;		      

    WRITECSR(base+R_BCM1480_MC_MCLK_CFG,mclkcfg);

    /*
     * In the case of 64-bit channels, do the same things on the
     * "ganged" channel so that they'll have the same timing.
     * Internally, they are ignored
     */
    if (mc->chantype == MC_64BIT_CHAN) {
	base = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx+2));
	WRITECSR(base+R_BCM1480_MC_TIMING1,timing1);
	WRITECSR(base+R_BCM1480_MC_DLL_CFG,mc->dllcfg);
	WRITECSR(base+R_BCM1480_MC_DRIVE_CFG,mc->drvcfg);
	WRITECSR(base+R_BCM1480_MC_MCLK_CFG,mclkcfg);
	}
}


/*  *********************************************************************
    *  BCM1480_MANUAL_TIMING(mcidx,mc)
    *  
    *  Program the timing registers, for the case of user-specified
    *  timing parameters (don't calculate values based on datasheet
    *  values, just stuff the info into the MC registers)
    *  
    *  Input parameters: 
    *  	   mcidx - memory controller index
    *  	   mc - memory controller data
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void bcm1480_manual_timing(int mcidx,mcdata_t *mc)
{
    unsigned int plldiv;
    unsigned int clk_ratio;
    unsigned int refrate;
    unsigned int ref_freq;
    unsigned int tCpuClk;
    unsigned int tMemClk;

    uint64_t timing1, mclkcfg, drvcfg;
    sbport_t base;

    /* 
     * We need our cpu clock for all sorts of things.
     */

#if defined(_FUNCSIM_)
    plldiv = 16;		/* 800MHz CPU for functional simulation */
#else
    plldiv = G_BCM1480_SYS_PLL_DIV(READCSR(PHYS_TO_K1(A_SCD_SYSTEM_CFG)));
#endif
    if (plldiv == 0) {
	/* XXX: should be common macro, also defaulted by boards' *_devs.c.  */
	plldiv = 6;
	}

    /* See comments in auto_timing for details */
    tCpuClk = 2000000/(BCM1480_REFCLK*plldiv);	/* tCpuClk is in picoseconds */

    /* Compute MAX(MIN_tMEMCLK,spd_tCK_25) */
    tMemClk = DECTO10THS(mc->tCK);
    if (mc->mintmemclk > tMemClk) tMemClk = mc->mintmemclk;

    clk_ratio = ((tMemClk*100) + tCpuClk - 1) / tCpuClk;
    clk_ratio = clk_ratio * 2;
    if (clk_ratio < 4) clk_ratio = 4;
    if (clk_ratio > 24) clk_ratio = 24;

    /* recompute tMemClk using the new clk_ratio */

    tMemClk = (10000 * clk_ratio)/(BCM1480_REFCLK * plldiv);

    /* Calculate the refresh rate */

    switch (mc->rfsh & JEDEC_RFSH_MASK) {
	case JEDEC_RFSH_64khz:	ref_freq = 64;  break;
	case JEDEC_RFSH_256khz:	ref_freq = 256; break;
	case JEDEC_RFSH_128khz:	ref_freq = 128; break;
	case JEDEC_RFSH_32khz:	ref_freq = 32;  break;
	case JEDEC_RFSH_8khz:	ref_freq = 16;  break;
	default: 		ref_freq = 8;   break;
	}

    refrate = ((plldiv * BCM1480_REFCLK * 1000 / 2) / (ref_freq*32*clk_ratio)) - 1;

    timing1 = mc->mantiming;

    base = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx));
    WRITECSR(base+R_BCM1480_MC_TIMING1,timing1);

#if _BCM1480_PASS1_WORKAROUNDS_
    /*
     * Start ZCLK-MCLK ratio at 24 (6:1) and increase gradually so master DLL locks
     * on right harmonic of the clock.
     */
    int i;
    for (i=24;i > clk_ratio;i--) {
	mclkcfg = V_BCM1480_MC_CLK_RATIO(i) |
	    V_BCM1480_MC_REF_RATE(refrate);
	WRITECSR(base+R_BCM1480_MC_MCLK_CFG,mclkcfg);
	}
#endif

    mclkcfg = V_BCM1480_MC_CLK_RATIO(clk_ratio) |
	V_BCM1480_MC_REF_RATE(refrate);

    WRITECSR(base+R_BCM1480_MC_MCLK_CFG,mclkcfg);

    /*
     * Memory DLL config and memory drive config registers.
     */
    WRITECSR(base+R_BCM1480_MC_DLL_CFG,mc->dllcfg);
    WRITECSR(base+R_BCM1480_MC_DRIVE_CFG,mc->drvcfg);

    /*
     * Chan 1 drive config bits (31:0 only) are used for chan 1,2,3.
     */
    if (mcidx > MC_CHAN0) {
	/* Need to mask out only 31:0 bits from current channel */ 
	drvcfg =  READCSR(base+R_BCM1480_MC_DRIVE_CFG) | _SB_MAKEMASK(32,0);
	WRITECSR(PHYS_TO_K1(A_BCM1480_MC_BASE(MC_CHAN1))+R_BCM1480_MC_DRIVE_CFG,drvcfg);
	}

    /*
     * In the case of 64-bit channels, do the same things on the
     * "ganged" channel so that they'll have the same timing.
     */
    if (mc->chantype == MC_64BIT_CHAN) {
	base = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx+2));
	WRITECSR(base+R_BCM1480_MC_TIMING1,timing1);
	WRITECSR(base+R_BCM1480_MC_MCLK_CFG,mclkcfg);
	WRITECSR(base+R_BCM1480_MC_DLL_CFG,mc->dllcfg);
	WRITECSR(base+R_BCM1480_MC_DRIVE_CFG,mc->drvcfg);
	}

}

/*  *********************************************************************
    *  BCM1480_SMBUS_INIT()
    *  
    *  Initialize SMBUS channel 
    *  
    *  Input parameters: 
    *  	   chan - SMBus channel number, 0 or 1
    *  	   
    *  Return value:
    *  	   smbus_base - KSEG1 address of SMBus channel
    *  
    *  Registers used:
    *  	   tmp0
    ********************************************************************* */

static sbport_t bcm1480_smbus_init(int chan)
{
    sbport_t base;

    base = PHYS_TO_K1(A_SMB_BASE(chan));

    WRITECSR(base+R_SMB_FREQ,K_SMB_FREQ_100KHZ);
    WRITECSR(base+R_SMB_CONTROL,0);

    return base;
}
	

/*  *********************************************************************
    *  BCM1480_SMBUS_WAITREADY()
    *  
    *  Wait for SMBUS channel to be ready.
    *  
    *  Input parameters: 
    *  	   smbus_base - SMBus channel base (K1seg addr)
    *  	   
    *  Return value:
    *  	   ret0 - 0 if no error occured, else -1
    *  
    *  Registers used:
    *  	   tmp0,tmp1
    ********************************************************************* */

static int bcm1480_smbus_waitready(sbport_t base)
{
    uint64_t status;

    /*	
     * Wait for busy bit to clear
     */

    for (;;) {
	status = READCSR(base+R_SMB_STATUS);
	if (!(status & M_SMB_BUSY)) break;
	}

    /*
     * Isolate error bit and clear error status
     */

    status &= M_SMB_ERROR;
    WRITECSR(base+R_SMB_STATUS,status);

    /*
     * Return status
     */

    return (status) ? -1 : 0;
}



/*  *********************************************************************
    *  BCM1480_SMBUS_READBYTE()
    *  
    *  Read a byte from a serial ROM attached to an SMBus channel
    *  
    *  Input parameters: 
    *      base - SMBus channel base address (K1seg addr)
    *  	   dev - address of device on SMBUS
    *      offset - address of byte within device on SMBUS
    *  	   
    *  Return value:
    *  	   byte from device (-1 indicates an error)
    ********************************************************************* */


static int bcm1480_smbus_readbyte(sbport_t base,unsigned int dev,unsigned int offset)
{
    int res;

    /*
     * Wait for channel to be ready
     */

    res = bcm1480_smbus_waitready(base);
    if (res < 0) return res;

    /*
     * Set up a READ BYTE command.  This command has no associated
     * data field, the command code is the data 
     */

    WRITECSR(base+R_SMB_CMD,offset);
    WRITECSR(base+R_SMB_START,dev | V_SMB_TT(K_SMB_TT_CMD_RD1BYTE));

    /* 
     * Wait for the command to complete
     */

    res = bcm1480_smbus_waitready(base);
    if (res < 0) return res;

    /*
     * Return the data byte
     */

    return (int) ((READCSR(base+R_SMB_DATA)) & 0xFF);
}


/*  *********************************************************************
    *  BCM1480_DRAM_GETINFO
    *  
    *  Process a single init table entry and move data into the
    *  memory controller's data structure.
    *  
    *  Input parameters: 
    *	   smbase - points to base of SMbus device to read from
    *  	   mc - memory controller data
    *      init - pointer to current user init table entry
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void bcm1480_dram_getinfo(unsigned int smbchan,
				unsigned int smbdev,
				mcdata_t *mc,
				int chipsel)

{
    int res;
    unsigned char spd[JEDEC_SPD_SIZE];
    int idx, sides, altCS;
    csdata_t *cs = &(mc->csdata[chipsel]);
    sbport_t smbase;

    smbase = bcm1480_smbus_init(smbchan);

    /*
     * Read just the memory type to see if the RAM is present.
     */

    res = bcm1480_smbus_readbyte(smbase,smbdev,JEDEC_SPD_MEMTYPE);

    if ((res < 0) || ((res != JEDEC_MEMTYPE_DDRSDRAM) && 
		      (res != JEDEC_MEMTYPE_DDRSDRAM2) &&
	              (res != SPD_MEMTYPE_FCRAM) &&
	              (res != SPD_MEMTYPE_DDR2))) {
	return;			/* invalid or no memory installed */
	}

    /*
     * Now go back and read everything.
     */

    res = 0;
    for (idx = 0; idx < JEDEC_SPD_SIZE; idx++) {
	res = bcm1480_smbus_readbyte(smbase,smbdev,idx);
	if (res < 0) break;
	spd[idx] = res;
	}

    if (res < 0) return;		/* some SMBus error */

    mc->numdimms++;
    cs->rows = spd[JEDEC_SPD_ROWS];
    cs->cols = spd[JEDEC_SPD_COLS];

    /*
     * Determine how many bits the banks represent.  Unlike
     * the rows/columns, the bank byte says how *many* banks
     * there are, not how many bits represent banks
     */

    switch (spd[JEDEC_SPD_BANKS]) {
	case 2:					/* 2 banks = 1 bits */
	    cs->banks = 1;
	    break;
	case 4:					/* 4 banks = 2 bits */
	    cs->banks = 2;
	    break;
	case 8:					/* 8 banks = 3 bits */
	    cs->banks = 3;
	    break;
	case 16:				/* 16 banks = 4 bits */
	    cs->banks = 4;
	    break;
	default:				/* invalid bank count */
	    return;
	}


    /*
     * Read timing parameters from the DIMM.  By this time we kind of trust
     */

    cs->spd_dramtype   = spd[JEDEC_SPD_MEMTYPE];
    cs->spd_tCK_25     = spd[JEDEC_SPD_tCK25];
    cs->spd_tCK_20     = spd[JEDEC_SPD_tCK20];
    cs->spd_tCK_10     = spd[JEDEC_SPD_tCK10];
    cs->spd_rfsh       = spd[JEDEC_SPD_RFSH];
    cs->spd_caslatency = spd[JEDEC_SPD_CASLATENCIES];
    cs->spd_attributes = spd[JEDEC_SPD_ATTRIBUTES];
    cs->spd_tRAS       = spd[JEDEC_SPD_tRAS];
    cs->spd_tRP        = spd[JEDEC_SPD_tRP];
    cs->spd_tRRD       = spd[JEDEC_SPD_tRRD];
    cs->spd_tRCD       = spd[JEDEC_SPD_tRCD];
    cs->spd_tRFC       = spd[JEDEC_SPD_tRFC];
    cs->spd_tRC        = spd[JEDEC_SPD_tRC];
    cs->spd_tRTP       = spd[JEDEC_SPD_tRTP];
    cs->spd_dimmtype   = spd[JEDEC_SPD_DIMMTYPE];

    /*
     * Okay, we got all the required data.  mark this CS present.
     */

    cs->flags = CS_PRESENT | CS_AUTO_TIMING;

    /*
     * If the module width is not 72 for any DIMM, disable ECC for this
     * channel.  All DIMMs must support ECC for us to enable it.
     */

    if (spd[JEDEC_SPD_WIDTH] != 72) mc->flags &= ~MCFLG_ECC_ENABLE;

    /*
     * If it was a double-sided DIMM, also mark the odd chip select
     * present.
     */
    
    sides = spd[JEDEC_SPD_SIDES] & 0x07;
    altCS = 1;
    if (cs->spd_dramtype == SPD_MEMTYPE_DDR2) {
	sides++; /* Jedec spec doe DDR2 is 3'b000 = 1 rank, 3'b001 = 2 ranks, etc */
	altCS = 2; /* For DDR2, second rank (on same dimm) is always +2 */
	if (mc->flags & MCFLG_NO_ODT_CS) altCS = 1;
	}

    if ((sides == 2) && !(mc->flags & MCFLG_BIGMEM)) {
	csdata_t *oddcs = &(mc->csdata[chipsel | altCS]);

	oddcs->rows  = cs->rows;
	oddcs->cols  = cs->cols;
	oddcs->banks = cs->banks;
	oddcs->flags = CS_PRESENT;

	oddcs->spd_dramtype   = spd[JEDEC_SPD_MEMTYPE];
	oddcs->spd_tCK_25     = spd[JEDEC_SPD_tCK25];
	oddcs->spd_tCK_20     = spd[JEDEC_SPD_tCK20];
	oddcs->spd_tCK_10     = spd[JEDEC_SPD_tCK10];
	oddcs->spd_rfsh       = spd[JEDEC_SPD_RFSH];
	oddcs->spd_caslatency = spd[JEDEC_SPD_CASLATENCIES];
	oddcs->spd_attributes = spd[JEDEC_SPD_ATTRIBUTES];
	oddcs->spd_tRAS       = spd[JEDEC_SPD_tRAS];
	oddcs->spd_tRP        = spd[JEDEC_SPD_tRP];
	oddcs->spd_tRRD       = spd[JEDEC_SPD_tRRD];
	oddcs->spd_tRCD       = spd[JEDEC_SPD_tRCD];
	oddcs->spd_tRFC       = spd[JEDEC_SPD_tRFC];
	oddcs->spd_tRC        = spd[JEDEC_SPD_tRC];
	oddcs->spd_tRTP	      = spd[JEDEC_SPD_tRTP];
	oddcs->spd_dimmtype   = spd[JEDEC_SPD_DIMMTYPE];
	}
    else if ((sides > 2) && !(mc->flags & MCFLG_BIGMEM)
	     && (chipsel == 0)) {
	
	/* More than 2 chip selects on a single DIMM. Start from cs 1 */
	csdata_t *loopcs;
	int i;
	for (i=chipsel+1;i<sides;i++) {
	    loopcs = &(mc->csdata[i]);

	    loopcs->rows  = cs->rows;
	    loopcs->cols  = cs->cols;
	    loopcs->banks = cs->banks;
	    loopcs->flags = CS_PRESENT;

	    loopcs->spd_dramtype   = spd[JEDEC_SPD_MEMTYPE];
	    loopcs->spd_tCK_25     = spd[JEDEC_SPD_tCK25];
	    loopcs->spd_tCK_20     = spd[JEDEC_SPD_tCK20];
	    loopcs->spd_tCK_10     = spd[JEDEC_SPD_tCK10];
	    loopcs->spd_rfsh       = spd[JEDEC_SPD_RFSH];
	    loopcs->spd_caslatency = spd[JEDEC_SPD_CASLATENCIES];
	    loopcs->spd_attributes = spd[JEDEC_SPD_ATTRIBUTES];
	    loopcs->spd_tRAS       = spd[JEDEC_SPD_tRAS];
	    loopcs->spd_tRP        = spd[JEDEC_SPD_tRP];
	    loopcs->spd_tRRD       = spd[JEDEC_SPD_tRRD];
	    loopcs->spd_tRCD       = spd[JEDEC_SPD_tRCD];
	    loopcs->spd_tRFC       = spd[JEDEC_SPD_tRFC];
	    loopcs->spd_tRC        = spd[JEDEC_SPD_tRC];
	    loopcs->spd_tRTP	   = spd[JEDEC_SPD_tRTP];
	    loopcs->spd_dimmtype   = spd[JEDEC_SPD_DIMMTYPE];
	    }
	}
}


/*  *********************************************************************
    *  BCM1480_DRAM_READPARAMS(d,init)
    *  
    *  Read all the parameters from the user parameter table and
    *  digest them into our local data structure.  This routine basically
    *  walks the table and calls the routine above to handle each
    *  entry.
    *  
    *  Input parameters: 
    *  	   d - our data structure (our RAM data)
    *  	   init - pointer to user config table
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void bcm1480_dram_readparams(initdata_t *d,const draminittab_t *init)
{
    mcdata_t *mc;
    csdata_t *cs;
    uint64_t sysrev; 

    sysrev = READCSR(PHYS_TO_K1(A_SCD_SYSTEM_REVISION));

    /* Assume we're staring on the first channel */
    mc = &(d->mc[MC_FIRSTCHANNEL]);
    cs = &(mc->csdata[0]);

    while (init->mcr.mcr_type != MCR_EOT) {

#ifdef _MCSTANDALONE_NOISY_
	printf("DRAM: Processing record '%s'\n",bcm1480_rectypes[init->mcr.mcr_type]);
#endif

	switch (init->mcr.mcr_type) {

	    case MCR_GLOBALS:
		/* Channel interleave type */
		d->cfg_chanintlv_type = init->gbl.gbl_intlv_ch;
		break;

	    case MCR_CHCFG:
		mc = &(d->mc[init->cfg.cfg_chan]);
		mc->mintmemclk = bcm1480_decto100ths(init->cfg.cfg_mintmemclk);

		mc->chantype   = init->cfg.cfg_chantype;
		mc->dramtype   = init->cfg.cfg_dramtype;
		mc->pagepolicy = init->cfg.cfg_pagepolicy;
		mc->cfgcsint   = init->cfg.cfg_intlv_cs;
		mc->numdimms   = 0; /* Zero for now, until we read the SPD */

		mc->dllcfg_override =  FALSE;
		if (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0)
		    mc->dllcfg = V_MC_DLLCONFIG_ADJ_DEFAULT_PASS2; /* Default unless overridden */
		else
		    mc->dllcfg = V_MC_DLLCONFIG_ADJ_DEFAULT; /* Default unless overridden */	
	
		mc->drvcfg = V_MC_DRVCONFIG_CLASS_DEFAULT; /* Default, no overridding */		    
		mc->flags = (init->cfg.cfg_ecc & (MCFLG_ECC_ENABLE | MCFLG_ECC_FORCE64)) |
		    (init->cfg.cfg_flags & (MCFLG_FORCEREG | MCFLG_BIGMEM)) |
		    (init->cfg.cfg_flags & (MCFLG_2T | MCFLG_DQO_SHIFT)) |
		    (init->cfg.cfg_flags & (MCFLG_DS_REDUCED | MCFLG_NO_ODT_CS)) ;

		mc->roundtrip  = DECTO10THS(init->cfg.cfg_roundtrip);
		if (mc->roundtrip == 0 && mc->dramtype != DRAM_TYPE_SPD) {
		    /* 
		     * Only set default roundtrip if mem type is specified, else wait
		     * to get type from SPD 
		     */
		    mc->roundtrip = (mc->dramtype == FCRAM) ?
			DEFAULT_MEMORY_ROUNDTRIP_TIME_FCRAM : DEFAULT_MEMORY_ROUNDTRIP_TIME;
		    }
		if (mc->dramtype == FCRAM) mc->pagepolicy = CLOSED; 	/*FCRAM must be closed page policy*/
		cs = &(mc->csdata[0]);
		break;

	    case MCR_TIMING:
		cs->spd_tCK_25 = init->tmg.tmg_tCK;
		cs->spd_tCK_20 = 0;
		cs->spd_tCK_10 = 0;
		cs->spd_rfsh = init->tmg.tmg_rfsh;
		cs->spd_caslatency = init->tmg.tmg_caslatency;
		cs->spd_attributes = init->tmg.tmg_attributes;
		cs->spd_tRAS = init->tmg.tmg_tRAS;
		cs->spd_tRP = init->tmg.tmg_tRP;
		cs->spd_tRRD = init->tmg.tmg_tRRD;
		cs->spd_tRCD = init->tmg.tmg_tRCD;
		cs->spd_tRFC = init->tmg.tmg_tRFC;
		cs->spd_tRC = init->tmg.tmg_tRC;
		break;

	    case MCR_TIMING2:
		mc->tAL = init->tmg2.tmg2_tAL;
		cs->spd_tRTP = init->tmg2.tmg2_tRTP;
		mc->tRAP = init->tmg2.tmg2_tRAP;     
		break;
	
	    case MCR_DLLCFG:

		mc->dllcfg_override = TRUE;

		if (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0) {
		    mc->dllcfg =
			V_BCM1480_MC_ADDR_FREQ_RANGE(init->dll.dll_addrfine) |
			V_BCM1480_MC_DQI_COARSE_ADJ(init->dll.dll_dqicoarse) |
			V_BCM1480_MC_DQI_FREQ_RANGE(init->dll.dll_dqifine) |
			V_BCM1480_MC_DQO_COARSE_ADJ(init->dll.dll_dqocoarse) |
			V_BCM1480_MC_DQO_FREQ_RANGE(init->dll.dll_dqofine) |
			V_BCM1480_MC_DLL_DEFAULT(init->dll.dll_default) |
			V_BCM1480_MC_DLL_FREQ_RANGE(init->dll.dll_dllfreq);
		    
		    if (init->dll.dll_dllbypass) mc->dllcfg |= _SB_MAKEMASK1(63);
		    }
		else {
		    mc->dllcfg =
			V_BCM1480_MC_ADDR_FINE_ADJ(init->dll.dll_addrfine) |
			V_BCM1480_MC_DQI_COARSE_ADJ(init->dll.dll_dqicoarse) |
			V_BCM1480_MC_DQI_FINE_ADJ(init->dll.dll_dqifine) |
			V_BCM1480_MC_DQO_COARSE_ADJ(init->dll.dll_dqocoarse) |
			V_BCM1480_MC_DQO_FINE_ADJ(init->dll.dll_dqofine);
		    }		    
		    
		break;

	    case MCR_ADDRCOARSE:
		mc->addrc_reg = init->addrc.addrcoarse_reg;
		mc->addrc_unbuf = init->addrc.addrcoarse_unbuf;		

		break;

	    case MCR_ODTCFG:

		mc->odtcfg = 
		            V_BCM1480_MC_ODT0(init->odt.odt_odt0) |
		            V_BCM1480_MC_ODT2(init->odt.odt_odt2) |
		            V_BCM1480_MC_ODT4(init->odt.odt_odt4) |
         		    V_BCM1480_MC_ODT6(init->odt.odt_odt6);

		if(init->odt.odt_odd_odt_en) mc->odtcfg |= M_BCM1480_MC_CS_ODD_ODT_EN;
		
		if(init->odt.odt_mc_value == ODT_150) mc->odt_mc = ODT_150;
		else if(init->odt.odt_mc_value == ODT_75) mc->odt_mc = ODT_75;
		else mc->odt_mc = ODT_OFF;

		if(init->odt.odt_dram_value == ODT_150)mc->odt_dram = ODT_150;
		else if(init->odt.odt_dram_value == ODT_75)mc->odt_dram = ODT_75;
		else if(init->odt.odt_dram_value == ODT_50)mc->odt_dram = ODT_50;
		else mc->odt_dram = ODT_OFF;

		break;

	    case MCR_ODTCFG2:
		
		/*
		 *  When we have 2 DIMMs, we need to have different ODT settings.
		 *  Use DRAM_CHAN_ODTCFG2.
		 */

		mc->odtcfg2 = 
		            V_BCM1480_MC_ODT0(init->odt2.odt_odt0) |
		            V_BCM1480_MC_ODT2(init->odt2.odt_odt2) |
		            V_BCM1480_MC_ODT4(init->odt2.odt_odt4) |
         		    V_BCM1480_MC_ODT6(init->odt2.odt_odt6);

		if(init->odt2.odt_odd_odt_en) mc->odtcfg2 |= M_BCM1480_MC_CS_ODD_ODT_EN;
		
		if(init->odt2.odt_mc_value == ODT_150) mc->odt2_mc = ODT_150;
		else if(init->odt2.odt_mc_value == ODT_75) mc->odt2_mc = ODT_75;
		else mc->odt2_mc = ODT_OFF;

		if(init->odt2.odt_dram_value == ODT_150) mc->odt2_dram = ODT_150;
		else if(init->odt2.odt_dram_value == ODT_75) mc->odt2_dram = ODT_75;
		else if(init->odt2.odt_dram_value == ODT_50) mc->odt2_dram = ODT_50;
		else mc->odt2_dram = ODT_OFF;

		break;


	    case MCR_GEOM:
		cs = &(mc->csdata[init->geom.geom_csel]);
		cs->rows = init->geom.geom_rows;
		cs->cols = init->geom.geom_cols;
		cs->banks = init->geom.geom_banks;
		cs->flags |= CS_PRESENT;
		break;

	    case MCR_SPD:
		cs = &(mc->csdata[init->spd.spd_csel]);
		bcm1480_dram_getinfo(init->spd.spd_smbuschan,
				    init->spd.spd_smbusdev,
				    mc,
				    init->spd.spd_csel);

		if (mc->dramtype == DRAM_TYPE_SPD) {
		    /* Use the DRAM type we get from the SPD */
		    if (cs->spd_dramtype == SPD_MEMTYPE_FCRAM){
			mc->dramtype = FCRAM;
			mc->pagepolicy = CLOSED;
			if (mc->roundtrip == 0) mc->roundtrip = DEFAULT_MEMORY_ROUNDTRIP_TIME_FCRAM;
			}
		    else {
			if(cs->spd_dramtype == SPD_MEMTYPE_DDR2){ 
			    mc->dramtype = JEDEC_DDR2;
			    }
			else {
			    mc->dramtype = JEDEC;
			    }
			if (mc->roundtrip == 0) mc->roundtrip = DEFAULT_MEMORY_ROUNDTRIP_TIME;		
			}
		    }
		/* 
		 * The line below lets you put a MCR_MANTIMING record
		 * before an MCR_SPD to work around some missing information
		 * on certain DIMMs.   Normally you have only one or the
		 * other.  
		 */
		mc->rfsh = cs->spd_rfsh;
		/* XXX flags ignored */
		break;

	    case MCR_MANTIMING:
		/* Manual timing - pick record up as bytes because we cannot
		   guarantee the alignment of the "mtm_timing" field in our
		   structure -- each row is 12 bytes, not good */
		mc->rfsh = (uint16_t) init->mtm.mtm_rfsh;	/* units: JEDEC refresh value */
		mc->tCK  = (uint16_t) init->mtm.mtm_tCK;	/* units: BCD, like SPD */
		mc->mantiming =
		    (((uint64_t) init->mtm.mtm_timing[0]) << 56) |
		    (((uint64_t) init->mtm.mtm_timing[1]) << 48) |
		    (((uint64_t) init->mtm.mtm_timing[2]) << 40) |
		    (((uint64_t) init->mtm.mtm_timing[3]) << 32) |
		    (((uint64_t) init->mtm.mtm_timing[4]) << 24) |
		    (((uint64_t) init->mtm.mtm_timing[5]) << 16) |
		    (((uint64_t) init->mtm.mtm_timing[6]) << 8) |
		    (((uint64_t) init->mtm.mtm_timing[7]) << 0);
		break;

	    default:
		break;
	    }

	init++;
	}

    /*
     * Okay, now we've internalized all the data from the SPDs
     * and/or the init table.
     */
    
}



/*  *********************************************************************
    *  BCM1480_DRAMINIT_DELAY
    *  
    *  This little routine delays at least 200 microseconds.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing.
    *  	   
    *  Registers used:
    *  	   tmp0,tmp1
    ********************************************************************* */

/* 200 microseconds = 5KHz, so delay 1GHz/5Khz = 200,000 cycles */

#if defined(_FASTEMUL_) && !defined(_DENALI_)
#define DRAMINIT_DELAY_CNT	"50"
#else
#define DRAMINIT_DELAY_CNT	 "(1000000000/5000)"
#endif

#if defined(_MCSTANDALONE_)
#if defined(_MCSTANDALONE_NOISY_)
#define DRAMINIT_DELAY()	printf("DELAY 200us\n");
#define bcm1480_draminit_smalldelay(x) printf("DELAY 200 MCLKS\n");
#else
#define DRAMINIT_DELAY()		/* not running on a 1250, no delays */
#define bcm1480_draminit_smalldelay(x)
#endif
#else
#define DRAMINIT_DELAY() bcm1480_draminit_delay()

static void bcm1480_draminit_delay(void)
{
    __asm("     li $9," DRAMINIT_DELAY_CNT " ; "
	  "     mtc0 $0,$9 ; "
	  "1:   mfc0 $8,$9 ; "
	  "     .set push ; .set mips64 ; ssnop ; ssnop ; .set pop ;"
	  "     blt $8,$9,1b ;");
}
static void bcm1480_draminit_smalldelay(uint64_t clks)
{
    __asm __volatile ("     move $9,%0 ; "
	  "     mtc0 $0,$9 ; "
	  "1:   mfc0 $8,$9 ; "
	  "     .set push ; .set mips64 ; ssnop ; ssnop ; .set pop ;"
	  "     blt $8,$9,1b ;" :: "r"(clks));
}
#endif

#if _BCM1480_PASS1_WORKAROUNDS_
static void bcm1480_cmd_active_check(sbport_t cmd)
{
    uint64_t status;

    for (;;) {
	status = READCSR(cmd);
	if (!(status & M_BCM1480_MC_CMD_ACTIVE)) break;
	}
}
#endif    

/*  *********************************************************************
    *  MAKEDRAMMASK(dest,width,pos)
    *  
    *  Create a 64-bit mask for the DRAM config registers
    *  
    *  Input parameters: 
    *  	   width - number of '1' bits to set
    *  	   pos - position (from the right) of the first '1' bit
    *  	   
    *  Return value:
    *  	   mask with specified with at specified position 
    ********************************************************************* */

#define MAKEDRAMMASK(width,pos) _SB_MAKEMASK(width,pos)

/*  *********************************************************************
    *  BCM1480_JEDEC_DDR2_INITCMDS
    *  
    *  Issue the sequence of DRAM init commands (JEDEC DDR2)
    *  
    *  Input parameters: 
    *      mcnum - memory controller index (0/1/2/3)
    *  	   mc - pointer to data for this memory controller
    *	   csel - which chip select to send init commands for
    *	   lastcs - last available chip select to set ODT
    *      lmbank - for largemem systems, the cs qualifiers to be
    *		    output on CS[2:3]
    *      tdata - chip select to use as a template for timing data
    *	   d - pointer to initdata_t   	   
    *
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void bcm1480_jedec_ddr2_initcmds(int mcnum,mcdata_t *mc,unsigned int csel,
					unsigned int lastcs, int lmbank,csdata_t *tdata,
					initdata_t *d)
{
    uint64_t csmask, odtcs;
    sbport_t cmd, mclkcfg;
    sbport_t mode;
    uint64_t modebits, casbits, wrbits, albits;
    uint64_t clk_ratio;
    uint64_t plldiv;
    uint64_t ns400;
    uint64_t mclkcfgbits;
    uint64_t sysrev;

    sysrev = READCSR(PHYS_TO_K1(A_SCD_SYSTEM_REVISION));

    /* clkratio = number of core clocks per mclk */
    mclkcfg = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_MCLK_CFG));
    clk_ratio = G_BCM1480_MC_CLK_RATIO(READCSR(mclkcfg));

#if defined(_FUNCSIM_)
    plldiv = 16;
#else
    plldiv = G_BCM1480_SYS_PLL_DIV(READCSR(PHYS_TO_K1(A_SCD_SYSTEM_CFG)));
#endif
    if (plldiv == 0) {
	plldiv = 6;
	}

    /*
     * During the init sequence we'll need to delay 400ns (or thereabouts).
     * Calculate this from the system PLL setting
     *
     *    Freq(MHz) = reference_clock * plldiv / 2
     * 
     * Now figure out how much 400ns is.  Since MHz is the same units as uSec,
     * multiply by 1000 first.
     *
     *	 400ns = (400 * 1000) / Freq(Mhz)
     *
     * Double delay to make sure commands get through
     */

    ns400 = (  6*(400 * 1000)) / (BCM1480_REFCLK * plldiv / 2);

    csmask = csel << S_BCM1480_MC_CS0;	/* move mask to right place */
    odtcs = lastcs << S_BCM1480_MC_CS0;

    if (mc->flags & MCFLG_BIGMEM) {
	/* 
	 * so that the banks will all get their precharge signals,
	 * put the CS qualifiers out on CS[2:3].
	 */	   
	csmask |= (uint64_t)(lmbank << 6);
	}

    cmd  = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_DRAMCMD));
    mode = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_DRAMMODE));

    /*
     * Using the data in the timing template, figure out which
     * CAS latency command to issue. Only whole CAS lat for DDR2.
     */

    casbits = 0;
    
    if (mc->chantype == MC_64BIT_CHAN) {

	/* Use 4-byte bursts for 64-bit channels */
	switch (tdata->flags & CS_CASLAT_MASK) {
	    case CS_CASLAT_60:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_DDR2_CAS6);
		break;
	    case CS_CASLAT_50:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_DDR2_CAS5);
		break;
	    default:
	    case CS_CASLAT_40:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS4);
		break;
	    case CS_CASLAT_30:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS3);
		break;
	    case CS_CASLAT_20:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS2);
		break;
	    }
	}
    else if (mc->chantype == MC_32BIT_CHAN) {
	/* Use 8-byte bursts for 32-bit channels */
	switch (tdata->flags & CS_CASLAT_MASK) {
	    case CS_CASLAT_60:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_DDR2_CAS6_BL8);
		break;
	    case CS_CASLAT_50:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_DDR2_CAS5_BL8);
		break;
	    case CS_CASLAT_40:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS4_BL8);
		break;
	    default:
	    case CS_CASLAT_30:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS3_BL8);
		break;
	    case CS_CASLAT_20:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS2_BL8);
		break;
	    }
	}

    switch (mc->tWR) {
        case 5:
            wrbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_WR_5);
            break;
        case 4:
            wrbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_WR_4);
            break;
	case 3:
	    wrbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_WR_3);
	    break;
	case 2:
	    wrbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_WR_2);
	    break;
	case 6:
	default:
	    wrbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_WR_6);
	    break;
	}    

    switch (mc->tAL) {
        case 7:
            albits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_MRVAL_AL_7);
            break;
        case 6:
            albits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_MRVAL_AL_6);
            break;
        case 5:
            albits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_MRVAL_AL_5);
            break;
        case 4:
            albits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_MRVAL_AL_4);
            break;
	case 3:
	    albits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_MRVAL_AL_3);
	    break;
	case 2:
	    albits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_MRVAL_AL_2);
	    break;
	case 1:
	    albits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_MRVAL_AL_1);
	    break;
	case 0:
	default:
	    albits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_MRVAL_AL_0);
	    break;
	}    

    /*
     * Set up for doing mode register writes to the SDRAMs
     * 
     * First time, we set bit 8 to reset the DLL on MR
     * 
     * Enable DLL on EMR
     */

    modebits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL) |
	       V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_RESETDLL) |
	       casbits | wrbits | albits;

    /*
     * Check to see if we need to set reduced drive strength.
     */
    if (mc->flags & MCFLG_DS_REDUCED)
	modebits |= V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_DS_REDUCED);

    /*
     * Bring the "bcm1480_draminit_smalldelay" routine into the ICache to
     * prevent the refresh from interfering between power-down-clear and
     * precharge.
     */

    bcm1480_draminit_smalldelay(1);

    /* 
     * Turn on output memory and wait for clock to stabilize.
     */
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_ENABLE_MCLK);
    bcm1480_draminit_smalldelay(ns400);

    /*
     * Remember to set the "ganged" bit properly if in 64-bit mode.
     * Also, set the page policy here.
     */

    WRITECSR(mode,modebits | mc->modebits );

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_CLRPWRDN);

    /*
     * Wait 400ns.
     */

    bcm1480_draminit_smalldelay(ns400);
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_cmd_active_check(cmd);
#endif

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_PRE);
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_draminit_smalldelay(ns400);
    bcm1480_cmd_active_check(cmd);
#endif

    /* 
     * EMRS2 and EMRS3. Send zeros in emode field for now.
     */
    WRITECSR(mode,mc->modebits);   
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_EMRS2);
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_EMRS3);


    WRITECSR(mode,modebits | mc->modebits );

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_EMRS);
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_draminit_smalldelay(ns400);
    bcm1480_cmd_active_check(cmd);
#endif

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_MRS);
    
    bcm1480_draminit_smalldelay(ns400);
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_cmd_active_check(cmd);
#endif

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_PRE);
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_cmd_active_check(cmd);
    bcm1480_draminit_smalldelay(ns400);   
#endif

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_AR);
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_draminit_smalldelay(ns400);
    bcm1480_cmd_active_check(cmd);
#endif

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_AR);
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_draminit_smalldelay(ns400);
    bcm1480_cmd_active_check(cmd);
#endif

    /*
     * This time, clear bit 8 to start the DLL
     */

    modebits &= ~V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_RESETDLL);
    WRITECSR(mode,modebits | mc->modebits );
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_MRS);
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_draminit_smalldelay(ns400);
    bcm1480_cmd_active_check(cmd);
#endif

    /*
     * Enable OCD default by setting bits 7,8,and 9 on EMR.
     */
    modebits |= V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_OCD_DEFAULT);	     
    WRITECSR(mode,modebits | mc->modebits );
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_EMRS);
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_draminit_smalldelay(ns400);
    bcm1480_cmd_active_check(cmd);
#endif

    /*
     * Enable OCD exit by clearing bits 7,8,and 9 on EMR.
     */
    modebits &= ~V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_OCD_DEFAULT); 
    WRITECSR(mode,modebits | mc->modebits );
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_EMRS);    
#if _BCM1480_PASS1_WORKAROUNDS_
    bcm1480_draminit_smalldelay(ns400);
    bcm1480_cmd_active_check(cmd);
#endif

    /* 
     * ODT setting
     *
     * FOR REV < B0 ONLY
     * We only want to set DRAM ODT to one CS (regardless of how many
     * CS's of memory we actually have). So just set last available
     * CS. 
     */	
    if (mc->numdimms <= 1) {
	if (mc->odt_dram == ODT_150)
	    modebits |= V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_RTT_150);
	else if (mc->odt_dram == ODT_75)
	    modebits |= V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_RTT_75);
	}
    else { /* 2 or more DIMMs */
	if (mc->odt2_dram == ODT_150) 
	    modebits |= V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_RTT_150);
	else if (mc->odt2_dram == ODT_75)
	    modebits |= V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_RTT_75);
       	else if (mc->odt2_dram == ODT_50)
            modebits |= V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_RTT_50);
	}
    WRITECSR(mode,modebits | mc->modebits );
    if (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0)
	WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_EMRS);
    else
	WRITECSR(cmd,odtcs | V_BCM1480_MC_COMMAND_EMRS);	

    /* Clear bit 16 auto_refresh_dis */
    if (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0) {
	mclkcfgbits = READCSR(mclkcfg);
	mclkcfgbits &= ~M_BCM1480_MC_AUTO_REF_DIS;
	WRITECSR(mclkcfg, mclkcfgbits);
	}
} 
    
/*  *********************************************************************
    *  BCM1480_JEDEC_INITCMDS
    *  
    *  Issue the sequence of DRAM init commands (JEDEC SDRAMs)
    *  
    *  Input parameters: 
    *      mcnum - memory controller index (0/1/2/3)
    *  	   mc - pointer to data for this memory controller
    *	   csel - which chip select to send init commands for
    *      lmbank - for largemem systems, the cs qualifiers to be
    *		    output on CS[2:3]
    *      tdata - chip select to use as a template for timing data
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void bcm1480_jedec_initcmds(int mcnum,mcdata_t *mc,unsigned int csel,
				  int lmbank,csdata_t *tdata)
{
    uint64_t csmask;
    sbport_t cmd;
    sbport_t mode;
    uint64_t modebits;
    uint64_t casbits;
    uint64_t clk_ratio;
    uint64_t plldiv;
    uint64_t ns400;

    /* clkratio = number of core clocks per mclk */
    cmd = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_MCLK_CFG));
    clk_ratio = G_BCM1480_MC_CLK_RATIO(READCSR(cmd));

#if defined(_FUNCSIM_)
    plldiv = 16;
#else
    plldiv = G_BCM1480_SYS_PLL_DIV(READCSR(PHYS_TO_K1(A_SCD_SYSTEM_CFG)));
#endif
    if (plldiv == 0) {
	plldiv = 6;
	}

    /*
     * During the init sequence we'll need to delay 400ns (or thereabouts).
     * Calculate this from the system PLL setting
     *
     *    Freq(MHz) = reference_clock * plldiv / 2
     * 
     * Now figure out how much 400ns is.  Since MHz is the same units as uSec,
     * multiply by 1000 first.
     *
     *	 400ns = (400 * 1000) / Freq(Mhz)
     */

    ns400 = (400 * 1000) / (BCM1480_REFCLK * plldiv / 2);

    csmask = csel << S_BCM1480_MC_CS0;	/* move mask to right place */

    if (mc->flags & MCFLG_BIGMEM) {
	/* 
	 * so that the banks will all get their precharge signals,
	 * put the CS qualifiers out on CS[2:3].
	 */	   
	csmask |= (uint64_t)(lmbank << 6);
	}

    cmd  = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_DRAMCMD));
    mode = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_DRAMMODE));

    /*
     * Using the data in the timing template, figure out which
     * CAS latency command to issue.
     */

    casbits = 0;
    
    if (mc->chantype == MC_64BIT_CHAN) {
	/* Use 4-byte bursts for 64-bit channels */
	switch (tdata->flags & CS_CASLAT_MASK) {
	    case CS_CASLAT_30:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS3);
		break;
	    default:
	    case CS_CASLAT_25:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS25);
		break;
	    case CS_CASLAT_20:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS2);
		break;
	    case CS_CASLAT_15:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS15);
		break;
	    }
	}
    else if (mc->chantype == MC_32BIT_CHAN) {
	/* Use 8-byte bursts for 32-bit channels */
	switch (tdata->flags & CS_CASLAT_MASK) {
	    case CS_CASLAT_30:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS3_BL8);
		break;
	    default:
	    case CS_CASLAT_25:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS25_BL8);
		break;
	    case CS_CASLAT_20:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS2_BL8);
		break;
	    case CS_CASLAT_15:
		casbits = V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_CAS15_BL8);
		break;
	    }
	}

    /*
     * Set up for doing mode register writes to the SDRAMs
     * 
     * First time, we set bit 8 to reset the DLL
     */

    modebits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL) |
	V_BCM1480_MC_MODE(JEDEC_SDRAM_MRVAL_RESETDLL);

    /*
     * Check to see if we need to set reduced drive strength.
     */
    if (mc->flags & MCFLG_DS_REDUCED)
	modebits |= V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_DS_REDUCED);

    /*
     * Bring the "bcm1480_draminit_smalldelay" routine into the ICache to
     * prevent the refresh from interfering between power-down-clear and
     * precharge.
     */

    bcm1480_draminit_smalldelay(1);

    /*
     * Remember to set the "ganged" bit properly if in 64-bit mode.
     * Also, set the page policy here.
     */

    WRITECSR(mode,modebits | casbits | mc->modebits);

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_CLRPWRDN);

    /*
     * Wait 400ns.
     */

    bcm1480_draminit_smalldelay(ns400);


    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_PRE);
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_EMRS);
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_MRS);
    
    bcm1480_draminit_smalldelay(ns400);

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_PRE);
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_AR);
    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_AR);

    /*
     * This time, clear bit 8 to start the DLL
     */

    modebits = V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL) |
	V_BCM1480_MC_MODE_DEFAULT;

    /*
     * Check to see if we need to set reduced drive strength.
     */
    if (mc->flags & MCFLG_DS_REDUCED)
	modebits |= V_BCM1480_MC_EMODE(JEDEC_SDRAM_EMRVAL_DS_REDUCED);

    WRITECSR(mode,modebits | casbits | mc->modebits);

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_MRS);

}

/*  *********************************************************************
    *  BCM1480_SGRAM_INITCMDS
    *  
    *  Issue the sequence of DRAM init commands. (SGRAMs)
    *  Note: this routine does not support "big memory" (external decode)
    *  
    *  Input parameters: 
    *      mcnum - memory controller index (0/1/2/3)
    *  	   mc - pointer to data for this memory controller
    *	   csel - which chip select to send init commands for
    *      tdata - chip select to use as a template for timing data
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void bcm1480_sgram_initcmds(int mcnum,mcdata_t *mc,
				   unsigned int csel,csdata_t *tdata)
{
    uint64_t csmask;
    sbport_t cmd;
    sbport_t mode;
    uint64_t modebits;
    uint64_t casbits;

    csmask = csel << S_BCM1480_MC_CS0;		/* move mask to right place */
    cmd  = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_DRAMCMD));
    mode = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_DRAMMODE));


    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_CLRPWRDN);
    DRAMINIT_DELAY();

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_PRE);
    DRAMINIT_DELAY();

    /*
     * Set up for doing mode register writes to the SDRAMs
     * 
     * mode 0x62 is "sequential 4-byte bursts, CAS Latency 2.5"
     * mode 0x22 is "sequential 4-byte bursts, CAS Latency 2"
     *
     * First time, we set bit 8 to reset the DLL
     */
    modebits = V_BCM1480_MC_EMODE(SGRAM_EMRVAL) |
	V_BCM1480_MC_MODE(SGRAM_MRVAL) | 
	V_BCM1480_MC_MODE(SGRAM_MRVAL_RESETDLL);

    /*
     * Remember to set the "ganged" bit properly if in 64-bit mode
     * Also, set the page policy here.
     */

    casbits = V_BCM1480_MC_PG_POLICY(K_BCM1480_MC_PG_POLICY_CAS_TIME_CHK);

    WRITECSR(mode,modebits | casbits | mc->modebits);

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_EMRS);
    DRAMINIT_DELAY();

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_MRS);
    DRAMINIT_DELAY();

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_PRE);
    DRAMINIT_DELAY();

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_AR);
    DRAMINIT_DELAY();

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_AR);
    DRAMINIT_DELAY();

    /*
     * This time, clear bit 8 to start the DLL
     */

    modebits = V_BCM1480_MC_EMODE(SGRAM_EMRVAL) | 
	V_BCM1480_MC_MODE(SGRAM_MRVAL);
    WRITECSR(mode,modebits | casbits | mc->modebits);

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_MRS);
    DRAMINIT_DELAY();

}

/*  *********************************************************************
    *  BCM1480_FCRAM_INITCMDS
    *  
    *  Issue the sequence of DRAM init commands.  (FCRAMs)
    *  Note: this routine does not support "big memory" (external decode)
    *  
    *  Input parameters: 
    *      mcnum - memory controller index (0/1/2/3)
    *  	   mc - pointer to data for this memory controller
    *	   csel - which chip select to send init commands for
    *      tdata - chip select to use as a template for timing data
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void bcm1480_fcram_initcmds(int mcnum,mcdata_t *mc,
				   unsigned int csel,csdata_t *tdata)
{
    uint64_t csmask;
    sbport_t cmd;
    sbport_t mode;
    uint64_t modebits;
    uint64_t casbits;

    csmask = csel << S_BCM1480_MC_CS0;		/* convert mask to right place */
    cmd  = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_DRAMCMD));
    mode = (sbport_t) PHYS_TO_K1(A_BCM1480_MC_REGISTER(mcnum,R_BCM1480_MC_DRAMMODE));


    /*
     * For FCRAMs the type must be set first, since much of the
     * init state machine is done in hardware.
     * Already set before. Set again? BV
     */

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_CLRPWRDN);
    DRAMINIT_DELAY();

    WRITECSR(cmd,csmask | K_BCM1480_MC_COMMAND_PRE);
    DRAMINIT_DELAY();

    /*
     * Set up for doing mode register writes to the FCRAMs
     * 
     * mode 0x32 is "sequential 4-byte bursts, CAS Latency 3.0"
     */

    modebits = V_BCM1480_MC_EMODE(FCRAM_EMRVAL) |
	V_BCM1480_MC_MODE(FCRAM_MRVAL);
    /*
     * Remember to set the "ganged" bit properly if in 64-bit mode
     * Page policy is always "closed" for FCRAMs.
     */

    casbits = V_BCM1480_MC_PG_POLICY(K_BCM1480_MC_PG_POLICY_CLOSED);

    WRITECSR(mode,modebits | casbits | mc->modebits);

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_EMRS);
    DRAMINIT_DELAY();

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_MRS);
    DRAMINIT_DELAY();

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_AR);
    DRAMINIT_DELAY();

    WRITECSR(cmd,csmask | V_BCM1480_MC_COMMAND_AR);
    DRAMINIT_DELAY();

    /*
     * Do 4 dummy writes, one to each bank, to get the
     * memory started.
     */

#ifndef _MCSTANDALONE_		/* only on real hardware */
    do {
	volatile uint64_t *ptr;

	ptr = (volatile uint64_t *) PHYS_TO_K1(0);
	*(ptr+(0x00>>3)) = 0;
	*(ptr+(0x20>>3)) = 0;
	*(ptr+(0x40>>3)) = 0;
	*(ptr+(0x60>>3)) = 0;
	} while (0);
#endif

}

/*  *********************************************************************
    *  BCM1480_DRAM_INTLV
    *  
    *  Do row/column/bank initialization for 128-byte interleaved
    *  mode, and also interleave across channels.  You need to have
    *  the same geometry DIMMs installed on memory channels for this 
    *  to work.
    *
    *  Interleaved modes will assign address bits in the following
    *  order:
    *    
    *         <------------Physical Address--------------->
    *  Bits:	RRRRRRR..R  CCCCCCCC..C  SS  BB  PP  CCc00
    * 
    *  Where:
    *     R = Row Address Bit     (MC_CSXX_ROW register)
    *     C = Column Address Bit  (MC_CSXX_COL register)
    *     S = Chip Select         (MC_CONFIG register)    
    *                             (when interleaving via chip selects)
    *     B = Bank Bit            (MC_CSXX_BA register)
    *     P = Channel Select Bit  (MC_GLB_INTLV register)  
    *                             (when interleaving memory channels)
    *	  c = Column Address Bit  (MC_CSXX_COL register for 32-bit channels)
    *		       		  (0 for 64-bit channels)
    *     0 = must be zero
    *
    *  Input parameters: 
    *  	   lots of stuff
    *  	   
    *  Return value:
    *  	   lots of stuff
    ********************************************************************* */

/*  *********************************************************************
    *  BCM1480_DRAM_INTLV_64BIT
    *  
    *  Do row/column/bank initialization for 64-bit channels.
    *  Do channel and CS interleaving if allowed. 
    *
    *  Registers written:
    *	MC_CSXX_BA
    *	MC_CSXX_COL
    *	MC_CSXX_ROW
    *	MC_CS_START
    *	MC_CS_END
    *	MC_CONFIG
    *	MC_GLB_INTLV
    *
    *  Input parameters: 
    *  	   initdata_t structure
    *  	   
    *  Return value:
    *  	   memory controller initialized
    ********************************************************************* */

static void bcm1480_dram_intlv_64bit(initdata_t *d)
{
    int ttlbits;			/* bit counter */
    int rows,columns,banks;
    int mcidx,csidx;
    sbport_t mcbase;
    sbport_t mcbase1;

    uint64_t dimmsize,highmem,highmem4gb;
    
    uint64_t mask;
    int bitnum;
    int chipsels;
    int csintlv0, csintlv1, csintlv2, chanintlv0;
    sbport_t col_row_spacing, bank_spacing, start_end_spacing;

    csintlv0 = csintlv1 = csintlv2 = chanintlv0 = 0;
    highmem = 0x100;
    highmem4gb = 0x200;

    /*
     * Loop through each memory channel and each chip select
     * within each memory controller.
     */

    for (mcidx = MC_FIRSTCHANNEL; mcidx < MC_64BIT_CHANNELS; mcidx++) {
	int csidx_start = (d->mc[mcidx].dramtype == JEDEC_DDR2) ? 2:1;

	if (d->mc[mcidx].flags & MCFLG_NO_ODT_CS) csidx_start = 1;

        int num_csint = csidx_start << d->mc[mcidx].csint;
	uint64_t channel_start = d->ttlbytes;
	uint64_t end_addr;
	uint64_t tmp;
	uint64_t bitmap;

	/* If current channel is not 64-bit, skip it */
	if (d->mc[mcidx].chantype != MC_64BIT_CHAN) continue;

	mcbase = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx));

	/* Chip selects are configured in pairs (0-1, 2-3, 4-5, and 6-7) */

	for (csidx = 0; csidx < MC_64BIT_CHIPSELS; csidx++) {

	
	    /* 
	     * 64-bit channels require "ganging" two 32-bit channels, 
	     * so we have to have the right registers for the given
	     * chip selects.	     
	     */
	    switch (csidx) {
		case 2: case 3:
		    col_row_spacing = 0x80;
		    bank_spacing = 0x20;
		    start_end_spacing = 0;
		    break;
		case 4: case 5:
		    col_row_spacing = bank_spacing = 0x2000;
		    start_end_spacing = 0x2000;
		    break;
		case 6: case 7:
		    col_row_spacing = 0x2080;
		    bank_spacing = 0x2020;
		    start_end_spacing = 0x2000;
		    break;
		default:
		    col_row_spacing = bank_spacing = start_end_spacing = 0;
		}

	    /* 
	     * Ignore this chipsel if we're not using it 
	     */
	    if (!(d->mc[mcidx].csdata[csidx].flags & CS_PRESENT)) continue;

	    /*
	     * Remember we did something to this MC.  We won't bother
	     * activating controllers we don't use.
	     */

	    d->inuse |= (1 << mcidx);

	    /* 
	     * Dig the geometry out of the structure
	     */

	    columns = d->mc[mcidx].csdata[csidx].cols;
	    rows    = d->mc[mcidx].csdata[csidx].rows;
	    banks   = d->mc[mcidx].csdata[csidx].banks;

	    /*
	     * The lowest 3 bits are never set in any mask. 
	     * They represent the byte width of the DIMM.
	     */
	    ttlbits = 3;

	    /* 
	     * For 64-bit channels, bits 3 and 4 are column bits. 
	     * It's not written to the register, but we do need to
	     * keep track of it.
	     */
	    ttlbits += 2;
	    columns -= 2;

	    /*
	     * Channel interleave if allowed. Only 1 bit interleaving
	     * between channel 0 and 1 in 64-bit channel mode.
	     */
	    if ( (d->mc[mcidx].chanintlvint == 1) ) {
		chanintlv0 = ttlbits;
		ttlbits += 1;
		}
			    
	    /*
	     * Now do the bank bits.
	     */
	    banks += ttlbits;
	    bitnum = S_BCM1480_MC_CS23_BANK0;	/* Start with first bank bit */
	    bitmap = 0;
	    while ( ttlbits < banks ) {
		bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		bitnum += K_BCM1480_MC_CSXX_BANKX_BIT_SPACING;
		ttlbits++;
		}
	    WRITECSR((mcbase+R_BCM1480_MC_CS01_BA+bank_spacing), bitmap);
	       	       
	    /*
	     * Get the chip select bits now and write to mc_config register down below.
	     */    
	    if ((d->mc[mcidx].csint > 0) && (csidx < num_csint)) {
		int i = 0;
		chipsels = d->mc[mcidx].csint + ttlbits;
		while (ttlbits < chipsels) {
		    switch (i) {
			case 0: csintlv0 = ttlbits; break;
			case 1: csintlv1 = ttlbits; break;
			case 2: csintlv2 = ttlbits; break;
			}
		    i++;
		    ttlbits++;
		    }
		}

	    /*
	     * Do the rest of the column bits
	     */
	    columns += ttlbits;
	    bitnum = S_BCM1480_MC_COL02;	/* Start with column bit 2, not 0!! */
	    bitmap = 0;
	    while ((ttlbits < columns) && (bitnum <= S_BCM1480_MC_COL07)) {
		bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		bitnum += K_BCM1480_MC_COLX_BIT_SPACING;
		ttlbits++;
		}
	    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_COL0+col_row_spacing),
		     bitmap);

	    bitnum = S_BCM1480_MC_COL08;	/* Start with column bit 8 for reg 1 */
	    bitmap = 0;
	    while ((ttlbits < columns) && (bitnum <= S_BCM1480_MC_COL14)) {		
		/* Skip column bit 10. Used for auto-precharge */
		if (bitnum != S_BCM1480_MC_COL10) {
		    bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		    }
		bitnum += K_BCM1480_MC_COLX_BIT_SPACING;
		ttlbits++;
		}
	    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_COL1+col_row_spacing),
		     bitmap);
	        	       
	    /*
	     * Finally, the row bits
	     */
	    rows += ttlbits;
	    bitnum = S_BCM1480_MC_ROW00;	/* Start with row bit 0 */
	    bitmap = 0;
	    while ((ttlbits < rows) && (bitnum <= S_BCM1480_MC_ROW07)) {
		bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		bitnum += K_BCM1480_MC_ROWX_BIT_SPACING;
		ttlbits++;
		}
	    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_ROW0+col_row_spacing),
		     bitmap);
	    bitnum = S_BCM1480_MC_ROW08; 	/* Start with row bit 8 for reg 1 */
	    bitmap = 0;
	    while ((ttlbits < rows) && (bitnum <= S_BCM1480_MC_ROW14)) {
		bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		bitnum += K_BCM1480_MC_ROWX_BIT_SPACING;
		ttlbits++;
		}
	    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_ROW1+col_row_spacing),
		     bitmap);

	    /*
	     * The total size of this DIMM is 1 << ttlbits.
	     */

	    dimmsize = ((uint64_t) 1) << ttlbits;

	    /*
	     * Program the start and end registers.  The start address is
	     * channel_start if csidx is cs-interleaved; otherwise, the
	     * start address is the current "ttlbytes".
	     *
	     * Separate based on channel interleaving.
	     */	    

	    if ((d->mc[mcidx].chanintlvint == 1) && (mcidx == 0)) { /* Channel interleaving */
		mcbase1 = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx+1));

		if (csidx < num_csint) {
		    mask = READCSR(mcbase+R_BCM1480_MC_CS_START+start_end_spacing);
		    tmp  = (channel_start >> 24);

		    if (d->ttlbytes >= 0xFFFFFFFF) {
			tmp &= ~highmem;
			mask |= ((tmp | highmem4gb) << (16*csidx));
			}
		    else {
			mask |= ((tmp | highmem) << (16*csidx));
			}

		    WRITECSR(mcbase+R_BCM1480_MC_CS_START+start_end_spacing,mask);
		    
		    mask = READCSR(mcbase1+R_BCM1480_MC_CS_START+start_end_spacing);
		    tmp  = (channel_start >> 24);

		    if (d->ttlbytes >= 0xFFFFFFFF) {
			tmp &= ~highmem;
			mask |= ((tmp | highmem4gb) << (16*csidx));
			}
		    else {
			mask |= ((tmp | highmem) << (16*csidx));
			}

		    WRITECSR(mcbase1+R_BCM1480_MC_CS_START+start_end_spacing,mask);
	    
		    d->ttlbytes += dimmsize >> d->mc[mcidx].csint;
		    } 
		else {
		    mask = READCSR(mcbase+R_BCM1480_MC_CS_START+start_end_spacing);
		    tmp  = d->ttlbytes >> 24;
		    mask |= ((tmp | highmem) << (16*csidx));
		    WRITECSR(mcbase+R_BCM1480_MC_CS_START+start_end_spacing,mask);
		    
		    mask = READCSR(mcbase1+R_BCM1480_MC_CS_START+start_end_spacing);
		    tmp  = d->ttlbytes >> 24;

		    if (d->ttlbytes >= 0xFFFFFFFF) {
			tmp &= ~highmem;
			mask |= ((tmp | highmem4gb) << (16*csidx));
			}
		    else {
			mask |= ((tmp | highmem) << (16*csidx));
			}

		    WRITECSR(mcbase1+R_BCM1480_MC_CS_START+start_end_spacing,mask);
	    
		    d->ttlbytes += dimmsize;
		    dimmsize = d->ttlbytes;
		    }

		mask = READCSR(mcbase+R_BCM1480_MC_CS_END+start_end_spacing);
		tmp  = dimmsize >> 24;

		if (d->ttlbytes >= 0xFFFFFFFF) {
		    tmp &= ~highmem;
		    mask |= ((tmp | highmem4gb) << (16*csidx));
		    }
		else {
		    mask |= ((tmp | highmem) << (16*csidx));
		    }

		WRITECSR(mcbase+R_BCM1480_MC_CS_END+start_end_spacing,mask);

		mask = READCSR(mcbase1+R_BCM1480_MC_CS_END+start_end_spacing);
		tmp  = dimmsize >> 24;

		if (d->ttlbytes >= 0xFFFFFFFF) {
		    tmp &= ~highmem;
		    mask |= ((tmp | highmem4gb) << (16*csidx));
		    }
		else {
		    mask |= ((tmp | highmem) << (16*csidx));
		    }

		WRITECSR(mcbase1+R_BCM1480_MC_CS_END+start_end_spacing,mask);
	
		}
	    else if(d->mc[mcidx].chanintlvint == 0) { /* No channel interleaving */		
		if (csidx < num_csint) {
		    mask = READCSR(mcbase+R_BCM1480_MC_CS_START+start_end_spacing);
		    tmp  = (channel_start >> 24);
		    
		    if (d->ttlbytes >= 0xFFFFFFFF) {
			tmp &= ~highmem;
			mask |= ((tmp | highmem4gb) << (16*csidx));
			}
		    else {
			mask |= ((tmp | highmem) << (16*csidx));
			}

		    WRITECSR(mcbase+R_BCM1480_MC_CS_START+start_end_spacing,mask);

		    d->ttlbytes += dimmsize >> d->mc[mcidx].csint;
		    end_addr = channel_start + dimmsize;
		    } 
		else {
		    mask = READCSR(mcbase+R_BCM1480_MC_CS_START+start_end_spacing);
		    tmp  = d->ttlbytes >> 24;

		    if (d->ttlbytes >= 0xFFFFFFFF) {
			tmp &= ~highmem;
			mask |= ((tmp | highmem4gb) << (16*csidx));
			}
		    else {
			mask |= ((tmp | highmem) << (16*csidx));
			}

		    WRITECSR(mcbase+R_BCM1480_MC_CS_START+start_end_spacing,mask);
	    
		    d->ttlbytes += dimmsize;
		    end_addr = d->ttlbytes;
		    }

		mask = READCSR(mcbase+R_BCM1480_MC_CS_END+start_end_spacing);
		tmp  = end_addr >> 24;

		/* 
		 *  QQQ BV Need a 4GB indicator in d (init_data_t).  ttlbytes is not be big enough as 
		 *  unsigned long long.
		 */ 

		if (d->ttlbytes >= 0xFFFFFFFF) {
		    tmp &= ~highmem;
		    mask |= ((tmp | highmem4gb)  << (16*csidx));
		    }
		else {
		    mask |= ((tmp | highmem)  << (16*csidx));
		    }
		WRITECSR(mcbase+R_BCM1480_MC_CS_END+start_end_spacing,mask);
		
		}
    
	    } /* Chip select loop */

	if (d->mc[mcidx].csint > 0) { /* Set memory channel config reg only if able to csintlv */
	    mask = V_BCM1480_MC_INTLV0(csintlv0) | V_BCM1480_MC_INTLV1(csintlv1) |  V_BCM1480_MC_INTLV2(csintlv2);
	    switch (d->mc[mcidx].csint) {
		case 1:
		    if (d->mc[mcidx].dramtype == JEDEC_DDR2)
			mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CS02_MODE);
		    else
			mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CS01_MODE);
		    break;
		case 2:
		    if (d->mc[mcidx].dramtype == JEDEC_DDR2)
			mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CS0246_MODE);
		    else {
			if (d->mc[mcidx].highest_cs_present > 5)
			    mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CS0167_MODE);
			else if (d->mc[mcidx].highest_cs_present > 3)
			    mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CS0145_MODE);
			else
			    mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CS0123_MODE);
			}
		    break;
		case 3:
		    mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CSFULL_MODE);
		    break;
		}
	    WRITECSR(mcbase+R_BCM1480_MC_CONFIG,mask);	   
	    }

	} /* Channel loop */


    /* Need to go back and set end registers if we're interleaving and we've hit the 4GB mark */
    if (d->ttlbytes >= 0xFFFFFFFF) {
	if ((d->mc[0].chanintlvint == 1) && (d->mc[0].csint > 0)) {
	    for (mcidx = MC_FIRSTCHANNEL; mcidx < MC_64BIT_CHANNELS; mcidx++) {
		for (csidx = 0; csidx < MC_64BIT_CHIPSELS; csidx++) {
		    uint64_t end_addr;
		    uint64_t tmp;

		    /* 
		     * Ignore this chipsel if we're not using it 
		     */
		    if (!(d->mc[mcidx].csdata[csidx].flags & CS_PRESENT)) continue;

		    /* 
		     * 64-bit channels require "ganging" two 32-bit channels, 
		     * so we have to have the right registers for the given
		     * chip selects.	     
		     */
		    switch (csidx) {
			case 2: case 3:
			    col_row_spacing = 0x80;
			    bank_spacing = 0x20;
			    start_end_spacing = 0;
			    break;
			case 4: case 5:
			    col_row_spacing = bank_spacing = 0x2000;
			    start_end_spacing = 0x2000;
			    break;
			case 6: case 7:
			    col_row_spacing = 0x2080;
			    bank_spacing = 0x2020;
			    start_end_spacing = 0x2000;
			    break;
			default:
			    col_row_spacing = bank_spacing = start_end_spacing = 0;
			}

		    mcbase = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx));
		    mcbase1 = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx+1));

		    end_addr = d->ttlbytes;
		   
		    mask = READCSR(mcbase+R_BCM1480_MC_CS_END+start_end_spacing);
		    mask ^= (_SB_MAKEMASK(12,(16*csidx)) & mask);

		    tmp  = end_addr >> 24;
		    tmp &= ~highmem;
		    mask |= ((tmp | highmem4gb)  << (16*csidx));
		    WRITECSR(mcbase+R_BCM1480_MC_CS_END+start_end_spacing,mask);
		    } // for cs

		} // for chan
	    } //if both chanintlv and csintlv
	else if (d->mc[0].csint > 0) {
	    for (csidx = 0; csidx < MC_64BIT_CHIPSELS; csidx++) {
		uint64_t end_addr;
		uint64_t tmp;

		/* 
		 * Ignore this chipsel if we're not using it 
		 */
		if (!(d->mc[MC_CHAN1].csdata[csidx].flags & CS_PRESENT)) continue;

		/* 
		 * 64-bit channels require "ganging" two 32-bit channels, 
		 * so we have to have the right registers for the given
		 * chip selects.	     
		 */
		switch (csidx) {
		    case 2: case 3:
			col_row_spacing = 0x80;
			bank_spacing = 0x20;
			start_end_spacing = 0;
			break;
		    case 4: case 5:
			col_row_spacing = bank_spacing = 0x2000;
			start_end_spacing = 0x2000;
			break;
		    case 6: case 7:
			col_row_spacing = 0x2080;
			bank_spacing = 0x2020;
			start_end_spacing = 0x2000;
			break;
		    default:
			col_row_spacing = bank_spacing = start_end_spacing = 0;
		    }

		    mcbase1 = PHYS_TO_K1(A_BCM1480_MC_BASE(MC_CHAN1));

		    end_addr = d->ttlbytes;
		   
		    mask = READCSR(mcbase1+R_BCM1480_MC_CS_END+start_end_spacing);
		    mask ^= (_SB_MAKEMASK(12,(16*csidx)) & mask);

		    tmp  = end_addr >> 24;
		    tmp &= ~highmem;
		    mask |= ((tmp | highmem4gb)  << (16*csidx));
		    WRITECSR(mcbase1+R_BCM1480_MC_CS_END+start_end_spacing,mask);
		} // for cs
	    } //else csintlv only
	}


    /* Global channel intlv register */
    if (chanintlv0) {
	mask = V_BCM1480_MC_INTLV0(chanintlv0) | V_BCM1480_MC_INTLV_MODE(K_BCM1480_MC_INTLV_MODE_01);
	WRITECSR(PHYS_TO_K1(A_BCM1480_MC_GLB_INTLV),mask);
	}
}

/*  *********************************************************************
    *  BCM1480_DRAM_INTLV_32BIT
    *  
    *  Do row/column/bank initialization for 32-bit channels.
    *  
    *  Input parameters: 
    *  	   initdata_t structure, starting channel, ending channel
    *	
    *  	   
    *  Return value:
    *  	   memory controller initialized
    ********************************************************************* */
static void bcm1480_dram_intlv_32bit(initdata_t *d)
{
    int ttlbits;			/* bit counter */
    int rows,columns,banks;
    int mcidx,csidx;
    sbport_t mcbase;
    sbport_t mcbase_idx;
    int chan_idx;

    uint64_t dimmsize;
     
    uint64_t mask;
    int bitnum;
    int chipsels;
    int csintlv0, csintlv1, chanintlv0, chanintlv1;

    csintlv0 = csintlv1 = chanintlv0 = chanintlv1 = 0;

    /*
     * Loop through each memory channel and each chip select
     * within each memory controller.
     */

    for (mcidx = MC_FIRSTCHANNEL; mcidx < MC_32BIT_CHANNELS; mcidx++) {
	int csidx_start = (d->mc[mcidx].dramtype == JEDEC_DDR2) ? 2:1;

	if (d->mc[mcidx].flags & MCFLG_NO_ODT_CS) csidx_start = 1;

        int num_csint = csidx_start << d->mc[mcidx].csint;
	uint64_t channel_start = d->ttlbytes;
	uint64_t end_addr;
	uint64_t tmp;
	uint64_t bitmap;

	/* If current channel is not 32-bit, skip it */
	if (d->mc[mcidx].chantype != MC_32BIT_CHAN) continue;

	mcbase = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx));

	/* Chip selects are configured one at a time */

	for (csidx = 0; csidx < MC_32BIT_CHIPSELS; csidx++) {

	    /* 
	     * Ignore this chipsel if we're not using it 
	     */
	    if (!(d->mc[mcidx].csdata[csidx].flags & CS_PRESENT)) continue;

	    /*
	     * Remember we did something to this MC.  We won't bother
	     * activating channels we don't use.
	     */

	    d->inuse |= (1 << mcidx);

	    /* 
	     * Dig the geometry out of the structure
	     */

	    columns = d->mc[mcidx].csdata[csidx].cols;
	    rows    = d->mc[mcidx].csdata[csidx].rows;
	    banks   = d->mc[mcidx].csdata[csidx].banks;

	    /*
	     * The lowest 2 bits are never set in any mask. 
	     * They represent the byte width of the DIMM.
	     */
	    ttlbits = 2;

	    /* 
	     * For 32-bit channels, bits 2:4 are column bits. 
	     * It's not written to the register, but we do need to
	     * keep track of it.
	     */
	    ttlbits += 3;
	    columns -= 3;

	    /*
	     * Channel interleave if allowed. One or two bits
	     * of channel interleaving. Write to global channel 
	     * intlv register below.
	     */
	    if ( (d->mc[mcidx].chanintlvint == 1) ) { 
		switch (mcidx) {
		    case MC_CHAN0: chanintlv0 = ttlbits; break;
		    case MC_CHAN2: chanintlv1 = ttlbits; break;
		    }
		ttlbits++;
		}
	    else if ( (d->mc[mcidx].chanintlvint == 2) ) {
		chanintlv0 = ttlbits;
		chanintlv1 = ttlbits + 1;
		ttlbits += 2;
		}
			    
	    /*
	     * Now do the bank bits.
	     */
	    banks += ttlbits;
	    bitnum = S_BCM1480_MC_CS23_BANK0;	/* Start with first bank bit */
	    bitmap = 0;
	    while ( ttlbits < banks ) {
		bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		bitnum += K_BCM1480_MC_CSXX_BANKX_BIT_SPACING;
		ttlbits++;
		}
	    switch (csidx) {
		case MC_CS0:
		case MC_CS1:
		    WRITECSR((mcbase+R_BCM1480_MC_CS01_BA), bitmap);
		    break;
		case MC_CS2:
		case MC_CS3:	
		    WRITECSR((mcbase+R_BCM1480_MC_CS23_BA), bitmap);
		    break;
		}

	    /*
	     * Get the chip select bits now and write to mc_config register down below.
	     */    
	    if ((d->mc[mcidx].csint > 0) && (csidx < num_csint)) {
		int i = 0;
		chipsels = d->mc[mcidx].csint + ttlbits;
		while (ttlbits < chipsels) {
		    switch (i) {
			case 0: csintlv0 = ttlbits; break;
			case 1: csintlv1 = ttlbits; break;
			}
		    i++;
		    ttlbits++;
		    }
		}

	    /*
	     * Do the rest of the column bits
	     */
	    columns += ttlbits;
	    bitnum = S_BCM1480_MC_COL03;	/* Start with column bit 3, not 0!! */
	    bitmap = 0;
	    while ((ttlbits < columns) && (bitnum <= S_BCM1480_MC_COL07)) {
		bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		bitnum += K_BCM1480_MC_COLX_BIT_SPACING;
		ttlbits++;
		}
	    switch (csidx) {
		case MC_CS0:
		case MC_CS1:
		    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_COL0),
			     bitmap);
		    break;
		case MC_CS2:
		case MC_CS3:
		    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_COL0+BCM1480_MC_CSX_SPACING),
			     bitmap);		    
		    break;
		}

	    bitnum = S_BCM1480_MC_COL08;	/* Start with column bit 8 for reg 1 */
	    bitmap = 0;
	    while ((ttlbits < columns) && (bitnum <= S_BCM1480_MC_COL14)) {		
		/* Skip column bit 10. Used for auto-precharge */
		if (bitnum != S_BCM1480_MC_COL10) {
		    bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		    }
		bitnum += K_BCM1480_MC_COLX_BIT_SPACING;
		ttlbits++;
		}
	    switch (csidx) {
		case MC_CS0:
		case MC_CS1:
		    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_COL1),
			     bitmap);
		    break;
		case MC_CS2:
		case MC_CS3:
		    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_COL1+BCM1480_MC_CSX_SPACING),
			     bitmap);
		    break;
		}
	        	       
	    /*
	     * Finally, the row bits
	     */
	    rows += ttlbits;
	    bitnum = S_BCM1480_MC_ROW00;	/* Start with row bit 0 */
	    bitmap = 0;
	    while ((ttlbits < rows) && (bitnum <= S_BCM1480_MC_ROW07)) {
		bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		bitnum += K_BCM1480_MC_ROWX_BIT_SPACING;
		ttlbits++;
		}
	    switch (csidx) {
		case MC_CS0:
		case MC_CS1:
		    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_ROW0),
			     bitmap);
		    break;
		case MC_CS2:
		case MC_CS3:
		    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_ROW0+BCM1480_MC_CSX_SPACING),
			     bitmap);		    
		    break;
		}

	    bitnum = S_BCM1480_MC_ROW08; 	/* Start with row bit 8 for reg 1 */
	    bitmap = 0;
	    while ((ttlbits < rows) && (bitnum <= S_BCM1480_MC_ROW14)) {
		bitmap |= _SB_MAKEVALUE(ttlbits,bitnum);
		bitnum += K_BCM1480_MC_ROWX_BIT_SPACING;
		ttlbits++;
		}
	    switch (csidx) {
		case MC_CS0:
		case MC_CS1:
		    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_ROW1),
			     bitmap);
		    break;
		case MC_CS2:
		case MC_CS3:
		    WRITECSR((mcbase+R_BCM1480_MC_CSX_BASE+R_BCM1480_MC_CSX_ROW1+BCM1480_MC_CSX_SPACING),
			     bitmap);
		    break;
		}

	    /*
	     * The total size of this DIMM is 1 << ttlbits.
	     */

	    dimmsize = ((uint64_t) 1) << ttlbits;

	    /*
	     * Program the start and end registers.  The start address is
	     * channel_start if csidx is cs-interleaved; otherwise, the
	     * start address is the current "ttlbytes".
	     */

	    switch (d->mc[mcidx].chanintlvint) {
		case 0: /* No channel intlv */
		    if (csidx < num_csint) {
			mask = READCSR(mcbase+R_BCM1480_MC_CS_START);
			tmp  = (channel_start >> 24);
			mask |= ((tmp | 0x100) << (16*csidx));
			WRITECSR(mcbase+R_BCM1480_MC_CS_START,mask);

			d->ttlbytes += dimmsize >> d->mc[mcidx].csint;
			end_addr = channel_start + dimmsize;
			} 
		    else {
			mask = READCSR(mcbase+R_BCM1480_MC_CS_START);
			tmp  = d->ttlbytes >> 24;
			mask |= ((tmp | 0x100) << (16*csidx));
			WRITECSR(mcbase+R_BCM1480_MC_CS_START,mask);
	    
			d->ttlbytes += dimmsize;
			end_addr = d->ttlbytes;
			}
	
		    mask = READCSR(mcbase+R_BCM1480_MC_CS_END);
		    tmp  = end_addr >> 24;
		    mask |= ((tmp | 0x100) << (16*csidx));
		    WRITECSR(mcbase+R_BCM1480_MC_CS_END,mask);
		    break;
		case 1:	/* Intlv channel 0-1 or 2-3 separately */
		    if ((mcidx == MC_CHAN1) || (mcidx == MC_CHAN3)) break;

		    if ((d->cfg_chanintlv_type == MC_01CHANINTLV) && 
			(mcidx == MC_CHAN2)) break;

		    if ((d->cfg_chanintlv_type == MC_23CHANINTLV) && 
			(mcidx == MC_CHAN0)) break;		    

		    if (csidx < num_csint) {
			for(chan_idx=mcidx; chan_idx <= (mcidx+1); chan_idx++) {
			    mcbase_idx = PHYS_TO_K1(A_BCM1480_MC_BASE(chan_idx));
			    mask = READCSR(mcbase_idx+R_BCM1480_MC_CS_START);
			    tmp  = (channel_start >> 24);
			    mask |= ((tmp | 0x100) << (16*csidx));
			    WRITECSR(mcbase_idx+R_BCM1480_MC_CS_START,mask);
			    }	    
			d->ttlbytes += dimmsize >> d->mc[mcidx].csint;
			if (d->cfg_chanintlv_type == MC_23CHANINTLV)
			    dimmsize = d->ttlbytes;
			} 
		    else {
			for(chan_idx=mcidx; chan_idx <= (mcidx+1); chan_idx++) {
			    mcbase_idx = PHYS_TO_K1(A_BCM1480_MC_BASE(chan_idx));
			    mask = READCSR(mcbase_idx+R_BCM1480_MC_CS_START);
			    tmp  = (d->ttlbytes >> 24);
			    mask |= ((tmp | 0x100) << (16*csidx));
			    WRITECSR(mcbase_idx+R_BCM1480_MC_CS_START,mask);
			    }	    
			d->ttlbytes += dimmsize;
			dimmsize = d->ttlbytes;
			}

		    for(chan_idx=mcidx; chan_idx <= (mcidx+1); chan_idx++) {
			mcbase_idx = PHYS_TO_K1(A_BCM1480_MC_BASE(chan_idx));
			mask = READCSR(mcbase_idx+R_BCM1480_MC_CS_END);
			tmp  = (dimmsize >> 24);
			mask |= ((tmp | 0x100) << (16*csidx));
			WRITECSR(mcbase_idx+R_BCM1480_MC_CS_END,mask);
			}
	    
		    break;
		case 2: /* Full (4) channel interleaving */
		    if (mcidx != MC_CHAN0) break;

		    if (csidx < num_csint) {
			for(chan_idx=MC_CHAN0; chan_idx <= MC_CHAN3; chan_idx++) {
			    mcbase_idx = PHYS_TO_K1(A_BCM1480_MC_BASE(chan_idx));
			    mask = READCSR(mcbase_idx+R_BCM1480_MC_CS_START);
			    tmp  = (channel_start >> 24);
			    mask |= ((tmp | 0x100) << (16*csidx));
			    WRITECSR(mcbase_idx+R_BCM1480_MC_CS_START,mask);
			    }	    
			d->ttlbytes += dimmsize >> d->mc[mcidx].csint;
			} 
		    else {
			for(chan_idx=MC_CHAN0; chan_idx <= MC_CHAN3; chan_idx++) {
			    mcbase_idx = PHYS_TO_K1(A_BCM1480_MC_BASE(chan_idx));
			    mask = READCSR(mcbase_idx+R_BCM1480_MC_CS_START);
			    tmp  = (d->ttlbytes >> 24);
			    mask |= ((tmp | 0x100) << (16*csidx));
			    WRITECSR(mcbase_idx+R_BCM1480_MC_CS_START,mask);
			    }	    
			d->ttlbytes += dimmsize;
			dimmsize = d->ttlbytes;
			}

		    for(chan_idx=MC_CHAN0; chan_idx <= MC_CHAN3; chan_idx++) {
			mcbase_idx = PHYS_TO_K1(A_BCM1480_MC_BASE(chan_idx));
			mask = READCSR(mcbase_idx+R_BCM1480_MC_CS_END);
			tmp  = (dimmsize >> 24);
			mask |= ((tmp | 0x100) << (16*csidx));
			WRITECSR(mcbase_idx+R_BCM1480_MC_CS_END,mask);
			}

		    break;
		} /* Channel intlv switch */

	    } /* Chip select loop */

	if (d->mc[mcidx].csint > 0) { /* Set memory channel config reg only if able to csintlv */
	    mask = V_BCM1480_MC_INTLV0(csintlv0) | V_BCM1480_MC_INTLV1(csintlv1);
	    switch (d->mc[mcidx].csint) {
		case 1:
		    if (d->mc[mcidx].dramtype == JEDEC_DDR2)
			mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CS02_MODE);
		    else
			mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CS01_MODE);			
		    break;
		case 2:
		    mask |= V_BCM1480_MC_CS_MODE(K_BCM1480_MC_CS0123_MODE);
		    break;
		}
	    WRITECSR(mcbase+R_BCM1480_MC_CONFIG,mask);
	    }

	} /* Channel loop */

    /* Global channel intlv register */
    mask = 0;
    if (chanintlv0 && chanintlv1 && (d->cfg_chanintlv_type == MC_FULLCHANINTLV))
	/* Intlv all four channels using 2 bits */
	mask = V_BCM1480_MC_INTLV0(chanintlv0) | V_BCM1480_MC_INTLV1(chanintlv1) | V_BCM1480_MC_INTLV_MODE_0123;	
    else if (chanintlv0 && chanintlv1 && (d->cfg_chanintlv_type == MC_01_23CHANINTLV))
	/* Intlv 0-1 and 2-3 separately using 1 bit each */
	mask = V_BCM1480_MC_INTLV0(chanintlv0) | V_BCM1480_MC_INTLV1(chanintlv1) | V_BCM1480_MC_INTLV_MODE_01_23;
    else if (chanintlv0 && (d->cfg_chanintlv_type == MC_01CHANINTLV))
	/* Intlv chan 0-1 using 1 bit */
	mask = V_BCM1480_MC_INTLV0(chanintlv0) | V_BCM1480_MC_INTLV_MODE_01;
    else if (chanintlv1 && (d->cfg_chanintlv_type == MC_23CHANINTLV))
	/* Intlv chan 2-3 using 1 bit */
	mask = V_BCM1480_MC_INTLV1(chanintlv1) | V_BCM1480_MC_INTLV_MODE_23;

    if (mask > 0)
	WRITECSR(PHYS_TO_K1(A_BCM1480_MC_GLB_INTLV),mask);

}

/*  *********************************************************************
    *  BCM1480_DRAM_ANALYZE(d)
    *  
    *  Analyze the DRAM parameters, determine if we can do 
    *  channel and/or chip select interleaving.
    *  
    *  Input parameters: 
    *  	   d - init data
    *  	   
    *  Return value:
    *  	   nothing (fields in initdata are updated)
    ********************************************************************* */

static void bcm1480_dram_analyze(initdata_t *d)
{
    int csidx;
    int num_cs;
    int mcidx;

    /*
     * Clear chanintlv and csintlv bits before we start.
     */
    for (mcidx = MC_FIRSTCHANNEL; mcidx < MC_MAX_CHANNELS; mcidx++) {
	d->mc[mcidx].chanintlvint = 0;
	d->mc[mcidx].csint = 0;
	}

    mcidx = MC_FIRSTCHANNEL;
    while (mcidx < MC_MAX_CHANNELS) {

	/* For 64-bit channels, no need to check channel 2-3 */
	if ( (d->mc[mcidx].chantype == MC_64BIT_CHAN) && (mcidx > MC_CHAN1) ) break;

	/*
	 * Determine if we can do channel interleaving.  This is possible if
	 * the DIMMs on each channel are the same.
	 *
	 * Start with no channel interleaving.
	 */
	if ( (d->cfg_chanintlv_type > MC_NOCHANINTLV) && (mcidx < MC_CHAN3) ) {	     
	    for (csidx = 0; csidx < MC_MAX_CHIPSELS; csidx++) {
		if (d->mc[mcidx].csdata[csidx].rows != d->mc[mcidx+1].csdata[csidx].rows) break;
		if (d->mc[mcidx].csdata[csidx].cols != d->mc[mcidx+1].csdata[csidx].cols) break;
		if (d->mc[mcidx].csdata[csidx].banks != d->mc[mcidx+1].csdata[csidx].banks) break;
		if (d->mc[mcidx].csdata[csidx].flags != d->mc[mcidx+1].csdata[csidx].flags) break;
		}
	    if (d->mc[mcidx].flags != d->mc[mcidx+1].flags) csidx = 0;	       
	    if (csidx == MC_MAX_CHIPSELS) {
		d->mc[mcidx].chanintlvint = 1;
		d->mc[mcidx+1].chanintlvint = 1; /* Set matching channel as well */
		}	
	    }
	    
	/*
	 * Determine how many CS interleave bits (0, 1, 2, or 3) will work.
	 * Memory channels are checked separately.  If port (i.e., channel)
	 * interleaving is allowed, each channel will end up with the same number
	 * of CS interleave bits.
	 * Note: CS 0 must always be present. No mixed-
	 */

        /* Forbid CS interleaving if not requested, in large mem mode or CS 0 is absent */
	if ( (d->mc[mcidx].cfgcsint > NOCSINTLV) &&
	     !(d->mc[mcidx].flags & MCFLG_BIGMEM) &&
	     (d->mc[mcidx].csdata[0].flags & CS_PRESENT) ) {
	    num_cs = 0; /* Number of CS's that matches CS0 */
	    csidx = (d->mc[mcidx].dramtype == JEDEC_DDR2) ? 2 : 1;
	    while (csidx < MC_MAX_CHIPSELS) {
		/* CS csidx must be present and attr matches */
		if ( (d->mc[mcidx].csdata[csidx].flags & CS_PRESENT) &&
		     (d->mc[mcidx].csdata[0].rows == d->mc[mcidx].csdata[csidx].rows) &&
		     (d->mc[mcidx].csdata[0].cols == d->mc[mcidx].csdata[csidx].cols) &&
		     (d->mc[mcidx].csdata[0].banks == d->mc[mcidx].csdata[csidx].banks) ) {
		    num_cs++;
		    if( csidx > d->mc[mcidx].highest_cs_present )
			d->mc[mcidx].highest_cs_present = csidx;
		    }
		csidx = (d->mc[mcidx].dramtype == JEDEC_DDR2) ? csidx+2 : csidx+1;
		}
	    switch (num_cs) {
		case 1: 
		    d->mc[mcidx].csint = 1; /* Use 1 bit for CS interleaving */
		    break;
		case 3:  
		    d->mc[mcidx].csint = 2; /* Use 2 bits */
		    break;
		case 7: 
		    d->mc[mcidx].csint = 3; /* Use 3 bits. 64-bit channels only */
		    break;
		default: d->mc[mcidx].csint = 0; /* All others, no CS interleaving */
		}

#ifdef _MCSTANDALONE_NOISY_
	    if (d->mc[mcidx].csint > NOCSINTLV)
		printf("DRAM: CS interleaving possible. Channel: %d Number of CS bits: %d.\n",
		       mcidx,d->mc[mcidx].csint);
#endif
	    }

	mcidx++;
	}

    /* Check for full channel interleave. */
    if ((d->cfg_chanintlv_type == MC_FULLCHANINTLV) && 
	(d->mc[MC_CHAN0].chanintlvint == 1) &&
	(d->mc[MC_CHAN1].chanintlvint == 1) &&
	(d->mc[MC_CHAN2].chanintlvint == 1) ) {

	for (mcidx = 0; mcidx < MC_MAX_CHANNELS; mcidx++) {
	    d->mc[mcidx].chanintlvint = 2;
	    }

	}
    
    /* Clear if we don't request */
    if (d->cfg_chanintlv_type == MC_01CHANINTLV) {
	d->mc[MC_CHAN2].chanintlvint = 0;
	d->mc[MC_CHAN3].chanintlvint = 0;
	}
    else if (d->cfg_chanintlv_type == MC_23CHANINTLV) {
	d->mc[MC_CHAN0].chanintlvint = 0;
	d->mc[MC_CHAN1].chanintlvint = 0;
	}
}

#if !defined(_MCSTANDALONE_)		/* When not linked into firmware, no RAM zeroing */

/*  *********************************************************************
    *  BCM1480_DRAM_ZERO1MB(d,addr)
    *  
    *  Zero one megabyte of memory starting at the specified address.
    *  'addr' is in megabytes.
    *  
    *  Input parameters: 
    *  	   d - initdata structure
    *  	   addr - starting address, expressed as a megabyte index
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

#ifndef _DMZERO_
static void bcm1480_dram_zero1mb(initdata_t *d,uint64_t addr)
{
    /*
     * We **MUST** align the stores to a cache-line boundary.  If they
     * get split over multiple cache lines, the first miss (to get the
     * *rest* of the stores into the icache) will cause a long enough
     * delay for the UAC write (not yet to the complete line) to be
     * flushed... which means it will hit the MC as a partial line,
     * causing a RMW, defeating the notion of writing all of memory to
     * clear ECC.
     */
    __asm(" .set push ; .set noreorder ; .set mips64 ; "
	  "  mfc0 $9,$12 ; "
	  "  ori  $8,$9,0x80 ; "
	  "  mtc0 $8,$12 ; "
	  "  bnel $0,$0,.+4 ; "
	  "  ssnop ; "
	  "  lui  $10,0xB800 ; "
	  "  dsll32 $10,$10,0 ; "
	  "  dsll   $12,%0,20 ;"
	  "  or     $10,$10,$12 ; "
	  "  lui    $11,0x10 ; "
	  "  .align 5 ; "
	  "1: "
	  "  sd     $0,0($10) ; " 
	  "  sd     $0,8($10) ; " 
	  "  sd     $0,16($10) ; " 
	  "  sd     $0,24($10) ; " 
	  "  sd     $0,32($10) ; " 
	  "  sd     $0,40($10) ; " 
	  "  sd     $0,48($10) ; " 
	  "  sd     $0,56($10) ; "
	  "  sub    $11,$11,64 ; "
	  "  bne    $11,$0,1b ;  "
	  "  dadd   $10,64 ; "
          "  mtc0   $9,$12 ; "
          "  bnel   $0,$0,.+4 ;"
	  "  ssnop ; "
	  " .set pop"
	  : : "r"(addr) : "$8","$9","$10","$11","$12");
}
#endif

/*  *********************************************************************
    *  BCM1480_DRAM_ZERO1MB(d,addr)
    *  
    *  Zero one megabyte of memory starting at the specified address.
    *  'addr' is in megabytes.
    *  
    *  Input parameters: 
    *  	   d - initdata structure
    *  	   addr - starting address, expressed as a megabyte index
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

#ifdef _DMZERO_
static void bcm1480_dram_zero1mb(initdata_t *d,uint64_t addr)
{
    sbport_t dmreg;
    uint64_t baseaddr;
    volatile int idx;

    /*
     * Build the descriptor
     */
    
    d->dscr[0] = (addr << 20) |
	M_DM_DSCRA_ZERO_MEM | 
	M_DM_DSCRA_UN_DEST | M_DM_DSCRA_UN_SRC |
	V_DM_DSCRA_DIR_SRC_CONST |
	V_DM_DSCRA_DIR_DEST_INCR;
    d->dscr[1] = V_DM_DSCRB_SRC_LENGTH(0);

    /* Flush the descriptor out.  We need to do this in Pass1
       because we're in cacheable noncoherent mode right now and
       the core will not respond to the DM's request for the descriptor. */

    __asm __volatile ("cache 0x15,0(%0) ; " :: "r"(d));

    /*
     * Give the descriptor to the data mover 
     */

    dmreg = PHYS_TO_K1(A_DM_REGISTER(0,R_DM_DSCR_BASE));
    baseaddr = (uint64_t) K0_TO_PHYS((long)d->dscr) | 
	V_DM_DSCR_BASE_PRIORITY(0) |
	V_DM_DSCR_BASE_RINGSZ(4) |
	M_DM_DSCR_BASE_ENABL |
	M_DM_DSCR_BASE_RESET;
    WRITECSR(dmreg,baseaddr);

    dmreg = PHYS_TO_K1(A_DM_REGISTER(0,R_DM_DSCR_COUNT));
    WRITECSR(dmreg,1);

    /*
     * Wait for the request to complete
     */

    while ((READCSR(dmreg) & 0xFFFF) > 0) {
	/* Do something that doesn't involve the ZBBus to give
	   the DM some extra time */
	for (idx = 0; idx < 10000; idx++) ; /* NULL LOOP */
	}

}
#endif


/*  *********************************************************************
    *  BCM1480_DRAM_ZERO(d)
    *  
    *  Zero memory, using the data mover.
    *  
    *  Input parameters: 
    *  	   d - initdata structure
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
static void bcm1480_dram_zero(initdata_t *d)
{
#if 0
    /* This is just for debugging */
    bcm1480_dram_zero1mb(d,1);
#else
    int idx;
    int maxmem;
    uint64_t curmb;			/* current address in megabytes */

    maxmem = (int) (d->ttlbytes >> 20);
    curmb = 0;

    for (idx = 0; idx < (int) maxmem; idx++) {
	bcm1480_dram_zero1mb(d,curmb);
	bcm1480_dram_zero1mb(d,curmb);	/* again just to make sure */
	curmb++;
	if (curmb == (REGION0_LOC+REGION0_SIZE))      curmb = REGION1_LOC;
	else if (curmb == (REGION1_LOC+REGION1_SIZE)) curmb = REGION2_LOC;
	else if (curmb == (REGION2_LOC+REGION2_SIZE)) curmb = REGION3_LOC;
	}
#endif
}

#endif /* !defined(_MCSTANDALONE_) */

/*  *********************************************************************
    *  BCM1480_DRAM_INIT()
    *  
    *  Initialize DRAM connected to the specified DRAM controller
    *  The DRAM will be configured without interleaving, as sequential
    *  blocks of memory.
    *  
    *  Input parameters: 
    *  	   a0 - zero to use default memconfig table
    *           or KSEG1 address of mem config table
    *  	   
    *  Return value:
    *  	   v0 - total amount of installed DRAM
    *  
    *  Registers used:
    *  	   all
    ********************************************************************* */

uint64_t bcm1480_dram_init_real(const draminittab_t *init,initdata_t *d);
uint64_t bcm1480_dram_init_real(const draminittab_t *init,initdata_t *d)
{
    uint64_t reg;
    sbport_t mcbase;
    sbport_t mgcfg;
    uint64_t gcfgbits;
    uint64_t dmbits;
    uint64_t sysrev;

    int mcidx;
    int csidx;
    unsigned int csmask, lastcs;
    int dramtype;
    csdata_t *tdata;
    mcdata_t *mc;

#if defined(_MCSTANDALONE_)
    /*
     * If compiled standalone (e.g., as the memconfig utility), we have
     * to zero the initdata structure.  On hardware, this is handled
     * by the initdata + stack init code.
     */
    memset (d, 0, sizeof (initdata_t));
#endif

#if !defined(_MCSTANDALONE_)
    if (!init) {
	return 0; /* No default tables available */
	}
#endif

    /*
     * Begin by initializing the memory channels to some known state.
     * Set the "BERR_DISABLE" bit for now while we initialize the channels,
     * this will be cleared again before the routine exits.
     * Set the "ECC_DISABLE" bit for all channels.
     */

#ifdef _MCSTANDALONE_NOISY_
    printf("DRAM: Initializing memory controller.\n");
#endif

    sysrev = READCSR(PHYS_TO_K1(A_SCD_SYSTEM_REVISION));
    mgcfg = PHYS_TO_K1(A_BCM1480_MC_GLB_CONFIG);
    gcfgbits = READCSR(mgcfg);
    gcfgbits |= M_BCM1480_MC_BERR_DISABLE;
    WRITECSR(mgcfg, gcfgbits);

    for (mcidx = MC_FIRSTCHANNEL; mcidx < MC_MAX_CHANNELS; mcidx++) {
	mcbase = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx));

	WRITECSR(mcbase+R_BCM1480_MC_CONFIG,0);
	WRITECSR(mcbase+R_BCM1480_MC_CS_START,0);
	WRITECSR(mcbase+R_BCM1480_MC_CS_END,0);
	WRITECSR(mcbase+R_BCM1480_MC_DRAMMODE,V_BCM1480_MC_DRAMMODE_DEFAULT | M_BCM1480_MC_GANGED | M_BCM1480_MC_ECC_DISABLE );
	WRITECSR(mcbase+R_BCM1480_MC_TEST_DATA,0);
	WRITECSR(mcbase+R_BCM1480_MC_TEST_ECC,0);
	}

    /*
     * Read the parameters
     */
    bcm1480_dram_readparams(d,init);

    /*
     * Analyze parameters 
     */
    bcm1480_dram_analyze(d);

    /*
     * Configure chip selects.  64-bit first, then 32-bit. Start with zero memory.
     */
    d->ttlbytes = 0;
    bcm1480_dram_intlv_64bit(d);      
    bcm1480_dram_intlv_32bit(d);

    /*
     * Okay, initialize the DRAM controller(s)
     */

    for (mcidx = MC_FIRSTCHANNEL; mcidx < MC_MAX_CHANNELS; mcidx++) {

#ifdef _MCSTANDALONE_NOISY_
	printf("DRAM: initializing channel %d: %s\n",
	       mcidx, (d->inuse & (1 << mcidx)) ? "in use" : "not in use");
#endif
	/*
	 * Skip this controller if we did nothing
	 */
	if (!(d->inuse & (1 << mcidx))) continue;

	/*
	 * Get the base address of the controller
	 */
	mcbase = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx));

	/* 
	 * Get our MC data 
	 */

	mc = &(d->mc[mcidx]);

	/*
	 * Program the clock config register.  This starts the clock to the
	 * SDRAMs.  Need to wait 200us after doing this. (6.4.6.1)
	 *
	 * Find the slowest chip/dimm among the chip selects on this 
	 * controller and use that for computing the timing values.
	 */ 

	csidx = bcm1480_find_timingcs(mc);
	if (csidx < 0) continue;		/* should not happen */

	tdata = &(d->mc[mcidx].csdata[csidx]);	/* remember for use below */

	if (mc->mantiming) {
	    bcm1480_manual_timing(mcidx,mc);
	    }
	else {
	    bcm1480_auto_timing(mcidx,mc,tdata);
	    }

	DRAMINIT_DELAY();

	dmbits = 0;

	if (d->mc[mcidx].chantype == MC_64BIT_CHAN) {
	    dmbits |= M_BCM1480_MC_GANGED; 
	    }

	dmbits |= V_BCM1480_MC_PG_POLICY(mc->pagepolicy);

	if ((mc->flags & MCFLG_2T) && (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0)) {
	    dmbits |= M_BCM1480_MC_2T_CMD;
	    }

	dramtype = d->mc[mcidx].dramtype;
	switch (dramtype) {
	    case FCRAM:
		dmbits |= V_BCM1480_MC_DRAM_TYPE(K_BCM1480_MC_DRAM_TYPE_FCRAM);
		break;

	    case JEDEC_DDR2:
		if (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0)
		    dmbits |= V_BCM1480_MC_DRAM_TYPE(K_BCM1480_MC_DRAM_TYPE_DDR2);
		else

		    dmbits |= V_BCM1480_MC_DRAM_TYPE(K_BCM1480_MC_DRAM_TYPE_DDR2_PASS1);		

		break;

	    default:
		dmbits |= V_BCM1480_MC_DRAM_TYPE(K_BCM1480_MC_DRAM_TYPE_JEDEC);
		break;
	    }

	d->mc[mcidx].modebits = dmbits;

	WRITECSR(mcbase+R_BCM1480_MC_DRAMMODE,dmbits);

        /*
	 * Okay, now do the following sequence:
	 * PRE-EMRS-MRS-PRE-AR-AR-MRS.   Figure out which
	 * chip selects we are using, then do this for all enabled
	 * chip selects.  Repeat the sequence for each channel.
	 */
	csmask = 0;
	lastcs = 0;
	for (csidx = 0; csidx < MC_MAX_CHIPSELS; csidx++) {
	    if (mc->csdata[csidx].flags & CS_PRESENT) {
#ifdef _MCSTANDALONE_NOISY_
		printf("DRAM: ..initializing CS%d\n",csidx);
#endif
		csmask |= (1 << csidx);

		/* Last available CS with DRAM present.  Used to set DRAM ODT. */
		lastcs = (1<< csidx);
		}
#ifdef _MCSTANDALONE_NOISY_
	    else printf("DRAM: ..CS%d not being used\n",csidx);
#endif
	    } /* for chipsels loop */

	if (csmask != 0) { 
	    switch (dramtype) {
		case JEDEC:
		    bcm1480_jedec_initcmds(mcidx,mc,csmask,0,tdata);
		    break;
		case JEDEC_DDR2:

		    /* SSTL_18 for DDR2 */
		    mgcfg = PHYS_TO_K1(A_BCM1480_MC_GLB_CONFIG);
		    gcfgbits = READCSR(mgcfg);
		    gcfgbits &= ~M_BCM1480_MC_SSTL_VOLTAGE;
		    WRITECSR(mgcfg, gcfgbits);

		    bcm1480_jedec_ddr2_initcmds(mcidx,mc,csmask,lastcs,0,tdata,d);
		    break;
		case SGRAM:
		    bcm1480_sgram_initcmds(mcidx,mc,csmask,tdata);
		    break;
		case FCRAM:
		    bcm1480_fcram_initcmds(mcidx,mc,csmask,tdata);
		    break;
		default:
#ifdef _MCSTANDALONE_NOISY_
		    printf("DRAM: Channel DRAM type declared as DRAM_TYPE_SPD, but no SPD DRAM type found.\n");
#endif
		    break;
		}
	    }

#ifdef _BCM1480_PASS2_WORKAROUNDS_

	/*
	 * Clear cs_odd_odt_en bit if it's a registered DDR2 DIMM
	 * SOC-199
	 */
	uint64_t odtcfgbits;
	
	if (G_SYS_REVISION(sysrev) >= K_SYS_REVISION_BCM1480_B0) {
	    
	    if ((tdata->spd_dimmtype & JEDEC_DIMMTYPE_RDIMM) ||
	        (tdata->spd_dimmtype & JEDEC_DIMMTYPE_MRDIMM) ||
		(mc->flags & MCFLG_FORCEREG)) { 
	
		odtcfgbits = READCSR(mcbase+R_BCM1480_MC_ODT);
		odtcfgbits &= ~M_BCM1480_MC_CS_ODD_ODT_EN;
		WRITECSR(mcbase+R_BCM1480_MC_ODT,odtcfgbits);
		}
	    }

#endif

	} /* for channel loop */

#if !defined(_MCSTANDALONE_)
    /*
     * Zero the contents of memory to set the ECC bits correctly.
     * Do it for all memory if either channel is enabled for ECC.
     */

    for (mcidx = MC_FIRSTCHANNEL; mcidx < MC_MAX_CHANNELS; mcidx++) {
	if (!(d->inuse & (1 << mcidx))) continue;
	if (d->mc[mcidx].flags & MCFLG_ECC_ENABLE) {	 /* XXX always zero DRAM to make good ECC */
	    bcm1480_dram_zero(d);
	    break;
	    }
	}
    WRITECSR(PHYS_TO_K1(A_BUS_L2_ERRORS),0);
    WRITECSR(PHYS_TO_K1(A_BUS_MEM_IO_ERRORS),0);
#endif
    /*
     * Kill the BERR_DISABLE bit
     */
    mgcfg = PHYS_TO_K1(A_BCM1480_MC_GLB_CONFIG);
    gcfgbits = READCSR(mgcfg);
    gcfgbits &= ~M_BCM1480_MC_BERR_DISABLE;
    WRITECSR(mgcfg, gcfgbits);


    /*
     * Turn on the ECC in the memory controller for those channels
     * that we've specified.
     */
    for (mcidx = MC_FIRSTCHANNEL; mcidx < MC_MAX_CHANNELS; mcidx++) {
	if (!(d->inuse & (1 << mcidx))) continue;
	if (!(d->mc[mcidx].flags & MCFLG_ECC_ENABLE)) {		/* ecc not enabled */
	    mcbase = PHYS_TO_K1(A_BCM1480_MC_BASE(mcidx));
	    reg = READCSR(mcbase+R_BCM1480_MC_DRAMMODE);
	    reg |= M_BCM1480_MC_ECC_DISABLE;
	    WRITECSR(mcbase+R_BCM1480_MC_DRAMMODE,reg);
	    }
	}

    /*
     * Return the total amount of memory initialized, in megabytes
     */

#ifdef _MCSTANDALONE_NOISY_
    printf("DRAM: Total memory: %dMB.\n",(unsigned int)(d->ttlbytes >> 20));
#endif

    return (d->ttlbytes >> 20);
}


/*  *********************************************************************
    *  XXBCM1480_DRAMINIT()
    *  
    *  This is a hideous hack.  To help keep things all in one module,
    *  and to aid in relocation (remember, it's tough to do a 
    *  PC-relative branch to an external symbol), here is an 
    *  assembly stub to get things ready to call the above C routine.
    *  We turn off the bus errors on both memory controllers, set up
    *  a small stack, and branch to the C routine to handle the rest.
    *  
    *  Input parameters: 
    *  	   register a0 - user initialization table
    *  	   
    *  Return value:
    *  	   register v0 - size of memory, in bytes
    ********************************************************************* */

#if !defined(_MCSTANDALONE_)
void xxbcm1480_draminit(const draminittab_t *init);
void xxbcm1480_draminit(const draminittab_t *init)
{
    /* Work area must fit into one cache way.  */
    if (WORK_AREA_SIZE > 8192)
	__asm __volatile ("ERROR WORK AREA TOO LARGE");

    __asm __volatile(" .globl bcm1480_dram_init ; "
	  "bcm1480_dram_init: ; "
	  " dli $14,0x0000000F2041AA00 ; "	/* Set BERR_DISABLE bit */
	  " lui $8,0xb005 ; "			/* in mc_glb_config register */
	  " sd	$14, 0x4100($8); "
	  " dli $13,0x0000019000000000 ; "	/* Set ECC_DISABLE bits */
	  " lui $9,0xb005 ; "			/* in all mc_drammode_ registers */
	  " sd	$13, 0x0420($9) ; "
	  " sd	$13, 0x1420($9) ; "
	  " sd	$13, 0x2420($9) ; "
	  " sd	$13, 0x3420($9) ; "	  
	  " sd  $0, 0x0120($9) ; "		/* Clear the CS_START/CS_END */
	  " sd  $0, 0x0140($9) ; "		/* registers so we won't really */
	  " sd  $0, 0x1120($9) ; "		/* try memory accesses till */
	  " sd  $0, 0x1140($9) ; "		/* memory is activated */

          /*
           * Make sure the writes have actually hit the memory
           * controller registers before we proceed.
           */
          " ld  $0, 0x1140($9) ; "
          " sync ; "

          /*
           * Calculate an area to use for the work area (initdata and
           * stasck).  For Verilog/BOOTRAM code, this is in the ROM
           * area (i.e. just below the KSEG1 boundary).  For normal
           * code, this goes at the bottom of DRAM.
           */
#if (defined( _VERILOG_) || CFG_BOOTRAM)
          " li $29,0xa0000000 ; "
          " daddiu $5, $29, -(%0) ; "
#else
          " li $5,0x80000000 ; "
          " daddiu $29, $5, %0 ; "
#endif

          /*
           * Zero all of the memory in the work area *right now*,
           * so that it's all in the L1 cache.  Otherwise, stack
           * usage may cause it to be requested in the time
           * between when the CS_START/_END registers are written
           * (in bcm1480_dram_intlv_64bit / bcm1480_dram_intlv_32bit)
           * and when the DRAM is actually enabled (later in
           * bcm1480_dram_init_real).
           *
           * From above:
           * $5   Base of work area.
           * $29  End of work area (SP).
           * $8   Scratch register.
           */
          " move   $8, $5 ; "
          "1: "
          " sd     $0,  0($8) ; "
          " sd     $0,  8($8) ; "
          " sd     $0, 16($8) ; "
          " sd     $0, 24($8) ; "
          " daddiu $8, $8, 0x20 ; "
          " bne    $8, $29, 1b ; "

          /*
           * Leave space on the stack for argument register stack slots
           * required by the o32 and o64 ABIs for calls.
           */
          " daddiu $29, $29, -32 ; "

#if CFG_RELOC
	  " la $25,bcm1480_dram_init_real ; "	/* SVR4 PIC linkage */
#endif
	  " b bcm1480_dram_init_real ; "	/* Branch to real init routine */
	  : : "I"(WORK_AREA_SIZE));

}

#else	/* _MCSTANDALONE_ */

/* 
 * SOCVIEW and non-CFE, non-MIPS things don't need any magic since they
 * are not running on the 1400.  Just call the main routine.
 */
uint64_t bcm1480_dram_init(const draminittab_t *init,initdata_t *d);
uint64_t bcm1480_dram_init(const draminittab_t *init,initdata_t *d)
{
    initdata_t initdata;
    return bcm1480_dram_init_real(init,&initdata);
}
#endif


/*  *********************************************************************
    *  End  (yes, 3000 lines of memory controller init code.  Sheesh!)
    ********************************************************************* */
