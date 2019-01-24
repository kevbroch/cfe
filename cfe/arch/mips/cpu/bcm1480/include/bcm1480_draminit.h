/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  DRAM Initialization routine definitions	File: bcm1480_draminit.h
    *  
    *  This file contains constants and data structures specific to
    *  the operation of bcm1480_draminit.c
    *  
    *  Author:  Mitch Lichtenberg
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

#ifndef _BCM1480_MC_H
#include "bcm1480_mc.h"			/* memory controller constants */
#endif


/*  *********************************************************************
    *  DRAMINFO macros - used to construct the DRAM information
    *  table.  There are two flavors, for "C" and assembly.
    ********************************************************************* */


/*
 * This macro generates a "decimals and tenths" hex value like those used
 * in an SPD.  'x' is the decimal portion and 't' is the tenths.  It's
 * sort of like a BCD value except 'x' can contain 0..15
 *
 * This macro needs to be used for the tCK parameter.
 *
 * For example, to specify 7.5, you could say DRT10(7,5)
 */

#define DRT10(x,t) (((x)<<4)|(t))

/*
 * This macro generates "decimals and quarters" like those used
 * in an SPD.  The 'q' parameter should be 0, 25, 50, or 75
 * This macro needs to be used by tRP, tRRD, rRCD, tRFC, adn tRC
 * 
 * For example, to specify 20.25, you could say DRT4(20,25)
 */

#define DRT4(x,q) (((x)<<2)|((q)/25))



#ifdef __ASSEMBLER__
#define DRAM_GLOBALS(chintlv) \
          .byte MCR_GLOBALS,chintlv,0,0,0,0,0,0,0,0,0,0 ;
#define DRAM_CHAN_CFG(chan,tMEMCLK,tROUNDTRIP,chantype,dramtype,pagepolicy,csintlv,ecc,flg) \
          .byte MCR_CHCFG,chan,tMEMCLK,tROUNDTRIP,chantype,dramtype,pagepolicy,csintlv,ecc,flg,0,0 ;
#define DRAM_CHAN_DLLCFG(addrfine,dqicoarse,dqifine,dqocoarse,dqofine) \
          .byte MCR_DLLCFG,addrfine,dqicoarse,dqifine,dqocoarse,dqofine,0,0,0,0,0,0 ;
#define DRAM_CHAN_DLLCFG2(addrfreq,dqicoarse,dqifreq,dqocoarse,dqofreq,dlldefault,dllfreq,dllbypass) \
          .byte MCR_DLLCFG,addrfreq,dqicoarse,dqifreq,dqocoarse,dqofreq,dlldefault,dllfreq,dllbypass,0,0,0 ;
#define DRAM_CHAN_ODTCFG(odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value) \
          .byte MCR_ODTCFG,odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value,0,0,0,0 ;
#define DRAM_CHAN_ODTCFG2(odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value) \
          .byte MCR_ODTCFG2,odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value,0,0,0,0 ;
#define DRAM_CHAN_ADDRCOARSE(addrcoarse_reg,addrcoarse_unb) \
          .byte MCR_ADDRCOARSE,addrcoarse_reg,addrcoarse_unb,0,0,0,0,0,0,0,0,0 ;
#define DRAM_CHAN_MANTIMING(tCK,rfsh,tval) \
          .byte MCR_MANTIMING,tCK,rfsh,0 ; \
          .byte (((tval) >> 56)&0xFF), (((tval) >> 48) & 0xFF) ; \
          .byte (((tval) >> 40)&0xFF), (((tval) >> 32) & 0xFF) ; \
          .byte (((tval) >> 24)&0xFF), (((tval) >> 16) & 0xFF) ; \
          .byte (((tval) >>  8)&0xFF), (((tval) >>  0) & 0xFF) ; 
#define DRAM_CS_TIMING(tCK,rfsh,caslatency,attributes,tRAS,tRP,tRRD,tRCD,tRFC,tRC) \
          .byte MCR_TIMING,tCK,rfsh,caslatency,attributes,tRAS,tRP,tRRD,tRCD,tRFC,tRC,0 ;
#define DRAM_CS_TIMING2(tAL,tRTP,tRAP) \
          .byte MCR_TIMING2,tAL,tRTP,tRAP,0,0,0,0,0,0,0,0 ;
#define DRAM_CS_GEOM(csel,rows,cols,banks,flags) \
          .byte MCR_GEOM,csel,rows,cols,banks,flags,0,0,0,0,0,0 ;
#define DRAM_CS_SPD(csel,flags,chan,dev) \
          .byte MCR_SPD,csel,flags,chan,dev,0,0,0,0,0,0,0 ;
#define DRAM_EOT \
          .byte MCR_EOT,0,0,0,0,0,0,0,0,0,0,0 ;
#else
#define DRAM_GLOBALS(chintlv) \
          {MCR_GLOBALS,chintlv,0,0,0,0,0,0,0,0,0,0}
#define DRAM_CHAN_CFG(chan,tMEMCLK,tROUNDTRIP,chantype,dramtype,pagepolicy,csintlv,ecc,flg) \
          {MCR_CHCFG,chan,tMEMCLK,tROUNDTRIP,chantype,dramtype,pagepolicy,csintlv,ecc,flg,0,0}
#define DRAM_CHAN_DLLCFG(addrfine,dqicoarse,dqifine,dqocoarse,dqofine) \
          {MCR_DLLCFG,addrfine,dqicoarse,dqifine,dqocoarse,dqofine,0,0,0,0,0,0}
#define DRAM_CHAN_DLLCFG2(addrfreq,dqicoarse,dqifreq,dqocoarse,dqofreq,dlldefault,dllfreq,dllbypass) \
          {MCR_DLLCFG,addrfreq,dqicoarse,dqifreq,dqocoarse,dqofreq,dlldefault,dllfreq,dllbypass,0,0,0}
#define DRAM_CHAN_ODTCFG(odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value) \
          {MCR_ODTCFG,odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value,0,0,0,0}
#define DRAM_CHAN_ODTCFG2(odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value) \
          {MCR_ODTCFG2,odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value,0,0,0,0}
#define DRAM_CHAN_ADDRCOARSE(addrcoarse_reg,addrcoarse_unb) \
          {MCR_ADDRCOARSE,addrcoarse_reg,addrcoarse_unb,0,0,0,0,0,0,0,0,0}
#define DRAM_CHAN_MANTIMING(tCK,rfsh,tval) \
          {MCR_MANTIMING,tCK,rfsh,0,  \
          (((tval) >> 56)&0xFF), (((tval) >> 48) & 0xFF),  \
          (((tval) >> 40)&0xFF), (((tval) >> 32) & 0xFF),  \
          (((tval) >> 24)&0xFF), (((tval) >> 16) & 0xFF),  \
          (((tval) >>  8)&0xFF), (((tval) >>  0) & 0xFF) } 
#define DRAM_CS_TIMING(tCK,rfsh,caslatency,attributes,tRAS,tRP,tRRD,tRCD,tRFC,tRC) \
          {MCR_TIMING,tCK,rfsh,caslatency,attributes,tRAS,tRP,tRRD,tRCD,tRFC,tRC,0}
#define DRAM_CS_TIMING2(tAL,tRTP,tRAP) \
          {MCR_TIMING2,tAL,tRTP,tRAP,0,0,0,0,0,0,0,0}
#define DRAM_CS_GEOM(csel,rows,cols,banks,flags) \
          {MCR_GEOM,csel,rows,cols,banks,flags,0,0,0,0,0,0}
#define DRAM_CS_SPD(csel,flags,chan,dev) \
          {MCR_SPD,csel,flags,chan,dev,0,0,0,0,0,0,0}
#define DRAM_EOT \
          {MCR_EOT,0,0,0,0,0,0,0,0,0,0,0}
#endif


#define MCR_GLOBALS     0
#define MCR_CHCFG 	1
#define MCR_TIMING	2
#define MCR_DLLCFG	3
#define MCR_GEOM	4
#define MCR_SPD		5
#define MCR_MANTIMING	6
#define MCR_TIMING2	7
#define MCR_ODTCFG      8
#define MCR_ODTCFG2	9
#define MCR_ADDRCOARSE	10
#define MCR_EOT		0xFF

#ifndef __ASSEMBLER__
typedef struct mc_initrec_s {
    uint8_t mcr_type;			/* record type */
    uint8_t mcr_reserved1;		/* pad to 12 bytes */
    uint8_t mcr_reserved2;		/* pad to 12 bytes */
    uint8_t mcr_reserved3;		/* pad to 12 bytes */
    uint8_t mcr_reserved4;		/* pad to 12 bytes */
    uint8_t mcr_reserved5;		/* pad to 12 bytes */
    uint8_t mcr_reserved6;		/* pad to 12 bytes */
    uint8_t mcr_reserved7;		/* pad to 12 bytes */
    uint8_t mcr_reserved8;		/* pad to 12 bytes */
    uint8_t mcr_reserved9;		/* pad to 12 bytes */
    uint8_t mcr_reserved10;		/* pad to 12 bytes */
    uint8_t mcr_reserved11;		/* pad to 12 bytes */
} mc_initrec_t;

typedef struct mc_globals_s {
    uint8_t gbl_type;
    uint8_t gbl_intlv_ch;		/* Channel interleave type */
    uint8_t gbl_reserved[10];		/* pad to 12 bytes */
} mc_globals_t;

typedef struct mc_cfgdata_s {
    uint8_t cfg_type;
    uint8_t cfg_chan;
    uint8_t cfg_mintmemclk;
    uint8_t cfg_roundtrip;
    uint8_t cfg_chantype;
    uint8_t cfg_dramtype;
    uint8_t cfg_pagepolicy;
    uint8_t cfg_intlv_cs;
    uint8_t cfg_ecc;
    uint8_t cfg_flags;
    uint8_t cfg_reserved[2];
} mc_cfgdata_t;

typedef struct mc_mantiming_s {
    uint8_t mtm_type;
    uint8_t mtm_tCK;
    uint8_t mtm_rfsh;
    uint8_t mtm_reserved;
    uint8_t mtm_timing[8];		/* one 64-bit word, as bytes */
} mc_mantiming_t;

typedef struct mc_dllcfg_s {
    uint8_t dll_type;
    uint8_t dll_addrfine;		/*addr freq range for pass 2*/
    uint8_t dll_dqicoarse;
    uint8_t dll_dqifine;		/*dqi freq range for pass 2*/
    uint8_t dll_dqocoarse;
    uint8_t dll_dqofine;		/*dqo freq range for pass 2*/
    uint8_t dll_default;
    uint8_t dll_dllfreq;
    uint8_t dll_dllbypass;
    uint8_t dll_reserved[3];
} mc_dllcfg_t;

typedef struct mc_addrcoarse_s {
    uint8_t addrcoarse_type;
    uint8_t addrcoarse_reg;
    uint8_t addrcoarse_unbuf;
    uint8_t addrcoarse_reserved[9];
} mc_addrcoarse_t;

typedef struct mc_timingdata_s {
    uint8_t tmg_type;			/* record type */
    uint8_t tmg_tCK;			/* tCK we should use */
    uint8_t tmg_rfsh;			/* SPD[12] Refresh Rate */
    uint8_t tmg_caslatency;		/* SPD[18] CAS Latencies Supported (set only one bit) */
    uint8_t tmg_attributes;		/* SPD[21] Attributes */
    uint8_t tmg_tRAS;			/* SPD[30] */
    uint8_t tmg_tRP;			/* SPD[27] */
    uint8_t tmg_tRRD;			/* SPD[28] */
    uint8_t tmg_tRCD;			/* SPD[29] */
    uint8_t tmg_tRFC;			/* SPD[42] */
    uint8_t tmg_tRC;			/* SPD[41] */
    uint8_t tmg_reserved[1];		/* pad to 12 bytes */
} mc_timingdata_t;

typedef struct mc_timing2data_s {
    uint8_t tmg2_type;			/* record type */    
    uint8_t tmg2_tAL;			/* Additive latency */
    uint8_t tmg2_tRTP;			/* SPD[38] */
    uint8_t tmg2_tRAP;			/* Active to auto-precharge delay */
    uint8_t tmg2_reserved[8];		/* pad to 12 bytes */  
} mc_timing2data_t;

typedef struct mc_geomdata_s {
    uint8_t geom_type;			/* record type */
    uint8_t geom_csel;			/* Chip Select number */
    uint8_t geom_rows;			/* rows */
    uint8_t geom_cols;			/* columns */
    uint8_t geom_banks;			/* banks */
    uint8_t geom_flags;			/* flags */
    uint8_t geom_reserved[6];		/* pad to 12 bytes */
} mc_geomdata_t;

typedef struct mc_spddata_s {
    uint8_t spd_type;			/* record type */
    uint8_t spd_csel;			/* Chip Select number */
    uint8_t spd_flags;			/* flags */
    uint8_t spd_smbuschan;		/* SMBus Channel */
    uint8_t spd_smbusdev;		/* SMBus device */
    uint8_t spd_reserved[7];		/* pad to 12 bytes */
} mc_spddata_t;

typedef struct mc_odtcfg_s {
    uint8_t odt_type;
    uint8_t odt_odt0;
    uint8_t odt_odt2;
    uint8_t odt_odt4;
    uint8_t odt_odt6;
    uint8_t odt_odd_odt_en;
    uint8_t odt_mc_value;		/* MC side - 75 ohms or 150 ohms */
    uint8_t odt_dram_value;		/* DRAM side - 75 ohms or 150 ohms */
    uint8_t odt_reserved[4];
} mc_odtcfg_t;

typedef union draminittab_s {
    mc_initrec_t mcr;			/* record with just header */
    mc_globals_t gbl;			/* global data */
    mc_cfgdata_t cfg;			/* channel configuration data */
    mc_timingdata_t tmg;		/* timing data */
    mc_dllcfg_t dll;			/* dll configuration */
    mc_geomdata_t geom;			/* geometry data */
    mc_spddata_t spd;			/* SPD data */
    mc_mantiming_t mtm;			/* manual timing */
    mc_timing2data_t tmg2;		/* timing2 data */
    mc_odtcfg_t odt;			/* odt data */
    mc_odtcfg_t odt2;			/* odt2 data */
    mc_addrcoarse_t addrc;		/* addrcoarse adj */
    uint8_t raw[12];			/* RAW data */
} draminittab_t;

#endif



/*  *********************************************************************
    *  Configuration parameter values and flag shorthands to improve
    *  readability of the tables.
    ********************************************************************* */

/* Channel interleavig options. DRAM_GLOBALS */
#define MC_NOCHANINTLV		0
#define MC_01CHANINTLV		1	
#define MC_23CHANINTLV		2	/* Valid in 32-bit channels only */
#define MC_01_23CHANINTLV	3     	/* Valid in 32-bit channels only */
#define MC_FULLCHANINTLV	4      	/* Valid ih 32-bit channels only */

/* Channels. DRAM_CHAN_CFG */
#define MC_CHAN0	0
#define MC_CHAN1	1
#define MC_CHAN2	2
#define MC_CHAN3	3

/* Channel type. DRAM_CHAN_CFG */
#define MC_32BIT_CHAN	1
#define MC_64BIT_CHAN	2	/* Set 'ganged' bit in chan 0 or 1. MC ignores for 2 or 3 */

/* DRAM type. DRAM_CHAN_CFG */ 
#define JEDEC			K_BCM1480_MC_DRAM_TYPE_JEDEC
#define FCRAM			K_BCM1480_MC_DRAM_TYPE_FCRAM
#define SGRAM			2 /* Not a register field, but might require separate timing stuff */			
#define DRAM_TYPE_SPD		3
#define JEDEC_DDR2		4

/* Page policy. DRAM_CHAN_CFG */ 
#define CLOSED			K_BCM1480_MC_PG_POLICY_CLOSED
#define CASCHECK		K_BCM1480_MC_PG_POLICY_CAS_TIME_CHK

/* Chip Select interleaving options. DRAM_CHAN_CFG */
#define NOCSINTLV	0
#define CSINTLV_2CS	1
#define CSINTLV_4CS	2
#define CSINTLV_8CS	3	/* 8 CS INTLV only available for 64-bit channels */

/* ECC enable. DRAM_CHAN_CFG */
#define ECCDISABLE		0
#define ECCENABLE		1  /* 8-bit ECC for 64-bit chan. 7-bit ECC for 32-bit chan */ 
#define ECCENABLE_FORCE64	8  /* "Double pumped" ECC. 32-bit channels only */

/* Flags for channel configuration. DRAM_CHAN_CFG */
#define MCFLG_ECC_ENABLE 	0x01   /* Used to compare ECC field in DRAM_CHAN_CFG */
#define MCFLG_ECC_FORCE64	0x08   /* Used to compare ECC field in DRAM_CHAN_CFG */  
#define MCFLG_BIGMEM		0x02
#define MCFLG_FORCEREG		0x04
#define MCFLG_2T		0x10
#define MCFLG_DQO_SHIFT		0x20
#define MCFLG_DS_REDUCED	0x40
#define MCFLG_NO_ODT_CS		0x80 	/* Set when no ODT chip selects are present and DRAM at CS 0/1, 2/3, etc */   

/* Chip Selects. DRAM_CS_GEOM or DRAM_CS_SPD */
#define MC_CS0		0    
#define MC_CS1		1
#define MC_CS2		2
#define MC_CS3		3
#define MC_CS4		4	/* CS 4-7 only available for 64-bit channels */
#define MC_CS5		5
#define MC_CS6		6
#define MC_CS7		7

#define ODT_OFF		0
#define ODT_75		1
#define ODT_150		2
#define ODT_50		3
#define CS_EVEN_ODT_EN	0
#define CS_ODD_ODT_EN	1

/* Additive latency in cycles. Timing2 register */
#define ADD_LATENCY0	0
#define ADD_LATENCY1	1
#define ADD_LATENCY2	2
#define ADD_LATENCY3	3
#define ADD_LATENCY4	4

/* DLL register freq ranges */
#define DLL_FREQ_RANGE_100_159	0xC
#define DLL_FREQ_RANGE_160_199	0x8
#define DLL_FREQ_RANGE_200_249	0x4
#define DLL_FREQ_RANGE_250_400	0x0

/*
 * End
 */


