/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  MEMC definitions				File: sbmemc.h
    *
    *  Constants and macros pertaining to SiliconBackplane 
    *  based MEMC cores
    *  
    *********************************************************************  
    *
    *  Copyright 2003,2004
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


#ifndef	_SBMEMC_H
#define	_SBMEMC_H

#define	MEMC_CONTROL		0x00
#define	MEMC_CONFIG		0x04
#define	MEMC_REFRESH		0x08
#define	MEMC_BISTSTAT		0x0c  /* unused */
#define	MEMC_MODEBUF		0x10
#define	MEMC_BKCLS		0x14  /* unused */
#define	MEMC_PRIORINV		0x18  /* unused */
#define	MEMC_DRAMTIM		0x1c
#define	MEMC_INTSTAT		0x20  /* unused */
#define	MEMC_INTMASK		0x24  /* unused */
#define	MEMC_INTINFO		0x28  /* unused */
#define	MEMC_NCDLCTL		0x30
#define	MEMC_RDNCDLCOR		0x34
#define	MEMC_WRNCDLCOR		0x38
#define	MEMC_MISCDLYCTL		0x3c
#define	MEMC_DQSGATENCDL	0x40
#define	MEMC_SPARE		0x44  /* unused */
#define	MEMC_TPADDR		0x48  /* unused */
#define	MEMC_TPDATA		0x4c  /* unused */
#define	MEMC_BARRIER		0x50  /* unused */
#define	MEMC_CORE		0x54  /* unused */


/* MEMC Core Init values (OCP ID 0x80f) */

/* For sdr: */
#define MEMC_SD_CONFIG_INIT	0x0004a000
#define MEMC_SD_DRAMTIM2_INIT	0x000754d8	/* CAS latency 2 */
#define MEMC_SD_DRAMTIM3_INIT	0x000754da	/* CAS latency 3 */
#define MEMC_SD_RDNCDLCOR_INIT	0x00000000
#define MEMC_SD_WRNCDLCOR_INIT	0x49351200
#define MEMC_SD1_WRNCDLCOR_INIT	0x14500200	/* For corerev 1  */
#define MEMC_SD_MISCDLYCTL_INIT	0x00061c1b
#define MEMC_SD1_MISCDLYCTL_INIT 0x00021416	/* For corerev 1  */
#define MEMC_SD_CONTROL_INIT0	0x00000002	/* SeqEn */
#define MEMC_SD_CONTROL_INIT1	0x00000008	/* PreCmd */
#define MEMC_SD_CONTROL_INIT2	0x00000004	/* RefCmd */
#define MEMC_SD_CONTROL_INIT3	0x00000010	/* MRSCmd */
#define MEMC_SD_CONTROL_INIT4	0x00000001	/* DRAMAccEn */
#define MEMC_SD_MODEBUF_INIT	0x00000000
#define MEMC_SD_REFRESH_INIT	0x0000840f

#if   defined(SDRM16X16X2)
/* This is for 64 MB as 16M X 16 X 2 (e.g., bcm95365rr) */
#define MEMC_GEOMETRY
#define MEMC_SDR_INIT           0x0008
#define MEMC_SDR_MODE           0x23
#define	MEMC_SDR_NCDL		0x00000000      /* was 0x00020032 */
#define	MEMC_SDR1_NCDL		0x00000000      /* was 0x0002020f */
#elif defined(SDRM4X32X1)
/* This is for 16 MB as 4M X 32 X 1 */
#define MEMC_GEOMETRY
#define MEMC_SDR_INIT           0x0000
#define MEMC_SDR_MODE           0x32
#define	MEMC_SDR_NCDL		0x00000000      /* was 0x00020032 */
#define	MEMC_SDR1_NCDL          0x00000000      /* was 0x0002020f */
#elif defined(SDRM8X8X4)
/* Default: 32MB as 8M x 8 x 4 */
#define MEMC_GEOMETRY
#define	MEMC_SDR_INIT		0x0008
#define	MEMC_SDR_MODE		0x32
#define	MEMC_SDR_NCDL		0x00020032
#define	MEMC_SDR1_NCDL		0x0002020f	/* For corerev 1  */
#elif defined(SDRM32X8X4)
/* This is for 128 MB as 32M X 8 X 4 */
#define MEMC_GEOMETRY
#define MEMC_SDR_INIT           0x0010		/* 1024 columns */
#define MEMC_SDR_MODE           0x32		/* CAS 3, Burst 4 */
#define	MEMC_SDR_NCDL		0x00000000      /* was 0x00020032 */
#define	MEMC_SDR1_NCDL          0x00000000      /* was 0x0002020f */
#elif defined(SDRM8X16X2)
/* This is for 32 MB as 8M X 16 X 2 */
#define MEMC_GEOMETRY
#define MEMC_SDR_INIT           0x0008		/* 512 columns */
#define MEMC_SDR_MODE           0x32		/* 0x30 = CAS 3, 0x02 = Burst Length 4 */
#define	MEMC_SDR_NCDL		0x00000000      /* was 0x00020032 */
#define	MEMC_SDR1_NCDL          0x00000000      /* was 0x0002020f */
#endif


/* For ddr: */
#define MEMC_CONFIG_INIT	0x00048000
#define MEMC_DRAMTIM2_INIT	0x000754d8	/* CL = 2   */
#define MEMC_DRAMTIM25_INIT	0x000754d9	/* CL = 2.5 */
#define MEMC_RDNCDLCOR_INIT	0x00000000
#define MEMC_WRNCDLCOR_INIT	0x49351200
#define MEMC_1_WRNCDLCOR_INIT	0x14500200
#define MEMC_DQSGATENCDL_INIT	0x00030000
#define MEMC_MISCDLYCTL_INIT	0x21061c1b
#define MEMC_1_MISCDLYCTL_INIT	0x21021400
#define MEMC_NCDLCTL_INIT	0x00002001
#define MEMC_CONTROL_INIT0	0x00000002
#define MEMC_CONTROL_INIT1	0x00000008
#define MEMC_MODEBUF_INIT0	0x00004000
#define MEMC_CONTROL_INIT2	0x00000010
#define MEMC_MODEBUF_INIT1	0x00000100
#define MEMC_CONTROL_INIT3	0x00000010
#define MEMC_CONTROL_INIT4	0x00000008
#define MEMC_REFRESH_INIT	0x0000840f
#define MEMC_CONTROL_INIT5	0x00000004
#define MEMC_MODEBUF_INIT2	0x00000000
#define MEMC_CONTROL_INIT6	0x00000010
#define MEMC_CONTROL_INIT7	0x00000001

#if   defined(DDRM16X16X2)
/* This is for 64 MB as 16M X 16 X 2 */
#define MEMC_GEOMETRY
#define	MEMC_DDR_INIT		0x0009
#define	MEMC_DDR_MODE		0x62
#define	MEMC_DDR_NCDL		0x00000000      /* was 0005050a */
#define	MEMC_DDR1_NCDL		0x00000a0a	/* For corerev 1 */
#elif defined(DDRM32X16X2)
/* This is for 128 MB as 32M X 16 X 2 (e.g., bcm95836cpci) */
#define MEMC_GEOMETRY
#define	MEMC_DDR_INIT		0x0011		/* for full 128MB */
#define	MEMC_DDR_MODE		0x62
#define	MEMC_DDR_NCDL		0x00000000	/* was 0x0005050a */
#define	MEMC_DDR1_NCDL		0x00000a0a	/* For corerev 1  */
#elif defined(DDRM64X16X2)
/* This is for 256 MB as 64M X 16 X 2 (e.g., bcm95836cpci Rev 2) */
#define MEMC_GEOMETRY
#define	MEMC_DDR_INIT		0x0011		/* for full 256MB (adds row) */
#define	MEMC_DDR_MODE		0x62
#define	MEMC_DDR_NCDL		0x00000000
#define	MEMC_DDR1_NCDL		0x00000a0a	/* For corerev 1  */
#endif

#ifndef MEMC_GEOMETRY
#error "sbmemc: geometry must be defined"
#endif

/* mask for sdr/ddr calibration registers */
#define MEMC_RDNCDLCOR_RD_MASK	0x000000ff
#define MEMC_WRNCDLCOR_WR_MASK	0x000000ff
#define MEMC_DQSGATENCDL_G_MASK	0x000000ff

/* masks for miscdlyctl registers */
#define MEMC_MISC_SM_MASK	0x30000000
#define MEMC_MISC_SM_SHIFT	28
#define MEMC_MISC_SD_MASK	0x0f000000
#define MEMC_MISC_SD_SHIFT	24

/* hw threshhold for calculating wr/rd for sdr memc */
#define MEMC_CD_THRESHOLD	128

/* Low bit of init register says if memc is ddr or sdr */
#define MEMC_CONFIG_DDR		0x00000001

#endif	/* _SBMEMC_H */
