/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Silicon Backplane utilities       	File: sb_utils.c
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

/*
 * Misc utility routines for accessing chip-specific features
 * of SiliconBackplane-based chips.
 */

#include "cfe.h"

#include "sbmips32.h"
#include "sb_bp.h"
#include "sb_utils.h"

/*
 * Depending on the chip, either the Chip Common or the External
 * Interface core provides global state and control.  These are
 * mutually exclusive.  The corresponding registers in the two cores
 * have compatible formats, but the core base address and the register
 * offsets depend on the core.
 */

#if defined(SB_EXTIF_BASE)
#include "sb_extif.h"
#elif defined(SB_CHIPC_BASE)
#include "sb_chipc.h"
#else
#error "Neither EXT_IF nor CHIPC defined"
#endif


#define ASSERT(x) \
  do { if (!(x)) printf("sb_utils: assertion failed\n"); } while (0)

/* Sharable clock related definitions and calculations. */

/* PLL types */

#define PLL_NONE        0x00000000
#define PLL_N3M         0x00010000
#define PLL_N4M         0x00020000
#define PLL_TYPE3       0x00030000
#define PLL_TYPE4       0x00008000

#define CC_CLOCK_BASE   24000000      /* Half the clock freq. */

/* bcm4710 (N3M) Clock Control magic field values */

#define CC_F6_2         0x02          /* A factor of 2 in */
#define CC_F6_3         0x03          /*  6-bit fields like */
#define CC_F6_4         0x05          /*  N1, M1 or M3 */
#define CC_F6_5         0x09
#define CC_F6_6         0x11
#define CC_F6_7         0x21

#define CC_F5_BIAS      5             /* 5-bit fields get this added */

#define CC_MC_BYPASS    0x08
#define CC_MC_M1        0x04
#define CC_MC_M1M2      0x02
#define CC_MC_M1M2M3    0x01
#define CC_MC_M1M3      0x11

/* bcm5836 (N4M) Clock Control magic field values (ditto) */

#define	CC_T2_BIAS      2             /* N1, N2, M1 & M3 bias */
#define	CC_T2M2_BIAS    3             /* M2 bias */

#define	CC_T2MC_M1BYP   1
#define	CC_T2MC_M2BYP   2
#define	CC_T2MC_M3BYP 	4


static inline uint32_t
factor6(uint32_t x)
{
    switch (x) {
	case CC_F6_2:	return 2;
	case CC_F6_3:	return 3;
	case CC_F6_4:	return 4;
	case CC_F6_5:	return 5;
	case CC_F6_6:	return 6;
	case CC_F6_7:	return 7;
	default:	return 0;
	}
}

/*
 * Calculate the PLL output frequency given a set of clockcontrol
 * values (CC_CLOCK_BASE assumed fixed).
 */

static uint32_t
sb_clock_rate(uint32_t pll_type, uint32_t n, uint32_t m)
{
    uint32_t n1, n2, clock, m1, m2, m3, mc;

    n1 = G_CCN_N1(n);
    n2 = G_CCN_N2(n);

    if (pll_type == PLL_N3M) {
	n1 = factor6(n1);
	n2 += CC_F5_BIAS;
	}
    else if (pll_type == PLL_N4M) {
	n1 += CC_T2_BIAS;
	n2 += CC_T2_BIAS;
	}
    else if (pll_type == PLL_TYPE3) {
	return 100000000;                 /* NB: for SB only */
	}
    else {
        ASSERT(0);
	return 0;
	}

    clock = CC_CLOCK_BASE * n1 * n2;

    if (clock == 0)
	return 0;

    m1 = G_CCM_M1(m);
    m2 = G_CCM_M2(m);
    m3 = G_CCM_M3(m);
    mc = G_CCM_MC(m);

    if (pll_type == PLL_N3M) {
	m1 = factor6(m1);
	m2 += CC_F5_BIAS;
	m3 = factor6(m3);

	switch (mc) {
	    case CC_MC_BYPASS:	return clock;
	    case CC_MC_M1:	return clock / m1;
	    case CC_MC_M1M2:	return clock / (m1 * m2);
	    case CC_MC_M1M2M3:	return clock / (m1 * m2 * m3);
	    case CC_MC_M1M3:	return clock / (m1 * m3);
	    default:		return 0;
	    }
	}
    else if (pll_type == PLL_N4M) {
	m1 += CC_T2_BIAS;
	m2 += CC_T2M2_BIAS;
	m3 += CC_T2_BIAS;
	if ((mc & CC_T2MC_M1BYP) == 0)
	    clock /= m1;
	if ((mc & CC_T2MC_M2BYP) == 0)
	    clock /= m2;
	if ((mc & CC_T2MC_M3BYP) == 0)
	    clock /= m3;
	return clock;
	}
    else {
	ASSERT(0);
	return 0;
	}
}


#if defined(SB_EXTIF_BASE)

/* Note: For EXTIF cores, the PLL must be N3M (aka TYPE1). */

/* Access EXTIF "enumeration" space */

#define READCSR(x)   \
  (*(volatile uint32_t *)PHYS_TO_K1(SB_EXTIF_BASE+(x)))
#define WRITECSR(x,v) \
  (*(volatile uint32_t *)PHYS_TO_K1(SB_EXTIF_BASE+(x)) = (v))

/*
 * Reset the entire chip and copy master clock registers to the slaves.
 */

void
sb_chip_reset(void)
{
    /* instant NMI from watchdog timeout after 1 tick */
    WRITECSR(R_WATCHDOGCNTR, 1);
    while (1)
	;
}


/* Return the current speed the SB is running at */
uint32_t
sb_clock(void)
{
    uint32_t clockcontrol_n, clockcontrol_sb;

    clockcontrol_n = READCSR(R_CLOCKCONTROLN);
    clockcontrol_sb = READCSR(R_CLOCKCONTROLSB);

    return sb_clock_rate(PLL_N3M, clockcontrol_n, clockcontrol_sb);
}

/* Return the current speed the CPU is running at. */
uint32_t
sb_cpu_clock(void)
{
    /* For EXTIF parts, cpu and backplane clocks are identical. */
    return sb_clock();
}


/* Set the current speed of the SB to the desired rate (as closely as
   possible) */
int
sb_setclock(uint32_t sb, uint32_t pci)
{
    uint32_t old_n, old_sb, old_pci;
    uint32_t new_n, new_sb, new_pci;
    uint i;
    static const struct {
		uint32_t clock;
		uint16_t n;
		uint32_t sb;
		uint32_t m33;
		uint32_t m25;
    } sb_clock_table[] = {
	{  96000000, 0x0303, 0x04020011, 0x11030011, 0x11050011 },
	/*  96.000 32.000 24.000 */
	{ 100000000, 0x0009, 0x04020011, 0x11030011, 0x11050011 },
	/* 100.000 33.333 25.000 */
	{ 104000000, 0x0802, 0x04020011, 0x11050009, 0x11090009 },
	/* 104.000 31.200 24.960 */
	{ 108000000, 0x0403, 0x04020011, 0x11050009, 0x02000802 },
	/* 108.000 32.400 24.923 */
	{ 112000000, 0x0205, 0x04020011, 0x11030021, 0x02000403 },
	/* 112.000 32.000 24.889 */
	{ 115200000, 0x0303, 0x04020009, 0x11030011, 0x11050011 },
	/* 115.200 32.000 24.000 */
	{ 120000000, 0x0011, 0x04020011, 0x11050011, 0x11090011 },
	/* 120.000 30.000 24.000 */
	{ 124800000, 0x0802, 0x04020009, 0x11050009, 0x11090009 },
	/* 124.800 31.200 24.960 */
	{ 128000000, 0x0305, 0x04020011, 0x11050011, 0x02000305 },
	/* 128.000 32.000 24.000 */
	{ 132000000, 0x0603, 0x04020011, 0x11050011, 0x02000305 },
	/* 132.000 33.000 24.750 */
	{ 136000000, 0x0c02, 0x04020011, 0x11090009, 0x02000603 },
	/* 136.000 32.640 24.727 */
	{ 140000000, 0x0021, 0x04020011, 0x11050021, 0x02000c02 },
	/* 140.000 30.000 24.706 */
	{ 144000000, 0x0405, 0x04020011, 0x01020202, 0x11090021 },
	/* 144.000 30.857 24.686 */
	{ 150857142, 0x0605, 0x04020021, 0x02000305, 0x02000605 },
	/* 150.857 33.000 24.000 */
	{ 152000000, 0x0e02, 0x04020011, 0x11050021, 0x02000e02 },
	/* 152.000 32.571 24.000 */
	{ 156000000, 0x0802, 0x04020005, 0x11050009, 0x11090009 },
	/* 156.000 31.200 24.960 */
	{ 160000000, 0x0309, 0x04020011, 0x11090011, 0x02000309 },
	/* 160.000 32.000 24.000 */
	{ 163200000, 0x0c02, 0x04020009, 0x11090009, 0x02000603 },
	/* 163.200 32.640 24.727 */
	{ 168000000, 0x0205, 0x04020005, 0x11030021, 0x02000403 },
	/* 168.000 32.000 24.889 */
	{ 176000000, 0x0602, 0x04020003, 0x11050005, 0x02000602 },
	/* 176.000 33.000 24.000 */
    };
    uint sb_clock_entries = sizeof(sb_clock_table)/sizeof(sb_clock_table[0]);

    /* Store the current clock reg values */
    old_n = READCSR(R_CLOCKCONTROLN);
    old_sb = READCSR(R_CLOCKCONTROLSB);
    old_pci = READCSR(R_CLOCKCONTROLPCI);

    /* keep current pci clock if not specified */
    if (pci == 0) {
	pci = sb_clock_rate(PLL_N3M, old_n, old_pci);
	}
    pci = (pci <= 25000000) ? 25000000 : 33000000;

    /* Find a supported clock setting no faster than the request. */
    if (sb < sb_clock_table[0].clock)
	return 0;
    for (i = sb_clock_entries-1; i > 0; i--) {
	if (sb >= sb_clock_table[i].clock)
	    break;
	}

    new_n = sb_clock_table[i].n;
    new_sb = sb_clock_table[i].sb;
    new_pci = pci == 25000000 ? sb_clock_table[i].m25 : sb_clock_table[i].m33;

    if (old_n != new_n || old_sb != new_sb || old_pci != new_pci) {
	/* Reset to install the new clocks if any changed. */
	WRITECSR(R_CLOCKCONTROLN, new_n);
	WRITECSR(R_CLOCKCONTROLSB, new_sb);
	WRITECSR(R_CLOCKCONTROLPCI, new_pci);
	/* Clock MII at 25 MHz. */
	WRITECSR(R_CLOCKCONTROLMII, sb_clock_table[i].m25);

	/* No return from chip reset. */
	sb_chip_reset();
	}

    return 1;
}

#elif defined(SB_CHIPC_BASE)

/* Access CHIPC "enumeration" space */

#define READCSR(x)    \
  (*(volatile uint32_t *)PHYS_TO_K1(SB_CHIPC_BASE+(x)))
#define WRITECSR(x,v) \
  (*(volatile uint32_t *)PHYS_TO_K1(SB_CHIPC_BASE+(x)) = (v))

void
sb_chip_reset(void)
{
    /* instant NMI from watchdog timeout after 1 tick */
    WRITECSR(R_WATCHDOGCNTR, 1);
    while (1)   /* Use 'wait' instead? */
	;
}

/* Return the current speed the SB is running at. */
uint32_t
sb_clock(void)
{
    uint32_t corecap;
    uint32_t clockcontrol_n, clockcontrol_sb;
    uint32_t pll_type;

    corecap = READCSR(R_CORECAPABILITIES);
    switch (G_CORECAP_PL(corecap)) {
	case K_PL_4710:  pll_type = PLL_N3M;  break;
	case K_PL_4704:  pll_type = PLL_N4M;  break;
	case K_PL_5365:  pll_type = PLL_TYPE3; break;
	default:         pll_type = PLL_NONE; break;
	}
    clockcontrol_n = READCSR(R_CLOCKCONTROLN);
    clockcontrol_sb = READCSR(R_CLOCKCONTROLSB);

    return sb_clock_rate(pll_type, clockcontrol_n, clockcontrol_sb);
}

/* Return the current speed the CPU is running at. */
uint32_t
sb_cpu_clock(void)
{
    uint32_t corecap;
    uint32_t clockcontrol_n, clockcontrol_m;
    uint32_t pll_type;

    corecap = READCSR(R_CORECAPABILITIES);
    clockcontrol_n = READCSR(R_CLOCKCONTROLN);
    switch (G_CORECAP_PL(corecap)) {
	case K_PL_4710:
	    pll_type = PLL_N3M;
	    clockcontrol_m = READCSR(R_CLOCKCONTROLM0);
	    break;
	case K_PL_4704:
	    pll_type = PLL_N4M;
	    clockcontrol_m = READCSR(R_CLOCKCONTROLM3);
	    break;
	case K_PL_5365:
	    pll_type = PLL_TYPE3;
	    return 200000000;     /* until PLL_TYPE3 is documented */
	    break;
	default:
	    pll_type = PLL_NONE;
	    clockcontrol_m = 0;
	    break;
	}

    return sb_clock_rate(pll_type, clockcontrol_n, clockcontrol_m);
}


/* Set the current speed of the CPU to the desired rate (as closely as
   possible).  XXX Currently, cannot change CPU:SB ratio. */
int
sb_setclock(uint32_t cpu, uint32_t pci)
{
    typedef struct {
	uint32_t mipsclock;
	uint32_t sbclock;
	uint16_t n;
	uint32_t sb;           /* aka m0 */
	uint32_t pci33;        /* aka m1 */
	uint32_t m2;           /* aka mii/uart/mipsref */
	uint32_t m3;           /* aka mips */
	uint32_t ratio;        /* cpu:sb */
	uint32_t ratio_parm;   /* for CP0 register 22, sel 3 (ClkSync) */
    } n4m_table_t;

    /* XXX 9:4 ratio parm was 0x012a0115.  Current value from BMIPS docs. */
    static const n4m_table_t type2_table[] = {
	{ 180000000,  80000000, 0x0403, 0x01010000, 0x01020300,
	  0x01020600, 0x05000100, 0x94, 0x012a00a9 },
	{ 180000000,  90000000, 0x0403, 0x01000100, 0x01020300,
	  0x01000100, 0x05000100, 0x21, 0x0aaa0555 },
	{ 200000000, 100000000, 0x0303, 0x01000000, 0x01000600,
	  0x01000000, 0x05000000, 0x21, 0x0aaa0555 },
	{ 211200000, 105600000, 0x0902, 0x01000200, 0x01030400,
	  0x01000200, 0x05000200, 0x21, 0x0aaa0555 },
	{ 220800000, 110400000, 0x1500, 0x01000200, 0x01030400,
	  0x01000200, 0x05000200, 0x21, 0x0aaa0555 },
	{ 230400000, 115200000, 0x0604, 0x01000200, 0x01020600,
	  0x01000200, 0x05000200, 0x21, 0x0aaa0555 },
	{ 234000000, 104000000, 0x0b01, 0x01010000, 0x01010700,
	  0x01020600, 0x05000100, 0x94, 0x012a00a9 },
	{ 240000000, 120000000,	0x0803,	0x01000200, 0x01020600,	
	  0x01000200, 0x05000200, 0x21, 0x0aaa0555 },
	{ 252000000, 126000000,	0x0504,	0x01000100, 0x01020500,
	  0x01000100, 0x05000100, 0x21, 0x0aaa0555 },
	{ 264000000, 132000000, 0x0903, 0x01000200, 0x01020700,
	  0x01000200, 0x05000200, 0x21, 0x0aaa0555 },
	{ 270000000, 120000000, 0x0703, 0x01010000, 0x01030400,
	  0x01020600, 0x05000100, 0x94, 0x012a00a9 },
	{ 276000000, 122666666, 0x1500, 0x01010000, 0x01030400,
	  0x01020600, 0x05000100, 0x94, 0x012a00a9 },
	{ 280000000, 140000000, 0x0503, 0x01000000, 0x01010600,
	  0x01000000, 0x05000000, 0x21, 0x0aaa0555 },
	{ 288000000, 128000000, 0x0604, 0x01010000, 0x01030400,
	  0x01020600, 0x05000100, 0x94, 0x012a00a9 },
	{ 288000000, 144000000, 0x0404, 0x01000000, 0x01010600,
	  0x01000000, 0x05000000, 0x21, 0x0aaa0555 },
	{ 300000000, 133333333, 0x0803, 0x01010000, 0x01020600,
	  0x01020600, 0x05000100, 0x94, 0x012a00a9 },
	{ 300000000, 150000000, 0x0803, 0x01000100, 0x01020600,
	  0x01000100, 0x05000100, 0x21, 0x0aaa0555 }
    };

    static const n4m_table_t type4_table[] = {
	{ 192000000,  96000000, 0x0702,	0x04020011, 0x11030011,
	  0x04020011, 0x04020003, 0x21, 0x0aaa0555 },
	{ 200000000, 100000000, 0x0009,	0x04020011, 0x11030011,
	  0x04020011, 0x04020003, 0x21, 0x0aaa0555 },
	{ 216000000, 108000000, 0x0211, 0x11020005, 0x11030303,
	  0x11020005, 0x04000005, 0x21, 0x0aaa0555 },
	{ 228000000, 101333333, 0x0e02, 0x11030003, 0x11210005,
	  0x11030305, 0x04000005, 0x94, 0x012a00a9 },
	{ 228000000, 114000000, 0x0e02, 0x11020005, 0x11210005,
	  0x11020005, 0x04000005, 0x21, 0x0aaa0555 },
	{ 240000000, 120000000,	0x0109,	0x11030002, 0x01050203,
	  0x11030002, 0x04000003, 0x21, 0x0aaa0555 },
	{ 252000000, 126000000,	0x0203,	0x04000005, 0x11050005,
	  0x04000005, 0x04000002, 0x21, 0x0aaa0555 },
	{ 264000000, 132000000, 0x0602, 0x04000005, 0x11050005,
	  0x04000005, 0x04000002, 0x21, 0x0aaa0555 },
	{ 272000000, 116571428, 0x0c02, 0x04000021, 0x02000909,
	  0x02000221, 0x04000003, 0x73, 0x254a14a9 },
	{ 280000000, 120000000, 0x0209, 0x04000021, 0x01030303,
	  0x02000221, 0x04000003, 0x73, 0x254a14a9 },
	{ 288000000, 123428571, 0x0111, 0x04000021, 0x01030303,
	  0x02000221, 0x04000003, 0x73, 0x254a14a9 },
	{ 300000000, 120000000, 0x0009, 0x04000009, 0x01030203,
	  0x02000902, 0x04000002, 0x52, 0x02520129 }
    };

    const n4m_table_t *clock_table;
    uint clock_entries;
    uint i;
    uint32_t old_n;
    uint32_t old_m0, old_m1, old_m2, old_m3;
    uint32_t new_n;
    uint32_t new_m0, new_m1, new_m2, new_m3;
    uint32_t old_ratio, new_ratio;
    uint32_t corecap;
    uint32_t pll_type;

    clock_table = NULL;  clock_entries = 0;   /* defaults */

    corecap = READCSR(R_CORECAPABILITIES);
    switch (G_CORECAP_PL(corecap)) {
	case K_PL_4710:    /* XXX Does this ever occur for CHIPC parts? */
	    pll_type = PLL_N3M;
	    break;
	case K_PL_4704:
	    pll_type = PLL_N4M;
	    clock_table = type2_table;
	    clock_entries = sizeof(type2_table)/sizeof(n4m_table_t);
	    break;
	case K_PL_5365:
	    pll_type = PLL_TYPE3;
	    clock_table = NULL;
	    return 200000000;     /* until PLL_TYPE3 is documented */
	    break;
	case 0:    /* XXX Fix for expanded field. */
	    pll_type = PLL_TYPE4;
	    clock_entries = sizeof(type4_table)/sizeof(n4m_table_t);
	    break;
	default:
	    pll_type = PLL_NONE;
	    break;
	}

    if (clock_table == NULL)
	return 0;

    /* Remember the current settings */
    old_n = READCSR(R_CLOCKCONTROLN);
    old_m0 = READCSR(R_CLOCKCONTROLM0);
    old_m1 = READCSR(R_CLOCKCONTROLM1);
    old_m2 = READCSR(R_CLOCKCONTROLM2);
    old_m3 = READCSR(R_CLOCKCONTROLM3);

    /* Match to deduce current cpu:sb ratio. */
    old_ratio = 0;
    for (i = 0; i < clock_entries; i++) {
	if (old_n == clock_table[i].n
	    && old_m0 == clock_table[i].sb && old_m1 == clock_table[i].pci33
	    && old_m2 == clock_table[i].m2 && old_m3 == clock_table[i].m3) {
	    old_ratio = clock_table[i].ratio;
	    break;
	    }
	}
    if (i == clock_entries) {
	/* No match; look for the supported ratios. */
	uint32_t mips_clk = sb_clock_rate(pll_type, old_n, old_m3);
	uint32_t sb_clk = sb_clock_rate(pll_type, old_n, old_m0);

	if (mips_clk == 2*sb_clk)
	    old_ratio = 0x21;
	else if (mips_clk == (sb_clk/4)*9)
	    old_ratio = 0x94;
	}
    if (old_ratio == 0)
	return 0;

    /* Find a supported CPU clock setting no faster than the request. */
    new_ratio = 0;
    if (cpu < clock_table[0].mipsclock)
	return 0;
    for (i = clock_entries-1; i > 0; i--) {
	if (cpu >= clock_table[i].mipsclock)
	    break;
	}

    new_ratio = clock_table[i].ratio;
    if (new_ratio != old_ratio)  /* For now, can't change ratios. */
	return 0;

    new_n = clock_table[i].n;
    new_m0 = clock_table[i].sb;
    new_m1 = clock_table[i].pci33;
    new_m2 = clock_table[i].m2;
    new_m3 = clock_table[i].m3;

    if (old_n != new_n || old_m0 != new_m0 || old_m1 != new_m1
	|| old_m2 != new_m2 || old_m3 != new_m3) {
	/* Reset to install the new clocks if any changed. */
	WRITECSR(R_CLOCKCONTROLN, new_n);
	WRITECSR(R_CLOCKCONTROLM0, new_m0);
	WRITECSR(R_CLOCKCONTROLM1, new_m1);
	WRITECSR(R_CLOCKCONTROLM2, new_m2);
	WRITECSR(R_CLOCKCONTROLM3, new_m3);

	/* No return from chip reset. */
	sb_chip_reset();
	}

    return 1;
}


/* Return the reference clock being supplied to the internal UART(s). */
uint32_t
sb_uart_clock(void)
{
    uint32_t coreid, corecap, corectl;
    uint32_t pll_type;
    uint32_t cc_n, cc_m;
    uint32_t clock, div;

    coreid = READCSR(R_SBIDHIGH);
    corecap = READCSR(R_CORECAPABILITIES);
    switch (G_CORECAP_PL(corecap)) {
	case K_PL_4710:  pll_type = PLL_N3M;   break;
	case K_PL_4704:  pll_type = PLL_N4M;   break;
	case K_PL_5365:  pll_type = PLL_TYPE3; break;
	default:         pll_type = PLL_NONE;  break;
	}

    cc_n = READCSR(R_CLOCKCONTROLN);
    if (pll_type == PLL_N3M) {
	cc_m = READCSR(R_CLOCKCONTROLM2);
        clock = sb_clock_rate(PLL_N3M, cc_n, cc_m);
	div = 1;
	}
    else if (pll_type == PLL_TYPE3) {
	/* 5365 type clock, not documented */
	clock = 100000000;
	div = 54;  /* clock/1843200 */
	WRITECSR(R_UARTCLOCKDIV, div);
	}      
    else {
	if (G_SBID_RV(coreid) >= 3) {
	    /* Internal backplane clock. */
	    cc_m = READCSR(R_CLOCKCONTROLSB);
	    clock = sb_clock_rate(PLL_N4M, cc_n, cc_m);
	    div = clock/1843200;
	    WRITECSR(R_UARTCLOCKDIV, div);
	    }
	else {
	    /* Fixed internal backplane clock (4310, certain 4306). */
	    clock = 88000000;
	    div = 48;
	    }
	}

    /* Clock source depends on strapping if UartClkOverride is unset. */
    corectl = READCSR(R_CORECONTROL);
    if ((G_SBID_RV(coreid) > 0) && ((corectl & M_CORECTL_CO) == 0)) {
	if (G_CORECAP_CS(corecap) == K_CS_INTERNAL) {
	    /* Internal divided backplane clock */
	    clock /= div;
            }
	else {
	    /* Assume external clock of 1.8432 MHz */
	    clock = 1843200;
	    }
	}

    return clock;
}
#endif /* SB_CHIPC_BASE */


/* Backplane interrupt mapping. */

#if defined(SB_MIPS_BASE)
#define SB_CPU_BASE SB_MIPS_BASE
#elif defined(SB_MIPS33_BASE)
#define SB_CPU_BASE SB_MIPS33_BASE
#else
#error "Neither MIPS nor MIPS33 core defined"
#endif

#define READREG(b,x)     (*(uint32_t *)PHYS_TO_K1((b)+(x)))
#define WRITEREG(b,x,v)  (*(uint32_t *)PHYS_TO_K1((b)+(x)) = (v))

/*
 * Map the interrupt index for a core (SBTPSFlagNum0 in SBTPSFlag
 * register) into the (external) interrupt priorities (IPn) defined by
 * the MIPS architecture.
 *
 * The mapping of backplane interrupts to MIPS interrupts is
 * controlled by the CPU core's SBIPSFLAG register.  Note that any
 * interrupt source not mapped by the four SBIPSFLAG fields will be
 * directed to IP0 (level 2) and must additionally be enabled in the
 * SBINTVEC register.  */

unsigned int
sb_map_irq(unsigned int flagnum)
{
    unsigned int ip;
    uint32_t ipsflags, intvec;

    ipsflags = READREG(SB_CPU_BASE, R_SBIPSFLAG);

    if (flagnum == G_SBISF_F1(ipsflags))
        ip = 1;
    else if (flagnum == G_SBISF_F2(ipsflags))
        ip = 2;
    else if (flagnum == G_SBISF_F3(ipsflags))
        ip = 3;
    else if (flagnum == G_SBISF_F4(ipsflags))
        ip = 4;
    else {
	intvec = READREG(SB_CPU_BASE, R_SBINTVEC);
	intvec |= (1 << flagnum);
	WRITEREG(SB_CPU_BASE, R_SBINTVEC, intvec);
	ip = 0;
	}

    return ip;
}
