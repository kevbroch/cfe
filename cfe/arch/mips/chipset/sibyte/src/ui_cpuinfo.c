/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  All you never wanted to know about CPUs	File: ui_cpuinfo.c
    *
    *  Routines to display CPU info (common to all CPUs)
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

#include "cfe.h"
#include "sbmips.h"
#include "ui_command.h"

#ifdef SIBYTE_SB1250
#include "sb1250_regs.h"
#include "sb1250_scd.h"
#include "sb1250_wid.h"
#endif
#ifdef SIBYTE_BCM1480
#include "bcm1480_regs.h"
#include "bcm1480_scd.h"
#include "bcm1480_l2c.h"
#endif

#include "env_subr.h"


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

/*
 * This lets us override the WID by poking values into our PromICE 
 */
#ifdef _MAGICWID_
#undef A_SCD_SYSTEM_REVISION
#define A_SCD_SYSTEM_REVISION 0x1FC00508
#undef A_SCD_SYSTEM_MANUF
#define A_SCD_SYSTEM_MANUF 0x1FC00518
#endif

/*  *********************************************************************
    *  Externs/forwards
    ********************************************************************* */

void sb1250_show_cpu_type(void);

/* XXXCGD: could be const, when env_setenv can cope.  */
#ifdef SIBYTE_SB1250
static char *show_cpu_type_bcm1250(char *, uint64_t syscfg, uint64_t sysrev);
static char *show_cpu_type_bcm112x(char *, uint64_t syscfg, uint64_t sysrev);
static void ibm_waferid_str(uint64_t wid, char *wid_str);
#endif
#ifdef SIBYTE_BCM1480
static char *show_cpu_type_bcm1480(char *, uint64_t syscfg, uint64_t sysrev);
#endif


/*  *********************************************************************
    *  ui_show_cpu_type()
    *  
    *  Display board CPU information
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
void sb1250_show_cpu_type(void)
{
    uint64_t syscfg, sysrev;
    /* XXXCGD: could be const, when env_setenv can cope.  */
    char *envval;
    char *cpuname;
    char *(*infofn)(char *, uint64_t, uint64_t);
    char temp[32];

    syscfg = SBREADCSR(A_SCD_SYSTEM_CFG);
    sysrev = SBREADCSR(A_SCD_SYSTEM_REVISION);

    switch (SYS_SOC_TYPE(sysrev)) {
#ifdef SIBYTE_SB1250
	case K_SYS_SOC_TYPE_BCM1250:
	    cpuname = "1250";
	    infofn = show_cpu_type_bcm1250;
	    break;

	case K_SYS_SOC_TYPE_BCM1120:
	    cpuname = "1120";
	    infofn = show_cpu_type_bcm112x;
	    break;

	case K_SYS_SOC_TYPE_BCM1125:
	    cpuname = "1125";
	    infofn = show_cpu_type_bcm112x;
	    break;

	case K_SYS_SOC_TYPE_BCM1125H:
	    cpuname = "1125H";
	    infofn = show_cpu_type_bcm112x;
	    break;
#endif /* SIBYTE_SB1250 */

#ifdef SIBYTE_BCM1480
	case K_SYS_SOC_TYPE_BCM1x80:
	    if (G_SYS_PART(sysrev) == K_SYS_PART_BCM1480)
		cpuname = "1480";
	    else if (G_SYS_PART(sysrev) == K_SYS_PART_BCM1280)
		cpuname = "1280";
	    else if (G_SYS_PART(sysrev) == K_SYS_PART_BCM1158)
		cpuname = "1158";
	    else {
		sprintf(temp, "unknown_1x80_%04x", (int)G_SYS_PART(sysrev));
		cpuname = temp;
		}
	    infofn = show_cpu_type_bcm1480;
	    break;

	case K_SYS_SOC_TYPE_BCM1x55:
	    if (G_SYS_PART(sysrev) == K_SYS_PART_BCM1455)
		cpuname = "1455";
	    else if (G_SYS_PART(sysrev) == K_SYS_PART_BCM1255)
		cpuname = "1255";
	    else {
		sprintf(temp, "unknown_1x55_%04x", (int)G_SYS_PART(sysrev));
		cpuname = temp;
		}
	    infofn = show_cpu_type_bcm1480;
	    break;
#endif /* SIBYTE_BCM1480 */

	default:
	    sprintf(temp, "unknown_%04x", (int)G_SYS_PART(sysrev));
	    cpuname = temp;
	    infofn = NULL;
	    break;
	}

    env_setenv("CPU_TYPE", cpuname,
	       ENV_FLG_BUILTIN | ENV_FLG_READONLY | ENV_FLG_ADMIN);

    envval = NULL;
    if (infofn != NULL)
	envval = (*infofn)(cpuname, syscfg, sysrev);
    if (envval == NULL) {
        sprintf(temp, "unknown_%02x", (int)G_SYS_REVISION(sysrev));
	envval = temp;
    }
    env_setenv("CPU_REVISION", envval,
	       ENV_FLG_BUILTIN | ENV_FLG_READONLY | ENV_FLG_ADMIN);

    /* Set # of CPUs based on 2nd hex digit of part number */
    sprintf(temp, "%d", (int)((G_SYS_PART(sysrev) >> 8) & 0x0F));
    env_setenv("CPU_NUM_CORES", temp,
	       ENV_FLG_BUILTIN | ENV_FLG_READONLY | ENV_FLG_ADMIN);

}


#ifdef SIBYTE_SB1250
static void
sb1xxx_show_syscfg(uint64_t syscfg)
{
    int plldiv;
    char temp[32];

    /*
     * Set variable that contains CPU speed, spit out config register
     */
    printf("SysCfg: %016llX [PLL_DIV: %d, IOB0_DIV: %s, IOB1_DIV: %s]\n",
	   syscfg,
	   (int)G_SYS_PLL_DIV(syscfg),
	   (syscfg & M_SYS_IOB0_DIV) ? "CPUCLK/3" : "CPUCLK/4",
	   (syscfg & M_SYS_IOB1_DIV) ? "CPUCLK/2" : "CPUCLK/3");

    plldiv = G_SYS_PLL_DIV(syscfg);
    if (plldiv == 0) {
	/* XXX: keep in synch with setting of CPU speed, above. */
	printf("PLL_DIV of zero found, assuming 6 (300MHz)\n");
	plldiv = 6;
    }

    sprintf(temp, "%d", plldiv * 50);
    env_setenv("CPU_SPEED", temp,
	       ENV_FLG_BUILTIN | ENV_FLG_READONLY | ENV_FLG_ADMIN);
}


/*  *********************************************************************
    *  show_cpu_type_bcm1250()
    *  
    *  Display CPU information for BCM1250 CPUs
    *  
    *  Input parameters: 
    *  	   revstr: pointer to string pointer, to be filled in with
    *      revision name.
    *  	   
    *  Return value:
    *  	   none.  fills in revstr.
    ********************************************************************* */
static char *
show_cpu_type_bcm1250(char *cpuname, uint64_t syscfg, uint64_t sysrev)
{
    char *revstr, *revprintstr;
    uint64_t cachetest;
    uint64_t sysmanuf, ibm_wid;
    uint32_t wid;
    int bin;
    unsigned int cpu_pass;
    char temp[32];
    static uint8_t cachesizes[16] = {4,2,2,2,2,1,1,1,2,1,1,1,2,1,1,0};
    static char *binnames[8] = {
	"2CPU_FI_1D_H2",
	"2CPU_FI_FD_F2 (OK)",
	"2CPU_FI_FD_H2",
	"2CPU_3I_3D_F2",
	"2CPU_3I_3D_H2",
	"1CPU_FI_FD_F2",
	"1CPU_FI_FD_H2",
	"2CPU_1I_1D_Q2"};

    cpu_pass = G_SYS_REVISION(sysrev);

    wid = G_SYS_WID(SBREADCSR(A_SCD_SYSTEM_REVISION));
    wid = WID_UNCONVOLUTE(wid);

    if ((wid != 0) && (cpu_pass == K_SYS_REVISION_BCM1250_A2)) {
	cpu_pass = K_SYS_REVISION_BCM1250_A6;
	}

    switch (cpu_pass) {
	case K_SYS_REVISION_BCM1250_PASS1:
	    revstr = "PASS1";
	    revprintstr = "Pass 1";
	    break;
	case K_SYS_REVISION_BCM1250_A1:
	    revstr = "A1";
	    revprintstr = "Pass 2.0 (wirebond)";
	    break;
	case K_SYS_REVISION_BCM1250_A2:
	    revstr = "A2";
	    revprintstr = "Pass 2.0 (flip-chip)";
	    break;
	case K_SYS_REVISION_BCM1250_A3:
	    revstr = "A3";
	    revprintstr = "A3 Pass 2.1 (flip-chip)";
	    break;
	case K_SYS_REVISION_BCM1250_A4:
	    revstr = "A4";
	    revprintstr = "A4 Pass 2.1 (wirebond)";
	    break;
	case K_SYS_REVISION_BCM1250_A6:
	    revstr = revprintstr = "A6";
	    break;
	case K_SYS_REVISION_BCM1250_A8:
	    revprintstr = "A8/A10";
	    revstr = "A8";
	    break;
	case K_SYS_REVISION_BCM1250_A9:
	    revstr = revprintstr = "A9";
	    break;
	case K_SYS_REVISION_BCM1250_B1:
	    revprintstr = "B0/B1";
	    revstr = "B1";
	    break;
	case K_SYS_REVISION_BCM1250_B2:
	    revstr = revprintstr = "B2";
	    break;
	case K_SYS_REVISION_BCM1250_C0:
	    revstr = revprintstr = "C0";
	    break;
	case K_SYS_REVISION_BCM1250_C1:
	    revstr = revprintstr = "C1";
	    break;
	case K_SYS_REVISION_BCM1250_C2:
	    revstr = revprintstr = "C2";
	    break;
	default:
	    revstr = NULL;
	    sprintf(temp, "rev 0x%x", (int)G_SYS_REVISION(sysrev));
	    revprintstr = temp;
	    break;
	}
    printf("CPU: BCM1250 %s\n", revprintstr);

    if (((G_SYS_PART(sysrev) >> 8) & 0x0F) == 1) {
	printf("[Uniprocessor CPU mode]\n");
	}

    /*
     * Report cache status if the cache was disabled, or the status of
     * the cache test for non-WID pass2 and pass3 parts.
     */
    printf("L2 Cache Status: ");
    if ((syscfg & M_SYS_L2C_RESET) != 0) {
	printf("disabled via JTAG\n");
	}
    else if ((cpu_pass == K_SYS_REVISION_BCM1250_A1) ||
	     (cpu_pass == K_SYS_REVISION_BCM1250_A2) ||
	     (cpu_pass == K_SYS_REVISION_BCM1250_C0) ||
	     (cpu_pass == K_SYS_REVISION_BCM1250_C1) ) {
	cachetest = (SBREADCSR(A_MAC_REGISTER(2, R_MAC_HASH_BASE)) & 0x0F);
	printf("0x%llX    Available L2 Cache: %dKB\n", cachetest,
	       ((int)cachesizes[(int)cachetest])*128);
	}
    else printf("OK\n");

    if (wid == 0) {
	printf("Wafer ID:  Not set\n");
	}
    else if (cpu_pass < K_SYS_REVISION_BCM1250_C0) {

	printf("Wafer ID:   0x%08X  [Lot %d, Wafer %d]\n", wid,
	       G_WID_LOTID(wid), G_WID_WAFERID(wid));

	bin = G_WID_BIN(wid);

	printf("Manuf Test: Bin %c [%s] ", "EABCDFGH"[bin], binnames[bin]);

	if (bin != K_WID_BIN_2CPU_FI_FD_F2)  {
	    printf("L2:%d ", G_WID_L2QTR(wid));
	    printf("CPU0:[I=%d D=%d]  ", G_WID_CPU0_L1I(wid), G_WID_CPU0_L1D(wid));
	    printf("CPU1:[I=%d D=%d]", G_WID_CPU1_L1I(wid), G_WID_CPU1_L1D(wid));
	    }
	printf("\n");
	}

    if (cpu_pass >= K_SYS_REVISION_BCM1250_C0) {
	/* Read system_manuf register for C0 or greater*/
	sysmanuf = SBREADCSR(A_SCD_SYSTEM_MANUF);

	printf("SysManuf:  %016llX [X: %d Y: %d] ", sysmanuf, (int)G_SYS_XPOS(sysmanuf), 
	                                            (int)G_SYS_YPOS(sysmanuf));

	ibm_wid = ((sysmanuf & 0xf000000000LL) >> 4) | (sysmanuf & 0xffffffff);
	char wid_str[7];
	ibm_waferid_str(ibm_wid, wid_str);
	printf("[Wafer ID: %s]", wid_str);

	printf("\n");
	}

    sb1xxx_show_syscfg(syscfg);

    return (revstr);
}

/*  *********************************************************************
    *  show_cpu_type_bcm112x()
    *  
    *  Display CPU information for BCM112x CPUs
    *  
    *  Input parameters: 
    *  	   revstr: pointer to string pointer, to be filled in with
    *      revision name.
    *  	   
    *  Return value:
    *  	   none.  fills in revstr.
    ********************************************************************* */
static char *
show_cpu_type_bcm112x(char *cpuname, uint64_t syscfg, uint64_t sysrev)
{
    char *revstr, *revprintstr;
    char temp[32];

    switch (G_SYS_REVISION(sysrev)) {
	case K_SYS_REVISION_BCM112x_A1:
	    revstr = revprintstr = "A1";
	    break;
	case K_SYS_REVISION_BCM112x_A2:
	    revstr = revprintstr = "A2";
	    break;
	case K_SYS_REVISION_BCM112x_A3:
	    revstr = revprintstr = "A3";
	    break;
	case K_SYS_REVISION_BCM112x_A4:
	    revstr = revprintstr = "A4";
	    break;
	default:
	    revstr = NULL;
	    sprintf(temp, "rev 0x%x", (int)G_SYS_REVISION(sysrev));
	    revprintstr = temp;
	    break;
	}
    printf("CPU: %s %s\n", env_getenv("CPU_TYPE"), revprintstr);

    printf("L2 Cache: ");
    if ((syscfg & M_SYS_L2C_RESET) != 0)
	printf("disabled via JTAG\n");
    else {
    /*1122 is 128K.  same soc type as 1125*/
       if (G_SYS_L2C_SIZE(sysrev) == K_SYS_L2C_SIZE_BCM1122)
    		printf("128KB\n");
        else
      		printf("256KB\n");
    }


    sb1xxx_show_syscfg(syscfg);
 
    return (revstr);
}

/* Decode IBMs WaferID code */
static void ibm_waferid_str(uint64_t wid, char *wid_str)
{
  int i;

  for (i=0; i<6; i++) {
    unsigned char bits;
    char code;

    bits = wid & 0x3f;
    wid >>= 6;
    if (bits < 0xa)
      code = bits + '0';
    else if (bits < 0x24)
      code = (bits - 0xa) + 'A';
    else if (bits == 0x3d)
      code = '-';
    else if (bits == 0x3e)
      code = '.';
    else if (bits == 0x3f)
      code = ' ';
    else
      code = '?';
    wid_str[5-i] = code;
  }
  wid_str[6] = '\0';
}
#endif /* SIBYTE_SB1250 */


#ifdef SIBYTE_BCM1480
static void
bcm14xx_show_syscfg(uint64_t syscfg)
{
    int plldiv;
    char temp[32];

    printf("SysCfg: %016llX [PLL_DIV:%d, SW_DIV:%d, CCNUMA:%s, IOB_DIV:%s]\n",
	   syscfg,
	   (int)G_BCM1480_SYS_PLL_DIV(syscfg),
	   (int)G_BCM1480_SYS_SW_DIV(syscfg),
	   (syscfg & M_BCM1480_SYS_CCNUMA_EN) ? "enable" : "disable",
	   (syscfg & M_BCM1480_SYS_IOB_DIV) ? "CPUCLK/3" : "CPUCLK/4");

    plldiv = G_BCM1480_SYS_PLL_DIV(syscfg);
    if (plldiv == 0) {
	/* XXX: keep in synch with setting of CPU speed, above. */
	printf("PLL_DIV of zero found, assuming 6 (300MHz)\n");
	plldiv = 6;
    }

    sprintf(temp, "%d", plldiv * 50);
    env_setenv("CPU_SPEED", temp,
	       ENV_FLG_BUILTIN | ENV_FLG_READONLY | ENV_FLG_ADMIN);
}


/*  *********************************************************************
    *  show_cpu_type_bcm1480()
    *  
    *  Display CPU information for BCM1480 CPUs
    *  
    *  Input parameters: 
    *  	   revstr: pointer to string pointer, to be filled in with
    *      revision name.
    *  	   
    *  Return value:
    *  	   none.  fills in revstr.
    ********************************************************************* */
static char *
show_cpu_type_bcm1480(char *cpuname, uint64_t syscfg, uint64_t sysrev)
{
    int cpu_pass = G_SYS_REVISION(sysrev);
    int enabled_cpus = bcm1480_num_cpus();
    uint64_t l2c_misc0_value;
    char *revstr, *revprintstr, *cachestr;
    char temp[32];
    int l2_disabled = 0;

#ifdef _BIGSUR_
    /*
     * This is a special hack just for the BCM1480 bringup board.  Customers,
     * please do not include this code in your ports!  The BCM1480 has no
     * way to read whether the L2 cache is being held in reset or not,
     * so we are using the least significant config bit (M_BCM1480_SYS_CONFIG)
     * to pass this information from the boot script to the firmware.
     */
    if (syscfg & (1<<S_BCM1480_SYS_CONFIG)) {
	l2_disabled = 1;
	}
#endif

    switch (cpu_pass) {
	case K_SYS_REVISION_BCM1480_S0:
	    revstr = "S0";
	    revprintstr = "S0 (pass1)";
	    break;
	case K_SYS_REVISION_BCM1480_A1:
	    revstr = "A1";
	    revprintstr = "A1 (pass1)";
	    break;
	case K_SYS_REVISION_BCM1480_A2:
	    revstr = "A2";
	    revprintstr = "A2 (pass1)";
	    break;
	case K_SYS_REVISION_BCM1480_A3:
	    revstr = "A3";
	    revprintstr = "A3 (pass1)";
	    break;
	case K_SYS_REVISION_BCM1480_B0:
	    revstr = "B0";
	    revprintstr = "B0 (pass2)";
	    break;
	default:
	    sprintf(temp, "rev 0x%x", (int)cpu_pass);
	    revprintstr = temp;
	    revstr = NULL;
	    break;
	}

    printf("CPU: %s %s, %d cpu%s", cpuname, revprintstr, enabled_cpus,
           enabled_cpus == 1 ? "" : "s");
    if (G_SYS_NUM_CPUS(sysrev) != enabled_cpus)
	printf(" enabled (%d disabled - fused)",
	       G_SYS_NUM_CPUS(sysrev) - enabled_cpus);
    printf("\n");

    if (l2_disabled) {
	/* Must not touch L2 cache registers if L2C is held in reset */
	cachestr = "disabled via JTAG";
	}
    else {
	l2c_misc0_value = SBREADCSR(A_BCM1480_L2_MISC0_VALUE);
	cachestr = "unknown size";
	if (G_SYS_L2C_SIZE(sysrev) == K_SYS_L2C_SIZE_1MB &&
	    G_BCM1480_L2C_MISC0_CACHE_DISABLE(l2c_misc0_value) != 0)
	    cachestr = "512KB (512KB disabled by software)";
	else if (G_SYS_L2C_SIZE(sysrev) == K_SYS_L2C_SIZE_1MB)
	    cachestr = "1MB";
	else if (G_SYS_L2C_SIZE(sysrev) == K_SYS_L2C_SIZE_512KB)
	    cachestr = "512KB";
	}

    printf("L2Cache: %s\n", cachestr);

    bcm14xx_show_syscfg(syscfg);

    return revstr;
}
#endif /* SIBYTE_BCM1480 */
