/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU Speed calculation			File: mpc8245_speed.c
    *  
    *  Routines to determine CPU and bus speeds.
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



#include "lib_types.h"
#include "ppcdefs.h"
#include "mpc824x.h"

#include "mpc8245_speed.h"

/*
 * This table maps the PLL configuration bits onto the multiplier
 * for the core clock.  It's stored in a fixed point format with
 * the low order bit meaning "1/2".  (e.g., value here is twice the
 * actual multiplier.
 *
 * Of course, this actually belongs somewhere else.
 */

typedef struct clkcfg_s {
    uint32_t sync_in;
    uint32_t pllmult;
} clkcfg_t;

#define MHZ25   25000000
#define MHZ50	50000000
#define MHZ33	33000000
#define MHZ66	66000000

static clkcfg_t clktable[32] = {
    {MHZ33,MULTVAL(3,0)},		/* 0 */
    {MHZ25,MULTVAL(3,0)},
    {MHZ50,MULTVAL(1,0)},
    {MHZ66,MULTVAL(1,0)},		/* bypass */

    {MHZ33,MULTVAL(2,0)},		/* 4 */
    {MHZ33,MULTVAL(2,0)},		/* not used */
    {MHZ33,MULTVAL(1,0)},		/* bypass */
    {MHZ66,MULTVAL(1,0)},	      

    {MHZ66,MULTVAL(1,0)},		/* 8 */
    {MHZ66,MULTVAL(2,0)},
    {MHZ25,MULTVAL(2,0)},
    {MHZ50,MULTVAL(1,5)},

    {MHZ33,MULTVAL(2,0)},		/* c */
    {MHZ50,MULTVAL(1,5)},
    {MHZ33,MULTVAL(2,0)},
    {MHZ25,MULTVAL(3,0)},

    {MHZ33,MULTVAL(3,0)},		/* 0x10 */
    {MHZ25,MULTVAL(4,0)},
    {MHZ66,MULTVAL(1,5)},
    {MHZ25,MULTVAL(4,0)},

    {MHZ33,MULTVAL(2,0)},		/* 0x14 */
    {MHZ25,MULTVAL(2,5)},
    {MHZ33,MULTVAL(2,0)},
    {MHZ33,MULTVAL(4,0)},		/* RESERVED */

    {MHZ33,MULTVAL(2,5)},		/* 0x18 */
    {MHZ50,MULTVAL(2,0)},
    {MHZ66,MULTVAL(1,0)},
    {MHZ33,MULTVAL(2,0)},

    {MHZ50,MULTVAL(1,5)},		/* 0x1c */
    {MHZ66,MULTVAL(1,5)},
    {MHZ33,MULTVAL(2,0)},		/* only rev D */
    {MHZ33,MULTVAL(1,0)}		/* PLL OFF: no clock */
};

static uint32_t cputable[32] = {
    MULTVAL(2,5), 
    MULTVAL(3,0), 
    MULTVAL(4,5), 
    MULTVAL(2,0), 

    MULTVAL(2,0), 
    MULTVAL(2,0), 	/* not used */
    MULTVAL(2,0), 	/* bypass */
    MULTVAL(3,0), 

    MULTVAL(3,0), 
    MULTVAL(2,0), 
    MULTVAL(4,5), 
    MULTVAL(3,0), 

    MULTVAL(2,5), 
    MULTVAL(3,5), 
    MULTVAL(3,0), 
    MULTVAL(3,5), 

    MULTVAL(2,0), 
    MULTVAL(2,5), 
    MULTVAL(2,0), 
    MULTVAL(3,0), 

    MULTVAL(3,5), 
    MULTVAL(4,0), 
    MULTVAL(4,0), 
    MULTVAL(2,0), 

    MULTVAL(3,0), 
    MULTVAL(2,5), 
    MULTVAL(4,0), 
    MULTVAL(3,0), 

    MULTVAL(3,0), 
    MULTVAL(2,5), 
    MULTVAL(1,0), 
    MULTVAL(3,5)
};

   
/*  *********************************************************************
    *  mpc8245_speeds(speed)
    *  
    *  Calculate the frequencies of various clocks.
    *  
    *  Input parameters: 
    *  	   speed - pointer to record to receive values
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
void mpc8245_speeds(mpc8245_speed_t *speed)
{
    /*
     * Use the crystal frequency (hopefully constant!) and the
     * PLL multiplier (read from core SPR) to calculate the
     * clock speed.
     */
    volatile uint32_t *cfgaddr = (volatile uint32_t *) A_MPC_CONFIG_ADDR;
    volatile uint8_t *cfgdata  = (volatile uint8_t *) A_MPC_CONFIG_DATA;
    uint8_t pll;	
    uint32_t hid1val;

    *cfgaddr = MPC_PCR;
    pll =  *(cfgdata + (MPC_PCR & 3));
    pll = (pll >> 3) & 0x1F;

    /* Read the HID1 register to get the PLL settings. */
    __asm __volatile("mfspr %0,1009" : "=r"(hid1val));

    if (speed->sync_in_clk) {
	speed->sys_logic_clk = speed->sync_in_clk;
	}
    else {
	speed->sys_logic_clk = clktable[pll].sync_in; 
	}


    speed->mem_pllmult = clktable[pll].pllmult;
    speed->cpu_pllmult = cputable[G_HID1_PLLCFG(hid1val)];

    speed->sys_logic_clk = ((((speed->sys_logic_clk << 1) * speed->mem_pllmult)) >> 2);

    speed->core_clk = ((speed->sys_logic_clk << 1) * speed->cpu_pllmult) >> 2;

}


/*  *********************************************************************
    *  mpc8245_cpu_speed()
    *  
    *  Calculate the frequencies of the CPU clock.
    *  
    *  Input parameters: 
    *  	   none
    *  	   
    *  Return value:
    *  	   CPU clock frequency in Hz.
    ********************************************************************* */
unsigned int mpc8245_cpu_speed(void)
{
    mpc8245_speed_t mpc8245_speed;

    mpc8245_speed.sync_in_clk = 0;		/* guess the SYNC_IN clock */
    mpc8245_speeds(&mpc8245_speed);

    return mpc8245_speed.core_clk;
}
