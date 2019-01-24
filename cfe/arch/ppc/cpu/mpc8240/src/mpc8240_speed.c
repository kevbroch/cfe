/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU Speed calculation			File: mpc8240_speed.c
    *  
    *  Routines to determine CPU and bus speeds.
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2004
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
#include "mpc824x.h"

extern uint32_t read_hid1(void);

/*
 * This table maps the PLL configuration bits onto the multiplier
 * for the core clock.  It's stored in a fixed point format with
 * the low order bit meaning "1/2".  (e.g., value here is twice the
 * actual multiplier.
 *
 * Of course, this actually belongs somewhere else.
 */
#define MULTVAL(amt,frac) (((amt)<<1)|((frac)/5))
static const int pllmults[32] = {
    MULTVAL(1,5),		/* 0 */
    MULTVAL(1,0),
    MULTVAL(1,0),
    MULTVAL(1,0),		/* PLL OFF: bypassed */

    MULTVAL(2,0),		/* 4 */
    MULTVAL(2,0),
    MULTVAL(2,5),
    MULTVAL(4,5),

    MULTVAL(3,0),		/* 8 */
    MULTVAL(5,5),
    MULTVAL(4,0),
    MULTVAL(5,0),

    MULTVAL(1,5),		/* c */
    MULTVAL(6,0),
    MULTVAL(3,5),
    MULTVAL(1,0),		/* PLL OFF: no clock */

    MULTVAL(3,0),		/* 0x10 */
    MULTVAL(2,5),
    MULTVAL(6,5),
    MULTVAL(1,0),		/* PLL OFF: bypassed */

    MULTVAL(7,0),		/* 0x14 */
    MULTVAL(1,0),		/* RESERVED */
    MULTVAL(7,5),
    MULTVAL(1,0),		/* RESERVED */

    MULTVAL(1,5),		/* 0x18 */
    MULTVAL(1,0),		/* RESERVED */
    MULTVAL(1,0),		/* RESERVED */
    MULTVAL(1,0),		/* RESERVED */

    MULTVAL(8,0),		/* 0x1c */
    MULTVAL(1,0),		/* RESERVED */
    MULTVAL(1,0),		/* RESERVED */
    MULTVAL(1,0)		/* PLL OFF: no clock */
};
    

/*  *********************************************************************
    *  mpc8240_cpu_speed()
    *  
    *  Calculate our CPU speed
    *  
    *  Input parameters: 
    *  	   none
    *  	   
    *  Return value:
    *  	   CPU clock frequency in Hz
    ********************************************************************* */

unsigned int mpc8240_cpu_speed(void);

unsigned int mpc8240_cpu_speed(void)
{
    unsigned int cpu_speed;
    uint32_t hid1;

    /*
     * Use the crystal frequency (hopefully constant!) and the
     * PLL multiplier (read from core SPR) to calculate the
     * clock speed.
     */

    hid1 = read_hid1();
    cpu_speed = 33000000;		/* crystal frequency */
    cpu_speed = (((cpu_speed << 1) * pllmults[G_HID1_PLLCFG(hid1)])) >> 1;

    return cpu_speed;
}

