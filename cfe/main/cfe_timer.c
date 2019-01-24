/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Timer routines				File: cfe_timer.c
    *  
    *  This module contains routines to keep track of the system time,.
    *  Since we don't have any interrupts in the firmware, even the
    *  timer is polled.  The timer must be called often enough
    *  to prevent missing the overflow of the CP0 COUNT
    *  register, approximately 2 billion cycles (half the count)
    * 
    *  Be sure to use the POLL() macro each time you enter a loop
    *  where you are waiting for some I/O event to occur or
    *  are waiting for time to elapse.
    *
    *  It is *not* a time-of-year clock.  The timer is only used
    *  for timing I/O events.
    *
    *  Internally, time is maintained in units of "CLOCKSPERTICK",
    *  which should be about tenths of seconds.
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

#include "cfe.h"

#ifndef CPUCFG_CYCLESPERCPUTICK
#define CPUCFG_CYCLESPERCPUTICK 1	/* CPU clock ticks per CP0 COUNT */
#endif

/*  *********************************************************************
    *  Externs
    ********************************************************************* */

extern uint32_t _getticks(void);	/* return value of CP0 COUNT */

/*  *********************************************************************
    *  Data
    ********************************************************************* */

volatile cfe_timer_t cfe_ticks;		/* current system time */

unsigned int cfe_cpu_speed;		/* CPU speed in clocks/second */

/* With current technology, we would like to accomodate
   sub-microsecond delays, but clocks per nanosecond may be a small
   number.  For more precision, we convert in terms of Kns, where
   one Kns = 1024 nsec, and scale by shifting. */

static unsigned int cfe_clocks_per_Kns;
static unsigned int cfe_clocks_per_usec;
static unsigned int cfe_clocks_per_tick;

static uint32_t cfe_oldcount;		/* For keeping track of ticks */
static uint32_t cfe_remticks;
static int cfe_timer_initflg = 0;

/*
 * C0_COUNT clocks per tick.  Some CPUs tick CP0 every 'n' cycles,
 * that's what CPUCFG_CYCLESPERCPUTICK is for.
 */
#define CFE_CLOCKSPERUSEC(cpu_speed) \
   ((cpu_speed)/1000000/(CPUCFG_CYCLESPERCPUTICK))
#define CFE_CLOCKSPERKNS(cpu_speed)  \
   ((cpu_speed)/976563/(CPUCFG_CYCLESPERCPUTICK))
#define CFE_CLOCKSPERTICK(cpu_speed) \
   ((cpu_speed)/(CFE_HZ)/(CPUCFG_CYCLESPERCPUTICK)) 


/*  *********************************************************************
    *  cfe_timer_task()
    *  
    *  This routine is called as part of normal device polling to 
    *  update the system time.   We read the CP0 COUNT register,
    *  add the delta into our current time, convert to ticks,
    *  and keep track of the COUNT register overflow
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */


static void cfe_timer_task(void *arg)
{
    uint32_t count;
    uint32_t deltaticks;
    uint32_t clockspertick;

    clockspertick = cfe_clocks_per_tick;

    count = _getticks();
    deltaticks    = (count - cfe_oldcount);
    cfe_remticks += deltaticks;

    /*
     * If CP0 COUNT jumped by a large value, use div/mod to update
     * the clock.  Otherwise,
     * assume it only moved by one tick and use a simple
     * loop to update it.  This loop probably will not
     * execute more than once.
     */

    if (cfe_remticks > (clockspertick << 4)) {
        cfe_ticks += (cfe_remticks / clockspertick);
        cfe_remticks %= clockspertick;
        }
    else {
        while (cfe_remticks > clockspertick) {
            cfe_remticks -= clockspertick;
            cfe_ticks++;
            }
        }

    cfe_oldcount = count;
}


/*  *********************************************************************
    *  cfe_timer_init(cpu_speed)
    *  
    *  Initialize the timer module.
    *  
    *  Input parameters: 
    *  	   CPU clock frequency in Hz (normally cfe_cpu_speed)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void cfe_timer_init(unsigned int cpu_speed)
{
    cfe_clocks_per_tick = CFE_CLOCKSPERTICK(cpu_speed);
    cfe_clocks_per_Kns = CFE_CLOCKSPERKNS(cpu_speed);
    if (cfe_clocks_per_Kns == 0)
	cfe_clocks_per_Kns = 1;    /* for the simulator */
    cfe_clocks_per_usec = CFE_CLOCKSPERUSEC(cpu_speed);
    if (cfe_clocks_per_usec == 0)
	cfe_clocks_per_usec = 1;    /* for the simulator */

    cfe_oldcount = _getticks();		/* get current COUNT register */
    cfe_ticks = 0;

    if (!cfe_timer_initflg) {
	cfe_bg_add(cfe_timer_task,NULL); /* add task for background polling */
	cfe_timer_initflg = 1;
	}
}


/*  *********************************************************************
    *  cfe_sleep(ticks)
    *  
    *  Sleep for 'ticks' ticks.  Background tasks are processed while
    *  we wait.
    *  
    *  Input parameters: 
    *  	   ticks - number of ticks to sleep (note: *not* clocks!)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void cfe_sleep(int ticks)
{
    int64_t timer;

    TIMER_SET(timer,ticks);
    while (!TIMER_EXPIRED(timer)) {	
	POLL();
	}
}



/*  *********************************************************************
    *  cfe_usleep(usec)
    *  
    *  Sleep for approximately the specified number of microseconds.
    *  
    *  Input parameters: 
    *  	   usec - number of microseconds to wait
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void cfe_usleep(int usec)
{
    uint32_t newcount;
    uint32_t now;

    /* XXX fix the wrap problem */

    now = _getticks();
    newcount = now + usec*cfe_clocks_per_usec;

    if (newcount < now)  	/* wait for wraparound */
        while (_getticks() > now)
	    ;
    

    while (_getticks() < newcount)
	;
}


/*  *********************************************************************
    *  cfe_nsleep(nsec)
    *  
    *  Sleep for approximately the specified number of nanoseconds.
    *  
    *  Input parameters: 
    *  	   nsec - number of nanoseconds to wait
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void cfe_nsleep(int nsec)
{
    uint32_t newcount;
    uint32_t now;

    /* XXX fix the wrap problem */

    now = _getticks();
    newcount = now + ((nsec*cfe_clocks_per_Kns + 512) >> 10);

    if (newcount < now)  	/* wait for wraparound */
        while (_getticks() > now)
	    ;
    
    while (_getticks() < newcount)
	;
}
