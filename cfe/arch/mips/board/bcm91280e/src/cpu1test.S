/*  *********************************************************************
    *  BCM1480 Board Support Package
    *  
    *  CPU1 test routines			File: CPU1TEST.S
    *
    *  This file gives CPU1 something to do.
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

#include "sbmips.h"
#include "bsp_config.h"
#include "cpu_config.h"
#include "mipsmacros.h"
#include "bcm91280e.h"
#include "bcm1480_regs.h"

		.text



#define SETLEDS(a,b,c,d)                     \
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
       JAL(cpu1_setleds)


		.text


/*  *********************************************************************
    *  BOARD_SETLEDS(x)
    *  
    *  Set LEDs for boot-time progress indication.  Not used if
    *  the board does not have progress LEDs.  This routine
    *  must not call any other routines, since it may be invoked
    *  either from KSEG0 or KSEG1 and it may be invoked 
    *  whether or not the icache is operational.
    *  
    *  Input parameters: 
    *  	   a0 - LED value (8 bits per character, 4 characters)
    *  	   
    *  Return value:
    *  	   nothing
    *  
    *  Registers used:
    *  	   t0,t1,t2,t3
    ********************************************************************* */


#define LED_CHAR0	(8*3)
#define LED_CHAR1	(8*2)
#define LED_CHAR2	(8*1)
#define LED_CHAR3	(8*0)

LEAF(cpu1_setleds)

		li	t0,PHYS_TO_K1(LEDS_PHYS)

		rol	a0,a0,8
		and	t1,a0,0xFF
		sb	t1,LED_CHAR0(t0)

		rol	a0,a0,8
		and	t1,a0,0xFF
		sb	t1,LED_CHAR1(t0)

		rol	a0,a0,8
		and	t1,a0,0xFF
		sb	t1,LED_CHAR2(t0)

		rol	a0,a0,8
		and	t1,a0,0xFF
		sb	t1,LED_CHAR3(t0)

		j	ra

END(cpu1_setleds)


/*  *********************************************************************
    *  CPU1PROC()
    *  
    *  This routine is started on the secondary processor.  It just
    *  makes some noise on the LEDs.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   does not return
    ********************************************************************* */

#ifdef _FUNCSIM_
#define WAITCOUNT 100000
#else
#define WAITCOUNT 300000000
#endif

delay:
		mtc0	zero,C0_COUNT
1:		li	t0,WAITCOUNT
		mfc0	t1,C0_COUNT
		blt	t1,t0,1b

		j	ra


#define GET_CUR_CPU(reg) \
		mfc0	reg,C0_PRID ; \
		srl	reg,reg,25 ; \
		and	reg,reg,7 

LEAF(cpu1proc)

		li	s2,PHYS_TO_K1(LEDS_PHYS+LED_CHAR3+1)

		/* Clear display to spaces */
		li	t0,' '
		sb	t0,-1(s2)
		sb	t0,-2(s2)
		sb	t0,-3(s2)
		sb	t0,-4(s2)

		/* Get index based on our processor number */
		GET_CUR_CPU(t0)
		beq	t0,0,1f		/* should not happen */
		sub	t0,t0,1
1:
		addi	t2,t0,'1'	/* cpu number */
		sub	s2,t1		/* s2 points at rightmost char */


		/* write processor number and twiddly thingy */

loop:

		li	t3,'/'
		sb	t3,-1(s2)

		bal	delay

		li	t3,'-'
		sb	t3,-1(s2)

		bal	delay

		li	t3,'\\'
		sb	t3,-1(s2)

		bal	delay

		li	t3,'|'
		sb	t3,-1(s2)

		bal	delay

		b	loop
		

END(cpu1proc)
	

