/*  *********************************************************************
    *  BCM47xx Board Support Package
    *  
    *  Board-specific initialization		File: bcm94704cpci_init.S
    *
    *  This module contains the assembly-language part of the init
    *  code for this board support package.  The routine
    *  "board_earlyinit" lives here.
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


#include "endian.h"
#include "cpu_config.h"
#include "sbmips32.h"
#include "sb_bp.h"
#include "sb_chipc.h"
#include "bsp_config.h"

		.text

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

/* LED digits are selected by M_ADDR[4:3] in this revision. */

#define LED_DIGIT(n)  ((n)<<3)
	

/*  *********************************************************************
    *  BOARD_EARLYINIT()
    *  
    *  Initialize board registers.  This is the earliest 
    *  time the BSP gets control.  This routine cannot assume that
    *  memory is operational, and therefore all code in this routine
    *  must run from registers only.  The $ra register must not
    *  be modified, as it contains the return address.
    *
    *  This routine will be called from uncached space, before
    *  the caches are initialized.  If you want to make
    *  subroutine calls from here, you must use the CALLKSEG1 macro.
    *
    *  Among other things, this is where the GPIO registers get 
    *  programmed to make on-board LEDs function, or other startup
    *  that has to be done before anything will work.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(board_earlyinit)


	#
	# Configure the external interface so we can access LEDs, etc.
	#

		li	a2, PHYS_TO_K1(SB_CHIPC_BASE)

	#
	# CS0 is used to access the following devices:	
	#   alphanumeric LEDs (Osram slr2016)
	#   TOD/NVRAM (Dallas ds1743)
	#
	# To avoid the various chipc/UART errata, the suggested workaround
	# is to set w1+w3 < 7.  We set w1 = 2 and w3 = 4 below; the w3 value
	# is out of spec for the LEDs at backplane frequencies above 100 MHz,
	# but the PLD (Altera epm7032ae) effectively extends w3 by its
	# (unspecified) minimum propagation delay.
	#

		li	a3, (V_CS_EM(2) | M_CS_EN)     /* enable the leds */
		sw	a3, R_CS01CONFIG(a2)
		li	a3, (V_CS_W0(16)|V_CS_W1(2)|V_CS_W2(14)|V_CS_W3(4))
		sw	a3, R_CS01MEMWAITCNT(a2)       /* 0x040E0210 */

		li	a3, M_CS_EN	               /* enable alt flash */
		sw	a3, R_CS4CONFIG(a2)
		li	a3, (V_CS_W0(16)|V_CS_W1(2)|V_CS_W2(14)|V_CS_W3(4))
		sw	a3, R_CS4WAITCNT(a2)           /* 0x040E0210 */

	#
	# Use CP0 Diagnostic Register to turn on the caches (I$ and D$)
	#
		mfc0	v0,C0_DIAGNOSTIC
		or	v0,(M_BCM0_DE | M_BCM0_IE)
		mtc0	v0,C0_DIAGNOSTIC

		li	a2, PHYS_TO_K1(BCM95836_CPCI_LED_ADDR)
		li	a3, 0x2A
		sw	a3, LED_DIGIT(0)(a2)

		jr	ra


END(board_earlyinit)


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
    *  	   t0
    ********************************************************************* */

	
LEAF(board_setleds)

		li	t0,PHYS_TO_K1(BCM95836_CPCI_LED_ADDR)
		rol	a0,a0,8
		sb	a0,LED_DIGIT(3)(t0)
		rol	a0,a0,8
		sb	a0,LED_DIGIT(2)(t0)
		rol	a0,a0,8
		sb	a0,LED_DIGIT(1)(t0)
		rol	a0,a0,8
		sb	a0,LED_DIGIT(0)(t0)
		j	ra

END(board_setleds)



/*  *********************************************************************
    *  Misc functions
    ********************************************************************* */

	.set	mips32

	.globl	read_config0
read_config0:
	mfc0	v0,C0_CONFIG,0
	j	ra

	.globl	read_config1
read_config1:
	mfc0	v0,C0_CONFIG,1
	j	ra

	.globl	read_bcm0
read_bcm0:
	mfc0	v0,C0_BRCMCFG,0
	j	ra

	.globl	read_bcm1
read_bcm1:
	mfc0	v0,C0_BRCMCFG,1
	j	ra

	.globl	read_bcm2
read_bcm2:
	mfc0	v0,C0_BRCMCFG,2
	j	ra

	.globl	read_bcm3
read_bcm3:
	mfc0	v0,C0_BRCMCFG,3
	j	ra

	.globl	read_bcm4
read_bcm4:
	mfc0	v0,C0_BRCMCFG,4
	j	ra

	.globl	read_bcm5
read_bcm5:
	mfc0	v0,C0_BRCMCFG,5
	j	ra

	.globl	read_bcm6
read_bcm6:
	mfc0	v0,C0_BRCMCFG,6
	j	ra

	.globl	read_bcm7
read_bcm7:
	mfc0	v0,C0_BRCMCFG,7
	j	ra
