/*  *********************************************************************
    *  BCM1280/BCM1480 Board Support Package
    *  
    *  Board-specific initialization		File: BCM91480HT_INIT.S
    *
    *  This module contains the assembly-language part of the init
    *  code for this board support package.  The routine
    *  "board_earlyinit" lives here.
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
#include "bcm1480_regs.h"
#include "bcm1480_scd.h"
#include "bcm1480_l2c.h"
#include "sb1250_genbus.h"
#include "bcm1480_draminit.h"
#include "jedec.h"
#include "bsp_config.h"
#include "cpu_config.h"
#include "mipsmacros.h"
#include "bcm91480ht.h"

		.text



/*  *********************************************************************
    *  Macros
    ********************************************************************* */


#define _SERIAL_PORT_LEDS_


#ifdef _SERIAL_PORT_LEDS_
#include "sb1250_uart.h"
#endif

/*  *********************************************************************
    *  LED macros
    ********************************************************************* */

#define SETLEDS1(a,b,c,d)                     \
	li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
	JAL_KSEG1(board_setleds)
#define SETLEDS(a,b,c,d)                     \
	li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
	JAL_KSEG1(board_setleds)


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
    *  subroutine calls from here, you must use the JAL_KSEG1 macro.
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

		move	s0, ra

       #
       # Configure the GPIOs
       #

		li	t0,PHYS_TO_K1(A_GPIO_DIRECTION)
		li	t1,GPIO_OUTPUT_MASK
		sd	t1,0(t0)

		li	t0,PHYS_TO_K1(A_GPIO_INT_TYPE)
		li	t1,GPIO_INTERRUPT_MASK
		sd	t1,0(t0)

#if CFG_L2_RAM  /* Variant for using L2 as memory via TLB for 32 bits */
       #
       # Set up the L2 cache to be used as SRAM by setting the
       # way_disable address (the actual flush will be done by
       # sb1250_l2cache_init).
       #
		li	t0,PHYS_TO_K1(A_BCM1480_L2_MAKE_WAY_ENABLE_LO(0x00))
		sd	zero,(t0)
		li	t0,PHYS_TO_K1(A_BCM1480_L2_MAKE_WAY_ENABLE_HI(0x08))
		sd	zero,(t0)

       # Use the result of a load to stall the pipe here.
       # Ref sec 5.4.2 (aka page 92, 1250_1125UM100-R).
       # XXX Probably not necessary for these early, global accesses.
		ld	t0,(t0)
		addu	t0,t0,t0
#endif


       #
       # Set the PCIX default frequency
       #
		li	t0,PHYS_TO_K1(A_GPIO_PIN_CLR)
		li	t1,(M_GPIO_PCIX_FREQALL)
		sd	t1,0(t0)

		li	t0,PHYS_TO_K1(A_GPIO_PIN_SET)
		li	t1,(M_GPIO_PCIX_FREQ33)
		sd	t1,0(t0)


       #
       # Configure the alternate boot ROM
       #     

		li	t0,PHYS_TO_K1(A_IO_EXT_CS_BASE(ALT_BOOTROM_CS))

		li	t1,ALT_BOOTROM_PHYS >> S_IO_ADDRBASE
		sd	t1,R_IO_EXT_START_ADDR(t0)

		li	t1,ALT_BOOTROM_SIZE-1
		sd	t1,R_IO_EXT_MULT_SIZE(t0)

		li	t1,ALT_BOOTROM_TIMING0
		sd	t1,R_IO_EXT_TIME_CFG0(t0)

		li	t1,ALT_BOOTROM_TIMING1
		sd	t1,R_IO_EXT_TIME_CFG1(t0)

		li	t1,ALT_BOOTROM_CONFIG
		sd	t1,R_IO_EXT_CFG(t0)


       #
       # Configure the LEDs
       #     
		li	t0,PHYS_TO_K1(A_IO_EXT_CS_BASE(LEDS_CS))
		li	t1,LEDS_PHYS >> S_IO_ADDRBASE
		sd	t1,R_IO_EXT_START_ADDR(t0)

		li	t1,LEDS_SIZE-1	/* Needs to be 1 smaller, se UM for details */
		sd	t1,R_IO_EXT_MULT_SIZE(t0)

		li	t1,LEDS_TIMING0
		sd	t1,R_IO_EXT_TIME_CFG0(t0)

		li	t1,LEDS_TIMING1
		sd	t1,R_IO_EXT_TIME_CFG1(t0)

		li	t1,LEDS_CONFIG
		sd	t1,R_IO_EXT_CFG(t0)

       #
       # Configure the BATTERY BACKUP CONTROLLER CPLD
       #     
		li	t0,PHYS_TO_K1(A_IO_EXT_CS_BASE(BATTERY_CS))
		li	t1,BATTERY_PHYS >> S_IO_ADDRBASE
		sd	t1,R_IO_EXT_START_ADDR(t0)

		li	t1,BATTERY_SIZE-1	/* Needs to be 1 smaller, se UM for details */
		sd	t1,R_IO_EXT_MULT_SIZE(t0)

		li	t1,BATTERY_TIMING0
		sd	t1,R_IO_EXT_TIME_CFG0(t0)

		li	t1,BATTERY_TIMING1
		sd	t1,R_IO_EXT_TIME_CFG1(t0)

		li	t1,BATTERY_CONFIG
		sd	t1,R_IO_EXT_CFG(t0)

	#
	# Make sure that the mailbox registers are cleared
	#

		li	t0,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CLR_CPU))
		nor	t1, zero, zero
		sd	t1,(t0)
		li	t0,PHYS_TO_K1(A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_1_CLR_CPU))
		sd	t1,(t0)
	
#ifdef _SERIAL_PORT_LEDS_

	# Program the mode register for 8 bits/char, no parity

		li	t0,PHYS_TO_K1(A_DUART_MODE_REG_1_A)	
		li	t1,V_DUART_BITS_PER_CHAR_8 | V_DUART_PARITY_MODE_NONE
		sd	t1,(t0)

	# Program the mode register for 1 stop bit, ignore CTS

		li	t0,PHYS_TO_K1(A_DUART_MODE_REG_2_A)	
		li	t1,M_DUART_STOP_BIT_LEN_1
		sd	t1,(t0)

	# Program the baud rate to 115200

		li	t0,PHYS_TO_K1(A_DUART_CLK_SEL_A)
		li	t1,V_DUART_BAUD_RATE(CFG_SERIAL_BAUD_RATE)
		sd	t1,(t0)

	# Dont use any interrupts

		li	t0,PHYS_TO_K1(A_DUART_IMR)
		ld	t1,(t0)
		and	t1,~M_DUART_IMR_ALL_A
		sd	t1,(t0)

	# Enable sending and receiving

		li	t0,PHYS_TO_K1(A_DUART_CMD_A)
		li	t1,M_DUART_RX_EN | M_DUART_TX_EN
		sd	t1,(t0)

#endif

		move	ra, s0
		j	ra

END(board_earlyinit)


/*  *********************************************************************
    *  BOARD_DRAMINFO
    *  
    *  Return the address of the DRAM information table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   v0 - DRAM info table, return 0 to use default table
    ********************************************************************* */


#define VPN2_256K(va)  (((va)>>(13+6))<<6)
#define PFN(pa)        ((pa)>>12)
#define PGMASK_256K    (0x3F<<13)
	
LEAF(board_draminfo)


#if CFG_L2_RAM  /* Variant for using L2 as memory via TLB for 32 bits */
	/*	
	 * Map 512K of L2 space to VA 0x300000 using a pair of 256K
	 * page frames.
	 * This is an abuse of the function, but it is called at the
	 * correct point in the initialization sequence, where adding
	 * new function calls is delicate.
	 */

		.set	push
		.set	mips64
	
		dli	v0,(VPN2_256K(0x00300000)<<13)
		dmtc0	v0,C0_TLBHI
		li	v0,(PFN(0xd0300000)<<6 | (5<<3) | 7)
		mtc0	v0,C0_TLBLO0
		li	v0,(PFN(0xd0340000)<<6 | (5<<3) | 7)
		mtc0	v0,C0_TLBLO1
		li	v0,PGMASK_256K
		mtc0	v0,C0_PGMASK
		li	v0,1
		mtc0	v0,C0_WIRED
		li	v0,0
		mtc0	v0,C0_INDEX
		tlbwi

		HAZARD

		.set	pop
#endif /* CFG_L2_RAM */

       /*
        * Return pointer to DRAM table.
	*/

		la	v0,dramtab
		j	ra


dramtab:
	/*
	 * Big Dipper DRAM
	 * Four 32 bit ECC channels
	 * channel 0 and 1 single slot with ODT enable
	 * channel 2 and 3 two slots ODT static on 
	 *
	 *  DRAM_CHAN_CFG(chan,tMEMCLK,tROUNDTRIP,chantype,dramtype,pagepolicy,csintlv,ecc,flg)
	 *  DRAM_CS_SPD(csel,flags,chan,dev)
	 *  DRAM_CS_TIMING2(tAL,tRTP,tRAP)
	 *  DRAM_CHAN_DLLCFG2(addrfreq,dqicoarse,dqifreq,dqocoarse,dqofreq,dlldefault,dllfreq,dllbypass)
	 *  DRAM_CHAN_ODTCFG(odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value)
	 *  DRAM_CHAN_ODTCFG2(odt0,odt2,odt4,odt6,odt_odd_en,odt_mc_value,odt_dram_value)
	 *  DRAM_CHAN_ADDRCOARSE(addrcoarse_reg,addrcoarse_unbuf)
	 *
	 *  tROUNDTRIP is trace delay + edge delay(600ps) of MCLK + DSQ
	 */

	DRAM_GLOBALS(MC_01CHANINTLV)	 
        DRAM_CHAN_CFG(MC_CHAN0, DRT10(3,0), DRT10(3,0), MC_32BIT_CHAN, DRAM_TYPE_SPD, CASCHECK,CSINTLV_2CS ,0 , 0 )
	DRAM_CS_SPD(MC_CS0,0,DRAM_SMBUS_CHAN_J4_1,DRAM_SMBUS_DEV_J4_1)
	DRAM_CHAN_ODTCFG(0x30,0x00,0x00,0x00,CS_ODD_ODT_EN,ODT_150,ODT_75)

        DRAM_CHAN_CFG(MC_CHAN1, DRT10(3,0), DRT10(3,0), MC_32BIT_CHAN, DRAM_TYPE_SPD, CASCHECK,CSINTLV_2CS , 0, 0 )
	DRAM_CS_SPD(MC_CS0,0,DRAM_SMBUS_CHAN_J4_1,DRAM_SMBUS_DEV_J4_1)
	DRAM_CHAN_ODTCFG(0x30,0x00,0x00,0x00,CS_ODD_ODT_EN,ODT_150,ODT_75)

	/* 
	 * Channels 2 and 3 
	 * 4 rank no ODT 
	 */
/*
 * MC_NOCHANINTLV
 * MC_01CHANINTLV		
 * MC_23CHANINTLV      	Valid in 32-bit channels only 
 * MC_01_23CHANINTLV	Valid in 32-bit channels only 
 * MC_FULLCHANINTLV 	Valid in 32-bit channels only 
 */

/*
 * NOCSINTLV
 * CSINTLV_2CS
 * CSINTLV_4CS
 * CSINTLV_8CS	Valid in 64-bit channels only
 */
	
        DRAM_CHAN_CFG(MC_CHAN2, DRT10(3,0), DRT10(3,5), MC_32BIT_CHAN, DRAM_TYPE_SPD, CASCHECK, 0, 0, MCFLG_2T | MCFLG_NO_ODT_CS)
	DRAM_CS_SPD(MC_CS0,0,DRAM_SMBUS_CHAN_J5_2,DRAM_SMBUS_DEV_J5_2)
	DRAM_CS_SPD(MC_CS2,0,DRAM_SMBUS_CHAN_J25_2,DRAM_SMBUS_DEV_J25_2)
	DRAM_CHAN_ODTCFG(0x00,0x00,0x00,0x00,0x00,ODT_150,ODT_75)
	DRAM_CHAN_ODTCFG2(0x00,0x00,0x00,0x00,0x00,ODT_150,ODT_75)
	DRAM_CHAN_ADDRCOARSE(0x0,0x0c)

        DRAM_CHAN_CFG(MC_CHAN3, DRT10(3,0), DRT10(3,5), MC_32BIT_CHAN, DRAM_TYPE_SPD, CASCHECK, 0, 0, MCFLG_2T | MCFLG_NO_ODT_CS)
	DRAM_CS_SPD(MC_CS0,0,DRAM_SMBUS_CHAN_J5_2,DRAM_SMBUS_DEV_J5_2)
	DRAM_CS_SPD(MC_CS2,0,DRAM_SMBUS_CHAN_J25_2,DRAM_SMBUS_DEV_J25_2)
	DRAM_CHAN_ODTCFG(0x00,0x00,0x00,0x00,0x00,ODT_150,ODT_75)
	DRAM_CHAN_ODTCFG2(0x00,0x00,0x00,0x00,0x00,ODT_150,ODT_75)
	DRAM_CHAN_ADDRCOARSE(0x0,0x0c)

	DRAM_EOT

END(board_draminfo)


/*  *********************************************************************
    *  BOARD_PIAI2_TXCHAR
    *  
    *  Transmit a single character via UART A
    *  
    *  Input parameters: 
    *  	   a0 - character to transmit (low-order 8 bits)
    *  	   
    *  Return value:
    *  	   nothing
    *  	   
    *  Registers used:
    *  	   t0,t1
    ********************************************************************* */

#ifdef _PROMICE_PORT_LEDS_

LEAF(board_piai2_txchar)

	# Wait until there is space in the transmit buffer

	        li      t0,PHYS_TO_K1(BOARD_PROMICE_BASE)

1:		lb	t1,BOARD_PROMICE_STATUS(t0)
		andi	t1,TDA
		bne	t1,zero,1b

	# Okay, now send the character.

		sb	a0,BOARD_PROMICE_ZERO(t0)

	# done!

		j	ra

END(board_piai2_txchar)
#endif

/*  *********************************************************************
    *  BOARD_UARTA_TXCHAR
    *  
    *  Transmit a single character via UART A
    *  
    *  Input parameters: 
    *  	   a0 - character to transmit (low-order 8 bits)
    *  	   
    *  Return value:
    *  	   nothing
    *  	   
    *  Registers used:
    *  	   t0,t1
    ********************************************************************* */

#ifdef _SERIAL_PORT_LEDS_
board_uarta_txchar:

	# Wait until there is space in the transmit buffer

1:		li	t0,PHYS_TO_K1(A_DUART_STATUS_A)
		ld	t1,(t0)			# Get status bits
		and	t1,M_DUART_TX_RDY	# test for ready
		beq	t1,0,1b			# keep going till ready

	# Okay, now send the character.

		li	t0,PHYS_TO_K1(A_DUART_TX_HOLD_A)
		sd	a0,(t0)

	# done!

		j	ra


#endif

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


#define LED_CHAR0	(32+8*3)
#define LED_CHAR1	(32+8*2)
#define LED_CHAR2	(32+8*1)
#define LED_CHAR3	(32+8*0)


LEAF(board_setleds)

	/*
	 * Sending to LEDs
	 */

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



#ifdef _SERIAL_PORT_LEDS_
		move	t3,ra
		move	t2,a0

		li	a0,'['
		bal	board_uarta_txchar

		move	a0,t2
		rol	a0,8
		bal	board_uarta_txchar
		rol	a0,8
		bal	board_uarta_txchar
		rol	a0,8
		bal	board_uarta_txchar
		rol	a0,8
		bal	board_uarta_txchar

		li	a0,']'
		bal	board_uarta_txchar
		li	a0,13
		bal	board_uarta_txchar
		li	a0,10
		bal	board_uarta_txchar

		move	a0,t2
		move	ra,t3

#endif

		j	ra

END(board_setleds)

LEAF(board_get_config)
		.set push
		.set noreorder
		.set nomacro
		lui	t0, %hi(PHYS_TO_K1(A_SCD_SYSTEM_CFG))
		ld	t0, %lo(PHYS_TO_K1(A_SCD_SYSTEM_CFG))(t0)
		dsrl	t0, t0, S_BCM1480_SYS_CONFIG
		li	v0, (M_BCM1480_SYS_CONFIG >> S_BCM1480_SYS_CONFIG)
		and	v0, v0, t0

		jr	ra
		 nop
		.set pop
END(board_get_config)



