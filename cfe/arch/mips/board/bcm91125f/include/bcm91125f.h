/*  *********************************************************************
    *  SB1250 Board Support Package
    *  
    *  BCM91125F  Definitions    		File: bcm91125f.h
    *
    *  This file contains I/O, chip select, and GPIO assignments
    *  for the BCM912500E checkout board.
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

/*
 * I/O Address assignments for the bcm91125f board
 *
 * Summary of address map:
 *
 * Address         Size   CSel    Description
 * --------------- ----   ------  --------------------------------
 * 0x1FC00000      2MB     CS0    Boot ROM
 * 			   CS1	  Unused
 * 			   CS2	  Unused
 * 			   CS3    Unused
 *                         CS4    Unused
 *                         CS5    Unused
 * 0x1D0A0000      64KB    CS6    LED dsplay
 *                         CS7    Unused
 *
 * GPIO assignments
 *
 * GPIO#    Direction   Description
 * -------  ---------   ------------------------------------------
 * GPIO0    Output	Debug LED
 * GPIO1    Input	RTC_OUT
 * GPIO2    		Unused
 * GPIO3    		Unused
 * GPIO4    		Unused
 * GPIO5    		Unused
 * GPIO6    		Unused
 * GPIO7    Input       PHY Interrupt               (interrupt)
 * GPIO8    Input       Nonmaskable Interrupt       (interrupt)
 * GPIO9    Input       Temperature Sensor Alert    (interrupt)
 * GPIO10   		Unused
 * GPIO11   		Unused
 * GPIO12   		Unused
 * GPIO13   		Unused
 * GPIO14   		Unused
 * GPIO15   Input	Switch SW4 dip8
 */

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#define MB (1024*1024)
#define K64 65536
#define NUM64K(x) (((x)+(K64-1))/K64)


/*  *********************************************************************
    *  GPIO pins
    ********************************************************************* */

#define GPIO_DEBUG_LED		0
#define GPIO_RTC_OUT		1
#define GPIO_BOARD_VER_0	2
#define GPIO_BOARD_VER_1	3
#define GPIO_PHY_INTERRUPT	7
#define GPIO_NONMASKABLE_INT	8
#define GPIO_TEMP_SENSOR_INT	9
#define GPIO_DIP_SWITCH		15

#define M_GPIO_DEBUG_LED	_SB_MAKEMASK1(GPIO_DEBUG_LED)
#define M_GPIO_RTC_OUT		_SB_MAKEMASK1(GPIO_RTC_OUT)
#define M_GPIO_BOARD_VER_0	_SB_MAKEMASK1(GPIO_BOARD_VER_0)
#define M_GPIO_BOARD_VER_1	_SB_MAKEMASK1(GPIO_BOARD_VER_1)

#define GPIO_BOARD_VER_MASK (M_GPIO_BOARD_VER_0 | M_GPIO_BOARD_VER_1)

#define GPIO_INTERRUPT_MASK ((V_GPIO_INTR_TYPEX(GPIO_PHY_INTERRUPT,K_GPIO_INTR_LEVEL)))

#define GPIO_OUTPUT_MASK (M_GPIO_DEBUG_LED | GPIO_BOARD_VER_MASK)

/*  *********************************************************************
    *  Generic Bus 
    ********************************************************************* */

/*
 * Boot ROM:  non-multiplexed, byte width, no parity, no ack
 * XXX: These are the (very slow) default parameters.   This can be sped up!
 */
#define BOOTROM_CS		0
#define BOOTROM_PHYS		0x1FC00000	/* address of boot ROM (CS0) */
#define BOOTROM_SIZE		NUM64K(16*MB)	/* size of boot ROM */
#define BOOTROM_TIMING0		V_IO_ALE_WIDTH(4) | \
                                V_IO_ALE_TO_CS(2) | \
                                V_IO_CS_WIDTH(24) | \
                                V_IO_RDY_SMPLE(1)
#define BOOTROM_TIMING1		V_IO_ALE_TO_WRITE(7) | \
                                V_IO_WRITE_WIDTH(7) | \
                                V_IO_IDLE_CYCLE(6) | \
                                V_IO_CS_TO_OE(0) | \
                                V_IO_OE_TO_CS(0)
#define BOOTROM_CONFIG		V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_1) | M_IO_NONMUX

/*
 * LEDs:  non-multiplexed, byte width, no parity, no ack
 */
#define LEDS_CS			6
#define LEDS_PHYS		0x1D0A0000
#define LEDS_SIZE		NUM64K(4)
#define LEDS_TIMING0		V_IO_ALE_WIDTH(4) | \
                                V_IO_ALE_TO_CS(2) | \
                                V_IO_CS_WIDTH(13) | \
                                V_IO_RDY_SMPLE(1)
#define LEDS_TIMING1		V_IO_ALE_TO_WRITE(2) | \
                                V_IO_WRITE_WIDTH(8) | \
                                V_IO_IDLE_CYCLE(6) | \
                                V_IO_CS_TO_OE(0) | \
                                V_IO_OE_TO_CS(0)
#define LEDS_CONFIG		V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_1) | M_IO_NONMUX
                              
/*  *********************************************************************
    *  SMBus 
    ********************************************************************* */
#define TEMPSENSOR_SMBUS_CHAN	0
#define TEMPSENSOR_SMBUS_DEV	0x2A
#define BIGEEPROM0_SMBUS_CHAN	0
#define BIGEEPROM0_SMBUS_DEV	0x50
#define SPDEEPROM_SMBUS_CHAN	0
#define SPDEEPROM_SMBUS_DEV	0x54

#define M41T81_SMBUS_CHAN	1     
#define M41T81_SMBUS_DEV	0x68	      
