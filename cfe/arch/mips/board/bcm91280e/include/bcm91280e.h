/*  *********************************************************************
    *  BCM1280/BCM1480 Board Support Package
    *  
    *  BCM91280E  Definitions  		   	File: bcm91280e.h
    *
    *  This file contains I/O, chip select, and GPIO assignments
    *  for the BCM1255/BCM1280/BCM1455/BCM1480 evaluation board.
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2005
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
 * I/O Address assignments for the bcm91280e board
 *
 * Summary of address map:
 *
 * Address         Size   CSel    Description
 * --------------- ----   ------  --------------------------------
 * 0x1FC00000      4MB     CS0    Boot ROM
 * 0x1F800000      4MB     CS1    Alternate Boot ROM 
 *                         CS2    Unused
 * 		      	   CS3    Unused
 * 0x100A0000      64KB    CS4    LED display
 *                         CS5    Unused
 * 	  	  	   CS6    Unused
 * 			   CS7    Unused
 *
 * GPIO assignments (GPIOs 9-15 connected to DIP switches)
 *
 * GPIO#    Direction   Description
 * -------  ---------   ------------------------------------------
 * GPIO0   
 * GPIO1
 * GPIO2
 * GPIO3
 * GPIO4
 * GPIO5    Input       Temperature Sensor Alert
 * GPIO6 
 * GPIO7
 * GPIO8
 * GPIO9    Input       EXT_SWM8 
 * GPIO10   Input       EXT_SWM7
 * GPIO11   Input       EXT_SWM6
 * GPIO12   Input       EXT_SWM5
 * GPIO13   Input       EXT_SWM4
 * GPIO14   Input       ROM_CS_SEL
 * GPIO15   Input       BFWP_H
 * 
 * SMBus assignments:
 *
 * Chan   Dev           Description
 * ----   ------        ------------------------------------------
 *  0     0x50          Microchip 24LC128 SMBus EEPROM
 *  1     0x51          Microchip 24LC128 SMBus EEPROM
 *  1     0x26          AD ADT7461AR Temperature Sensor
 *  1     0x57          ST Micro M41T81M Real-time clock
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

#define GPIO_TEMP_SENSOR	5

#define M_GPIO_TEMP_SENSOR	_SB_MAKEMASK1(GPIO_TEMP_SENSOR)

/* Leave bidirectional pins in "input" state at boot. */

#define GPIO_OUTPUT_MASK (0)

#define GPIO_INTERRUPT_MASK (0)

/*  *********************************************************************
    *  Generic Bus 
    ********************************************************************* */

/*
 * Boot ROM:  non-multiplexed, byte width, no parity, no ack
 * XXX: These are the (very slow) default parameters.   This can be sped up!
 */
#define BOOTROM_CS		0
#define BOOTROM_PHYS		0x1FC00000	/* address of boot ROM (CS0) */
#define BOOTROM_SIZE		NUM64K(4*MB)	/* size of boot ROM */
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
 * Alternate Boot ROM:  non-multiplexed, byte width, no parity, no ack
 * XXX: These are the (very slow) default parameters.   This can be sped up!
 */
#define ALT_BOOTROM_CS		1
#define ALT_BOOTROM_PHYS	0x1F800000	/* address of alternate boot ROM (CS1) */
#define ALT_BOOTROM_SIZE	NUM64K(4*MB)	/* size of alternate boot ROM */
#define ALT_BOOTROM_TIMING0	V_IO_ALE_WIDTH(4) | \
                                V_IO_ALE_TO_CS(2) | \
                                V_IO_CS_WIDTH(24) | \
                                V_IO_RDY_SMPLE(1)
#define ALT_BOOTROM_TIMING1	V_IO_ALE_TO_WRITE(7) | \
                                V_IO_WRITE_WIDTH(7) | \
                                V_IO_IDLE_CYCLE(6) | \
                                V_IO_CS_TO_OE(0) | \
                                V_IO_OE_TO_CS(0)
#define ALT_BOOTROM_CONFIG	V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_1) | M_IO_NONMUX

/*
 * LEDs:  non-multiplexed, byte width, no parity, no ack
 *
 */
#define LEDS_CS			4
#define LEDS_PHYS		0x100A0000	/* same address as SWARM */
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
    *  SMBus Devices
    ********************************************************************* */

#define TEMPSENSOR_SMBUS_CHAN	1
#define TEMPSENSOR_SMBUS_DEV	0x4C

#define BIGEEPROM_SMBUS_CHAN_1	1		/* This one is for CFE */
#define BIGEEPROM_SMBUS_DEV_1	0x51

#define BIGEEPROM_SMBUS_CHAN	0		/* This one is for customer use */
#define BIGEEPROM_SMBUS_DEV	0x50

#define M41T81_SMBUS_CHAN	1
#define M41T81_SMBUS_DEV	0x57

/*  *********************************************************************
    *  Board revision numbers
    ********************************************************************* */

/* Maps from SYSTEM_CFG config[1:0] register to actual board rev #'s */

#define BOARD_REV_0	0


/*  *********************************************************************
    *  Board configuration switches
    ********************************************************************* */

/*
 * The board revision and configuration switches are in config[5:0]
 * They can be obtained by calling the board_get_config() function, 
 * which returns them as:
 *    return value bits [ 5: 0]:	config[5:0].
 */

#define BOARD_CFG_REV_MASK	0x03
#define BOARD_CFG_SWM0		0x04
#define BOARD_CFG_SWM1		0x08
#define BOARD_CFG_SWM2		0x10
#define BOARD_CFG_SWM3		0x20

#define BOARD_CFG_SWM4		0
#define BOARD_CFG_SWM5		0
#define BOARD_CFG_SWM6		0 
#define BOARD_CFG_SWM7		0
#define BOARD_CFG_SWM8		0

/* Console type: 1 bit for now, maybe 2 later.  */
#define	BOARD_CFG_CONS_MASK	BOARD_CFG_SWM0

#define	BOARD_CFG_CONS_UART0	0		/* UART0 */
#define	BOARD_CFG_CONS_PROMICE	BOARD_CFG_SWM0	/* PromICE */

/* Reserved: BOARD_CFG_SWM1 */

/* Set if PCI/HT should be initialized.  */
#define	BOARD_CFG_INIT_PCI	BOARD_CFG_SWM2

/* Set if STARTUP environment variable should be used.  */
#define BOARD_CFG_DO_STARTUP	BOARD_CFG_SWM3

/* Disable all but half of L2 cache.  */
#define	BOARD_CFG_HALF_L2	BOARD_CFG_SWM4

/* Disable all but two CPUs.  */
#define	BOARD_CFG_2CPU		BOARD_CFG_SWM5

/* Reserved: BOARD_CFG_SWM6 */

#ifdef __LANGUAGE_C
int	board_get_config(void);
#endif
