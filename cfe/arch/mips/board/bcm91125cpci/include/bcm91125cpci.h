/*  *********************************************************************
    *  SB1250 Board Support Package
    *  
    *  BCM91125CPCI  Definitions    		File: bcm91125cpci.h
    *
    *  This file contains I/O, chip select, and GPIO assignments
    *  for the BCM91125CPCI board.
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
 * I/O Address assignments for the bcm91125cpci board
 *
 * Summary of address map:
 *
 * Address         Size   CSel    Description
 * --------------- ----   ------  --------------------------------
 * 0x1FC00000      16MB    CS0    Boot ROM
 * 0x1EC00000	   16MB    CS1	  Alternate Boot ROM
 * 			   CS2	  Unused
 * 		           CS3    Unused
 * 0x1D0A0000      64KB    CS4    LED display
 *                         CS5    Unused
 * 0x11000000      64KB    CS6    CompactFlash/PCMCIA
 *                         CS7    Unused
 *
 * GPIO assignments
 *
 * GPIO#    Direction   Description
 * -------  ---------   ------------------------------------------
 * GPIO0    Output      Debug LED
 * GPIO1    Input       Boot Mode                   
 * GPIO2    Input       NMI                         (interrupt)
 * GPIO3    Input       RTC Interrupt		    (interrupt)
 * GPIO4    Input       PHY Interrupt		    (interrupt)
 * GPIO5    Input       Temperature Sensor Alert    (interrupt)
 * GPIO6    N/A         PCMCIA interface
 * GPIO7    N/A         PCMCIA interface
 * GPIO8    N/A         PCMCIA interface
 * GPIO9    N/A         PCMCIA interface
 * GPIO10   N/A         PCMCIA interface
 * GPIO11   N/A         PCMCIA interface
 * GPIO12   N/A         PCMCIA interface
 * GPIO13   N/A         PCMCIA interface
 * GPIO14   N/A         PCMCIA interface
 * GPIO15   N/A         PCMCIA interface
 * 
 * SMBus assignments:
 *
 * Chan   Dev           Description
 * ----   ------        ------------------------------------------
 *  0     0x2A          Temperature Sensor 
 *  1     0x68          ST Micro M41T81 Real-time clock
 *  0     0x54		SPD for DIMM slot 0
 *  0     0x50          Microchip 24LC128 SMBus EEPROM (400KHz capable)
 */

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#define __MB (1024*1024)
#define __KB (1024)
#define __K64 65536
#define NUM64K(x) (((x)+(__K64-1))/__K64)


/*  *********************************************************************
    *  GPIO pins
    ********************************************************************* */

#define GPIO_DEBUG_LED		0
#define GPIO_BOOT_MODE		1
#define GPIO_NONMASKABLE_INT	2
#define GPIO_RTC_INTERRUPT	3
#define GPIO_PHY_INTERRUPT	4
#define GPIO_TEMP_SENSOR_INT	5

#define M_GPIO_DEBUG_LED	_SB_MAKEMASK1(GPIO_DEBUG_LED)
#define M_GPIO_BOOT_MODE	_SB_MAKEMASK1(GPIO_BOOT_MODE)

#define GPIO_INTERRUPT_MASK ((V_GPIO_INTR_TYPEX(GPIO_PHY_INTERRUPT,K_GPIO_INTR_LEVEL)))

#define GPIO_OUTPUT_MASK (M_GPIO_DEBUG_LED)

/*  *********************************************************************
    *  Generic Bus 
    ********************************************************************* */

/*
 * Boot ROM:  non-multiplexed, byte width, no parity, no ack
 * XXX: These are the (very slow) default parameters.   This can be sped up!
 * Boot rom: 16MB Intel StrataFlash
 */
#define BOOTROM_CS		0
#define BOOTROM_PHYS		0x1FC00000	/* address of boot ROM (CS0) */
#define BOOTROM_SIZE		NUM64K(16*__MB)	/* size of boot ROM */
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
 * Boot rom: 512KB AMD PLCC
 */
#define ALT_BOOTROM_CS		1
#define ALT_BOOTROM_PHYS	0x1EC00000	/* address of alternate boot ROM (CS1) */
#define ALT_BOOTROM_SIZE	NUM64K(512*__KB)	/* size of alternate boot ROM */
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
 */
#define LEDS_CS			4
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
                              

/*
 * PCMCIA: this information was derived from chapter 12, table 12-5
 */
#define PCMCIA_CS		6
#define PCMCIA_PHYS		0x11000000
#define PCMCIA_SIZE		NUM64K(64*__MB)
#define PCMCIA_TIMING0		V_IO_ALE_WIDTH(3) | \
                                V_IO_ALE_TO_CS(1) | \
                                V_IO_CS_WIDTH(17) | \
                                V_IO_RDY_SMPLE(1)
#define PCMCIA_TIMING1		V_IO_ALE_TO_WRITE(8) | \
                                V_IO_WRITE_WIDTH(8) | \
                                V_IO_IDLE_CYCLE(2) | \
                                V_IO_CS_TO_OE(0) | \
                                V_IO_OE_TO_CS(0)
#define PCMCIA_CONFIG		V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_2)

                              
/*  *********************************************************************
    *  SMBus 
    ********************************************************************* */

#define TEMPSENSOR_SMBUS_CHAN	0
#define TEMPSENSOR_SMBUS_DEV	0x2A
#define EEPROM0_SMBUS_CHAN	0
#define EEPROM0_SMBUS_DEV	0x50
#define M41T81_SMBUS_CHAN	1     
#define M41T81_SMBUS_DEV	0x68	      

#define SODIMM1_SMBUS_CHAN	0
#define SODIMM1_SMBUS_DEV	0x54
