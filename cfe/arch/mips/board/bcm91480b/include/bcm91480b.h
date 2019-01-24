/*  *********************************************************************
    *  BCM1280/BCM1480 Board Support Package
    *  
    *  BCM91480B  Definitions  		   	File: bcm91480b.h
    *
    *  This file contains I/O, chip select, and GPIO assignments
    *  for the BCM1255/BCM1280/BCM1455/BCM1480 evaluation board.
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
 * I/O Address assignments for the bcm91480b board
 *
 * Summary of address map:
 *
 * Address         Size   CSel    Description
 * --------------- ----   ------  --------------------------------
 * 0x1FC00000      4MB     CS0    Boot ROM
 * 0x1F800000      4MB     CS1    Alternate Boot ROM 
 *                         CS2    Unused
 * 0x100A0000      64KB    CS3    LED display
 * 0x100C0000      64KB    CS4    FPGA Settings
 *                         CS5    Unused
 * 0x11000000      64MB    CS6    PCMCIA #0
 * 0x15000000      64MB    CS7    PCMCIA #1
 *
 * GPIO assignments
 *
 * GPIO#    Direction   Description
 * -------  ---------   ------------------------------------------
 * GPIO0    Output      PCMCIA_MODE0
 * GPIO1    Output	PCMCIA_MODE1
 * GPIO2    Output	PCMCIA_MODE2
 * GPIO3    Output	PCIX_CLKSEL1
 * GPIO4    Output	PCIX_CLKSEL0
 * GPIO5    Input       Temperature Sensor Alert
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
 *  0     0x50          Microchip 24LC128 SMBus EEPROM
 *  1     0x51          Microchip 24LC128 SMBus EEPROM
 *  1     0x4C          AD ADT7461AR Temperature Sensor
 *  1     0x68          ST Micro M41T81M Real-time clock
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

#define GPIO_PCMCIA_MODE0	0
#define GPIO_PCMCIA_MODE1	1
#define GPIO_PCMCIA_MODE2	2
#define GPIO_PCIX_CLKSEL0	3
#define GPIO_PCIX_CLKSEL1	4
#define GPIO_TEMP_SENSOR	5

#define M_GPIO_PCMCIA_MODE0	_SB_MAKEMASK1(GPIO_PCMCIA_MODE0)
#define M_GPIO_PCMCIA_MODE1	_SB_MAKEMASK1(GPIO_PCMCIA_MODE1)
#define M_GPIO_PCMCIA_MODE2	_SB_MAKEMASK1(GPIO_PCMCIA_MODE2)
#define M_GPIO_PCIX_CLKSEL0	_SB_MAKEMASK1(GPIO_PCIX_CLKSEL0)
#define M_GPIO_PCIX_CLKSEL1	_SB_MAKEMASK1(GPIO_PCIX_CLKSEL1)
#define M_GPIO_TEMP_SENSOR	_SB_MAKEMASK1(GPIO_TEMP_SENSOR)

/* Leave bidirectional pins in "input" state at boot. */

#define GPIO_OUTPUT_MASK (M_GPIO_PCIX_CLKSEL0 | \
                          M_GPIO_PCIX_CLKSEL1 | \
                          M_GPIO_PCMCIA_MODE0 | \
                          M_GPIO_PCMCIA_MODE1 | \
                          M_GPIO_PCMCIA_MODE2)

#define GPIO_INTERRUPT_MASK (0)

/* used for resetting */
#define M_GPIO_PCIX_FREQALL (M_GPIO_PCIX_CLKSEL0 | \
                          M_GPIO_PCIX_CLKSEL1)

/* 
 * Frequency Settings:
 *   P1 P0   Frequency
 *    0 0    33MHz
 *    0 1    66MHz
 *    1 0    100MHz
 *    1 1    133MHz
 */

#define M_GPIO_PCIX_FREQ133 (M_GPIO_PCIX_CLKSEL0 | \
                          M_GPIO_PCIX_CLKSEL1)

#define M_GPIO_PCIX_FREQ100 (M_GPIO_PCIX_CLKSEL1)

#define M_GPIO_PCIX_FREQ66 (M_GPIO_PCIX_CLKSEL0)

#define M_GPIO_PCIX_FREQ33 (0)


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
#define LEDS_CS			3
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

/*
 * FPGA settings: non-multiplexed, byte width, no parity, no ack
 *
 */
#define FPGA_CS			4
#define FPGA_PHYS		0x100C0000	/* same address as SWARM */
#define FPGA_SIZE		NUM64K(4)
#define FPGA_TIMING0		V_IO_ALE_WIDTH(4) | \
                                V_IO_ALE_TO_CS(2) | \
                                V_IO_CS_WIDTH(13) | \
                                V_IO_RDY_SMPLE(1)
#define FPGA_TIMING1		V_IO_ALE_TO_WRITE(2) | \
                                V_IO_WRITE_WIDTH(8) | \
                                V_IO_IDLE_CYCLE(6) | \
                                V_IO_CS_TO_OE(0) | \
                                V_IO_OE_TO_CS(0)
#define FPGA_CONFIG		V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_1) | M_IO_NONMUX


/*
 * PCMCIA0: this information was derived from section 16
 */
#define PCMCIA0_CS		6
#define PCMCIA0_PHYS		0x11000000
#define PCMCIA0_SIZE		NUM64K(64*MB)
#define PCMCIA0_TIMING0		V_IO_ALE_WIDTH(3) | \
                                V_IO_ALE_TO_CS(1) | \
                                V_IO_CS_WIDTH(17) | \
                                V_IO_RDY_SMPLE(1)
#define PCMCIA0_TIMING1		V_IO_ALE_TO_WRITE(8) | \
                                V_IO_WRITE_WIDTH(8) | \
                                V_IO_IDLE_CYCLE(2) | \
                                V_IO_CS_TO_OE(0) | \
                                V_IO_OE_TO_CS(0)
#define PCMCIA0_CONFIG		V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_2)

/*
 * PCMCIA1: this information was derived from section 16
 */
#define PCMCIA1_CS		7
#define PCMCIA1_PHYS		0x15000000
#define PCMCIA1_SIZE		NUM64K(64*MB)
#define PCMCIA1_TIMING0		V_IO_ALE_WIDTH(3) | \
                                V_IO_ALE_TO_CS(1) | \
                                V_IO_CS_WIDTH(17) | \
                                V_IO_RDY_SMPLE(1)
#define PCMCIA1_TIMING1		V_IO_ALE_TO_WRITE(8) | \
                                V_IO_WRITE_WIDTH(8) | \
                                V_IO_IDLE_CYCLE(2) | \
                                V_IO_CS_TO_OE(0) | \
                                V_IO_OE_TO_CS(0)
#define PCMCIA1_CONFIG		V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_2)


/*  *********************************************************************
    *  PCMCIA mode
    ********************************************************************* */

/* Use one-slot mode for now.  See sb1250_genbus.h for list of modes */
/* #define BCM91480_PCMCIA_MODE	K_PCMCIA_MODE_PCMA_NOB */		/* one-slot */
#define BCM91480_PCMCIA_MODE	K_PCMCIA_MODE_PCMA_PCMB		/* two-slots */

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
#define M41T81_SMBUS_DEV	0x68

/*
 * 64-bit memory channel 0,2 -> SMBus devices 0x54, 0x55
 * 64-bit memory channel 1,3 -> SMBus devices 0x56, 0x57
 */

#define DRAM_SMBUS_CHAN		0
#define DRAM_SMBUS_DEV		0x54


/*  *********************************************************************
    *  FPGA defines
    ********************************************************************* */

/*
 * These registers are implemented on all BCM91480B boards, and should be
 * implemented on BCM91480A rev 2 and later.
 */

#define FPGA_PHYS_SREG0		(FPGA_PHYS + 0)
#define FPGA_PHYS_SREG1		(FPGA_PHYS + 1)

#define FPGA_SREG0_SWM4		0x01
#define FPGA_SREG0_SWM5		0x02
#define FPGA_SREG0_SWM6		0x04
#define FPGA_SREG0_BWFP_H	0x08
#define FPGA_SREG0_ROM_CS_SEL	0x10			/* set -> ROM */
/*	reserved		0x20 */
/*	reserved		0x40 */
/*	reserved		0x80 */

#define FPGA_SREG1_HT_EN_L0	0x01
#define FPGA_SREG1_HT_EN_L1	0x02
#define FPGA_SREG1_SPI_EN_L0	0x04
#define FPGA_SREG1_SPI_EN_L1	0x08
#define FPGA_SREG1_LVDS_EN_L0	0x10
#define FPGA_SREG1_LVDS_EN_L1	0x20
/*	reserved		0x40 */
/*	reserved		0x80 */


/*  *********************************************************************
    *  Board revision numbers
    ********************************************************************* */

/* Maps from SYSTEM_CFG config[1:0] register to actual board rev #'s */

#define BOARD_REV_1	0


/*  *********************************************************************
    *  Board configuration switches
    ********************************************************************* */

/*
 * The board revision and configuration switches are in config[5:0]
 * and in two registers in the FPGA.  They can be obtained by calling the
 * bcm91480a_get_config() function, which returns them as:
 *    return value bits [ 5: 0]:	config[5:0]
 *    return value bits [15: 8]:	FPGA reg 0
 *    return value bits [23:16]:	FPGA reg 1
 */

#define BOARD_CFG_REV_MASK	0x03
#define BOARD_CFG_SWM0		0x04
#define BOARD_CFG_SWM1		0x08
#define BOARD_CFG_SWM2		0x10
#define BOARD_CFG_SWM3		0x20

#ifdef _BCM91480B_

#define BOARD_CFG_SWM4		(FPGA_SREG0_SWM4 << 8)
#define BOARD_CFG_SWM5		(FPGA_SREG0_SWM5 << 8)
#define BOARD_CFG_SWM6		(FPGA_SREG0_SWM6 << 8)
#define BOARD_CFG_BWFP_H	(FPGA_SREG0_BFWP_H << 8)
#define BOARD_CFG_ROM_CS_SEL	(FPGA_SREG0_ROM_CS_SEL << 8)

#define BOARD_CFG_HT_EN_L0	(FPGA_SREG1_HT_EN_L0 << 16)
#define BOARD_CFG_HT_EN_L1	(FPGA_SREG1_HT_EN_L1 << 16)
#define BOARD_CFG_SPI_EN_L0	(FPGA_SREG1_SPI_EN_L0 << 16)
#define BOARD_CFG_SPI_EN_L1	(FPGA_SREG1_SPI_EN_L1 << 16)
#define BOARD_CFG_LVDS_EN_L0	(FPGA_SREG1_LVDS_EN_L0 << 16)
#define BOARD_CFG_LVDS_EN_L1	(FPGA_SREG1_LVDS_EN_L1 << 16)

#endif /* _BCM91480B_ */

#ifdef _BCM91480A_
#define BOARD_CFG_SWM4		0
#define BOARD_CFG_SWM5		0
#define BOARD_CFG_SWM6		0 

#endif /* _BCM91480A_ */

/* Console type: 1 bit for now, maybe 2 later.  */
#define	BOARD_CFG_CONS_MASK	BOARD_CFG_SWM0

#define	BOARD_CFG_CONS_UART0	0		/* UART0 */
#define	BOARD_CFG_CONS_PROMICE	BOARD_CFG_SWM0	/* PromICE */

/* HT node identification: 1 bit for now.  */
#define BOARD_CFG_NODE_ID	BOARD_CFG_SWM1

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
