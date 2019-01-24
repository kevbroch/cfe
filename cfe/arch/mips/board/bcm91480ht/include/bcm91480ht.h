/*  *********************************************************************
    *  BCM1280/BCM1480 Board Support Package
    *  
    *  BCM91480HT  Definitions  		   File: bcm91480ht.h
    *
    *  This file contains I/O, chip select, and GPIO assignments
    *  for the BCM1255/BCM1280/BCM1455/BCM1480 evaluation board.
    *  
    *  Author:  Binh Vo
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
 * I/O Address assignments for the bcm91480ht board
 *
 * Summary of address map:
 *
 * Address         Size   CSel    Description
 * --------------- ----   ------  --------------------------------
 * 0x1FC00000      4MB     CS0    Boot ROM
 * 0x1F800000      4MB     CS1    Alternate Boot ROM 
 *                         CS2    Unused
 * 0x100A0000      64KB    CS4    LED display
 * 0x100C0000      64KB    CS5    CPLD and Battery Backup Control
 *      		   CS6    Unused
 *		      	   CS7    Unused
 *
 * GPIO assignments
 *
 * GPIO#    Direction   Description
 * -------  ---------   ------------------------------------------
 * GPIO0    Input	HT2000 Fatal
 * GPIO1    Input	HT2000 Alert
 * GPIO2    Input	HT1000 CPU Reset
 * GPIO3    Output	PCIX_CLKSEL1
 * GPIO4    Output	PCIX_CLKSEL0
 * GPIO5    Input       Temperature Sensor Alert
 * GPIO6    Input	HT2000GEIRQ1
 * GPIO7    Input	HT2000GEIRQ2
 * GPIO8    Input	SPI0 Reset
 * GPIO9    Input	PCIE_IRQ0
 * GPIO10   Input	PCIE_IRQ1
 * GPIO11   Input	PCIE_IRQ2
 * GPIO12   Input	PCIE_IRQ3
 * GPIO13   Input	Power Fail
 * GPIO14   Input	Charger IRQ
 * GPIO15   Input	SPI1 Reset
 * 
 * SMBus assignments:
 *
 * Chan   Dev           Description
 * ----   ------        ------------------------------------------
 *  0     0x70          SMBus switch
 *  0     0x09		Charger 1759
 *  0	  0x44		A2D
 *  0	  0x		Battery pack
 *  0	  0x51		MC chan 2 / J25
 *  0	  0x52	        MC chan 2 / J5
 *  0     0x55	 	MC chan 3 / J25
 *  0     0x56		MC chan 3 / J5
 *  0	  0x57		MC chan 0 / J4
 *  0 	  0x53		MC chan 1 / J4	
 *  1     0x70          SMBus switch
 *  1     0x50          Microchip 24LC128 SMBus EEPROM	
 *  1     0x51          Microchip 24LC128 SMBus EEPROM
 *  1     0x4c          AD ADT7461AR Temperature Sensor
 *  1     0x68          ST Micro M41T81M Real-time clock
 *  1	  0x6E		CLK SYNTH 9FG104
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

#define GPIO_HT200O_FATAL	0
#define GPIO_HT2000_ALERT	1
#define GPIO_HT1000_RESET	2
#define GPIO_PCIX_CLKSEL0	3
#define GPIO_PCIX_CLKSEL1	4
#define GPIO_TEMP_SENSOR	5
#define GPIO_HT2000_IRQ1	6
#define GPIO_HT2000_IRQ2	7
#define GPIO_SPIO_RESET		8
#define GPIO_PCIE_IRQ3		9
#define GPIO_PCIE_IRQ2		10
#define GPIO_PCIE_IRQ1		11
#define GPIO_PCIE_IRQ0		12
#define GPIO_POWER_FAIL		13
#define GPIO_CHARGER_IRQ	14
#define GPIO_SPI1_RESET		15

#define M_GPIO_HT200O_FATAL	_SB_MAKEMASK1(GPIO_HT200O_FATAL)
#define M_GPIO_HT2000_ALERT	_SB_MAKEMASK1(GPIO_HT2000_ALERT)
#define M_GPIO_HT1000_RESET	_SB_MAKEMASK1(GPIO_HT1000_RESET)
#define M_GPIO_PCIX_CLKSEL0	_SB_MAKEMASK1(GPIO_PCIX_CLKSEL0)
#define M_GPIO_PCIX_CLKSEL1	_SB_MAKEMASK1(GPIO_PCIX_CLKSEL1)
#define M_GPIO_TEMP_SENSOR	_SB_MAKEMASK1(GPIO_TEMP_SENSOR)
#define M_GPIO_HT2000_IRQ1	_SB_MAKEMASK1(GPIO_HT2000_IRQ1)
#define M_GPIO_HT2000_IRQ2	_SB_MAKEMASK1(GPIO_HT2000_IRQ2)
#define M_GPIO_SPIO_RESET	_SB_MAKEMASK1(GPIO_SPIO_RESET)
#define M_GPIO_PCIE_IRQ3	_SB_MAKEMASK1(GPIO_PCIE_IRQ3)
#define M_GPIO_PCIE_IRQ2	_SB_MAKEMASK1(GPIO_PCIE_IRQ2)
#define M_GPIO_PCIE_IRQ1	_SB_MAKEMASK1(GPIO_PCIE_IRQ1)
#define M_GPIO_PCIE_IRQ0	_SB_MAKEMASK1(GPIO_PCIE_IRQ0)
#define M_GPIO_POWER_FAIL	_SB_MAKEMASK1(GPIO_POWER_FAIL)
#define M_GPIOCHARGER_IRQ_	_SB_MAKEMASK1(GPIO_CHARGER_IRQ)
#define M_GPIO_SPI1_RESET	_SB_MAKEMASK1(GPIO_SPI1_RESET)

/* Leave bidirectional pins in "input" state at boot. */

#define GPIO_OUTPUT_MASK (M_GPIO_PCIX_CLKSEL0 | \
                          M_GPIO_PCIX_CLKSEL1)
                          
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

/*
 * Battery backup settings: non-multiplexed, byte width, no parity, no ack
 *
 */
#define BATTERY_CS		5
#define BATTERY_PHYS		0x100C0000	/* same address as SWARM */
#define BATTERY_SIZE		NUM64K(4)
#define BATTERY_TIMING0		V_IO_ALE_WIDTH(4) | \
                                V_IO_ALE_TO_CS(2) | \
                                V_IO_CS_WIDTH(13) | \
                                V_IO_RDY_SMPLE(1)
#define BATTERY_TIMING1		V_IO_ALE_TO_WRITE(2) | \
                                V_IO_WRITE_WIDTH(8) | \
                                V_IO_IDLE_CYCLE(6) | \
                                V_IO_CS_TO_OE(0) | \
                                V_IO_OE_TO_CS(0)
#define BATTERY_CONFIG		V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_1) | M_IO_NONMUX



/*  *********************************************************************
    *  SMBus Devices
    ********************************************************************* */

#define TEMPSENSOR_SMBUS_CHAN	1
#define TEMPSENSOR_SMBUS_DEV	0x4c

#define BIGEEPROM_SMBUS_CHAN_1	1		/* This one is for CFE */
#define BIGEEPROM_SMBUS_DEV_1	0x51

#define BIGEEPROM_SMBUS_CHAN	1		/* This one is for customer use */
#define BIGEEPROM_SMBUS_DEV	0x50

#define M41T81_SMBUS_CHAN	1
#define M41T81_SMBUS_DEV	0x68

#define CLK_SYNTH_CHAN		1
#define CLK_SYNTH_DEV		0x6E

#define SMBUS_SWITCH_CHAN_0	0
#define SMBUS_SWITCH_DEV_0	0x70

#define SMBUS_SWITCH_CHAN_1	1
#define SMBUS_SWITCH_DEV_1	0x70

#define CHARGER_CHAN		0
#define CHARGER_DEV		0x09

#define A2D_CHAN		0
#define A2D_DEV			0x44

#define BATTERY_CHAN
#define BATTERY_DEV


#define DRAM_SMBUS_CHAN_J4_0		0
#define DRAM_SMBUS_DEV_J4_0		0x57
#define DRAM_SMBUS_CHAN_J4_1		0
#define DRAM_SMBUS_DEV_J4_1		0x53

#define DRAM_SMBUS_CHAN_J5_2		0
#define DRAM_SMBUS_DEV_J5_2		0x52
#define DRAM_SMBUS_CHAN_J5_3		0
#define DRAM_SMBUS_DEV_J5_3		0x56

#define DRAM_SMBUS_CHAN_J25_2		0
#define DRAM_SMBUS_DEV_J25_2		0x51
#define DRAM_SMBUS_CHAN_J25_3		0
#define DRAM_SMBUS_DEV_J25_3		0x55


/*  *********************************************************************
    *  FPGA defines
    ********************************************************************* */

/*
 * These registers are implemented on all BCM91480HT boards.
 */

#define FPGA_PHYS_SREG0		(FPGA_PHYS + 0)
#define FPGA_PHYS_SREG1		(FPGA_PHYS + 1)

#define FPGA_SREG0_SWM4		0x01
#define FPGA_SREG0_SWM5		0x02
#define FPGA_SREG0_SWM6		0x04
#define FPGA_SREG0_SWM7		0x08
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

#define BOARD_CFG_REV_MASK	0x03

#define BOARD_CFG_SWM0		0x04
#define BOARD_CFG_SWM1		0x08
#define BOARD_CFG_SWM2		0x10
#define BOARD_CFG_SWM3		0x20

#define BOARD_CFG_SWM4		(FPGA_SREG0_SWM4 << 8)
#define BOARD_CFG_SWM5		(FPGA_SREG0_SWM5 << 8)
#define BOARD_CFG_SWM6		(FPGA_SREG0_SWM6 << 8)
#define BOARD_CFG_SWM7		(FPGA_SREG0_SWM7 << 8)
#define BOARD_CFG_ROM_CS_SEL	(FPGA_SREG0_ROM_CS_SEL << 8)

#define BOARD_CFG_HT_EN_L0	(FPGA_SREG1_HT_EN_L0 << 16)
#define BOARD_CFG_HT_EN_L1	(FPGA_SREG1_HT_EN_L1 << 16)
#define BOARD_CFG_SPI_EN_L0	(FPGA_SREG1_SPI_EN_L0 << 16)
#define BOARD_CFG_SPI_EN_L1	(FPGA_SREG1_SPI_EN_L1 << 16)
#define BOARD_CFG_LVDS_EN_L0	(FPGA_SREG1_LVDS_EN_L0 << 16)
#define BOARD_CFG_LVDS_EN_L1	(FPGA_SREG1_LVDS_EN_L1 << 16)


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

#ifdef __LANGUAGE_C
int	board_get_config(void);
#endif




