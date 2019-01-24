/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: bcm91125c_devs.c
    *  
    *  This is the "C" part of the board support package.  The
    *  routines to create and initialize the console, wire up 
    *  device drivers, and do other customization live here.
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
#include "sbmips.h"
#include "env_subr.h"
#include "cfe_smbus.h"

#include "sb1250_defs.h"
#include "sb1250_regs.h"
#include "sb1250_scd.h"
#include "sb1250_smbus.h"

#include "bsp_config.h"

#include "bcm91125pcix.h"

#include "dev_newflash.h"

/*  *********************************************************************
    *  Devices we're importing
    ********************************************************************* */

extern cfe_driver_t promice_uart;		/* promice serial port */
extern cfe_driver_t sb1250_uart;		/* SB1250 serial ports */
extern cfe_driver_t sb1250_ether;		/* SB1250 MACs */

extern cfe_smbus_t sb1250_smbus;
extern cfe_driver_t smbus_24lc128;		/* Microchip EEPROM */
extern cfe_driver_t smbus_m41t81clock;		/* M41T81 SMBus RTC */

extern cfe_driver_t newflashdrv;		/* AMD-style flash */
extern cfe_driver_t pcmciadrv;			/* PCMCIA card */

#if CFG_PCI
extern int clkgen_init(void);			/* Init Cypress CLK generator */
extern int ui_init_clkgencmds(void);
extern void pci_add_devices(int init);          /* driver collection du jour */
#endif

extern int ui_init_bcm91125ccmds(void);
extern int ui_init_corecmds(void);
extern int ui_init_soccmds(void);
extern int ui_init_testcmds(void);
extern int ui_init_disktestcmds(void);
extern int ui_init_tempsensorcmds(void);
extern int ui_init_toyclockcmds(void);
extern int ui_init_memtestcmds(void);
extern int ui_init_resetcmds(void);
extern int ui_init_spdcmds(void);
extern int ui_init_flashtestcmds(void);
extern int ui_init_ethertestcmds(void);
extern int ui_init_phycmds(void);

/*  *********************************************************************
    *  Other externs
    ********************************************************************* */

extern void sb1250_show_cpu_type(void);

/*  *********************************************************************
    *  Some board-specific parameters
    ********************************************************************* */

/*
 * Note!  Configure the PROMICE for burst mode zero (one byte per
 * access).
 */

#define PROMICE_BASE	(0x1FDFFC00)
#define PROMICE_WORDSIZE 1

/*  *********************************************************************
    *  SysConfig switch settings and related parameters
    ********************************************************************* */

int board_rev;
int config_switch;

#define CONFIG_CONSOLE_MASK		0x03 /* SW10 switch 1 and 2 */  
#define CONFIG_UART_CONSOLE		0x00
#define CONFIG_PROMICE_CONSOLE		0x01
#define CONFIG_CONSOLE_RESERVED_0	0x02
#define CONFIG_CONSOLE_RESERVED_1	0x03

#define CONFIG_PCI_INIT			0x04 /* SW10 switch 3 */
#define CONFIG_STARTUP_ENV		0x08 /* SW10 switch 4 */

#define CONFIG_RESERVED_0		0x10 /* SW10 switch 5 */
#define CONFIG_RESERVED_1		0x20 /* SW10 switch 6 */

/*  *********************************************************************
    *  board_console_init()
    *  
    *  Add the console device and set it to be the primary
    *  console.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void board_console_init(void)
{
    uint64_t syscfg = SBREADCSR(A_SCD_SYSTEM_CFG);

    /*
     * Read the config switch and decide how we are going to set up
     * the console.  This is actually board revision dependent.
     */
    board_rev = 0;		/* all the config bits go to switches */
    config_switch = G_SYS_CONFIG(syscfg) & 0x3F;

    cfe_startflags = 0;
    if (config_switch & CONFIG_PCI_INIT)
	cfe_startflags = CFE_INIT_PCI;

    /* Initialize console */
    cfe_add_device(&sb1250_uart,A_DUART,0,0);
    cfe_add_device(&promice_uart,PROMICE_BASE,PROMICE_WORDSIZE,0);

    switch (config_switch & CONFIG_CONSOLE_MASK) {
	case CONFIG_PROMICE_CONSOLE:
	    cfe_set_console("promice0");
	    break;
	case CONFIG_UART_CONSOLE:
	default:
	    cfe_set_console("uart0");
	    break;
	}

    /*
     * SMBus buses - need to be initialized before we attach
     * devices that use them.
     */

    cfe_add_smbus(&sb1250_smbus,A_SMB_BASE(0),0);
    cfe_add_smbus(&sb1250_smbus,A_SMB_BASE(1),0);
     
    /*
     * NVRAM
     */
    cfe_add_device(&smbus_24lc128,BIGEEPROM0_SMBUS_CHAN,BIGEEPROM0_SMBUS_DEV,0);
    cfe_set_envdevice("eeprom0");	/* Connect NVRAM to 24lc128 */

#if 0
    /*
     * Program Clock Generator
     */
    if (config_switch & CONFIG_PCI_INIT)     
	clkgen_init();
#endif

}

/*  *********************************************************************
    *  board_device_init()
    *  
    *  Initialize and add other devices.  Add everything you need
    *  for bootstrap here, like disk drives, flash memory, UARTs,
    *  network controllers, etc.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void board_device_init(void)
{
    /*
     * Print out the board version number.
     */
    printf("%s board revision %d\n", CFG_BOARDNAME,board_rev+1);

    /*
     * UART channel B
     */
    cfe_add_device(&sb1250_uart,A_DUART,1,0);

    /* 
     * Boot ROM 
     */
    cfe_add_device(&newflashdrv,
		   BOOTROM_PHYS,
		   (BOOTROM_SIZE*K64) | FLASH_FLG_BUS8 | FLASH_FLG_DEV16,
		   NULL);
    cfe_add_device(&newflashdrv,
		   ALT_BOOTROM_PHYS,
		   (ALT_BOOTROM_SIZE*K64) | FLASH_FLG_BUS8 | FLASH_FLG_DEV16,
		   NULL);

    /*
     * TOY clock
     */
    cfe_add_device(&smbus_m41t81clock,M41T81_SMBUS_CHAN,M41T81_SMBUS_DEV,0);
       
    /* 
     * MACs - must init after environment, since the hw address is stored there 
     */
    cfe_add_device(&sb1250_ether,A_MAC_BASE_0,0,env_getenv("ETH0_HWADDR"));
    cfe_add_device(&sb1250_ether,A_MAC_BASE_1,1,env_getenv("ETH1_HWADDR"));

    /*
     * PCMCIA support
     */

    cfe_add_device(&pcmciadrv,PCMCIA_PHYS,0,NULL);

#if CFG_PCI
    pci_add_devices(cfe_startflags & CFE_INIT_PCI);
#endif

    /*
     * Set variable that contains CPU speed, spit out config register
     */
    printf("Config switch: %d\n", config_switch);

    sb1250_show_cpu_type();

}



/*  *********************************************************************
    *  board_device_reset()
    *  
    *  Reset devices.  This call is done when the firmware is restarted,
    *  as might happen when an operating system exits, just before the
    *  "reset" command is applied to the installed devices.   You can
    *  do whatever board-specific things are here to keep the system
    *  stable, like stopping DMA sources, interrupts, etc.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void board_device_reset(void)
{

    /* Nothing to do. */
}


/*  *********************************************************************
    *  board_final_init()
    *  
    *  Do any final initialization, such as adding commands to the
    *  user interface.
    *
    *  If you don't want a user interface, put the startup code here.  
    *  This routine is called just before CFE starts its user interface.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void board_final_init(void)
{

    int flag;
    
    ui_init_bcm91125ccmds();
    ui_init_corecmds();
    ui_init_soccmds();
    ui_init_testcmds();
    ui_init_disktestcmds();
    ui_init_toyclockcmds();
    ui_init_tempsensorcmds();
    ui_init_memtestcmds();
    ui_init_resetcmds();
    ui_init_spdcmds();
    ui_init_flashtestcmds();
    ui_init_ethertestcmds();
    ui_init_phycmds();
    ui_init_clkgencmds();

    if (config_switch & CONFIG_STARTUP_ENV) {
	/* Change STARTUP's flags so it can run or error message if not set */
	if (env_getenv("STARTUP") == NULL) {
	    printf("*** STARTUP environment variable not set.\n\n");
	}
	else {
	    flag = env_envtype("STARTUP");
	    flag &= ~ENV_FLG_STARTUP_NORUN;
	    env_setflags("STARTUP",flag);
	    }
	}
    else {
	if (env_getenv("STARTUP") != NULL) {
	    /* Don't run the commands in STARTUP */
	    flag = env_envtype("STARTUP");
	    flag |= ENV_FLG_STARTUP_NORUN;
	    env_setflags("STARTUP",flag);
	    }
	}
}

