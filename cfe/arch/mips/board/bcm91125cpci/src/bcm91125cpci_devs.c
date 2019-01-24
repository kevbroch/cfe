/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: bcm91125cpci_devs.c
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
#include "sb1250_mac.h"

#include "bcm91125cpci.h"

#include "dev_newflash.h"

#include "cfe_loader.h"
#include "cfe_autoboot.h"

#include "jedec.h"

/*  *********************************************************************
    *  Devices we're importing
    ********************************************************************* */

extern cfe_driver_t sb1250_uart;		/* SB1250 serial ports */
extern cfe_driver_t sb1250_ether;		/* SB1250 MACs */

extern cfe_driver_t newflashdrv;		/* CFI flash */
extern cfe_driver_t pcmciadrv;			/* PCMCIA card */

#if CFG_PCI
/* No PCI device drivers in the default build. */
#endif

extern cfe_smbus_t sb1250_smbus;
extern cfe_driver_t smbus_24lc128;		/* Microchip EEPROM */
extern cfe_driver_t smbus_m41t81clock;		/* M41T81 SMBus RTC */

/*  *********************************************************************
    *  Commands we're importing
    ********************************************************************* */

extern int ui_init_soccmds(void);
extern int ui_init_testcmds(void);
extern int ui_init_resetcmds(void);
extern int ui_init_phycmds(void);
extern int ui_init_tempsensorcmds(void);
extern int ui_init_toyclockcmds(void);
extern int ui_init_memtestcmds(void);
extern int ui_init_flashtestcmds(void);
extern int ui_init_spdcmds(void);
extern int ui_init_disktestcmds(void);
extern int ui_init_vxbootcmds(void);

/*  *********************************************************************
    *  Some other stuff we use
    ********************************************************************* */

extern void sb1250_show_cpu_type(void);
extern int cfe_device_download(int boot, char *options);


/*  *********************************************************************
    *  SysConfig switch settings and related parameters
    ********************************************************************* */

int board_rev;
int config_switch;

#define CONFIG_STARTUP_ENV	       	0x01	/* switch 1 */
#define CONFIG_PCI_INIT			0x02	/* switch 2 */

static int64_t blinky_timer;			/* for blinky */

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

    /* Console */
    cfe_add_device(&sb1250_uart,A_DUART,0,0);

    /* 
     * Fill in board_rev, config_switch
     */
    board_rev = (G_SYS_CONFIG(syscfg) & 3);
    config_switch = (G_SYS_CONFIG(syscfg) >> 2) & 0x0f;

    cfe_startflags = 0;
    if (config_switch & CONFIG_PCI_INIT) {
	cfe_startflags = CFE_INIT_PCI;
	}

    cfe_set_console("uart0");

    /*
     * SMBus buses - need to be initialized before we attach
     * devices that use them.
     */

    cfe_add_smbus(&sb1250_smbus,A_SMB_BASE(0),0);
    cfe_add_smbus(&sb1250_smbus,A_SMB_BASE(1),0);
     
    /* 
     * NVRAM (environment variables)
     */

    cfe_add_device(&smbus_24lc128,EEPROM0_SMBUS_CHAN,EEPROM0_SMBUS_DEV,0);
    cfe_set_envdevice("eeprom0");	/* Connect NVRAM to 2nd 24lc128 */

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
    uint64_t gpio;
    newflash_probe_t fprobe;

    /*
     * Print out the board version number.
     */
    printf("%s board revision %d\n", CFG_BOARDNAME,board_rev);

    /*
     * BCM91125CPCI has a 512K PLCC boot ROM (AM29LV040B) and a
     * 16M (8M x 16) Intel 28F128J3 flash chip.  Boot mode is
     * selected by SW1, position 2, which is also connected to GPIO3.
     *
     * Sense the GPIO pin to configure the
     * flash or PLCC as flash0, so we can program either in place.
     */

    gpio = SBREADCSR(A_GPIO_READ);

    /* 
     * Boot ROM (16MB), using "new" flash driver.  Partition the flash.
     *
     * Partitions are as follows:
     *
     *	1MB   - "boot"	CFE
     *  7MB   - "os"	VxWorks image
     *  8MB  -  "filesys" - VxWorks filesystem
     */
    /*
     * "Alternate" boot ROM, the PLCC flash
     */

    if (gpio & M_GPIO_BOOT_MODE) {	/* 1 = onboard */
	memset(&fprobe,0,sizeof(fprobe));
	fprobe.flash_phys = BOOTROM_PHYS;
	fprobe.flash_size = BOOTROM_SIZE*__K64;
	fprobe.flash_flags =  FLASH_FLG_BUS8 | FLASH_FLG_DEV16;
	fprobe.flash_nparts = 2;
	fprobe.flash_parts[0].fp_size = 1*__MB;
	fprobe.flash_parts[0].fp_name = "boot";
	fprobe.flash_parts[1].fp_size = 15*__MB;
	fprobe.flash_parts[1].fp_name = "os";

	cfe_add_device(&newflashdrv,0,0,&fprobe);

	memset(&fprobe,0,sizeof(fprobe));
	fprobe.flash_phys = ALT_BOOTROM_PHYS;
	fprobe.flash_size = ALT_BOOTROM_SIZE*__K64;
	fprobe.flash_flags =  FLASH_FLG_BUS8 | FLASH_FLG_DEV8;
	fprobe.flash_nparts = 1;
	fprobe.flash_parts[0].fp_size = 512*__KB;
	fprobe.flash_parts[0].fp_name = "boot";
	cfe_add_device(&newflashdrv,0,0,&fprobe);
	}
    else {		/* boot from PLCC */
	memset(&fprobe,0,sizeof(fprobe));
	fprobe.flash_phys = BOOTROM_PHYS;
	fprobe.flash_size = ALT_BOOTROM_SIZE*__K64;
	fprobe.flash_flags =  FLASH_FLG_BUS8 | FLASH_FLG_DEV8;
	fprobe.flash_nparts = 1;
	fprobe.flash_parts[0].fp_size = 512*__KB;
	fprobe.flash_parts[0].fp_name = "boot";
	cfe_add_device(&newflashdrv,0,0,&fprobe);

	memset(&fprobe,0,sizeof(fprobe));
	fprobe.flash_phys = ALT_BOOTROM_PHYS;
	fprobe.flash_size = BOOTROM_SIZE*__K64;
	fprobe.flash_flags =  FLASH_FLG_BUS8 | FLASH_FLG_DEV16;
	fprobe.flash_nparts = 2;
	fprobe.flash_parts[0].fp_size = 1*__MB;
	fprobe.flash_parts[0].fp_name = "boot";
	fprobe.flash_parts[1].fp_size = 15*__MB;
	fprobe.flash_parts[1].fp_name = "os";

	cfe_add_device(&newflashdrv,0,0,&fprobe);
	}


    /* 
     * MACs - must init after environment, since the hw address is stored there.
     *
     */
    cfe_add_device(&sb1250_ether,A_MAC_BASE_0,0,env_getenv("ETH0_HWADDR"));

    /*
     * Reset the MAC address for MAC 1, since it's not hooked up to anything.
     * This prevents the OS from probing it.
     */
    SBWRITECSR(A_MAC_REGISTER(1,R_MAC_ETHERNET_ADDR),0);

    /*
     * PCMCIA support
     */

    cfe_add_device(&pcmciadrv,PCMCIA_PHYS,0,NULL);

    /*
     * Real-time clock 
     */
    cfe_add_device(&smbus_m41t81clock,M41T81_SMBUS_CHAN,M41T81_SMBUS_DEV,0);

    /*
     * PCI devices.
     */

#if CFG_PCI
    /* No device drivers in the default build. */
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
    /*Nothing to do. */
}


/*  *********************************************************************
    *  board_blinkylight(arg)
    *  
    *  Blink the LED once per second
    *  
    *  Input parameters: 
    *  	   arg - not used
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void board_blinkylight(void *arg)
{
    static int light = 0;
    intptr_t reg;

    if (TIMER_EXPIRED(blinky_timer)) {
	light = !light;
	reg = light ? A_GPIO_PIN_SET : A_GPIO_PIN_CLR;
	SBWRITECSR(reg,M_GPIO_DEBUG_LED);
	TIMER_SET(blinky_timer,CFE_HZ);
	}
}


/*  *********************************************************************
    *  board_setup_autoboot()
    *  
    *  Set up autoboot methods.  This routine sets up the list of 
    *  places to find a boot program.
    *  
    *  Input parameters: 
    *  	   nothing
    *  
    *  Return value:
    *  	   nothing
    ********************************************************************* */
static void board_setup_autoboot(void)
{
    /*
     * If you had partitioned your flash, you could boot from it like this:
     */

    cfe_add_autoboot(CFE_AUTOBOOT_RAW,0,"flash0.os","elf","raw",NULL);

    /*
     * Now try running a script (file containing CFE commands) from
     * the TFTP server.   Your DHCP server must set option 130
     * to contain the name of the script.  Option 130 gets stored
     * in "BOOT_SCRIPT" when a DHCP reply is received.
     */

    cfe_add_autoboot(CFE_AUTOBOOT_NETWORK,LOADFLG_BATCH,
		     "eth0","raw","tftp","$BOOT_SERVER:$BOOT_SCRIPT");

    /*
     * Finally, try loading whatever the DHCP server says is the boot
     * program.  Do this as an ELF file, and failing that, try a
     * raw binary.
     */

    cfe_add_autoboot(CFE_AUTOBOOT_NETWORK,0,"eth0","elf","tftp",NULL);
    cfe_add_autoboot(CFE_AUTOBOOT_NETWORK,0,"eth0","raw","tftp",NULL);

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

    ui_init_soccmds();
    ui_init_testcmds();
    ui_init_resetcmds();
    ui_init_tempsensorcmds();
    ui_init_toyclockcmds();
    ui_init_memtestcmds();
    ui_init_phycmds();
    ui_init_flashtestcmds();
    ui_init_spdcmds();
    ui_init_disktestcmds();
    ui_init_vxbootcmds();

    cfe_bg_add(board_blinkylight,NULL);
    TIMER_SET(blinky_timer,CFE_HZ);
    
    board_setup_autoboot();

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
