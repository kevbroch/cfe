/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: bmw_devs.c
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
#include "env_subr.h"

#include "bmw.h"
#include "ppcdefs.h"
#include "dev_newflash.h"
#include "mpc824x.h"
#include "mpc8245_speed.h"

/*  *********************************************************************
    *  Devices we're importing
    ********************************************************************* */

extern cfe_driver_t ns16550_uart;		/* 16550 serial ports */
extern cfe_driver_t newflashdrv;		/* flash driver */
extern cfe_driver_t ds1743_nvram;		/* NVRAM */
extern cfe_driver_t ds1743_clock;		/* Time-of-day clock */

#if CFG_MSYS
extern cfe_driver_t bdkdrv;
#endif

extern char *vxboot_mac_addr;

extern int  ui_init_bmwcmds(void);
extern int  ui_init_resetcmds(void);
extern int  ui_init_toyclockcmds(void);
extern int  ui_init_flashtestcmds(void);
extern int  ui_init_disktestcmds(void);
extern int  ui_init_vxbootcmds(void);
#if CFG_PCI
extern void cpci_add_devices(int init);         /* driver collection du jour */
#endif
    
static cfe_timer_t blinky_timer;    

static mpc8245_speed_t mpc8245_speed;

/*  *********************************************************************
    *  Some board-specific parameters
    ********************************************************************* */


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
    /* Need CPU speed for UART clock */
    mpc8245_speed.sync_in_clk = 0;		/* guess the SYNC_IN clock */
    mpc8245_speeds(&mpc8245_speed);

    cfe_cpu_speed = mpc8245_speed.core_clk;	/* report speed via core clock */

    /* Console.  UART speed's BAUD_BASE is our sys_logic_clock */
    cfe_add_device(&ns16550_uart,A_BMW_SERIAL_0,mpc8245_speed.sys_logic_clk,0);
    cfe_set_console("uart0");

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

/*
 * JP1  BMW
 * 1 2 3
 * o-o o  = Boot from PLCC (512K AM29LV040B)
 * o o-o  = Boot from Intel 28F320J3A-110 (4MB)
 * 
 * Note: BCM98245CPCI has a 512K PLCC boot rom and a 2M (1M x 16)
 *       Intel AM29LV160DB flash chip. Access is mutually
 *       exclusive, as set by JP1.
 *
 * Define the config constant below to:
 * 
 * 1 = flash device corresponds to socket (PLCC)
 * 0 = flash device corresponds to 2MB AMD flash on board
 */

#define CFG_PLCC_FLASH 0       

void board_device_init(void)
{
    newflash_probe_t bootflash;

    printf("CPU Multiplier:  %d.%d\n",MULTWHOLE(mpc8245_speed.cpu_pllmult),
	   MULTFRAC(mpc8245_speed.cpu_pllmult));
    printf("Mem Multiplier:  %d.%d\n",MULTWHOLE(mpc8245_speed.mem_pllmult),
	   MULTFRAC(mpc8245_speed.mem_pllmult));
    printf("Sys logic clock: %dMHz\n",SPEED_MHZ(mpc8245_speed.sys_logic_clk));

#if CFG_PLCC_FLASH

    /*
     * Socketed flash is not CFI compatible.  Sector it manually.
     */
    bootflash.flash_phys = 0xFFF00000;
    bootflash.flash_size = 512*1024;
    bootflash.flash_flags = FLASH_FLG_BUS8|FLASH_FLG_DEV8|FLASH_FLG_MANUAL;
    bootflash.flash_type = FLASH_TYPE_FLASH;
    bootflash.flash_cmdset = FLASH_CFI_CMDSET_AMD_STD;
    bootflash.flash_nsectors = 1;
    bootflash.flash_sectors[0] = FLASH_SECTOR_RANGE(8,65536);
    bootflash.flash_nchips = 1;
    bootflash.flash_nparts = 0;
    bootflash.flash_ioctl_hook = 0;
    bootflash.flash_engine_hook = 0;
#else

    /*
     * The boot ROM actually starts below the start address of
     * the PowerPC core.  The first megabyte lives *below* the 
     * start address.  We'll partition the flash into two chunks
     * in case we want to store something like OS data in the flash.
     *	
     * XXX: the only reason we're manually sectoring is that newflash
     *      doesn't re-probe on open, so if you want to move the
     *      flash jumper from PLCC to onboard flash you'll lose.
     */

    bootflash.flash_phys = A_BMW_BOOTROM;
    bootflash.flash_size = BMW_BOOTROM_SIZE;
    bootflash.flash_flags = FLASH_FLG_BUS8|FLASH_FLG_DEV16|FLASH_FLG_MANUAL;
    bootflash.flash_type = FLASH_TYPE_FLASH;
    bootflash.flash_cmdset = FLASH_CFI_CMDSET_AMD_STD;
    bootflash.flash_nsectors = 4;
    bootflash.flash_sectors[0] = FLASH_SECTOR_RANGE(1,16384);
    bootflash.flash_sectors[1] = FLASH_SECTOR_RANGE(2,8192);
    bootflash.flash_sectors[2] = FLASH_SECTOR_RANGE(1,32768);
    bootflash.flash_sectors[3] = FLASH_SECTOR_RANGE(31,65536);
    bootflash.flash_nchips = 1;
    bootflash.flash_nparts = 2;
    bootflash.flash_parts[0].fp_size = 1*1024*1024;
    bootflash.flash_parts[0].fp_name = "os";
    bootflash.flash_parts[1].fp_size = 1*1024*1024;
    bootflash.flash_parts[1].fp_name = "boot";
    bootflash.flash_ioctl_hook = 0;
    bootflash.flash_engine_hook = 0;
#endif

    cfe_add_device(&newflashdrv,0,0,&bootflash);

    /*
     * M-Systems DoC
     */

#if CFG_MSYS
    cfe_add_device(&bdkdrv,A_BMW_TSOP_DOC,BMW_TSOP_DOC_SIZE,NULL);
#endif

    /*
     * Real-time clock and NVRAM
     */

    cfe_add_device(&ds1743_nvram,A_BMW_TOYCLOCK,A_BMW_VX_NVRAM,0);
    cfe_set_envdevice("nvram0");
    cfe_add_device(&ds1743_clock,A_BMW_TOYCLOCK,1,0);
    /* un write-protect the TOY clock and the NVRAM */
    *((volatile uint8_t *) (A_BMW_PLD + R_BMW_PLD_TODWP)) = 0;

    /* Set magic address for VxWorks boot */
    vxboot_mac_addr = (char*)A_BMW_VX_MAC;

    /*
     * PCI devices.
     */
#if CFG_PCI
    cpci_add_devices(cfe_startflags & CFE_INIT_PCI);
#endif


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

}



/*  *********************************************************************
    *  board_blinky()
    *  
    *  Blink the LED at 1HZ
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
static void board_blinky(void *arg)
{
    static int light = 0;
    volatile uint8_t *ptr = (volatile uint8_t *) (A_BMW_PLD + R_BMW_PLD_LED);

    if (TIMER_EXPIRED(blinky_timer)) {
	light = !light;
	*ptr = light ? M_BMW_PLD_FLG : 0;
	TIMER_SET(blinky_timer,CFE_HZ);
	}
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
    ui_init_bmwcmds();
    ui_init_resetcmds();
    ui_init_toyclockcmds();
    ui_init_flashtestcmds();
    ui_init_disktestcmds();
    ui_init_vxbootcmds();

    cfe_bg_add(board_blinky,NULL);
    TIMER_SET(blinky_timer,CFE_HZ);
}

