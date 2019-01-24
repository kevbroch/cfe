/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: mousse_devs.c
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
#include "dev_newflash.h"

#include "mousse.h"


/*  *********************************************************************
    *  Devices we're importing
    ********************************************************************* */

extern cfe_driver_t ns16550_uart;		/* 16550 serial ports */
extern cfe_driver_t m48txx_nvram;		/* TOY clock & NVRAM */
extern cfe_driver_t newflashdrv;		/* AMD-style flash */

extern int  ui_init_moussecmds(void);
extern int  ui_init_flashtestcmds(void);
#if CFG_PCI
extern void mpc824x_pci_add_devices(int init);  /* driver collection du jour */
#endif



/*  *********************************************************************
    *  Some board-specific parameters
    ********************************************************************* */

static cfe_timer_t blinky_timer;    



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
    //cfe_startflags = 0;

    /* Console */
    cfe_add_device(&ns16550_uart,A_MOUSSE_SERIAL_1,18432000,0);
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

void board_device_init(void)
{
    newflash_probe_t bootflash;
#if 0

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

    cfe_add_device(&newflashdrv,0,0,&bootflash);
#else
    bootflash.flash_phys = 0xFFF00000;
    bootflash.flash_size = 2*1024*1024;
    bootflash.flash_flags = FLASH_FLG_BUS8|FLASH_FLG_DEV16|FLASH_FLG_MANUAL;
    bootflash.flash_type = FLASH_TYPE_FLASH;
    bootflash.flash_cmdset = FLASH_CFI_CMDSET_AMD_STD;
    bootflash.flash_nsectors = 1;
    bootflash.flash_sectors[0] = FLASH_SECTOR_RANGE(32,65536);
    bootflash.flash_nchips = 1;
    bootflash.flash_nparts = 0;
    bootflash.flash_ioctl_hook = 0;
    bootflash.flash_engine_hook = 0;

    cfe_add_device(&newflashdrv,0,0,&bootflash);

//    cfe_add_device(&newflashdrv,0xFFF00000,(2*1024*1024)|FLASH_FLG_BUS8|FLASH_FLG_DEV16,NULL);

#endif

    cfe_add_device(&m48txx_nvram,A_MOUSSE_TOYCLOCK,MOUSSE_TOYCLOCK_SIZE,0);
    cfe_set_envdevice("nvram0");
#if CFG_PCI
    mpc824x_pci_add_devices(cfe_startflags & CFE_INIT_PCI);
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
    volatile uint8_t *ptr = (volatile uint8_t *) (A_MOUSSE_PLD + R_MOUSSE_PLD_LED);

    if (TIMER_EXPIRED(blinky_timer)) {
	light = !light;
	*ptr = light ? M_MOUSSE_PLD_FLG : 0;
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
    ui_init_moussecmds();
    ui_init_flashtestcmds();
    cfe_bg_add(board_blinky,NULL);
    TIMER_SET(blinky_timer,CFE_HZ);

}

