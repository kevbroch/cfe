/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: bcm95836cpci_devs.c
    *  
    *  This is the "C" part of the board support package.  The
    *  routines to create and initialize the console, wire up 
    *  device drivers, and do other customization live here.
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2004
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
#include "bcmnvram.h"

#include "sbmips32.h"
#include "sb_bp.h"

#include "dev_newflash.h"
#include "sb_utils.h"


/*  *********************************************************************
    *  Devices we're importing
    ********************************************************************* */

extern cfe_driver_t ns16550_uart;
extern cfe_driver_t sb_mac;
extern cfe_driver_t newflashdrv;

#if CFG_PCI
extern void cpci_add_devices(int init_pci);
#endif

extern cfe_driver_t ds1743_nvram;
extern cfe_driver_t ds1743_clock;

extern void ui_init_bcm95836cpcicmds(void);
extern int ui_init_toyclockcmds(void);
extern void ui_init_flashtestcmds(void);

#if CFG_USB
static void board_usb_init(void);
extern int usb_init(uint32_t addr);
extern cfe_driver_t usb_disk;
extern int ui_init_usbcmds(void);
#endif

extern void board_setleds(uint32_t);
void board_led_msg(char *msg);

/*  *********************************************************************
    *  Some board-specific parameters
    ********************************************************************* */

typedef struct initenv_s {
    const char *name;
    const char *value;
    const char *def;
} initenv_t;

/* Note: et1 MAC port is not brought out on current revs */
const initenv_t bcm95836cpci_envvars[] = {
    {"et0macaddr","$$NVRAM","02-10-18-58-36-10"},
    {"et1macaddr","$$NVRAM","02-10-18-58-36-11"},
    {"et0phyaddr","2",NULL},
    {"et1phyaddr","1",NULL},
    {"et0mdcport","0",NULL},
    {"et1mdcport","1",NULL},
    {"boardtype","bcm95836cpci",NULL},
    {NULL,NULL}};


/*  *********************************************************************
    *  board_led_msg()
    *  
    *   Write characters to OSRAM 4-char LED display.
    *  
    *  Input parameters: 
    *  	  String of 1 character or more in length
    *  	   
    *  Return value:
    *  	   nothing
    ******************************************************************** */
void board_led_msg(char * msg)
{
    int len = strlen(msg);
    uint32_t a0;
    int i;

    for (a0 = 0,i = 0; i < 4; i++) {
	a0 <<= 8;
	a0 |= (i < len) ? msg[i] : ' ';
	}
    board_setleds(a0);
}


/*  *********************************************************************
    *  bcm95836cpci_initenv()
    *  
    *  Initialize default environment variables.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
static void bcm95836cpci_initenv(void)
{
    const initenv_t *ini;
    char *txt;

    ini = bcm95836cpci_envvars;

    /* Assign either the forced value, or the value
       of another environment variable if the name starts
       with a dollar sign.  If that name is not defined
       either, then use the default from the table. */

    while (ini->name) {
	if (strcmp(ini->value, "$$NVRAM") == 0) {
	    txt = (char *)nvram_get(ini->name);
	    if (!txt) txt = (char *) ini->def;
	    }
	else if (ini->value[0] == '$') {
	    txt = env_getenv(&(ini->value[1]));
	    if (!txt) txt = (char *) ini->def;
	    }
	else {
	    txt = (char *) ini->value;
	    }
	if (txt) {
	    if (!env_getenv(ini->name)) {
		env_setenv(ini->name,txt,ENV_FLG_BUILTIN);
		}
	    }
	ini++;
	}

}


/*  *********************************************************************
    *  bcm95836cpci_probeflash()
    *  
    *  Probe the flash and initialize as required.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    *
    * Note: BCM95836CPCI has a 512K PLCC boot rom and a 16M (8M x 16)
    *       Intel 28F128J3A flash chip.  However, accesses to flash 
    *       using CS4, required for the nvram partition, are limited
    *       to 8M and other operating systems using that partition
    *       are currently limited to 4M.  We therefore size each
    *       partition of the Intel flash to reside in the low 4M.
    ********************************************************************* */

#define BOOT_SPACE   (512*KB)
#define TRX_SPACE    (1*KB)     /* Must hold a trx_hdr structure */
#define OS_SPACE     ((4*MB) - BOOT_SPACE - TRX_SPACE - NVRAM_SPACE)
#define NVRAM_OFFSET (BOOT_SPACE + TRX_SPACE + OS_SPACE)

static void bcm95836cpci_probeflash(void)
{
#ifdef INCLUDE_FLASH_DRIVERS
    newflash_probe_t fprobe;

    /*
     * JP1001 BCM95836CPCI
     * 1 2
     * o o (open)   = Boot from PLCC (512K AM29LV040B)
     * o-o (closed) = Boot from Intel 28F128J3A-110 (16MB)
     * 
     * To put CFE on the Intel flash, burn cfe.srec into PLCC,
     * move jumper JP1001 from pins 2-3 to jump pins 1-2, and
     * then flash cfe.flash into it.
     *
     * We configure 4 flash devices on the BCM95836CPCI:
     *
     *	flash0:	boot flash (PLCC or Intel flash, per jumper above), 512kB. 
     *	flash1:	Intel flash, 8MB (boot/trx/os/nvram partitions).
     *	flash2:	Intel flash, 8MB (boot/trx/nvram partitions).
     *	flash3:	Intel flash, 8MB (single partition).
     */

    memset(&fprobe,0,sizeof(fprobe));

    /* We configure flash0 w/o partitions so that the "flash" UI command
     * will find it correctly by default.  Ugh.
     */
    fprobe.flash_phys = 0x1FC00000;	/* boot flash.  */
    fprobe.flash_size = 512 * KB;   /*REAL_BOOTROM_SIZE;*/
    fprobe.flash_flags = FLASH_FLG_BUS8 | FLASH_FLG_DEV8;
    fprobe.flash_nparts = 0;
    cfe_add_device(&newflashdrv, 0, 0, &fprobe);	/* flash0 */

    memset(&fprobe,0,sizeof(fprobe));

    /* following bcm947xx_devs.c as modified by the switch group */
    fprobe.flash_phys = 0x1B000000;    /* shadow (Intel) flash at CS4 */
    fprobe.flash_size = 16 * MB;
    fprobe.flash_flags = FLASH_FLG_BUS8 | FLASH_FLG_DEV16;

    /* Because CFE can only boot from the beginning of a partition */
    fprobe.flash_nparts = 4;
    fprobe.flash_parts[0].fp_size = BOOT_SPACE;
    fprobe.flash_parts[0].fp_name = "boot";
    fprobe.flash_parts[1].fp_size = TRX_SPACE;   /* currently 7 words used */
    fprobe.flash_parts[1].fp_name = "trx";
    fprobe.flash_parts[2].fp_size = OS_SPACE;
    fprobe.flash_parts[2].fp_name = "os";
    fprobe.flash_parts[3].fp_size = NVRAM_SPACE;
    fprobe.flash_parts[3].fp_name = "nvram";
    cfe_add_device(&newflashdrv, 0, 0, &fprobe);	/* flash1 */

    /* Because CFE can only flash an entire partition */
    fprobe.flash_nparts = 3;
    fprobe.flash_parts[0].fp_size = BOOT_SPACE;
    fprobe.flash_parts[0].fp_name = "boot";
    fprobe.flash_parts[1].fp_size = TRX_SPACE + OS_SPACE;
    fprobe.flash_parts[1].fp_name = "trx";
    fprobe.flash_parts[2].fp_size = NVRAM_SPACE;
    fprobe.flash_parts[2].fp_name = "nvram";
    cfe_add_device(&newflashdrv, 0, 0, &fprobe);	/* flash2 */

    /* Because sometimes we want to program the entire device */
    fprobe.flash_nparts = 0;
    cfe_add_device(&newflashdrv, 0, 0, &fprobe);	/* flash3 */

    board_led_msg("FLASH");
    
#endif /* INCLUDE_FLASH_DRIVERS */
}


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
    uint32_t uart_clock;

    uart_clock = sb_uart_clock();

    /* This hack prevents a divider value of 0 in the UART registers.  It's
       not obviously the right thing to do here. */
    if (uart_clock < (CFG_SERIAL_BAUD_RATE*16))
	uart_clock = 16*CFG_SERIAL_BAUD_RATE;

    /* Console on BCM95836cpci is COM1, note: no HW Flow control */
    cfe_add_device(&ns16550_uart, BCM95836_COM1, uart_clock, 0);
    cfe_set_console("uart0");

    /* Auxiliary UART */
    cfe_add_device(&ns16550_uart,BCM95836_COM2,uart_clock,0);

    /* Make nvram variables available early.  These are located in
     * the flash1/flash2 "nvram" partition.
     */
    nvram_init((uint8_t *)PHYS_TO_K1(0x1B000000 + NVRAM_OFFSET), NVRAM_SPACE);

#if CFG_PCI
    cfe_startflags |= CFE_INIT_PCI;
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
    /* NVRAM is DS1743 TOD/NVRAM (Battery backed SRAM) */
    cfe_add_device(&ds1743_nvram,
		   BCM95836_CPCI_NVRAM_ADDR,
		   BCM95836_CPCI_NVRAM_SIZE,0);
    cfe_set_envdevice("nvram0");
    cfe_add_device(&ds1743_clock, BCM95836_CPCI_NVRAM_ADDR, 0, 0);

    /* We would like the environment variables to be set up early.
     * For compatibility with HND conventions, some of the device
     * attach routines, e.g, for the MACs, may interrogate the
     * environment.
     */

    bcm95836cpci_probeflash();
    bcm95836cpci_initenv();

    cfe_add_device(&sb_mac, 0, 2, env_getenv("et0macaddr"));
#if 0   /* MAC 1 is not brought out */
    cfe_add_device(&sb_mac, 1, 1, env_getenv("et1macaddr"));
#endif

#if CFG_USB
    board_usb_init();
    usb_init(SB_USB_BASE);
    cfe_add_device(&usb_disk,0,0,0);
#endif

#if CFG_PCI
    /* We are a PCI host in a CPCI system.  Most of the supported PCI
       devices are not currently available for CPCI; we link only
       a subset. */
    cpci_add_devices(1);
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

#if CFG_USB
/*  *********************************************************************
    *  board_usb_init()
    *  
    *  Turn on the OHCI controller at the core.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
static void board_usb_init(void)
{
    volatile uint32_t *reg;

    /* host mode (M_SBTS_UH=1) sequence taken from the BCM4710 UM */

    reg = (volatile uint32_t *) PHYS_TO_K1(SB_USB_BASE + R_SBTMSTATELOW);

    *reg = M_SBTS_RS | M_SBTS_CE | M_SBTS_FC | M_SBTS_UH;
    cfe_usleep(100);
    *reg = M_SBTS_CE | M_SBTS_FC | M_SBTS_UH;
    cfe_usleep(100);
    *reg = M_SBTS_CE | M_SBTS_UH;
    cfe_usleep(100);
}
#endif


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
    ui_init_bcm95836cpcicmds();
    ui_init_toyclockcmds();
    ui_init_flashtestcmds();

#if CFG_USB
    ui_init_usbcmds();
#endif
    board_led_msg("CFE ");
}
