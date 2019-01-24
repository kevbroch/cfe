/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: bcm91480ht_devs.c
    *  
    *  This is the "C" part of the board support package.  The
    *  routines to create and initialize the console, wire up 
    *  device drivers, and do other customization live here.
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


#include "cfe.h"
#include "cfe_smbus.h"
#include "env_subr.h"

#include "bcm1480_regs.h"
#include "bcm1480_scd.h"
#include "bcm1480_mc.h"
#include "sb1250_genbus.h"
#include "lib_physio.h"

#include "bcm91480ht.h"

#include "dev_newflash.h"

/*  *********************************************************************
    *  Devices we're importing
    ********************************************************************* */

extern cfe_driver_t promice_uart;		/* promice serial port */
extern cfe_driver_t sb1250_uart;		/* SB1250 serial ports */
extern cfe_driver_t sb1250_ether;		/* SB1250 MACs */
extern cfe_driver_t newflashdrv;		/* AMD-style flash */
#if CFG_PCI
extern void pci_add_devices(int init);
#endif
#if CFG_TCP
extern cfe_driver_t tcpconsole;			/* TCP/IP console */
#endif

extern cfe_smbus_t sb1250_smbus;       		/* SiByte SMBus host */

extern cfe_driver_t smbus_24lc128;		/* Microchip EEPROM */
extern cfe_driver_t smbus_m41t81clock;		/* ST Micro clock */
extern cfe_driver_t smbus_switch;		/* PCA9543A SMBus switch */


/*  *********************************************************************
    *  Commands we're importing
    ********************************************************************* */

extern int ui_docommands(char *);

extern void ui_init_cpu1cmds(void);
extern void ui_init_bcm91480htcmds(void);
extern int ui_init_soccmds(void);
extern int ui_init_testcmds(void);
extern int ui_init_toyclockcmds(void);
extern int ui_init_tempsensorcmds(void);
extern int ui_init_memtestcmds(void);
extern int ui_init_resetcmds(void);
extern int ui_init_phycmds(void);
extern int ui_init_spdcmds(void);
extern int ui_init_disktestcmds(void);
extern int ui_init_ethertestcmds(void);
extern int ui_init_flashtestcmds(void);
#if CFG_LDT
extern int ui_init_pmcmds(void);
extern int ui_init_swtrccmds(void);
extern int ui_init_ccncmds(void);
#endif

/*  *********************************************************************
    *  Some other stuff we use
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

#define REAL_BOOTROM_SIZE	(2*1024*1024)	/* region is 4MB, but rom is 2MB */

/*  *********************************************************************
    *  SysConfig switch settings and related parameters
    ********************************************************************* */

static int board_config;
static int board_rev;

#define PROMICE_CONSOLE		0x00000001



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
    uint64_t sysrev;
    uint64_t syscfg;

    sysrev = SBREADCSR(A_SCD_SYSTEM_REVISION);
    syscfg = SBREADCSR(A_SCD_SYSTEM_CFG);

    /* Console */
    cfe_add_device(&sb1250_uart,A_BCM1480_DUART(0),0,0);
    cfe_add_device(&promice_uart,PROMICE_BASE,PROMICE_WORDSIZE,0);

    /*
     * Read the config switch and decide how we are going to set up
     * the console.
     *
     * Note that the human-readable board revision is the revision
     * encoded by the revision bits + 1.
     */
    board_config = board_get_config();
    board_rev = (board_config & BOARD_CFG_REV_MASK) + 1;

    cfe_startflags = 0;

    /* Set up CFE start flags based on config switch */

    switch (board_config & BOARD_CFG_CONS_MASK) {
    case BOARD_CFG_CONS_UART0:
    default:
	break;

    case BOARD_CFG_CONS_PROMICE:
	cfe_startflags |= PROMICE_CONSOLE;
	break;
    }

    /* XXX This is a kludge.

       There are not enough config switch bits to deal with HT modes
       and node configuration options.  However, if the node ID is set
       to non-zero, HT DMA becomes non-coherent and existing CFE
       drivers will not work for devices behind the PLX bridge.

       For now, if CFG_INIT_PCI is set, leave the node ID as zero 
       and auto-configure PCI.  Otherwise, assign the node ID 
       according to CFG_NODE_ID and rely on an explicit node_enable 
       call to initialize PCI/HT.

       Unfortunately, late initialization does not install the 
       drivers for a device attached to the otherwise usable
       PCI-X interface.
    */
       
    if ((board_config & BOARD_CFG_INIT_PCI) != 0) {
	cfe_startflags |= CFE_INIT_PCI;
	}
    else {
	unsigned int node_id = (board_config & BOARD_CFG_NODE_ID ? 5 : 4);

	syscfg &= ~M_BCM1480_SYS_NODEID;
	syscfg |= V_BCM1480_SYS_NODEID(node_id);

	/* XXX workaround: write value to SCD before updating sys_cfg */
	SBWRITECSR(A_BCM1480_SCD_SCRATCH, syscfg);

	SBWRITECSR(A_SCD_SYSTEM_CFG, syscfg);
	}

    /* Configure console */

    if (cfe_startflags & PROMICE_CONSOLE) {
	cfe_set_console("promice0");
	}
    else {
	cfe_set_console("uart0");
	}

    /*
     * SMBus buses - need to be initialized before we attach
     * devices that use them.
     */

    cfe_add_smbus(&sb1250_smbus,A_SMB_BASE(0),0);
    cfe_add_smbus(&sb1250_smbus,A_SMB_BASE(1),0);
     
    /* 
     * NVRAM (environment variables)
     */

    cfe_add_device(&smbus_24lc128,BIGEEPROM_SMBUS_CHAN_1,BIGEEPROM_SMBUS_DEV_1,0);
    cfe_set_envdevice("eeprom0");	/* Connect NVRAM subsystem to EEPROM */

    cfe_add_device(&smbus_switch,SMBUS_SWITCH_CHAN_0,SMBUS_SWITCH_DEV_0,0);

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
    uint64_t syscfg;
    uint64_t mcreg;

    syscfg = SBREADCSR(A_SCD_SYSTEM_CFG);

    /*
     * Print out the board version number and config straps
     */

    printf("%s board revision %d\n",CFG_BOARDNAME,board_rev);
    printf("%s configuration switches: 0x%x\n",CFG_BOARDNAME,board_config);

    /* Enable token ring timeouts.  By default, references to
       unimplemented south ring addresses will give no response and
       thus hang the CPU. */
    phys_write64(A_BCM1480_NC_SR_TIMEOUT_COUNTER_SEL, 0x1);
    
    /* Similarly for node controller timeouts. */
    phys_write64(A_BCM1480_NC_TIMEOUT_COUNTER_SEL, 0x1);
    
    /*
     * UART channel B on primary DUART
     */

    cfe_add_device(&sb1250_uart,A_BCM1480_DUART(0),1,0);

    /*
     * UARTs on second DUART, if enabled
     */

    if (syscfg & M_BCM1480_SYS_DUART1_ENABLE) {
	cfe_add_device(&sb1250_uart,A_BCM1480_DUART(2),0,0);
	cfe_add_device(&sb1250_uart,A_BCM1480_DUART(2),1,0);
	}

#ifndef _FUNCSIM_
    /* 
     * Boot ROM, using "new" flash driver
     */

    cfe_add_device(&newflashdrv,
		   BOOTROM_PHYS,
		   REAL_BOOTROM_SIZE | FLASH_FLG_BUS8 | FLASH_FLG_DEV16,
		   NULL);
    cfe_add_device(&newflashdrv,
		   ALT_BOOTROM_PHYS,
		   REAL_BOOTROM_SIZE | FLASH_FLG_BUS8 | FLASH_FLG_DEV16,
		   NULL);
#endif /*_FUNCSIM_*/

    /*
     * This is the 24LC128 on SMBus0.  CFE doesn't use it for anything,
     * but you can load data into it and then boot from it by changing a jumper.
     */

    cfe_add_device(&smbus_24lc128,BIGEEPROM_SMBUS_CHAN,BIGEEPROM_SMBUS_DEV,0);

    /* 
     * MACs - must init after environment, since the hw address is stored there 
     */

#if (!CFG_BOOTRAM && !CFG_L2_RAM)
    cfe_add_device(&sb1250_ether,A_MAC_BASE_0,0,env_getenv("ETH0_HWADDR"));
    cfe_add_device(&sb1250_ether,A_MAC_BASE_1,1,env_getenv("ETH1_HWADDR"));
    cfe_add_device(&sb1250_ether,A_MAC_BASE_2,2,env_getenv("ETH2_HWADDR"));
#endif
    cfe_add_device(&sb1250_ether,A_MAC_BASE_3,3,env_getenv("ETH3_HWADDR"));

#if CFG_PCI
    pci_add_devices(cfe_startflags & CFE_INIT_PCI);
#endif

    /*
     * Real-time clock
     */

    cfe_add_device(&smbus_m41t81clock,M41T81_SMBUS_CHAN,M41T81_SMBUS_DEV,0);

    /*
     * Display config register and CPU type
     */

    sb1250_show_cpu_type();

    mcreg = SBREADCSR(A_BCM1480_MC_REGISTER(0,R_BCM1480_MC_CLOCK_CFG));
    if (G_BCM1480_MC_CLK_RATIO(mcreg)) {
	printf("Memory controller #0: %dMHz\n",
	       (cfe_cpu_speed / 2000000) * 4 / ((int)G_BCM1480_MC_CLK_RATIO(mcreg)));
	}

    mcreg = SBREADCSR(A_BCM1480_MC_REGISTER(1,R_BCM1480_MC_CLOCK_CFG));
    if (G_BCM1480_MC_CLK_RATIO(mcreg)) {
	printf("Memory controller #1: %dMHz\n",
	       (cfe_cpu_speed / 2000000) * 4 / ((int)G_BCM1480_MC_CLK_RATIO(mcreg)));
	}

    printf("Switch Clock: %dMHz\n",
	   (cfe_cpu_speed * 2 / 1000000) / ((int)G_BCM1480_SYS_SW_DIV(syscfg)));

    if (G_BCM1480_SYS_NODEID(syscfg) != 0) {
	printf("Node Id: %d\n", G_BCM1480_SYS_NODEID(syscfg));
	}

    /*
     * Some misc devices go here, mostly for playing.
     */

#if CFG_TCP
    cfe_add_device(&tcpconsole,0,0,0);
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

    ui_init_cpu1cmds();
    ui_init_bcm91480htcmds();
    ui_init_soccmds();
    ui_init_testcmds();
    ui_init_toyclockcmds();
    ui_init_tempsensorcmds();
    ui_init_memtestcmds();
    ui_init_resetcmds();
    ui_init_phycmds();
    ui_init_spdcmds();
    ui_init_disktestcmds();
    ui_init_ethertestcmds();
    ui_init_flashtestcmds();
#if CFG_LDT
    ui_init_pmcmds();
    ui_init_swtrccmds();
    ui_init_ccncmds();
#endif

    if ((board_config & BOARD_CFG_DO_STARTUP) != 0) {
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


