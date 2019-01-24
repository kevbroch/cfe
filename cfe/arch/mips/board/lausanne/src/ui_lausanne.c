/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  LAUSANNE-specific commands		      File: ui_lausanne.c
    *  
    *  A temporary sandbox for misc test routines and commands.
    *  
    *  Author:  Mitch Lichtenberg
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
#include "ui_command.h"
#if CFG_PCI
#include "pcivar.h"
#endif

#include "sbmips.h"
#include "sb1250_regs.h"
#include "sb1250_genbus.h"
#include "lausanne.h"


/*  *********************************************************************
    *  Configuration
    ********************************************************************* */

#ifndef DEBUG_CS
#define DEBUG_CS  0
#endif


/*  *********************************************************************
    *  prototypes
    ********************************************************************* */

int ui_init_swarmcmds(void);

int ui_init_cpldcmds(void);
static int ui_cmd_writecpld(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_readcpld(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_burstwcpld(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_burstrcpld(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_config_flash(ui_cmdline_t *cmd,int argc,char *argv[]);

#if CFG_PCI
static int ui_cmd_map_pci(ui_cmdline_t *cmd,int argc,char *argv[]);
#endif

#if CFG_VGACONSOLE
static int ui_cmd_vgadump(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_vgainit(ui_cmdline_t *cmd,int argc,char *argv[]);
extern int vga_biosinit(void);
extern void vgaraw_dump(char *tail);
#endif

static int burst_mode(unsigned int cs, int burst_on);
static int intel_flash_data_width_mode(unsigned int cs, int flash_16bit);


/*  *********************************************************************
    *  Data
    ********************************************************************* */

 
/*  *********************************************************************
    *  ui_init_swarmcmds()
    *  
    *  Add SWARM-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */


int ui_init_swarmcmds(void)
{
#if CFG_PCI
    cmd_addcmd("map pci",
	       ui_cmd_map_pci,
	       NULL,
	       "Define a BAR0 window available to PCI devices",
	       "map pci offset size paddr [-off] [-l2ca] [-matchbits]\n\n"
	       "Map the region of size bytes starting at paddr to appear\n"
	       "at offset relative to BAR0\n",
	       "-off;Remove the region|"
	       "-l2ca;Make L2 cachable|"
	       "-matchbits;Use match bits policy");
#endif
#if CFG_VGACONSOLE
    cmd_addcmd("vga init",
	       ui_cmd_vgainit,
	       NULL,
	       "Initialize the VGA adapter.",
	       "vgainit",
	       "");

    cmd_addcmd("vga dumpbios",
	       ui_cmd_vgadump,
	       NULL,
	       "Dump the VGA BIOS to the console",
	       "vga dumpbios",
	       "");
#endif


    return 0;
}


#if CFG_PCI
static uint64_t parse_hex(const char *num)
{
    uint64_t x = 0;
    unsigned int digit;

    if ((*num == '0') && (*(num+1) == 'x')) num += 2;

    while (*num) {
        if ((*num >= '0') && (*num <= '9')) {
            digit = *num - '0';
            }
        else if ((*num >= 'A') && (*num <= 'F')) {
            digit = 10 + *num - 'A';
            }
        else if ((*num >= 'a') && (*num <= 'f')) {
            digit = 10 + *num - 'a';
            }
        else {
            break;
            }
        x *= 16;
        x += digit;
        num++;
        }

    return x;
}

static int ui_cmd_map_pci(ui_cmdline_t *cmd,int argc,char *argv[])
{
    unsigned long offset, size;
    uint64_t paddr;
    int l2ca, endian;
    int enable;
    int result;

    enable = !cmd_sw_isset(cmd, "-off");
    if (enable) {
	offset = parse_hex(cmd_getarg(cmd, 0));
	size = parse_hex(cmd_getarg(cmd, 1));
	paddr = parse_hex(cmd_getarg(cmd, 2));
	l2ca = cmd_sw_isset(cmd,"-l2ca");
	endian = cmd_sw_isset(cmd, "-matchbits");
	result = pci_map_window(paddr, offset, size, l2ca, endian);
	}
    else {
	offset = parse_hex(cmd_getarg(cmd, 0));
	size = parse_hex(cmd_getarg(cmd, 1));
	result = pci_unmap_window(offset, size);
	}

    return result;
}
#endif


#if CFG_VGACONSOLE
static int ui_cmd_vgainit(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int res;

    res = vga_biosinit();

    xprintf("vgaraw_init returns %d\n",res);
    
    return 0;
}

static int ui_cmd_vgadump(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;

    x = cmd_getarg(cmd,0);
    if (!x) x = "";

    vgaraw_dump(x);
    
    return 0;
}
#endif


int ui_init_cpldcmds(void)
{

    cmd_addcmd("write cpld",
	       ui_cmd_writecpld,
	       NULL,
	       "Write bytes to the cpld",
	       "write cpld",
	       "");

    cmd_addcmd("read cpld",
	       ui_cmd_readcpld,
	       NULL,
	       "Read bytes from the cpld",
	       "read cpld",
	       "");

    cmd_addcmd("burstw cpld",
	       ui_cmd_burstwcpld,
	       NULL,
	       "Write to the cpld in 8-bit BURST MODE",
	       "burstw cpld",
	       "");

    cmd_addcmd("burstr cpld",
	       ui_cmd_burstrcpld,
	       NULL,
	       "Read from the cpld in 8-bit BURST MODE",
	       "burstr cpld",
	       "");

    cmd_addcmd("config flash",
	       ui_cmd_config_flash,
	       NULL,
	       "Configure width and burst mode for the Intel flash.",
	       "config flash [-n|-w] [-b|-o]",
	       "-n;switch flash1 to 8-bit mode|"
	       "-w;switch flash1 to 16-bit mode|"
	       "-b;burst on|"	
	       "-o;burst off");
    return 0;
}


/* NORMAL WRITE to the cpld
 * Write 8'b01010101 to address 8'b00000001 of the cpld
 */
static int ui_cmd_writecpld(ui_cmdline_t *cmd,int argc,char *argv[])
{
    /* Make sure burst mode is DISABLED. */
    burst_mode(CPLD_CS, 0);

    xprintf ("writing 0x55 to cpld address 0x01\n");
    *((volatile uint8_t *) PHYS_TO_K1(CPLD_PHYS+0x01)) = (uint8_t) (0x55);
    return 0;
}


/* NORMAL READ to the cpld
 * The cpld is programmed to output the current cpld SUM if address 0x00 is
 * read.  However, if any other address is read, then the cpld will output
 * 5'b00000 concatenated with the lowest three address bits.
 * ie.
 * reading address 0xFF will return 0x07.
 * reading address 0x4A will return 0x02.
 * reading address 0x09 will return 0x01.
 */
static int ui_cmd_readcpld(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint8_t data;

    /* make sure burst mode is DISABLED. */
    burst_mode(CPLD_CS, 0);

    data = *((volatile uint8_t *) PHYS_TO_K1(CPLD_PHYS+0xFC));
    xprintf ("CPLD address 0xFC contains 0x%2X\n", data);
    xprintf ("This value should be 0x04\n");
    data = *((volatile uint8_t *) PHYS_TO_K1(CPLD_PHYS));
    xprintf ("CPLD address 0x00 contains 0x%2X\n", data);

    return 0;
}


/* BURST WRITE to the cpld
 * Maximum burst size (without doing a UAC store) to cpld is 8 bytes.
 * byte1: 0x01
 * byte2: 0x02
 * byte3: 0x04
 * byte4: 0x08
 * byte5: 0x10
 * byte6: 0x20
 * byte7: 0x40
 * byte8: 0x80
 * To do the burst, write the 64-bit value 0x8040201008040201 to the cpld.
 * At the end of the burst, the cpld SUM register should contain 0xFF.
 */
static int ui_cmd_burstwcpld(ui_cmdline_t *cmd,int argc,char *argv[])
{
    /* Enable burst mode. */
    burst_mode(CPLD_CS, 1);

    xprintf("burst writing 8 bytes (0x8040201008040201) to cpld address 0x00\n");
    *((volatile uint64_t *) PHYS_TO_K1(CPLD_PHYS)) = (uint64_t) (0x8040201008040201ULL);

    return 0;
}


/* BURST READ to the cpld
 * Burst reading the cpld at address 0x00 should return
 * 0x07060504030201 concatenated with the cpld SUM[7:0]
 */
static int ui_cmd_burstrcpld(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint64_t data;

    /* Enable burst mode */
    burst_mode(CPLD_CS, 1);

    data = *((volatile uint64_t *) PHYS_TO_K1(CPLD_PHYS));
    xprintf ("Address 0x00 of cpld contains (by burst reading) 0x%16X\n", data);
    xprintf ("This value should be 0x07060504030201FF if a burst write was just executed\n");

    return 0;
}

static int ui_cmd_config_flash(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint64_t gpio_data;
    unsigned int cs;
    int flash_16bit = 0;
    int burst_on = 0;
 
    /* Burst and 16-bit modes make sense only for the Intel flash; the
       alternative "flash" is a PromICE connector.  Do not change mode
       while running from that flash, and do not attempt to update the
       flash except in 8-bit, non-burst mode (the default). */

    /* Boot flash selection is at pin 9 on GPIO; refer to lausanne.h */
    gpio_data = SBREADCSR(A_GPIO_READ);
    cs = (gpio_data & _SB_MAKEMASK1(GPIO_BOOT_SELECT)) ?
	  BOOTROM_CS : ALT_BOOTROM_CS;

    if (cmd_sw_isset(cmd,"-w")) {
	/*switch to 16-bit mode*/
	flash_16bit = 1;
	}
    else if (cmd_sw_isset(cmd,"-n")) {
	/*switch to 8-bit mode*/
	flash_16bit = 0;
	}	
    intel_flash_data_width_mode(cs, flash_16bit);

    if (cmd_sw_isset(cmd,"-b")) {
	burst_on = 1;
	}
    else if (cmd_sw_isset(cmd,"-o")) {
	burst_on = 0;
	}    
    burst_mode(cs, burst_on);

    return 0;

}


/*  *********************************************************************
    *  Utilities
    ********************************************************************* */

static int intel_flash_data_width_mode(unsigned int cs, int flash_16bit)
{
    uint64_t gpio_direction;
    uint64_t io_ext_cfg;
 
    gpio_direction = SBREADCSR(A_GPIO_DIRECTION);

    io_ext_cfg = SBREADCSR(A_IO_EXT_CS_BASE(cs));
    io_ext_cfg &= ~M_IO_WIDTH_SEL;

    if (flash_16bit) {
	gpio_direction |= _SB_MAKEMASK1(GPIO_FLASH_BYTE_EN);
	SBWRITECSR(A_GPIO_DIRECTION,gpio_direction);

	if (DEBUG_CS) xprintf("FLASH CS %d: 16-BIT MODE\n", cs);
	SBWRITECSR(A_GPIO_PIN_SET,_SB_MAKEMASK1(GPIO_FLASH_BYTE_EN));

	/* GENBUS CS: mux'ed and 2 bytes wide */
	io_ext_cfg |= V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_2);
	io_ext_cfg &= ~M_IO_NONMUX;   /* make sure we're mux'ed!! */
	SBWRITECSR(A_IO_EXT_CS_BASE(cs),io_ext_cfg);
	}
    else {  /* 8 bit flash */
	gpio_direction &= ~_SB_MAKEMASK1(GPIO_FLASH_BYTE_EN);  /* XXX eh?? */
	SBWRITECSR(A_GPIO_DIRECTION,gpio_direction);

	if (DEBUG_CS) xprintf("FLASH CS %d: 8-BIT MODE\n", cs);	
	SBWRITECSR(A_GPIO_PIN_CLR,_SB_MAKEMASK1(GPIO_FLASH_BYTE_EN));

	/* GENBUS CS: nonmux'ed and 1 byte wide */
	io_ext_cfg |= M_IO_NONMUX | V_IO_WIDTH_SEL(K_IO_WIDTH_SEL_1);
	SBWRITECSR(A_IO_EXT_CS_BASE(cs),io_ext_cfg);
	}
    return 0;
}  

static int burst_mode(unsigned int cs, int burst_on)
{
    uint64_t io_ext_cfg;
 
    /*GENBUS CS */
    io_ext_cfg = SBREADCSR(A_IO_EXT_CS_BASE(cs));

    if (burst_on) {
	io_ext_cfg |= M_IO_BURST_EN; 
	SBWRITECSR(A_IO_EXT_CS_BASE(cs), io_ext_cfg);
	if (DEBUG_CS) xprintf("FLASH CS %d: BURST ON\n",cs);
	}
    else { /*burst off*/
	io_ext_cfg &= ~M_IO_BURST_EN;
	SBWRITECSR(A_IO_EXT_CS_BASE(cs),io_ext_cfg);
	if (DEBUG_CS) xprintf("FLASH CS %d: BURST OFF\n",cs);
	}   
 
    return 0;
}
