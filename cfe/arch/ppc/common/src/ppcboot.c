/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PPCBoot Linux environment		File: ppcboot.c
    *  
    *  Build board information to pass to the Linux kernel as done
    *  by the PPCBoot (U-Boot) bootloader. This is an alternative
    *  to using the CFE API.
    *  
    *********************************************************************  
    *
    *  Copyright 2004
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

#include "ppcboot.h"

#include "initdata.h"
#include "env_subr.h"

#include "net_ebuf.h"
#include "net_enet.h"
#include "net_ether.h"

#include "net_ip.h"
#include "net_api.h"

static bd_t bd_info;

/*  *********************************************************************
    *  get_flash_info(devname,info)
    *  
    *  Get size and base address of flash device.
    *
    *  Input parameters: 
    *  	   devname - flash device name
    *  	   info - flash device information
    *  	   
    *  Return value:
    *  	   0 if ok, else error code
    ********************************************************************* */

static int get_flash_info(char *devname,flash_info_t *info)
{
    int fd;
    int retlen;

    fd = cfe_open(devname);
    if (fd < 0) {
	return fd;
    }
    return cfe_ioctl(fd,IOCTL_FLASH_GETINFO,(uint8_t *)info,
                     sizeof(flash_info_t),&retlen,0);
}


/*  *********************************************************************
    *  cfe_launch(ept)
    *  
    *  Build board info in PPCBoot-format and start user program.
    *
    *  Input parameters: 
    *  	   ept - program entry vector
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void cfe_launch(unsigned long ept)
{
    void (*kernel)(bd_t *,void *,void *,void *,void *) = (void *)ept;
    char *envp;
    char *cmd_start,*cmd_end;
    flash_info_t info;
    bd_t *kbd = &bd_info;

    /*
     * Put board info on stack
     */

    memset(kbd,0,sizeof(bd_t));

    /* DRAM Info */
    kbd->bi_memstart = 0;
    kbd->bi_memsize = mem_totalsize << 20;

    /* Flash Info */
    if (get_flash_info("flash1.boot",&info) == 0 ||
        get_flash_info("flash0.boot",&info) == 0) {
        kbd->bi_flashstart = (unsigned long)info.flash_base;
        kbd->bi_flashsize = info.flash_size;
    }

    /* SRAM Info */
    kbd->bi_sramstart = 0;
    kbd->bi_sramsize = 0;

    /* Misc Info */
    kbd->bi_bootflags = 1;
    kbd->bi_intfreq = cfe_cpu_speed;
    kbd->bi_busfreq = cfe_bus_speed;
    kbd->bi_baudrate = CFG_SERIAL_BAUD_RATE;

    /* Ethernet MAC address */
    if ((envp = env_getenv("ETH0_HWADDR")) == NULL) {
        envp = "02-10-18-11-22-33";
    }
    enet_parse_hwaddr(envp,kbd->bi_enetaddr);

    /* IP address */
    if ((envp = env_getenv("NET_IPADDR")) != NULL) {
        parseipaddr(envp,(uint8_t *)&kbd->bi_ip_addr);
    }

    /*
     * Put Linux commandline on stack
     */
    if ((envp = env_getenv("LINUX_CMDLINE")) == NULL) {
        envp = "";
    }

    cmd_start = envp;
    cmd_end = envp + strlen(envp);

    /*
     * Linux Kernel Parameters:
     *   r3: ptr to board info data
     *   r4: initrd_start or 0 if no initrd
     *   r5: initrd_end - unused if r4 is 0
     *   r6: Start of command line string
     *   r7: End   of command line string
     */
    (*kernel) (kbd,0,0,cmd_start,cmd_end);
}
