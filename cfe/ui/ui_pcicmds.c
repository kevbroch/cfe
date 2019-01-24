/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PCI Commands				File: ui_pcicmds.c
    *  
    *  PCI user interface routines
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
#include "ui_command.h"
#include "lib_memfuncs.h"

#if CFG_PCI
#include "pcivar.h"
extern int pci_probe_tag(pcitag_t tag);   /* currently internal */
void	  pci_break_tag(pcitag_t, int *, int *, int *, int *); /* currently internal */
#include "pcireg.h"
#ifdef SIBYTE_BCM1480
#include "bcm1480_pci.h"
#endif
#endif

int ui_init_pcicmds(void);

#if CFG_PCI
static int pci_print_summary(pcitag_t tag)
{
    pcireg_t id, class;
    char devinfo[256];

    class = pci_conf_read(tag, PCI_CLASS_REG);
    id = pci_conf_read(tag, PCI_ID_REG);

    pci_devinfo(id, class, 1, devinfo);
    pci_tagprintf (tag, "%s\n", devinfo);

    return 0;	   
}

static int pci_print_concise(pcitag_t tag)
{
    pci_tagprintf (tag, "\n");
    pci_conf_print(tag);	

    return 0;
}

static int ui_cmd_pci(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *argp;

    if (cmd_sw_isset(cmd,"-init")) {
	pci_flags_t flags;
	char *str;
	extern cons_t pci_optnames[];

	flags = PCI_FLG_NORMAL;
#if (CFG_LDT && CFG_LDT_REV_017)
	flags |= PCI_FLG_LDT_REV_017;
#endif
	str = env_getenv("PCI_OPTIONS");
	setoptions(pci_optnames,str,&flags);
	
	xprintf("Initializing PCI. [%s]\n",str ? str : "normal");
	pci_configure(flags);

	return 0;
	}

    argp = cmd_getarg(cmd,0);

    if (argp == NULL) {
	if (cmd_sw_isset(cmd,"-v")) {
	    pci_foreachdev(pci_print_concise);
	    }
	else {
	    pci_foreachdev(pci_print_summary);
	    }
	}
    else {
	/* parse the tuple */
	int n, port, bus, dev, func;
	pcitag_t tag;
	char *p;

	port = bus = dev = func = 0;
	p = argp;  n = 0;

	while (*p >= '0' && *p <= '9') {
	    n = n*10 + (*p - '0');
	    p++;
	    }
	if (*p == '/')
	    bus = n;
	else  if (*p == ':') {
	    port = n;
	    p++;
	    while (*p >= '0' && *p <= '9') {
		bus = bus*10 + (*p - '0');
		p++;
		}
	    }
	else
	    goto fail;
	p++;
	while (*p >= '0' && *p <= '9') {
	    dev = dev*10 + (*p - '0');
	    p++;
	    }
	if (*p != '/')
	    goto fail;
	p++;
	while (*p >= '0' && *p <= '9') {
	    func = func*10 + (*p - '0');
	    p++;
	    }
	if (*p != '\000')
	    goto fail;

	tag = pci_make_tag(port,bus,dev,func);	

	pci_print_concise(tag);
	}

    return 0;

fail:
    printf("invalid PCI tuple %s\n", argp);
    return -1;
}


static int ui_cmd_pcidump(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int port, bus, dev, func;
    int alldev;
    int d1, d2, d;
    int idx;
    pcitag_t tag;
    pcireg_t v0, v1, v2, v3;
    int printed, zeros;
    char *arg;
    char *x;

    port = bus = dev = func = 0;       /* defaults */
    alldev = 0;

    arg = cmd_getarg(cmd, 0);
    if (arg) {
	if ((x = strchr(arg, ':'))) {
	    *x++ = '\0';
	    port = atoi(arg);
	    arg = x;
	    }
	if ((x = strchr(arg, '/'))) {
	    *x++ = '\0';
	    bus = atoi(arg);
	    arg = x;
	    if ((x = strchr(arg, '/'))) {
		*x++ = '\0';
		if (arg[0] == '*') alldev = 1;
		dev = atoi(arg);
		arg = x;
		func = atoi(arg);
		}
	    }
	}

    if (alldev) {
	d1 = 0; d2 = 32;
	}
    else {
	d1 = dev; d2 = dev+1;
	}

    for (d = d1; d < d2; d++) {

	tag = pci_make_tag(port, bus, d, func);
	if (pci_probe_tag(tag)) {
	    if (cmd_sw_isset(cmd, "-scan")) {
		v0 = pci_conf_read(tag, 0);
    		pci_tagprintf (tag, "VID %04X DID %04X\n",
			v0 & 0xFFFF, v0 >> 16);
		}
	    else {
    		pci_tagprintf (tag, "\n");
		printed = zeros = 0;
		for (idx = 0; idx < 0x100; idx += 0x10) {
		    v0 = pci_conf_read(tag, idx+0x0);
		    v1 = pci_conf_read(tag, idx+0x4);
		    v2 = pci_conf_read(tag, idx+0x8);
		    v3 = pci_conf_read(tag, idx+0xC);
		    if (!printed || (v0 | v1 | v2 | v3) != 0) {
			xprintf(" %04X: %08X %08X %08X %08X\n",
				idx, v0, v1, v2, v3);
			printed = 1;
			zeros = 0;
			}
		    else if (!zeros) {
			xprintf(" ...\n");
			zeros = 1;
			}
		    }
        	}
	    }
	else {
    	    pci_tagprintf (tag, "no device\n");
	    }
	}

    return 0;

}


static int ui_cmd_pcimemtest(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int            vid, did, enumidx, barnum;
    int            reg, i;
    pcitag_t       tag;
    hsaddr_t       addr = 0;
    uint64_t       memsize = 0, dsize;
    uint64_t       *src;
    uint64_t       pattern;
    pcireg_t       old, mask;
    pcireg_t       oldhi, maskhi;
    int            port, bus, device, function;
    int            error;
    
   
    if (argc < 4) {
       printf("\n Insufficient arguments, please see help");
       return 0;
    }
    
    vid = atoi(argv[0]);
    did = atoi(argv[1]);
    enumidx = atoi(argv[2]);
    barnum = atoi(argv[3]);

    pci_find_device(vid, did, enumidx, &tag);
    pci_break_tag(tag, &port, &bus, &device, &function);
    printf("\n Found device on Port: %d, Bus: %d, Device: %d, Func: %d", port, bus, device, function);
    
    /* Get the Base address and size of memory region on the target */
    reg = 0x10 + barnum * 4;
    old = pci_conf_read(tag, reg);
    pci_conf_write(tag, reg, 0xffffffff);
    mask = pci_conf_read(tag, reg);
    pci_conf_write(tag, reg, old);

    if(PCI_MAPREG_MEM_TYPE(mask) == PCI_MAPREG_MEM_TYPE_64BIT) {

       oldhi = pci_conf_read(tag, reg + 4);
	 pci_conf_write(tag, reg + 4, 0xffffffff);
	 maskhi = pci_conf_read(tag, reg + 4);
	 pci_conf_write(tag, reg + 4, oldhi);

       addr = ((uint64_t)oldhi) << 32;
       memsize = ((uint64_t)maskhi) << 32;
    }

    addr |= old;
    addr &= 0xFFFFFFFFFFFFFFF0ULL; 
    memsize |= mask;
    memsize = PCI_MAPREG_MEM64_SIZE(memsize);
    printf("\n BAR: %lx, Size: %lx", addr, memsize);

    /* Append virtual address base */
    addr |= 0x9000000000000000ULL;
    printf("\n Addres %016llx",addr);

    /* Allocate mem region on Host and fill with test data */
    if(memsize > 0x10000)
       dsize = 0x10000;
    else
       dsize = memsize;
    src = KMALLOC(dsize,sizeof(uint64_t));
    if(src == NULL) {
       printf("\n Insufficient memory: Unable to allocate memory for test data");
       return 0;
    }

    /* Reading and Saving the Data from PCI Target */
    for(i = 0; i < dsize/8; i++) {
        error = mem_peek(&src[i],addr + i*8, MEM_QUADWORD);
        if(error != 0) {
           printf("\n Failed to read data from Target address:%016llX", addr+i*8);
           return(error);
        }
    }
    
    /* Now write the test data on to Target, Read it back and compare */
 
    printf("\n Writing Test Data to PCI Target");

    pattern = 0xAAAAAAAAAAAAAAAAULL;
    printf("\n Writing memory region of size: %016llX at Address: %016llX with Pattern: %llX",
             dsize/4, addr, pattern);
    for(i = 0; i < (dsize/32); i++) {
        error = mem_poke(addr + i*8,pattern,MEM_QUADWORD); 
        if(error != 0) {
           printf("\n Failed to write data to the Target at Address: %016llX, Pattern: %016llX",addr+i*8, pattern);
           return(error);
        }
    }

    pattern = 0x5555555555555555ULL;
    printf("\n Writing memory region of size: %016llX at Address: %016llX with Pattern: %llX",
             dsize/4, addr + dsize/4, pattern);
    for(i = dsize/32; i < (dsize/16); i++) {
        error = mem_poke(addr + i*8,pattern,MEM_QUADWORD); 
        if(error != 0) {
           printf("\n Failed to write data to the Target at Address: %016llX, Pattern: %016llX",addr+i*8, pattern);
           return(error);
        }
    }

    pattern = 0;
    printf("\n Writing memory region of size: %016llX at Address: %016llX with Pattern: %llX",
             dsize/4, addr + dsize/2, pattern);
    for(i = dsize/16; i < (dsize *3/32); i++) { 
        error = mem_poke(addr + i*8,pattern,MEM_QUADWORD); 
        if(error != 0) {
           printf("\n Failed to write data to the Target at Address: %016llX, Pattern: %016llX",addr+i*8, pattern);
           return(error);
        }
    }

    pattern = 0xFFFFFFFFFFFFFFFFULL;
    printf("\n Writing memory region of size: %016llX at Address: %016llX with Pattern: %llX",
             dsize/4, addr + (dsize*3)/4, pattern);
    for(i = dsize*3/32; i < dsize/8; i++) {
        error = mem_poke(addr + i*8,pattern,MEM_QUADWORD); 
        if(error != 0) {
           printf("\n Failed to write data to the Target at Address: %016llX, Pattern: %016llX",addr+i*8, pattern);
           return(error);
        }
    }

    printf("\n Writing Test Data: Done");

    printf("\n Now Reading the Test data from PCI device"); 
    for(i = 0; i < (dsize/32); i++) {
        error = mem_peek(&pattern,addr + i*8, MEM_QUADWORD);
        if(error != 0) {
           printf("\n Failed to read data from Target address:%016llX", addr+i*8);
           return(error);
        }
 
       if(pattern != 0xAAAAAAAAAAAAAAAAULL) {
          printf("\n Test failed at Address: %llX", addr + i * 8);
          printf("\n Value read: %016llX", pattern);
          printf("\n Must be: 0xAAAAAAAAAAAAAAAA");
          return 0;
       }
    }

    printf("\n Testing with Pattern 0xAAAAAAAAAAAAAAAA : Done");

    for(i = dsize/32; i < (dsize/16); i++) {
        error = mem_peek(&pattern,addr + i*8, MEM_QUADWORD);
        if(error != 0) {
           printf("\n Failed to read data from Target address:%016llX", addr+i*8);
           return(error);
        }
 
       if(pattern != 0x5555555555555555ULL) {
          printf("\n Test failed at Address: %llX", addr + i * 8);
          printf("\n Value read: %016llX", pattern);
          printf("\n Must be: 0x5555555555555555");
          return 0;
       }
    }
    printf("\n Testing with Pattern 0x5555555555555555: Done");

    for(i = dsize/16; i < (dsize *3/32); i++) {
        error = mem_peek(&pattern,addr + i*8, MEM_QUADWORD);
        if(error != 0) {
           printf("\n Failed to read data from Target address:%016llX", addr+i*8);
           return(error);
        }
 
       if(pattern != 0) {
          printf("\n Test failed at Address: %llX", addr + i * 8);
          printf("\n Value read: %016llX", pattern);
          printf("\n Must be: 0x0");
          return 0;
       }
    }
    printf("\n Testing with Pattern 0x0 : Done");

    for(i = dsize*3/32; i < dsize/8; i++) {
        error = mem_peek(&pattern,addr + i*8, MEM_QUADWORD);
        if(error != 0) {
           printf("\n Failed to read data from Target address:%016llX", addr+i*8);
           return(error);
        }
 
       if(pattern != 0xFFFFFFFFFFFFFFFFULL) {
          printf("\n Test failed at Address: %llX", addr + i * 8);
          printf("\n Value read: %016llX", pattern);
          printf("\n Must be: 0xFFFFFFFFFFFFFFFF");
          return 0;
       }
    }
    printf("\n Testing with Pattern 0xFFFFFFFFFFFFFFFF: Done");

    /* Write the original data back to the target */
    for(i = 0; i < dsize/8; i++) {
        error = mem_poke(addr + i*8,src[i], MEM_QUADWORD);
        if(error != 0) {
           printf("\n Failed to write saved data to the Target at address:%016llX", addr+i*8);
           return(error);
        }
    }

    printf("\n PCI R/W test passed");
    return 0;
}

#ifdef SIBYTE_BCM1480

/* Sets Map table entry in 1280/1480 PCI config space */
static int ui_cmd_pcimte(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int        entry;
    uint64_t   base;
    pcireg_t   addr;

    if(argc < 2){
       printf("\n Too few arguements...");
       return 0;
    }

    entry = atoi(argv[0]);
    if(entry >= BCM1480_PHB_MAPENTRIES) {
       printf("\n Invalid map table entry");
       return 0;
    }

    base = atoi(argv[1]);
    addr = ((uint32_t)((base & ~0xFFFFF) >> 8)) | M_BCM1480_PHB_MAP_ENABLE;
    
    pci_conf_write(0, R_BCM1480_PHB_MAP(entry), addr);
    return 0;
}
#endif
#endif


int ui_init_pcicmds(void)
{

#if CFG_PCI
    cmd_addcmd("show pci",
	       ui_cmd_pci,
	       NULL,
	       "Display information about PCI buses and devices",
	       "show pci [-v] [[port:]bus/dev/func]\n\n"
	       "Displays information about PCI and LDT buses and devices in the\n"
	       "system.  If you specify a bus/dev/func triplet, only that device\n"
	       " will be displayed.",
	       "-v;Display verbose information|"
	       "-init;Reinitialize and rescan the PCI bus");

    cmd_addcmd("pci dump",
	       ui_cmd_pcidump,
	       NULL,
	       "Dump PCI configuration space.",
	       "pci dump [[port:]bus/dev/func]",
	       "-scan;Just see if there is a device there.");

    cmd_addcmd("pci memtest",
               ui_cmd_pcimemtest,
               NULL,
               "Tests Data transactions between PCI Host and Target",
               "pci memtest VendorId DeviceId Instance BAR#",
               "");

#ifdef SIBYTE_BCM1480
    cmd_addcmd("pci setmtentry",
               ui_cmd_pcimte,
               NULL,
               "Sets Map table entry in PCI Config space",
               "pci setmtentry Entry# 'Base address(40bit) of 1MB region' ",
               "");
#endif
#endif

    return 0;
}
