/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PCI IDE disk driver			File: dev_ide_frodo.c
    *  
    *  This is a simple driver for IDE hard disks that are connected
    *  ServerWorks "Frodo" Serial ATA controllers
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

#include "dev_ide_common.h"

#include "dev_ide.h"

#include "pcivar.h"
#include "pcireg.h"

#include "lib_physio.h"


#define BCM_BYTESWAP32(value)                       \
       ((((uint32_t)(value) & 0xFF000000) >> 24) |  \
        (((uint32_t)(value) & 0x00FF0000) >> 8)  |  \
        (((uint32_t)(value) & 0x0000FF00) << 8)  |  \
        (((uint32_t)(value) & 0x000000FF) << 24))

#define BCM_BYTESWAP16(value)                       \
       ((((uint16_t)(value) & 0xFF00) >> 8)  |      \
        (((uint16_t)(value) & 0x00FF) << 8))


#if   defined(ENDIAN_BIG)
#define BCM_LE32_TO_HOST(x)     BCM_BYTESWAP32(x) 
#define BCM_HOST_TO_LE32(x)     BCM_BYTESWAP32(x) 
#define BCM_LE16_TO_HOST(x)     BCM_BYTESWAP16(x) 
#define BCM_HOST_TO_LE16(x)     BCM_BYTESWAP16(x) 
#else
#define BCM_LE32_TO_HOST(x)     (x) 
#define BCM_HOST_TO_LE32(x)     (x) 
#define BCM_LE16_TO_HOST(x)     (x) 
#define BCM_HOST_TO_LE16(x)     (x) 
#endif

#define BCM_REG_RD32_LE( p_reg32 )       BCM_LE32_TO_HOST(phys_read32( ((physaddr_t) p_reg32))) 
#define BCM_REG_WR32_LE( p_reg32, val )  phys_write32(((physaddr_t)p_reg32), BCM_HOST_TO_LE32( val ) ) 
#define BCM_REG_RD16_LE( p_reg16 )       BCM_LE16_TO_HOST( phys_read16( ((physaddr_t)p_reg16 ))) 
#define BCM_REG_WR16_LE( p_reg16, val )  phys_write16(((physaddr_t)p_reg16), BCM_HOST_TO_LE16( val ) ) 
#define BCM_REG_RD8( p_reg8 )            phys_read8(  ((physaddr_t)p_reg8))
#define BCM_REG_WR8(p_reg8, val)         phys_write8( ((physaddr_t)p_reg8), val) 

//#include "sb1250_defs.h"
#include "sbmips.h"


#define FRODO_TESTCTRLREG               0x10f0
#define FRODO_MDIOCTRLREG               0x8c
#define FRODO_PLLCTRLREG                0x84
#define FRODO_SCR2REG                   0x48

#define FRODO_TEST_CTRL_VALUE           0x40000000
#define FRODO_SCR2_RESET_PHY       0x00000001
#define FRODO_SCR2_CLEAR           0x00000000

/* assume all Frodo deives have 4 ports for now... */
#define FRODO_NUM_PORTS(dev, class)     4

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#if ENDIAN_BIG
//#define _BYTESWAP_ 	/* don't byteswap these disks */
#endif

#define OUTB(x,y) outb(x,y)
#define OUTW(x,y) outw(x,y)
#define INB(x) inb(x)
#define INW(x) inw(x)

/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */

extern void _wbflush(void);

static void idedrv_probe(cfe_driver_t *drv,
			      unsigned long probe_a, unsigned long probe_b, 
			      void *probe_ptr);

/*  *********************************************************************
    *  Device Dispatch
    ********************************************************************* */

static cfe_devdisp_t idedrv_dispatch = {
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

const cfe_driver_t frododrv = {
    "FRODO SATA disk",
    "sata",
    CFE_DEV_DISK,
    &idedrv_dispatch,
    idedrv_probe
};


/*  *********************************************************************
    *  Supported PCI devices
    ********************************************************************* */

#define DEVID(vid,pid) (((pid)<<16)|(vid))

static uint32_t pciidedrv_devlist[] = {
    DEVID(0x1166,0x0212),		/* SW */
    DEVID(0x1166,0x0213),		/* SW */
    DEVID(0x1166,0x0241),		/* SW */
    DEVID(0x1166,0x0242),		/* SW */
    DEVID(0x1166,0x024a),		/* SW */ 
    0xFFFFFFFF
};


/*  *********************************************************************
    *  Port I/O routines
    *  
    *  These routines are called back from the common code to do
    *  I/O cycles to the IDE disk.  We provide routines for
    *  reading and writing bytes, words, and strings of words.
    ********************************************************************* */

static uint8_t idedrv_inb(idecommon_dispatch_t *disp,uint32_t reg)
{
    return BCM_REG_RD8((reg*4)+disp->baseaddr);
}

static uint16_t idedrv_inw(idecommon_dispatch_t *disp,uint32_t reg)
{
    return  BCM_REG_RD16_LE((reg*4)+disp->baseaddr);
}

static void idedrv_ins(idecommon_dispatch_t *disp,uint32_t reg,hsaddr_t buf,int len)
{
    uint16_t data;

    while (len > 0) {
        data = BCM_REG_RD16_LE((reg*4)+disp->baseaddr);

#ifdef _BYTESWAP_
	hs_write8(buf,(data >> 8) & 0xFF);
	buf++;
	hs_write8(buf,(data & 0xFF));
	buf++;
#else
	hs_write8(buf,(data & 0xFF));
	buf++;
	hs_write8(buf,(data >> 8) & 0xFF);
	buf++;
#endif
    /* xprintf("FRODO: %04X:%04X:%02X%02X\n", (int) len, (int) data, (int) *((uint8_t *) (int)buf-2), (int) *((uint8_t *) (int)buf-1)); */

	len--;
	len--;
	}

}

static void idedrv_outb(idecommon_dispatch_t *disp,uint32_t reg,uint8_t val)
{
    BCM_REG_WR8((reg*4)+disp->baseaddr,val);
}

static void idedrv_outw(idecommon_dispatch_t *disp,uint32_t reg,uint16_t val)
{
    BCM_REG_WR16_LE((reg*4)+disp->baseaddr,val);
}

static void idedrv_outs(idecommon_dispatch_t *disp,uint32_t reg,hsaddr_t buf,int len)
{
    uint16_t data;

    while (len > 0) {
#ifdef _BYTESWAP_
	data = (uint16_t) hs_read8(buf+1) + ((uint16_t) hs_read8(buf+0) << 8);
#else
	data = (uint16_t) hs_read8(buf+0) + ((uint16_t) hs_read8(buf+1) << 8);
#endif

	BCM_REG_WR16_LE((reg*4)+disp->baseaddr,data);

	buf++;
	buf++;
	len--;
	len--;
	}
}


/*  *********************************************************************
    *  pciidedrv_find(devid,list)
    *  
    *  Find a particular product ID on the list.  Return >= 0 if
    *  the ID is valid.
    *  
    *  Input parameters: 
    *  	   devid - product and device ID we have
    *  	   list - list of product and device IDs we're looking for
    *  	   
    *  Return value:
    *  	   index into table, or -1 if not found
    ********************************************************************* */
static int pciidedrv_find(uint32_t devid,uint32_t *list)
{
    int idx = 0;

    while (list[idx] != 0xFFFFFFFF) {

	if (list[idx] == devid) return idx;
    
	idx++;
	}

    return -1;
}


/*  *********************************************************************
    *  idedrv_probe(drv,probe_a,probe_b,probe_ptr)
    *  
    *  Our probe routine.  Attach an IDE device to the firmware.
    *  
    *  Input parameters: 
    *  	   drv - driver structure
    *  	   probe_a - physical address of IDE registers
    *  	   probe_b - unit number
    *  	   probe_ptr - not used
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void idedrv_probe(cfe_driver_t *drv,
			      unsigned long probe_a, unsigned long probe_b, 
			      void *probe_ptr)
{
    idecommon_t *softc;
    idecommon_dispatch_t *disp;
    char descr[80];
    char unitstr[50];
    pcitag_t tag;
    int index;
    uint32_t devid,classid;
    uint32_t reg;
    int res;
    int unit;
    cfe_driver_t *realdrv;
    int attached = 0;

    /* 
     * probe_a is unused
     * probe_b is unused
     * probe_ptr is unused.
     */

    index = 0;

    for (;;) 
    {
        if (pci_find_class(PCI_CLASS_MASS_STORAGE,index,&tag) != 0) break;
        index++;

        devid = pci_conf_read(tag,PCI_ID_REG);
        classid = pci_conf_read(tag,PCI_CLASS_REG);

        /* 
         * This driver will only accept certain device IDs.
         */
        if (pciidedrv_find(devid,pciidedrv_devlist) < 0) {
            continue;
	    }

        /*
         * Mapping register #5 is the MMIO BAR.
         */

        reg = pci_conf_read(tag,PCI_MAPREG(5));

        reg &= ~PCI_MAPREG_TYPE_MASK;


        /* 
         * Make sure BAR is valid, HT1000 has second device instance without MM bar 
         */
        if (reg == 0) 
        {
            continue;
	    }

 

        /* Create CFE device instance for each port */
        for (unit = 0; unit < FRODO_NUM_PORTS(devid, classid); unit++) 
        {
            /* unit = (int) probe_b; */

            softc = (idecommon_t *) KMALLOC(sizeof(idecommon_t),0);
            disp = (idecommon_dispatch_t *) KMALLOC(sizeof(idecommon_dispatch_t),0);

            if (!softc || !disp) {
                if (softc) KFREE(softc);
                if (disp) KFREE(disp);
                return;		/* out of memory, stop here */
            }

            softc->idecommon_addr = reg + (unit * 0x100);
            disp->ref = softc;
            disp->baseaddr = softc->idecommon_addr;
            softc->idecommon_deferprobe = 0;
            softc->idecommon_dispatch = disp;
            softc->idecommon_unit = 0;

            disp->outb = idedrv_outb;
            disp->outw = idedrv_outw;
            disp->outs = idedrv_outs;

            disp->inb = idedrv_inb;
            disp->inw = idedrv_inw;
            disp->ins = idedrv_ins;

            {
                cfe_usleep( 10000 );

                uint32_t port_select, ncqrdval, ncqfixval, mdioctrl;
                // Enable MDIO Space
                BCM_REG_WR32_LE( reg + FRODO_TESTCTRLREG, FRODO_TEST_CTRL_VALUE | 0x00000001); 
                cfe_usleep( 10000 );

                port_select = (((1 << unit) << 16) | (0x2007));
                BCM_REG_WR32_LE( reg + FRODO_MDIOCTRLREG, port_select);
                cfe_usleep( 10000 );

                ncqrdval = 0x0000400d;
                BCM_REG_WR32_LE( reg + FRODO_MDIOCTRLREG, ncqrdval );
                cfe_usleep( 10000 );
	
                ncqfixval = BCM_REG_RD32_LE( reg + FRODO_MDIOCTRLREG );
                ncqfixval = (ncqfixval & 0xFFFF0000);
                ncqfixval = (ncqfixval | 0x0004200d);
                BCM_REG_WR32_LE( reg + FRODO_MDIOCTRLREG, ncqfixval);
                cfe_usleep( 10000 );
    
                // Disable MDIO Access
                mdioctrl =  BCM_REG_RD32_LE( reg + FRODO_TESTCTRLREG );
                mdioctrl &= (~FRODO_TEST_CTRL_VALUE);
                BCM_REG_WR32_LE( reg + FRODO_TESTCTRLREG, mdioctrl);

                BCM_REG_WR32_LE( disp->baseaddr + FRODO_SCR2REG, FRODO_SCR2_RESET_PHY);
                cfe_usleep( 10000 );
                BCM_REG_WR32_LE( disp->baseaddr + FRODO_SCR2REG, FRODO_SCR2_CLEAR);
                cfe_usleep( 10000 );
            }

            /*
             * If we're autoprobing, do it now.  Loop back if we have
             * trouble finding the device.  
             * 
             * If not autoprobing, assume the device is there and set the
             * common routines to double check later.
             */
    
            if (IDE_PROBE_GET_TYPE(probe_b,unit) == IDE_DEVTYPE_AUTO) {
                res = idecommon_devprobe(softc,1);
                if (res < 0) {
                    KFREE(softc);
                    KFREE(disp);
                    continue;
                }
            }
            else {
                idecommon_init(softc,IDE_PROBE_GET_TYPE(probe_b,unit));
                softc->idecommon_deferprobe = 1;
            }

            xsprintf(descr,"%s unit %d at %04X",drv->drv_description,
                     softc->idecommon_unit,softc->idecommon_addr);
            xsprintf(unitstr,"%d",unit);

            realdrv = (cfe_driver_t *) &frododrv;

            idecommon_attach(&idedrv_dispatch);
            cfe_attach(realdrv,softc,unitstr,descr);
            attached++;
        }
    }

    xprintf("FRODO: %d controllers found\n",attached);
}


