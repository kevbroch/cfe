/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PCI device selection and initialization		File: pci_devs.c
    *  
    *  These are the routines to include the PCI device drivers
    *  for which CPCI cards are available.
    *  
    *********************************************************************  
    *
    *  Copyright 2003
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


#if CFG_PCI
#include "cfe.h"
#include "env_subr.h"

extern cfe_driver_t tulipdrv;                   /* Tulip Ethernet */

#if CFG_DOWNLOAD
extern cfe_driver_t bcm1250drv;                 /* BCM1250 as a device */
#endif

void cpci_add_devices(int init_pci);
void cpci_add_devices(int init_pci)
{
    if (init_pci) {
	cfe_add_device(&tulipdrv,0,0,env_getenv("TULIP0_HWADDR"));
	 /* ... more as we get suitable cards. */

#if CFG_DOWNLOAD
	/* Access to bcm1250 in PCI device mode */
	cfe_add_device(&bcm1250drv,0,0,NULL);
#endif
	 }
}
#endif /* CFG_PCI */
