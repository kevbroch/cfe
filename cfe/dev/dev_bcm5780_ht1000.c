/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  HT1000 Bridge Support			File: dev_bcm5780_ht1000.c
    *  
    *********************************************************************  
    *
    *  Copyright 2002,2003
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
#include "lib_types.h"
#include "lib_physio.h"

#include "pcireg.h"
#include "pcivar.h"
#include "pci_internal.h"

void bcm5780devs_enable_preset(pcitag_t tag);

/* BCM5780 (HT1000) specific definitions */

/* BCM5780 specific registers */

/* BCM5780 configuration registers */

#define BCM5780_HOST_CFG_FEATURE_ENABLE0      0x0064
#define BCM5780_H0ST_FE0_USB_ENABLE           (1 <<  8)
#define BCM5780_H0ST_FE0_IDE_ENABLE           (1 << 14)

#define BCM5780_HOST_CFG_FEATURE_ENABLE2      0x0084
#define BCM5780_H0ST_FE2_SATA_ENABLE          (1 << 0)


void bcm5780devs_enable_preset(pcitag_t tag)
{
    pcireg_t ctrl;


    printf("BCM5780 (HT1000) PCI bridge discovered.  Enabling devices...\n");
    /* 
     * Enable BCM5780 - HT1000 devices supported under CFE 
     */
    ctrl = pci_conf_read(tag, BCM5780_HOST_CFG_FEATURE_ENABLE0);
    /* Enable Single Channel IDE support */
    ctrl |= BCM5780_H0ST_FE0_IDE_ENABLE;
    /* Enable USB support */
    ctrl |= BCM5780_H0ST_FE0_USB_ENABLE;
    pci_conf_write(tag, BCM5780_HOST_CFG_FEATURE_ENABLE0, ctrl);
    ctrl = pci_conf_read(tag, BCM5780_HOST_CFG_FEATURE_ENABLE0);   /* push */


    ctrl = pci_conf_read(tag, BCM5780_HOST_CFG_FEATURE_ENABLE2);
    /* Enable Frodo SATA support */
    ctrl |= BCM5780_H0ST_FE2_SATA_ENABLE;
    pci_conf_write(tag, BCM5780_HOST_CFG_FEATURE_ENABLE2, ctrl);
    ctrl = pci_conf_read(tag, BCM5780_HOST_CFG_FEATURE_ENABLE2);   /* push */
}

