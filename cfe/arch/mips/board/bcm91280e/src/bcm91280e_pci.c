/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: bcm91280e_pci.c
    *  
    *  This is the part of the board support package for boards
    *  that support PCI. It describes the board-specific slots/devices
    *  and wiring thereof.
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

#include "sbmips.h"
#include "bcm91280e.h"
#include "bcm1480_regs.h"

#include "pcireg.h"
#include "pcivar.h"
#include "pci_internal.h"

/* PCI interrupt mapping on the Cupertino (BCM91280E) board:
   The BCM91280E is wired for PCI Device mode and has no interrupt
   inputs.
*/

/* Return the base shift of a slot or device on the motherboard.
   This is board specific, for the bcm91280e only. */
uint8_t
pci_int_shift_0(pcitag_t tag)
{
    return 0;
}

/* Return the mapping of a bcm91280e device/function interrupt to an
   interrupt line.  For the BCM1480, return 1-4 to indicate the
   pci_inta - pci_intd inputs to the interrupt mapper, respectively,
   or 0 if there is no mapping. */
uint8_t
pci_int_map_0(pcitag_t tag)
{
    return 0;
}

/* PCI-X clock initialization functions. */

void
pci_clock_reset(void)
{
    /* Device mode: no on-board clock generator. */
}

void
pci_clock_enable(int on)
{
    /* No separate enable. */
}

unsigned int
pci_clock_select(unsigned int freq)
{
    unsigned int selected;

    /* Clear and selectively reset the controlling GPIO pins. */


    if (freq >= 133) {
	selected = 133;
	}
    else if (freq >= 100) {
	selected = 100;
	}
    else if (freq >= 66) {
	selected = 66;
	}
    else {
	selected = 33;
	}

    return selected;
}


/* Local HT topology (master/slave) information. */

int
ldt_slave_mode(int port)
{
    /* For now, all ports are nominally masters. */

    return 0;
}
