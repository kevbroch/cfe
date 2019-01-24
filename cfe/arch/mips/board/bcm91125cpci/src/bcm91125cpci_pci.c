/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: bcm91125cpci_pci.c
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

#include "lib_types.h"

#include "pcireg.h"
#include "pcivar.h"
#include "pci_internal.h"

extern int _pciverbose;

/* PCI interrupt mapping on the BCM91125CPCI board.  The board is
   wired as a PCI host intended to occupy the CPCI System slot.  The
   mapping follows the standard for CPCI backplanes, with
   IDSEL 31:25 (devices 20:14) -> CPCI logical slots 2 to 8.
 */

/* Return the base shift of a slot or device on the motherboard.
   This is board specific, for the bcm91125cpci only. */
uint8_t
pci_int_shift_0(pcitag_t tag)
{
    int bus, device;

    pci_break_tag(tag, NULL, &bus, &device, NULL);

    if (bus == 0 && (device >= 14 && device <= 20))
	return ((device - 5) % 4);
    else
	return 0;
}

/* Return the mapping of a CPCI device/function interrupt to an
   interrupt line.  For the BCM1125CPCI, return 1-4 to indicate the
   pci_inta - pci_intd inputs to the interrupt mapper, respectively,
   or 0 if there is no mapping. */
uint8_t
pci_int_map_0(pcitag_t tag)
{
    pcireg_t data;
    int pin, bus, device;

    data = pci_conf_read(tag, PCI_BPARAM_INTERRUPT_REG);
    pin = PCI_INTERRUPT_PIN(data);
    if (pin == 0) {
	/* No IRQ used. */
	return 0;
    }
    if (pin > 4) {
	if (_pciverbose >= 1)
	    pci_tagprintf(tag, "pci_map_int: bad interrupt pin %d\n", pin);
	return 0;
    }

    pci_break_tag(tag, NULL, &bus, &device, NULL);

    if (bus == 0 && (device >= 14 && device <= 20))
	return (((pin - 1) + pci_int_shift_0(tag)) % 4) + 1;
    else
	return 0;
}
