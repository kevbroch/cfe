/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board-specific PCI description		File: bmw_pci.c
    *  
    *  This file describes the board-specific PCI slots/devices
    *  and wiring thereof.
    *  
    *  Author:  Ed Satterthwaite
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

#include "lib_types.h"

#include "pcireg.h"
#include "pcivar.h"
#include "pci_internal.h"

/* PCI interrupt mapping on the BMW card.  The only on-board device has
   id 13 (bcm5703 10/100/1000 NIC).  There is also a CompactPCI connector
   configured for a cPCI System Slot.

   All interrupts are routed via the Altera EPM7064 CPLD to the mpc8245
   as IRQ1 .. IRQ4 (?).  The bcm5703 interrupt is connected directly,
   not via INT[A-D].

     Slot    IDSEL   DevID  IRQ{0,1,2,3,4}  shift
     (HB)     gnd      0      {-,-,-,-}       -  (NC)
    (5703)    13      13      {M,-,-,-}       0

   PCI INTA through INTD from the CPCI connector are wired as
   separate signals.  A standard backplane maps IDSEL 31:25 to CPCI
   logical slots 2 to 8:
             IDSEL   DevID  cPCI Logical Slot  shift
              31      10         2               1
              25      25         8               3               
              26      26         7               2
              27      27         6               1
              28      28         5               0
              29      29         4               3
              30      30         3               2
*/

/* Return the base shift of a slot or device on the motherboard.
   This is board specific, for the BMW only. */
uint8_t
pci_int_shift_0(pcitag_t tag)
{
    int bus, device;

    pci_break_tag(tag, NULL, &bus, &device, NULL);
    if (bus == 0) {
	switch (device) {
	    case 13:  return 0;

	    /* CPCI backplane */
	    case 10:  return 1;
	    case 25:  return 3;
	    case 26:  return 2;
	    case 27:  return 1;
	    case 28:  return 0;
	    case 29:  return 3;
	    case 30:  return 2;

	    default:  return 0;   /* for now */
	    }
	}
    else
	return 0;
}

/* Return the mapping of a device/function interrupt to an
   interrupt line (IRQ number).  XXX CONJECTURED XXX */
uint8_t
pci_int_map_0(pcitag_t tag)
{
    pcireg_t data;
    int pin, bus, device;

    data = pci_conf_read(tag, PCI_BPARAM_INTERRUPT_REG);
    pin = PCI_INTERRUPT_PIN(data);
    if (pin == 0) {
	/* No IRQ used. */
	return 0xFF;
	}

    pci_break_tag(tag, NULL, &bus, &device, NULL);

    if (bus == 0) {
	switch (device) {
	    /* Assume MAC -> IRQ[0] */
	    case 13:  return 0;

            /* via cPCI connector */
	    /* Assume INT{A,B,C,D} -> IRQ[{1,2,3,4}] */
	    case 10:  return 1+1;
	    case 25:  return 1+3;
	    case 26:  return 1+2;
	    case 27:  return 1+1;
	    case 28:  return 1+0;
	    case 29:  return 1+3;
	    case 30:  return 1+2;

	    default: return 0xFF;
	    }
	}
    else
	return 0xFF;
}
