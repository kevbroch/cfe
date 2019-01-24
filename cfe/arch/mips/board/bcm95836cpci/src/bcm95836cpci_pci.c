/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board-specific PCI description		File: bcm94702cpci_pci.c
    *  
    *  This file describes the board-specific PCI slots/devices
    *  and wiring thereof.
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

#include "lib_types.h"

#include "pcireg.h"
#include "pcivar.h"
#include "pci_internal.h"

/* The BCM95836CPCI card can function as either host or device.  As a
   host, it plugs into a CompactPCI System Slot.  The mapping from
   System to Logical Slot for interrupts on the backplane is defined
   by the CompactPCI spec (see Section 3.2.6 and Figure 8).

   Backplane:
   Slot    IDSEL   DevID   INT{A,B,C,D}  shift
    1 (HB)  16       0       {A,B,C,D}     0
    2       31      15       {B,C,D,A}     1
    3       30      14       {C,D,A,B}     2
    4       29      13       {D,A,B,C}     3
    5       28      12       {A,B,C,D}     0
    6       27      11       {B,C,D,A}     1
    7       26      10       {C,D,A,B}     2
    8       25       9       {D,A,B,C}     3

   However, INTA and INTC signals on the connector are mapped to INTA on
   the card, and the INTB and INTD signals are mapped to INTB. */

/* Return the base shift of a slot or device on the motherboard.
   This is board specific, for the BCM95836CPCI only. */
uint8_t
pci_int_shift_0(pcitag_t tag)
{
    int bus, device;

    pci_break_tag(tag, NULL, &bus, &device, NULL);

    if (bus == 0 && (device >= 9 && device <= 15))
	return ((16 - device) % 4);
    else
	return 0;
}

/* Return the mapping of a device/function interrupt to an interrupt
   line.  For the BCM95836CPCI, all interrupt signals from the
   backplane are mapped to the single interrupt eventually generated
   by the PCI core. */
uint8_t
pci_int_map_0(pcitag_t tag)
{
    pcireg_t data;
    int pin;

    data = pci_conf_read(tag, PCI_BPARAM_INTERRUPT_REG);
    pin = PCI_INTERRUPT_PIN(data);
    return (pin == 0) ? 0 : (((pin - 1) + pci_int_shift_0(tag)) % 2) + 1;
}
