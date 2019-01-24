/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: bcm91480ht_pci.c
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

#include "bcm91480ht.h"
#include "bcm1480_regs.h"

#include "pcireg.h"
#include "pcivar.h"
#include "pci_internal.h"

/* PCI interrupt mapping on the BCM91480ht board:
   Only device id 1 is implemented as a PCI(X) connector, and
   there are no on-board devices.  Note 1280-style IDSEL mappings.

   Slot    IDSEL   DevID  INT{A,B,C,D}   shift
  (PHB)      -       0       {A,-,-,-}     0
    1       17       1       {A,B,C,D}     0 (identity)
    -        -       -                     1 (A->B, B->C, C->D, D->A)
    -                                      2 (A->C, B->D, C->A, D->B)
    -                                      3 (A->D, B->A, C->B, D->C) 
*/

extern int _pciverbose;

/* Return the base shift of a slot or device on the motherboard.
   This is board specific, for the bcm91480ht only. */
uint8_t
pci_int_shift_0(pcitag_t tag)
{
    int bus, device;

    pci_break_tag(tag, NULL, &bus, &device, NULL);

    if (bus != 0)
	return 0;
    switch (device) {
    case 0: case 1:
	return 0;
    default:
	return 0;
    }
  return 0;
}

/* Return the mapping of a bcm91480ht device/function interrupt to an
   interrupt line.  For the BCM1480, return 1-4 to indicate the
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

    if (bus != 0)
	return 0;

    switch (device) {
    case 0:
    case 1:
        return (((pin - 1) + pci_int_shift_0(tag)) % 4) + 1;
    default:
        return 0;
    }
    return 0;
}


/* PCI-X clock initialization functions. */

void
pci_clock_reset(void)
{
    SBWRITECSR(A_GPIO_PIN_CLR, M_GPIO_PCIX_FREQALL);
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

    SBWRITECSR(A_GPIO_PIN_CLR, M_GPIO_PCIX_FREQALL);

    if (freq >= 133) {
	selected = 133;
	SBWRITECSR(A_GPIO_PIN_SET, M_GPIO_PCIX_FREQ133);
	}
    else if (freq >= 100) {
	selected = 100;
	SBWRITECSR(A_GPIO_PIN_SET, M_GPIO_PCIX_FREQ100);
	}
    else if (freq >= 66) {
	selected = 66;
	SBWRITECSR(A_GPIO_PIN_SET, M_GPIO_PCIX_FREQ66);
	}
    else {
	selected = 33;
	SBWRITECSR(A_GPIO_PIN_SET, M_GPIO_PCIX_FREQ33);
	}

    return selected;
}


/* Local HT topology (master/slave) information. */

int
ldt_slave_mode(int port)
{
    unsigned int board_config;

    /* For now, treat ports 0 and 1 the same, as controlled by the
       setting of the CFG_NODE_ID bit in sys_cfg.  Port 2 is dedicated 
       to the PLX bridge and does not support node interconnect. */

    board_config = board_get_config();

    switch (port) {
	case 0:  case 1:
	    return ((board_config & BOARD_CFG_NODE_ID) != 0);
	default:
	    return 0;
	}
}
