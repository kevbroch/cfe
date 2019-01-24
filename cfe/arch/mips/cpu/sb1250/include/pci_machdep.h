/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  PCI machine dependent stuff		File: pci_machdep.h
    *  
    *  This module contains cpu or architecture specific stuff for PCI.
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

#ifndef _PCI_MACHDEP_H_
#define _PCI_MACHDEP_H_

/*
 * Machine-specific definitions for PCI autoconfiguration.
 *
 * See the comments in pci_machdep.c for more explanation.
 */

#include "lib_types.h"

/*
 * Address types, as integers.
 */
typedef uint32_t pci_addr_t;
typedef uint64_t phys_addr_t;   /* ZBbus physical addresses. */

/*
 * Configuration tag; created from a {bus,device,function} triplet by
 * pci_make_tag(), and passed to pci_conf_read() and pci_conf_write().
 */
typedef uint32_t pcitag_t;

/*
 * Type of a value read from or written to a configuration register.
 * Always 32 bits.
 */
typedef uint32_t pcireg_t;

/*
 * The number of rooted bus trees to be configured (i.e., host bridges
 * with independent address spaces).
 */
#define PCI_HOST_PORTS 1

/* All mappings through the PCI host bridge use match bits mode. */
#define PHYS_TO_PCI(a) ((uint32_t) (a) | 0x20000000)
#define PCI_TO_PHYS(a) ((uint32_t) (a) & 0x1FFFFFFF)



#endif /* _PCI_MACHDEP_H_ */
