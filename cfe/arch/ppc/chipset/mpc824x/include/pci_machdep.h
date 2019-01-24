/*
 * Copyright (c) 2004 Broadcom Corporation.  All rights reserved.
 */

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
typedef uint32_t phys_addr_t;   /* MPC824X physical addresses. */

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

/*
 * PCI addresses and memory addresses are the same.
 */

#define PHYS_TO_PCI(a) ((uint32_t) (a))
#define PCI_TO_PHYS(a) ((uint32_t) (a))

#endif /* _PCI_MACHDEP_H_ */
