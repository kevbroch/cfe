/*
 * Copyright (c) 2001,2002,2003 SiByte, Inc.  All rights reserved.
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
#define PCI_HOST_PORTS 2

/* All mappings through the PCI host bridge use match bits mode. */
#define PHYS_TO_PCI(a) ((uint32_t) (a) | 0x20000000)
#define PCI_TO_PHYS(a) ((uint32_t) (a) & 0xDFFFFFFF)

/* export routine to issue a ht link reset */
void ht_port_link_reset( unsigned int port );


/*  *********************************************************************
    *  PCI register stuff.  In HT mode, we need to talk to the
    *  HT bridges to set up PoHT functions
    ********************************************************************* */

#define	BCM1480_PCI_MAKE_TAG(p,b,d,f)					\
    (((p) << 24) | ((b) << 16) | ((d) << 11) | ((f) << 8))
#define BCM1480_HOST_PORT(tag) (((tag) >> 24) & 0xFF)


/* The BCM1480 integrated host bridges. */

#define	BCM1480_LDT_BRIDGE	(BCM1480_PCI_MAKE_TAG(1,0,4,0))

/* The BCM1480 integrated external PCI/HT bridges. */

#define BCM1480_EXT0_BRIDGE     (BCM1480_PCI_MAKE_TAG(1,0,0,0))
#define BCM1480_EXT1_BRIDGE     (BCM1480_PCI_MAKE_TAG(1,0,1,0))
#define BCM1480_EXT2_BRIDGE     (BCM1480_PCI_MAKE_TAG(1,0,2,0))
#define BCM1480_EXTx_BRIDGE(x)  (BCM1480_PCI_MAKE_TAG(1,0,(x),0))

#define BCM1480_EXTx_BRIDGE_BUS1(x)  (BCM1480_PCI_MAKE_TAG(1,1,(x),0))
#define BCM1480_EXTx_BRIDGE_BUS2(x)  (BCM1480_PCI_MAKE_TAG(1,2,(x),0))

#endif /* _PCI_MACHDEP_H_ */
