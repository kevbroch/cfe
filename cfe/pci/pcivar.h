/*
 * Copyright (c) 1994 Charles Hannum.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Charles Hannum.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PCIVAR_H_
#define _PCIVAR_H_

/*
 * Definitions for PCI autoconfiguration.
 *
 * This file describes types and functions which are exported for PCI
 * configuration and address mapping.
 */

#include "pci_machdep.h"

/* From <cpu>_pci_machdep.c */

pcitag_t  pci_make_tag(int, int, int, int);
pcireg_t  pci_conf_read(pcitag_t, int);
void	  pci_conf_write(pcitag_t, int, pcireg_t);

typedef enum {
    PCI_MATCH_BYTES = 0,
    PCI_MATCH_BITS  = 1
} pci_endian_t;

int       pci_map_io(pcitag_t, int, pci_endian_t, phys_addr_t *);
int       pci_map_mem(pcitag_t, int, pci_endian_t, phys_addr_t *);

phys_addr_t pci_msi_clear_addr(uint64_t);

uint8_t   inb(unsigned int port);
uint16_t  inw(unsigned int port);
uint32_t  inl(unsigned int port);
void      outb(unsigned int port, uint8_t val);
void      outw(unsigned int port, uint16_t val);
void      outl(unsigned int port, uint32_t val);

int       pci_map_window(phys_addr_t pa,
			 unsigned int offset, unsigned int len,
			 int l2ca, int endian);
int       pci_unmap_window(unsigned int offset, unsigned int len);

/* From pciconf.c */

/* Flags controlling PCI/LDT configuration options */

typedef unsigned int pci_flags_t;

#define PCI_FLG_NORMAL	      0x00000001
#define PCI_FLG_VERBOSE       0x00000003
#define PCI_FLG_LDT_REV_017   0x00000008

void	  pci_configure(pci_flags_t flags);
void      pci_show_configuration(void);
int	  pci_foreachdev(int (*fn)(pcitag_t tag));
int	  pci_cacheline_log2 (void);
int	  pci_maxburst_log2 (void);

int pci_find_device(uint32_t vid, uint32_t did, int enumidx, pcitag_t *tag);
int pci_find_class(uint32_t class, int enumidx, pcitag_t *tag);

/* From pci_subr.c */

void	  pci_devinfo(pcireg_t, pcireg_t, int, char *);
void	  pci_conf_print(pcitag_t tag);

void	  pci_tagprintf (pcitag_t tag, const char *fmt, ...)
#ifdef __long64
	    __attribute__((__format__(__printf__,2,3)))
#endif
            ;

#endif /* _PCIVAR_H_ */
