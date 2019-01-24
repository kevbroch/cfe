/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *
    *  PCI Table Generator			File: mkpcidb.c
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *  This program munges the PCI table into a form that uses
    *  fewer embedded pointers.  Pointers are evil for the
    *  relocatable version of CFE since they chew up valuable
    *  initialized data segment space, and we only have
    *  64KB of that.
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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned short pci_vendor_id_t;
typedef unsigned short pci_product_id_t;

struct pci_knowndev {
    pci_vendor_id_t     vendor;
    pci_product_id_t    product;
    int                 flags;
    char                *vendorname, *productname;
};

#include "pcidevs.h"
#define PCI_KNOWNDEV_NOPROD 0x01
#include "pcidevs_data.h" 



struct pci_knowndev2 {
    pci_vendor_id_t vendor;
    pci_product_id_t product;
    int vendorname;
    int productname;
};

#define MAXPCIDEVS 5000
#define MAXSTRINGTABLE (1024*1024)

struct pci_knowndev2 knowndevs[MAXPCIDEVS];
char stringtable[MAXSTRINGTABLE];
int curstringptr = 0;

static void initintern (void);
static int intern (char *string);

int main(int argc,char *argv[])
{
    struct pci_knowndev2 *outdev;
    const struct pci_knowndev *indev;
    char *stringoff_type;
    int cnt = 0;
    int idx;

    indev = pci_knowndevs;
    outdev = knowndevs;
    cnt = 0;

    initintern();

    while (indev->vendorname) {
	assert (cnt < MAXPCIDEVS);
	outdev->vendor = indev->vendor;
	outdev->product = indev->product;
	outdev->vendorname = intern(indev->vendorname);
	outdev->productname = intern(indev->productname);
	cnt++;
	indev++;
	outdev++;
	}

    outdev->vendor = 0;
    outdev->product = 0;
    outdev->vendorname = -1;
    outdev->productname = -1;
    cnt++;

    if (curstringptr <= 65534)
      stringoff_type = "unsigned short";
    else
      stringoff_type = "unsigned int";

    fprintf(stderr,"%d total devices (%d bytes), %d bytes of strings\n",
	   cnt,
	   cnt * (4 + 2 * (curstringptr <= 65534 ? 2 : 4)),
	   curstringptr);

    printf("struct pci_knowndev2 {\n");
    printf("    pci_vendor_id_t     vendor;\n");
    printf("    pci_product_id_t    product;\n");
    printf("    %s vendorname, productname;\n", stringoff_type);
    printf("};\n");

    printf("\n\n\n");
    printf("const static struct pci_knowndev2 _pci_knowndevs[] __attribute__ ((section (\".text\"))) = {\n");
    for (idx = 0; idx < cnt; idx++) {
	printf("\t{0x%04X,0x%04X,%d,%d},\n",
	       knowndevs[idx].vendor,
	       knowndevs[idx].product,
	       knowndevs[idx].vendorname,
	       knowndevs[idx].productname);
	}
    printf("};\n\n\n");
    printf("const static char _pci_knowndevs_text[] __attribute__ ((section (\".text\"))) = {\n");
    for (idx = 0; idx < curstringptr; idx++) {
	if ((idx % 16) == 0) printf("\t");
	printf("0x%02X,",stringtable[idx]);
	if ((idx % 16) == 15) printf("\n");
	}
    printf("};\n\n");

    printf("static const struct pci_knowndev2 *pci_knowndevs = _pci_knowndevs;\n");
    printf("static const char *pci_knowndevs_text = _pci_knowndevs_text;\n");
    printf("#define PCI_STRING_NULL ((%s)-1)\n", stringoff_type);
    printf("#define PCI_STRING(x) (&pci_knowndevs_text[(x)])\n\n");



    exit(0);
}


/*
 * Internalize strings so that duplicates aren't emitted to the string
 * table.
 */
char *istrings[MAXPCIDEVS * 2];
int nistrings;

static void
initintern(void)
{
    nistrings = 0;
}

static int
intern(char *string)
{
    int ptr;
    int si;

    if (!string) return -1;

    /* This is N^2, but will usually hit in the first entry checked if
       it hits at all.  */
    for (si = nistrings - 1; si >= 0; si--) {
      if (strcmp(string, istrings[si]) == 0)
        return (istrings[si] - stringtable);
    }

    assert (si < (MAXPCIDEVS * 2));

    ptr = curstringptr;
    istrings[nistrings++] = &stringtable[ptr];

    strcpy(&stringtable[ptr],string);
    curstringptr += strlen(string)+1;

    return ptr;
}
