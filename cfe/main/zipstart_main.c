/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  ZipStart main module			File: zipstart_main.c
    *  
    *  Main "C" entry point for ZipStart, our mini loader.
    *  
    *  Author:  Mitch Lichtenberg (mpl@broadcom.com)
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
#include "zipstart.h"


/*  *********************************************************************
    *  Binary data area -- the linker script creates these variables
    *  for us, and the data is placed into the text section
    *  since it is decompressed/copied directly from the flash.
    ********************************************************************* */

#if CFG_ZLIB
extern uint8_t __attribute__ ((section(".text"))) _binary_ramcfe_bin_gz_start;
extern uint8_t __attribute__ ((section(".text"))) _binary_ramcfe_bin_gz_end;
#else
extern uint32_t __attribute__ ((section(".text"))) _binary_ramcfe_bin_start;
extern uint32_t __attribute__ ((section(".text"))) _binary_ramcfe_bin_end;
#endif

/*  *********************************************************************
    *  Globals
    ********************************************************************* */

static uint8_t *heapptr;		/* our pretend "heap" */

/*  *********************************************************************
    *  External Data
    ********************************************************************* */

extern long mem_heapstart;		/* Where to put our pretend "heap" */

/*  *********************************************************************
    *  cfe_ledstr(str)
    *  
    *  Display a message on the LEDs via the BSP's "board_setleds"
    *  routine.  
    *  
    *  Input parameters: 
    *  	   str - pointer to 4-character string
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void cfe_ledstr(const char *str)
{
    uint32_t led = 
	(((uint32_t) str[0]) << 24) | 
	(((uint32_t) str[1]) << 16) | 
	(((uint32_t) str[2]) << 8) | 
	(((uint32_t) str[3]) << 0);

    board_setleds(led);
}

/*  *********************************************************************
    *  putstr(str)
    *  
    *  Display a text string to the console via board_conout()
    *  
    *  Input parameters: 
    *  	   str - pointer to null-terminated string
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void putstr(const char *str)
{
    while (*str) board_conout(*str++);
}


/*  *********************************************************************
    *  putnum(x,base)
    *  
    *  Display a number via board_conout.
    *  
    *  Input parameters: 
    *  	   x - number to display
    *  	   base - number base, usually 10 or 16
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void putnum(uint32_t x,int base)
{
    int idx;
    int digit;
    char num[9];

    for (idx = 0; idx < 8; idx++) {
	digit = (x % base);
	num[7-idx] = (digit > 9) ? (digit - 10 + 'A') : (digit + '0');
	x /= base;
	if ((base == 10) && (x == 0)) {
	    idx++;
	    break;
	    }
	}
    num[8] = 0;
    putstr(&num[8-idx]);
}

/*  *********************************************************************
    *  puthex(x)
    *  
    *  Display a number in hex with a leading 0x prefix.
    *  
    *  Input parameters: 
    *  	   x - number
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void puthex(uint32_t x)
{
    putstr("0x");
    putnum(x,16);
}

/*  *********************************************************************
    *  putdec(x)
    *  
    *  Display a number in decimal.
    *  
    *  Input parameters: 
    *  	   x - number
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void putdec(uint32_t x)
{
    putnum(x,10);
}


/*  *********************************************************************
    *  zs_exception(code,framedata)
    *  
    *  This routine is called when we encounter an exception.
    *  
    *  Input parameters: 
    *  	   code - which exception vector was called
    *  	   framedata - points at stack frame (see exception.h)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static char *regnames = "SR\0CAUSE\0EPC\0VADDR\0PRID\0";

void zs_exception(uint32_t code,uint64_t *framedata)
{
    int idx;
    char *label = regnames;

    putstr("*** Exception "); puthex(code); putstr("\r\n");

    /* XXX Kinda MIPS specific here */
    for (idx = 0; idx < 5; idx++) {
	putstr(label);
	while (*label) label++;
	label++;
	putstr(" = ");
	puthex((uint32_t)framedata[idx]);
	putstr("\r\n");
	}
    
    for (;;) ;
}


/*  *********************************************************************
    *  zs_copybin()
    *  
    *  Copy a raw binary into memory.  The destination address
    *  is fromt he configuration parameter CFG_TEXT_START, and the
    *  source address is from the linker script (wherever the 
    *  CFE binary ended up in zipstart's image).  
    *  
    *  This routine is used for "static relocation" of CFE. 
    *  Dynamic relocation is typicaly done directly fromt he flash
    *  using "embedded PIC" or SVR4 PIC.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

#if !CFG_ZLIB
static void zs_copybin(void)
{
    uint32_t *dest;
    uint32_t *src;
    uint32_t *last;
    
    dest = (uint32_t *) CFE_TEXT_START;
    src = &_binary_ramcfe_bin_start;
    last = &_binary_ramcfe_bin_end;

#ifdef _ZIPSTART_VERBOSE_
    putstr("Copying "); puthex((uint32_t)src); putstr("-"); puthex((uint32_t)last); 
    putstr(" to "); puthex((uint32_t)dest); putstr("\r\n");
#endif

    while (src < last) *dest++ = *src++;
}
#endif

/*  *********************************************************************
    *  zs_unzip()
    *  
    *  Decompresses CFE into memory.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

#if CFG_ZLIB
static int zs_unzip(void)
{
    uint8_t *dest;
    uint8_t *src;
    uint8_t *last;
    int res;
    
    dest = (uint8_t *) CFE_TEXT_START;
    src = (uint8_t *) &_binary_ramcfe_bin_gz_start;
    last = (uint8_t *) &_binary_ramcfe_bin_gz_end;

#ifdef _ZIPSTART_VERBOSE_
    putstr("Unzipping "); puthex((uint32_t)src); putstr("-"); puthex((uint32_t)last); 
    putstr(" to "); puthex((uint32_t)dest); putstr("\r\n");
#endif

    res = rz_init((uint8_t *) src,(last-src)*sizeof(uint32_t));

    if (res < 0) {
	putstr("rz_init failed\r\n");
	return -1;
	}

    for (;;) {
	res = rz_read(dest,512);
	if (res < 0) {
	    putstr("rz_read failed\r\n");
	    return -1;
	    }
	if (res == 0) break;
	dest += res;
	}

    return 0;
}
#endif


/*  *********************************************************************
    *  zs_main(a,b,c,d)
    *  
    *  The main "C" entry point for zipstart.  
    *  
    *  Input parameters: 
    *  	   a,b,c,d - parameters from assembly language
    *  	   
    *  Return value:
    *  	   nothing (does not return)
    ********************************************************************* */

void zs_main(long a,long b,long c,long d)
{
    /*
     * Initialize our pretend heap
     */

    heapptr = (uint8_t *) mem_heapstart;

    /*
     * Say hello.
     */

    putstr("\r\nCFE bootstrap v"); 
    putdec(CFE_VER_MAJOR); putstr("."); putdec(CFE_VER_MINOR); 
    putstr("."); putdec(CFE_VER_BUILD); 
    putstr(" for "); putstr(CFG_BOARDNAME); putstr("\r\n");

#if CFG_ZLIB
    putstr("Decompressing firmware.\r\n");
    SETLEDS("UNZP");
#else
    putstr("Loading firmware.\r\n");
    SETLEDS("LOAD");
#endif

    /*
     * Copy CFE into final location.
     */

#if CFG_ZLIB
    zs_unzip();
#else
    zs_copybin();
#endif

    /*
     * Start CFE.  Note that cfe_launch calls the board package's
     * cache routines to flush/invalidate the caches.
     * We also assume that the entry point is the first
     * instruction in the binary.
     */

    putstr("Starting firmware.\r\n");
    cfe_launch(CFE_TEXT_START);

    /* does not return */
}


/*  *********************************************************************
    *  zs_malloc(size)
    *  
    *  Allocate some memory.  Since we don't anticpate using
    *  this very much or very often, we don't bother to free 
    *  anything - just grab a chunk and return it.
    *  
    *  Input parameters: 
    *  	   size - number of bytes to allocate
    *  	   
    *  Return value:
    *  	   pointer to allocated memory
    ********************************************************************* */

void *zs_malloc(int size)
{
    uint8_t *ptr = heapptr;
    heapptr = (uint8_t *) ((((long) heapptr) + size + 31) & ~31);
    return ptr;
}

/*  *********************************************************************
    *  zs_free(ptr)
    *  
    *  Free allocated memory.
    *  
    *  (or not - we don't really do this)
    *  
    *  Input parameters: 
    *  	   ptr - pointer to memory block
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void zs_free(void *x)
{
}

