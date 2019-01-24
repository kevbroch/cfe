/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Flash write module		File: dev_generic_flashop.c
    *  
    *  Generic "C" version of the flashop engine that we can use
    *  on any platform that supports relocation.
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2004,2005
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
#include "dev_newflash.h"
#include "lib_physio.h"

#if (!FLASH_XOR_ADDR)
#define FLASHCMD_8(base,offset,value) phys_write8((base)+(offset),(value))
#define FLASHCMD_16(base,offset,value) phys_write16((base)+((offset)<<1),(value))
#define FLASHCMD_16B(base,offset,value) phys_write8((base)+((offset)<<1),(value))
#else
/* Invert the low order bits so that hw inversion will restore them. */
#define FLASHCMD_8(base,offset,value) phys_write8(((base)+(offset))^3,(value))
#define FLASHCMD_16(base,offset,value) phys_write16(((base)+((offset)<<1))^2,(value))
#define FLASHCMD_16B(base,offset,value) phys_write8(((base)+((offset)<<1))^3,(value))
#endif

long flashop_engine_generic(flashinstr_t *inst);

long flashop_engine_generic(flashinstr_t *inst)
{
    long result = 0;
    volatile uint8_t *dest8;
    volatile uint16_t *dest16;
    volatile uint8_t *src8;
    volatile uint16_t *src16;
    uint16_t w16,b16;
    uint8_t w8,b8,wrrev;
    uint32_t w32;
    long cnt,cnt2,wrbufsz;
    physaddr_t base;
    physaddr_t pptr;
    physaddr_t dsave;
    

    while (inst->fi_op != FEOP_RETURN) {

	base = inst->fi_base;
	cnt = inst->fi_cnt;

	switch (inst->fi_op) {
	    case FEOP_RETURN :
		break;

	    case FEOP_REBOOT :
		break;

	    case FEOP_READ8 :
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;
		while (cnt > 0) {
		    *dest8++ = phys_read8(pptr++);
		    cnt--;
		    }
		break;

	    case FEOP_READ16 :
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;

		if ((cnt > 0) && (pptr & 1)) {
		    w16 = phys_read16(pptr & ~1);
#if !ENDIAN_BIG
		    w16 >>= 8;
#endif
		    *dest8++ = (uint8_t) w16;
		    pptr++;
		    cnt--;
		    }

		while (cnt > 1) {
		    w16 = phys_read16(pptr);
#if ENDIAN_BIG
		    *dest8++ = (uint8_t) (w16 >> 8);
		    *dest8++ = (uint8_t) (w16 & 0xFF);
#else
		    *dest8++ = (uint8_t) (w16 & 0xFF);
		    *dest8++ = (uint8_t) (w16 >> 8);
#endif
		    pptr+=2;
		    cnt-=2;
		    }

		if (cnt > 0) {
		    w16 = phys_read16(pptr);
#if ENDIAN_BIG
		    w16 >>= 8;
#endif
		    *dest8++ = (uint8_t) w16;
		    }

		    break;

/* CFI - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if (FLASH_DRIVERS & FLASH_DRIVER_CFI)
	    case FEOP_CFIQUERY8 :
#if FLASH_XOR_ADDR
		wrrev = 3;
#else
		wrrev = 0;
#endif
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;

		FLASHCMD_8(base,FLASH_CFI_QUERY_ADDR,FLASH_CFI_QUERY_MODE);

		while (cnt > 0) {
		    *dest8++ = phys_read8(pptr^wrrev);
		    pptr++;
		    cnt--;
		    }
		
		FLASHCMD_8(base,FLASH_CFI_QUERY_ADDR,FLASH_CFI_QUERY_EXIT);
		break;

	    case FEOP_CFIQUERY16 :
#if FLASH_XOR_ADDR
		wrrev = 2;
#else
		wrrev = 0;
#endif
		pptr = base + inst->fi_src;
		dest16 = (volatile uint16_t *) inst->fi_dest;

		FLASHCMD_16(base,FLASH_CFI_QUERY_ADDR,FLASH_CFI_QUERY_MODE);

		while (cnt > 1) {
		    *dest16++ = phys_read16(pptr^wrrev);
		    pptr += 2;
		    cnt -= 2;
		    }
		
		FLASHCMD_16(base,FLASH_CFI_QUERY_ADDR,FLASH_CFI_QUERY_EXIT);
		break;

	    case FEOP_CFIQUERY16B :
#if FLASH_XOR_ADDR
		wrrev = 3;
#else
		wrrev = 0;
#endif
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;

		FLASHCMD_16B(base,FLASH_CFI_QUERY_ADDR,FLASH_CFI_QUERY_MODE);

		while (cnt > 0) {
		    *dest8++ = phys_read8(pptr^wrrev);
		    pptr++;
		    cnt--;
		    }
		
		FLASHCMD_16B(base,FLASH_CFI_QUERY_ADDR,FLASH_CFI_QUERY_EXIT);
		break;
#endif

	    case FEOP_MEMCPY :
		dest8 = (volatile uint8_t *) inst->fi_dest;
		src8 = (volatile uint8_t *) inst->fi_src;
		while (cnt > 0) {
		    *dest8++ = *src8++;
		    cnt--;
		    }
		break;

/* AMD  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if (FLASH_DRIVERS & FLASH_DRIVER_AMD)
	    case FEOP_AMD_ERASE8 :
		pptr = base + inst->fi_dest;

		/* Do an "unlock write" sequence  (cycles 1-2) */

		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);

		/* send the erase command (cycle 3) */

		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_ERASE_3);

		/* Do an "unlock write" sequence (cycles 4-5) */

		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);

		/* Send the "erase sector" qualifier (cycle 6) */

		FLASHCMD_8(pptr,0,AMD_FLASH_ERASE_SEC_6);

		/* Wait for the erase to complete */

		while (phys_read8(pptr) != 0xFF) ; /* null loop */

		break;

	    case FEOP_AMD_ERASE16 :
		pptr = base + inst->fi_dest;

		/* Do an "unlock write" sequence  (cycles 1-2) */

		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);

		/* send the erase command (cycle 3) */

		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_ERASE_3);

		/* Do an "unlock write" sequence (cycles 4-5) */

		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);

		/* Send the "erase sector" qualifier (cycle 6) */

		FLASHCMD_16(pptr,0,AMD_FLASH_ERASE_SEC_6);

		/* Wait for the erase to complete */

		while ((phys_read16(pptr) & 0xFF) != 0xFF) ; /* null loop */

		break;

	    case FEOP_AMD_ERASE16B :
		pptr = base + inst->fi_dest;

		/* Do an "unlock write" sequence  (cycles 1-2) */

		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);

		/* send the erase command (cycle 3) */

		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_ERASE_3);

		/* Do an "unlock write" sequence (cycles 4-5) */

		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);

		/* Send the "erase sector" qualifier (cycle 6) */

		FLASHCMD_16B(pptr,0,AMD_FLASH_ERASE_SEC_6);

		/* Wait for the erase to complete */

		while ((phys_read8(pptr) & 0xFF) != 0xFF) ; /* null loop */

		break;

	    case FEOP_AMD_PGM8 :
		pptr = base + inst->fi_dest;
		src8 = (volatile uint8_t *) inst->fi_src;

		while (cnt > 0) {

		    FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		    FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);

		    /* Send a program command (cycle 3) */

		    FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_PROGRAM);

		    /* Write a byte (cycle 4) */

		    b8 = *src8;
		    phys_write8(pptr,b8);

		    /* Wait for write to complete */

		    for (;;) {
			w8 = phys_read8(pptr);
			if ((w8 & 0x80) == (b8 & 0x80)) break;
			if ((w8 & 0x20) != 0x20) continue;
			break;
			}

		    cnt--;
		    pptr++;
		    src8++;
		    }

		break;

	    case FEOP_AMD_PGM16 :
		pptr = base + inst->fi_dest;
		src16 = (volatile uint16_t *) inst->fi_src;

		while (cnt > 0) {

		    /* Do an "unlock write" sequence  (cycles 1-2) */

		    FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		    FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);

		    /* Send a program command (cycle 3) */

		    FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_PROGRAM);

		    /* Write a byte (cycle 4) */

		    b16 = *src16;
		    phys_write16(pptr,b16);

		    /* Wait for write to complete */

		    for (;;) {
			w16 = phys_read16(pptr);
			if ((w16 & 0x80) == (b16 & 0x80)) break;
			if ((w16 & 0x20) != 0x20) continue;
			break;
			}

		    pptr += 2;
		    cnt -= 2;
		    src16++;
		    }

		break;

	    case FEOP_AMD_PGM16B :
		pptr = base + inst->fi_dest;
		src8 = (volatile uint8_t *) inst->fi_src;

		while (cnt > 0) {

		    FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		    FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);

		    /* Send a program command (cycle 3) */

		    FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_PROGRAM);

		    /* Write a byte (cycle 4) */

		    b8 = *src8;
		    phys_write8(pptr,b8);

		    /* Wait for write to complete */

		    for (;;) {
			w8 = phys_read8(pptr);
			if ((w8 & 0x80) == (b8 & 0x80)) break;
			if ((w8 & 0x20) != 0x20) continue;
			break;
			}

		    cnt--;
		    pptr++;
		    src8++;
		    }

		break;

	    case FEOP_AMD_DEVCODE8 :
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;

		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);
		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_AUTOSEL);
      
		w8 = phys_read8(pptr);
		*dest8 = w8;

		phys_write8(base,AMD_FLASH_RESET);

		break;

	    case FEOP_AMD_DEVCODE16 :
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;

		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);
		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_AUTOSEL);
      
		w32 = phys_read32(pptr);
#if (ENDIAN_BIG && !FLASH_XOR_ADDR)
		*dest8 = (uint8_t)(w32 >> 8);
#else
		*dest8 = (uint8_t)(w32 >> 16);
#endif
		phys_write8(base,AMD_FLASH_RESET);

		break;

	    case FEOP_AMD_DEVCODE16B :
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;

		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);
		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_AUTOSEL);
      
		w32 = phys_read32(pptr);
#if (ENDIAN_BIG && !FLASH_XOR_ADDR)
		*dest8 = (uint8_t) w32;
#else
		*dest8 = (uint8_t) (w32 >> 16);
#endif
		phys_write8(base,AMD_FLASH_RESET);

		break;

	    case FEOP_AMD_MANID8 :
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;

		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);
		FLASHCMD_8(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_AUTOSEL);

		w8 = phys_read8(pptr + AMD_FLASH_MANID);
		*dest8 = w8;

		phys_write8(base,AMD_FLASH_RESET);

		break;

	    case FEOP_AMD_MANID16 :
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;

		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);
		FLASHCMD_16(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_AUTOSEL);

		w32 = phys_read32(pptr);

#if (ENDIAN_BIG && !FLASH_XOR_ADDR)
		*dest8 = (uint8_t) (w32 >> ((3-AMD_FLASH_MANID)*8));
#else
		*dest8 = (uint8_t) (w32 >> (AMD_FLASH_MANID*8));
#endif

		phys_write8(base,AMD_FLASH_RESET);

	    case FEOP_AMD_MANID16B :
		pptr = base + inst->fi_src;
		dest8 = (volatile uint8_t *) inst->fi_dest;

		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_MAGIC_1);
		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_2,AMD_FLASH_MAGIC_2);
		FLASHCMD_16B(base,AMD_FLASH_MAGIC_ADDR_1,AMD_FLASH_AUTOSEL);

		w8 = phys_read8(pptr + AMD_FLASH_MANID);

		*dest8 = w8;

		phys_write8(base,AMD_FLASH_RESET);
		break;
#endif

/* INTEL  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#if (FLASH_DRIVERS & FLASH_DRIVER_INTEL)
	    case FEOP_INTEL_ERASE8 :

		pptr = base + inst->fi_dest;

		FLASHCMD_8(pptr,0,INTEL_FLASH_ERASE_BLOCK);
		FLASHCMD_8(pptr,0,INTEL_FLASH_ERASE_CONFIRM);

		while ((phys_read8(pptr) & 0x80) == 0) ; /* NULL LOOP */

		FLASHCMD_8(pptr,0,INTEL_FLASH_READ_MODE);
		break;

	    case FEOP_INTEL_ERASE16 :
		pptr = base + inst->fi_dest;

		FLASHCMD_16(pptr,0,INTEL_FLASH_ERASE_BLOCK);
		FLASHCMD_16(pptr,0,INTEL_FLASH_ERASE_CONFIRM);

		while ((phys_read16(pptr) & 0x80) == 0) ; /* NULL LOOP */

		FLASHCMD_16(pptr,0,INTEL_FLASH_READ_MODE);
		break;

	    case FEOP_INTEL_PGM8 :
		pptr = base + inst->fi_dest;
		src8 = (volatile uint8_t *) inst->fi_src;

		while (cnt > 0) {
		    FLASHCMD_8(pptr,0,INTEL_FLASH_PROGRAM);

		    b8 = *src8;
		    phys_write8(pptr,b8);

		    while ((phys_read8(pptr) & 0x80) == 0) ; /* NULL LOOP */
		    
		    src8++;
		    pptr++;
		    cnt--;
		    }

		FLASHCMD_8(pptr,0,INTEL_FLASH_READ_MODE);
		break;

	    case FEOP_INTEL_PGM16 :
		pptr = base + inst->fi_dest;
		src16 = (volatile uint16_t *) inst->fi_src;

		while (cnt > 0) {
		    FLASHCMD_16(pptr,0,INTEL_FLASH_PROGRAM);

		    b16 = *src16;
		    phys_write16(pptr,b16);

		    while ((phys_read16(pptr) & 0x80) == 0) ; /* NULL LOOP */
		    
		    src16++;
		    pptr+=2;
		    cnt-=2;
		    }

		FLASHCMD_16(pptr,0,INTEL_FLASH_READ_MODE);
		break;

	    case FEOP_INTEL_PGM32B :
#if FLASH_XOR_ADDR
                /* Write in reverse order (no swap!).

		   XXX This trick works only if the source and destination
		   have the same alignment with respect to 32-bit word
		   boundaries.  For simplicity, the current code assumes
		   32-bit alignment and a length that is a multiple of
		   32 bytes.  The dev_newflash driver always writes full
		   sectors and guarantees this.  Other applicatons (e.g., 
		   dribble updates) should not use this operation.
		*/
		wrrev = 3;
#else
		wrrev = 0;
#endif
		pptr = base + inst->fi_dest;
		src8 = (volatile uint8_t *) inst->fi_src;
		dsave = pptr;
		
		while (cnt > 0) {
		    FLASHCMD_8(pptr,0,INTEL_FLASH_WRITE_BUFFER);
		    while ((phys_read8(pptr) & 0x80) == 0) ; /* NULL LOOP */
		    wrbufsz = 32;
		    phys_write8(pptr,(wrbufsz-1));	/* Device wants n-1 value */
		    for (cnt2 = 0; cnt2 < wrbufsz; cnt2++) {
			phys_write8(pptr^wrrev,src8[cnt2^wrrev]);
			pptr++;
			cnt--;
			}
		    src8 += wrbufsz;
		    FLASHCMD_8(dsave,0,INTEL_FLASH_ERASE_CONFIRM); /* Program buffer to flash */
		    while ((phys_read8(dsave) & 0x80) == 0) ; /* NULL LOOP */ 
		    }

		FLASHCMD_8(dsave,0,INTEL_FLASH_READ_STATUS);
		while ((phys_read8(dsave) & 0x80) == 0) ; /* NULL LOOP */

		phys_read8(dsave);

		FLASHCMD_8(dsave,0,INTEL_FLASH_READ_MODE);
		break;
#endif /* FLASH_DRIVER_INTEL */

	    }

	inst++;
	}

    return result;
}

void *flashop_engine_ptr = flashop_engine_generic;
int flashop_engine_len  = 1024; /* gack! */
