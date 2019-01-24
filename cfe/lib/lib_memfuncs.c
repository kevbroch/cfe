/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Memory mangling functions		File: lib_memfuncs.c
    *  
    *  Routines that the user interface commands use to modify
    *  memory (peek, poke, etc.)  Most are wrapped in exception
    *  handler clothing.
    *  
    *  Author:  Binh Vo, Mitch Lichtenberg
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
#include "lib_hssubr.h"
#include "lib_try.h"
#include "lib_memfuncs.h"



/*  *********************************************************************
    *  mem_peek(d,addr,type)
    *  
    *  Read memory of the specified type at the specified address.
    *  Exceptions are caught in the case of a bad memory reference.
    *  
    *  Input parameters: 
    *  	   d - pointer to where data should be placed
    *  	   addr - address to read
    *  	   type - type of read to do (MEM_BYTE, etc.)
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else error code
    ********************************************************************* */

int mem_peek(void *d, hsaddr_t addr, int type)
{

    jmpbuf_t *jb;

    jb = exc_initialize_block();
    if( jb == NULL ) {
	return CFE_ERR_NOMEM;
	}

    if (exc_try(jb) == 0) {
  
	switch (type) {
	    case MEM_BYTE:
		*(uint8_t *)d = hs_read8(addr);
		break;
	    case MEM_HALFWORD:
		*(uint16_t *)d = hs_read16(addr);
		break;
	    case MEM_WORD:
		*(uint32_t *)d = hs_read32(addr);
		break;
	    case MEM_QUADWORD:
		*(uint64_t *)d = hs_read64(addr);
		break;
	    default:
		return CFE_ERR_INV_PARAM;
	    }

	exc_cleanup_block(jb);
	}
    else {
	/*Exception handler*/

	exc_cleanup_handler(jb, EXC_NORMAL_RETURN);
	return CFE_ERR_GETMEM;
	}

    return 0;
}

/* *********************************************************************
   *  Write memory of type at address addr with value val. 
   *  Exceptions are caught, handled (error message) and function 
   *  returns with 0. 
   *
   *  1 success
   *  0 failure
   ********************************************************************* */

int mem_poke(hsaddr_t addr, uint64_t val, int type)
{

    jmpbuf_t *jb;

    jb = exc_initialize_block();
    if( jb == NULL ) {
	return CFE_ERR_NOMEM;
	}

    if (exc_try(jb) == 0) {
  
	switch (type) {
	    case MEM_BYTE:
		hs_write8(addr,(uint8_t)val);
		break;
	    case MEM_HALFWORD:
		hs_write16(addr,(uint16_t)val);
		break;
	    case MEM_WORD:
		hs_write32(addr,(uint32_t)val);
		break;
	    case MEM_QUADWORD:
		hs_write64(addr,(uint64_t)val);
		break;
	    default:
		return CFE_ERR_INV_PARAM;
	    }

	exc_cleanup_block(jb);
	}
    else {
	/*Exception handler*/

	exc_cleanup_handler(jb, EXC_NORMAL_RETURN);
	return CFE_ERR_SETMEM;
	}

    return 0;
}
 



