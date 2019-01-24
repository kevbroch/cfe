/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Hyperspace Subroutines			File: lib_hssubr.S
    *  
    *  Little stub routines to allow access to KXSEG from 32-bit progs.
    *
    *  This module is intended for use on 64-bit platforms only.
    *  The KX bit in the status register is assumed to be SET.
    *  We also assume we're compiling for mips4 and pass 64-bit values
    *  around in registers.
    *
    *  There's no reason this couldn't have been done with "C" macros
    *  or inline assembly.
    *  
    *  Author:  Mitch Lichtenberg
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


#include "sbmips.h"
#include "cpu_config.h"		/* for definition of HAZARD */
#include "mipsmacros.h"


/*  *********************************************************************
    *  hs_read8 - read 8-bit bytes
    ********************************************************************* */


LEAF(hs_read8)
		lbu	v0,(a0)
		j	ra
END(hs_read8)

/*  *********************************************************************
    *  hs_read16 - read 16-bit shorts
    ********************************************************************* */

LEAF(hs_read16)
		lhu	v0,(a0)
		j	ra
END(hs_read16)

/*  *********************************************************************
    *  hs_read32 - read 32-bit ints
    ********************************************************************* */

LEAF(hs_read32)
		lw	v0,(a0)
		j	ra
END(hs_read32)

/*  *********************************************************************
    *  hs_read64 - read 64-bit longs
    ********************************************************************* */

#if CPUCFG_REGS64
LEAF(hs_read64)
		ld	v0,(a0)
		j	ra
END(hs_read64)
#endif

/*  *********************************************************************
    *  hs_write8 - write 8-bit bytes
    ********************************************************************* */

LEAF(hs_write8)
		sb	a1,(a0)
		j	ra
END(hs_write8)

/*  *********************************************************************
    *  hs_write16 - write 16-bit shorts
    ********************************************************************* */

LEAF(hs_write16)
		sh	a1,(a0)
		j	ra
END(hs_write16)

/*  *********************************************************************
    *  hs_write32 - write 32-bit longs
    ********************************************************************* */

LEAF(hs_write32)
		sw	a1,(a0)
		j	ra
END(hs_write32)

/*  *********************************************************************
    *  hs_write64 - write 64-bit longs
    ********************************************************************* */

#if CPUCFG_REGS64
LEAF(hs_write64)
		sd	a1,(a0)
		j	ra
END(hs_write64)
#endif

/*  *********************************************************************
    *  hs_memcpy(dest,src,cnt)
    *  
    *  Copy data to/from hyperspace.   This routine is slimy; it
    *  assumes that we're compiled -mips4 and we'll pass addresses
    *  on the stack as 64 bits.
    *  
    *  Input parameters: 
    *  	   dest - destination address (hsaddr)
    *  	   src - source address (regular pointer)
    *  	   cnt - count
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(hs_memcpy_from_hs)
	b	_hs_memcpy_to_hs
END(hs_memcpy_from_hs)

LEAF(hs_memcpy_to_hs)
_hs_memcpy_to_hs:

1:	beqz	a2,2f

	lbu	t0,0(a1)		/* copying one byte at a time */
	sb	t0,0(a0)		/* because I'm lazy and it isn't worth it. */
	ADDPTR	a0,a0,1
	ADDPTR	a1,a1,1
	subu	a2,a2,1
	b	1b

2:	j	ra

END(hs_memcpy_to_hs)

/*  *********************************************************************
    *  hs_memset(dest,c,cnt)
    *  
    *  Write values to a buffer.  Like memcpy, we do this knowing we're
    *  compiled __misp64 so we can access 64-bit registers.
    *  
    *  Input parameters: 
    *  	   dest - destination buffer
    *  	   c - character to write
    *  	   cnt - number of bytes to write
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(hs_memset)

1:	beqz	a2,2f

	sb	a1,0(a0)
	addiu	a0,a0,1
	subu	a2,a2,1
	b	1b	

2:	j	ra

END(hs_memset)


/*  *********************************************************************
    *  End
    ********************************************************************* */

