/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Physical memory peek/poke routines	File: lib_physio.S
    *  
    *  Little stub routines to allow access to arbitrary physical
    *  addresses.  In most cases this should not be needed, as
    *  many physical addresses are within kseg1, but this handles
    *  the cases that are not automagically, so we don't need
    *  to mess up the code with icky macros and such.
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


#include "ppcdefs.h"
#include "ppcmacros.h"
#include "cpu_config.h"		/* for definition of HAZARD */


/*  *********************************************************************
    *  phys_read8 - read 8-bit bytes
    ********************************************************************* */


LEAF(phys_read8)
	lbz	r3,0(r3)
	blr
END(phys_read8)

/*  *********************************************************************
    *  phys_read16 - read 16-bit shorts
    ********************************************************************* */

LEAF(phys_read16)
	lhz	r3,0(r3)
	blr
END(phys_read16)

/*  *********************************************************************
    *  phys_read32 - read 32-bit ints
    ********************************************************************* */

LEAF(phys_read32)
	lwz	r3,0(r3)
	blr
END(phys_read32)

/*  *********************************************************************
    *  phys_read64 - read 64-bit longs
    ********************************************************************* */

LEAF(phys_read64)
	lwz	r4,4(r3)
	lwz	r3,0(r3)
	blr

END(phys_read64)

/*  *********************************************************************
    *  phys_write8 - write 8-bit bytes
    ********************************************************************* */

LEAF(phys_write8)
	stb	r4,0(r3)
	eieio
	blr
END(phys_write8)

/*  *********************************************************************
    *  phys_write16 - write 16-bit shorts
    ********************************************************************* */

LEAF(phys_write16)
	sth	r4,0(r3)
	eieio
	blr
END(phys_write16)

/*  *********************************************************************
    *  phys_write32 - write 32-bit longs
    ********************************************************************* */

LEAF(phys_write32)
	stw	r4,0(r3)
	eieio
	blr
END(phys_write32)

/*  *********************************************************************
    *  phys_write64 - write 64-bit longs
    ********************************************************************* */

LEAF(phys_write64)
	stw	r4,0(r3)
	stw	r5,4(r3)
	blr
END(phys_write64)



/*  *********************************************************************
    *  phys_read16_swapped - read 16-bit shorts with byte swap
    ********************************************************************* */

LEAF(phys_read16_swapped)
	lhbrx	r3,0,r3
	blr
END(phys_read16_swapped)

/*  *********************************************************************
    *  phys_read32_swapped - read 32-bit ints with byte swap
    ********************************************************************* */

LEAF(phys_read32_swapped)
	lwbrx	r3,0,r3
	blr
END(phys_read32_swapped)

/*  *********************************************************************
    *  phys_write16_swapped - write 16-bit shorts with byte swap
    ********************************************************************* */

LEAF(phys_write16_swapped)
	sthbrx	r4,0,r3
	eieio
	blr
END(phys_write16_swapped)

/*  *********************************************************************
    *  phys_write32_swapped - write 32-bit longs with byte swap
    ********************************************************************* */

LEAF(phys_write32_swapped)
	stwbrx	r4,0,r3
	eieio
	blr
END(phys_write32_swapped)

/*  *********************************************************************
    *  End
    ********************************************************************* */
