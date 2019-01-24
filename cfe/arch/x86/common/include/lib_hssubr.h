/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  "Hyperspace" access routines		File: lib_hssubr.h
    *  
    *  Routines to muck with memory independently of pointer size.
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


#ifndef _LIB_HSSUBR_H
#define _LIB_HSSUBR_H

/*
 * The macros and functions in this file are used to wrangle
 * 64-bit addresses on 32-bit versions of CFE.  Generally CFE
 * is 32-bits on all platforms, even those with 64-bit addressability.
 * This is done for toolchain reasons, since 64-bitness interferes
 * in nasty ways with relocation.
 *
 * There are two other macros of note:
 *
 *  PTR2HSADDR(x)   Converts a pointer to an hsaddr, with appopriate 
 *                  sign extensions
 * 
 *  HSADDR2PTR(x)   Shortens an hsaddr into a regular pointer, assuming
 *                  that pointer is already narrow enough to fit into
 *                  a regular pointer.  Used mostly in places where
 *                  CFE is sending pointers to itself.
 */

#if (CPUCFG_REGS32)

/* We can use the macros. */

typedef long hsaddr_t;		/* longs are already pointer-sized */

#define hs_write8(a,b) *((volatile uint8_t *) (a)) = (b)
#define hs_write16(a,b) *((volatile uint16_t *) (a)) = (b)
#define hs_write32(a,b) *((volatile uint32_t *) (a)) = (b)
#define hs_write64(a,b) *((volatile uint64_t *) (a)) = (b)
#define hs_read8(a) *((volatile uint8_t *) (a))
#define hs_read16(a) *((volatile uint16_t *) (a))
#define hs_read32(a) *((volatile uint32_t *) (a))
#define hs_read64(a) *((volatile uint64_t *) (a))
#define hs_memcpy_to_hs(d,s,c) memcpy((void *)(d),(s),(c))
#define hs_memcpy_from_hs(d,s,c) memcpy((d),(void *)(s),(c))
#define hs_memset(d,x,c) memset((void *)(d),(x),(c))
#define PTR2HSADDR(x) ((hsaddr_t)(x))
#define HSADDR2PTR(x) ((void *)(long)(x))

#else	

/* Need functions, pointers and registers are different */

#error "X86 package doesn't support this yet."

typedef long long hsaddr_t; 	/* Make sure pointers are 64 bits */

extern void hs_write8(hsaddr_t a,uint8_t b);
extern void hs_write16(hsaddr_t a,uint16_t b);
extern void hs_write32(hsaddr_t a,uint32_t b);
extern void hs_write64(hsaddr_t a,uint64_t b);
extern uint8_t hs_read8(hsaddr_t a);
extern uint16_t hs_read16(hsaddr_t a);
extern uint32_t hs_read32(hsaddr_t a);
extern uint64_t hs_read64(hsaddr_t a);
extern void hs_memset(hsaddr_t dest,int c,int cnt);
extern void hs_memcpy_to_hs(hsaddr_t dest,void *src,int cnt);
extern void hs_memcpy_from_hs(void *dest,hsaddr_t src,int cnt);

#define PTR2HSADDR(x) ((hsaddr_t)(long)(x))
#define HSADDR2PTR(x) ((void *)(long)(x))
#endif

#endif
