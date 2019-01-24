/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Cache operations				File: cfe_cache.h
    *  
    *  Macros that we use for common cache-related operations
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

#ifndef _CFE_CACHE_H
#define _CFE_CACHE_H


/*  *********************************************************************
    *  cfe_cacheops bits
    *  
    *  These bits are used in the "flags" parameter to the cfe_cacheops
    *  routine that eventually winds its way into the CPU package.
    *  Since they're also defined in cfe_iocb.h we duplicate them here
    ********************************************************************* */


/*
 * Duplicates from cfe_iocb.h -- warning!
 */

#ifndef CFE_CACHE_FLUSH_D

#define CFE_CACHE_FLUSH_D	1
#define CFE_CACHE_INVAL_I	2
#define CFE_CACHE_INVAL_D	4
#define CFE_CACHE_INVAL_L2	8
#define CFE_CACHE_FLUSH_L2	16
#define CFE_CACHE_INVAL_RANGE	32
#define CFE_CACHE_FLUSH_RANGE	64

#endif


#ifndef __ASSEMBLER__

/*  *********************************************************************
    *  Routines that implement cache operations
    ********************************************************************* */

extern void CPUCFG_INVALRANGE(volatile void *,int);
extern void CPUCFG_SYNCRANGE(volatile void *,int);

/*  *********************************************************************
    *  cache macros
    ********************************************************************* */

/*
 * Macros to flush or invalidate the cache (always)
 */
#define CACHE_SYNC(ptr,len) CPUCFG_SYNCRANGE((ptr),(len))
#define CACHE_INVAL(ptr,len) CPUCFG_INVALRANGE((ptr),(len))

/*
 * Macros to flush or invalidate the cache depending on whether
 * we support coherent DMA or not
 */

#if CPUCFG_COHERENT_DMA
#define CACHE_DMA_SYNC(ptr,len)
#define CACHE_DMA_INVAL(ptr,len) 
#define CACHE_DMA_SHARED(ptr) (ptr)
#define CACHE_DMA_CACHEABLE(ptr) ((uint8_t *) (ptr))
#else
#define CACHE_DMA_SYNC(ptr,len) CPUCFG_SYNCRANGE((ptr),(len))
#define CACHE_DMA_INVAL(ptr,len) CPUCFG_INVALRANGE((ptr),(len))
#define CACHE_DMA_SHARED(ptr) ((uint8_t *)UNCADDR(PHYSADDR((uintptr_t)(ptr))))
#define CACHE_DMA_CACHEABLE(ptr) ((uint8_t *)KERNADDR(PHYSADDR((uintptr_t)(ptr))))
#endif

#endif

#endif 
