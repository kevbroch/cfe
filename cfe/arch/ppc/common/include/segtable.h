/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Segment Table definitions		File: segtable.h
    *
    *  The 'segment table' (bad name) is just a list of addresses
    *  of important stuff used during initialization.  We use these
    *  indirections to make life less complicated during code
    *  relocation.
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


#if !defined(__ASSEMBLER__)
#ifndef _TBLIDX
#define _TBLIDX(x)   (x)		/* C handles indexing for us */
#endif
typedef long segtable_t;		/* 32 for long32, 64 for long64 */
#else
#ifndef _TBLIDX
#define _TBLIDX(x) (4*(x))
#endif
#endif

/*
 * Definitions for the segment_table
 */

#define R_SEG_ETEXT	     _TBLIDX(0)		/* end of text segment */
#define R_SEG_FDATA	     _TBLIDX(1)		/* Beginning of data segment */
#define R_SEG_EDATA	     _TBLIDX(2)		/* end of data segment */
#define R_SEG_END	     _TBLIDX(3)		/* End of BSS */
#define R_SEG_FTEXT          _TBLIDX(4)		/* Beginning of text segment */
#define R_SEG_FBSS           _TBLIDX(5)		/* Beginning of BSS */
#define R_SEG_GP	     _TBLIDX(6)		/* Global Pointer */
#define R_SEG_ROMDATA        _TBLIDX(7)		/* Start of ROM DATA */
#define R_SEG_RESERVED       _TBLIDX(8)		/* Reserved */
#define R_SEG_APIENTRY       _TBLIDX(9)		/* API Entry address */

