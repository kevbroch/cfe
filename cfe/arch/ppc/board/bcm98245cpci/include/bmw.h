/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Board device initialization		File: bmw.h
    *  
    *  Stuff particular to the BMW board
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


/*  *********************************************************************
    *  This is where we put all the on-chip features
    ********************************************************************* */

#define A_BMW_EUMBBAR_VAL 0xFC000000
#define A_BMW_EUMBSIZE	  0x00100000

#define A_BMW_SERIAL_0		(A_BMW_EUMBBAR_VAL+0x4500)
#define A_BMW_SERIAL_1		(A_BMW_EUMBBAR_VAL+0x4600)

/*  *********************************************************************
    *  Most things on the generic bus are relative to PORTX_BASE
    ********************************************************************* */

#define PORTX_BASE	0xFF000000	/* base of X-bus on MPC8240 */
#define A_BMW_TSOP_DOC	(PORTX_BASE+0)	/* TSOP Disk-on-Chip on RCS1 */
#define BMW_TSOP_DOC_SIZE (32*1024*1024)

#define A_BMW_BOOTROM	0xFFE00000	/* note: starts 1MB below PowerPC start address */
#define BMW_BOOTROM_SIZE (2*1024*1024)


#define A_BMW_XROM	0x7c000000	/* Extended ROM space on RCS2 */

#define A_BMW_PLD	(A_BMW_XROM + 0)
#define A_BMW_LED	(A_BMW_XROM + 0x2000)
#define A_BMW_TOYCLOCK	(A_BMW_XROM + 0x4000)
#define A_BMW_VX_NVRAM  0x1000      /* VxWorks reserved */

/* VxWorks-specific NVRAM addresses */
#define A_BMW_VX_MAC    (A_BMW_TOYCLOCK + 0x0400)


#define A_BMW_DOC	0x70000000     	/* 32-pin DIP socket on RCS3 */

/*  *********************************************************************
    *  PLD definitions
    ********************************************************************* */

#define R_BMW_PLD_REVID	0
#define R_BMW_PLD_LED	1
#define R_BMW_PLD_TODEN	2
#define R_BMW_PLD_TODWP	3

#define M_BMW_PLD_FLG	0x80
#define M_BMW_PLD_REVID	0x7F


