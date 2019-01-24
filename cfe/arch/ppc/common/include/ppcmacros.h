/*  *********************************************************************
    *  PowerPC Board Support Package
    *  
    *  PPC CPU macros			File: ppcmacros.h
    * 
    *  This module contains constants and macros specific to the
    *  PowerPC core.
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2003
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


#define _LONG_ .long

#ifndef _TBLIDX
#define _TBLIDX(x) ((x)*4)
#endif

/*  *********************************************************************
    *  Macros for common instruction sequences
    ********************************************************************* */

#if CFG_RELOC
#define LDADDR(reg,val) lwz reg,GOT(val)
#else
#define LDADDR(r,val)  addis r,0,(val)@ha ; addi r,r,(val)@l
#endif

#define LDCONST(r,val) addis r,0,(val)@ha ; addi r,r,(val)@l

/*  *********************************************************************
    *  Macros for generating assembly language routines
    ********************************************************************* */

#if defined(__ASSEMBLER__)

/* global leaf function (does not call other functions) */
#define LEAF(name)		\
  	.align 2;               \
        .globl name;		\
        .type name,@function;   \
name:


/* end of a global function */
#define END(name) \
  	.size	name,.-name; 

/* define & export a symbol */
#define EXPORT(name)		\
  	.globl name;		\
name:

/* import a symbol */
#define	IMPORT(name, size)	\
	.extern	name,size

/* define a zero-fill common block (BSS if not overridden) with a global name */
#define COMM(name,size)		\
	.comm	name,size

/* define a zero-fill common block (BSS if not overridden) with a local name */
#define LCOMM(name,size)		\
  	.lcomm	name,size


/*  *********************************************************************
    *  Relocatable Code macros (GOT stuff)
    ********************************************************************* */

#if CFG_RELOC

#define GOT_REG r30			/* register we use to access the GOT */

/*
 * BEGIN_GOT() -- open the .got2 section.  Following this are one or
 * more GOT_ENTRY macros, then an END_GOT().
 */


#define BEGIN_GOT()   \
        .section  ".got2","aw" ; \
_GPPTR = . + 0x8000

/*
 * END_GOT() - close the .got2 section.
 */

#define END_GOT()	 .text

/*
 * GOT_ENTRY(x) - declare a GOT entry and a symbol that refers to the
 * variable referenced by the GOT entry.
 */
#define GOT_ENTRY(x)	_GE_ ## x = . - _GPPTR ; .long x

/*
 * GOT(x) - generate the symbol we created above in GOT_ENTRY.  Used
 * to reference variables that we placed in the GOT.
 */
#define GOT(x)	_GE_ ## x (GOT_REG)

/*
 * LOAD_GOT(reg) - determine the value for the GOT and place into 'reg'
 * we do this with PIC code since the GOT is near us.
 */
#define LOAD_GOT(reg) \
	bl	2f ;  \
	.text	2 ;   \
1:	.long	_GPPTR-2f ; \
	.text	;     \
2:	mflr	reg ; \
	lwz	r0,1b-2b(reg) ; \
	add	reg,r0,reg  

/* 
 * SAVE_GOT, RESTORE_GOT
 * Simple macros to save/restore previously computed GOT value.
 * In the non-relocation case, these expand to nothing.
 */

#define SAVE_GOT(reg)  mr reg,GOT_REG
#define RESTORE_GOT(reg) mr GOT_REG,reg
#define SAVE_LR(reg) mflr reg
#define RESTORE_LR(reg) mtlr reg

#else

/*
 * Not relocating, most of this stuff is NULL
 */

#define BEGIN_GOT()
#define END_GOT()
#define LOAD_GOT(x)
#define GOT_ENTRY(x)
#define GOT(x) x
#define SAVE_GOT(reg)
#define RESTORE_GOT(reg)
#define SAVE_LR(reg)
#define RESTORE_LR(reg)
#endif /* CFG_RELOC */


#endif









