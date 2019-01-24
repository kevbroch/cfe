/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  MIPS Macros				File: mipsmacros.h
    *
    *  Macros to deal with various mips-related things.
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

#ifndef _MIPSMACROS_H
#define _MIPSMACROS_H

/*  *********************************************************************
    *  32/64-bit macros
    ********************************************************************* */

#if ((CPUCFG_REGS32+CPUCFG_REGS64) != 1)
#error "You must define exactly ONE of CPUCFG_REGS32,CPUCFG_REGS64 in cpu_config.h"
#endif

#if ((CFG_RELOC) && defined(__long64))
#error "Relocation is not compatible with 64-bit pointer types"
#endif

/*
 * These macros are used when we know we're dealing with 64-bit
 * pointers.  Normally CFE is a 32-bit app, but there are a few 
 * cases where we explicitly deal with pointers that are greater
 * than the word size, like the dump/examine commands, the flashop
 * engine, etc.
 */

/* 
 * CPUCFG_REGS64 tells us that our architecture has 64-bit
 * registers.  We define macros that expand to 64-bit ops
 * regardless of the setting of __long64.  
 *
 * These macros are used in cases where we really care whether
 * the registers are 64 bits wide or not.  Places like exception
 * handlers and other code that needs to see the entire register
 * even though we're in a 32-bit app.
 * 
 * One special case is when we're dealing with 64-bit pointers.  We
 * need 64-bit ALU ops to add/subtract these even if they are 
 * pointing at 32-bit data.
 */
#if CPUCFG_REGS64
#define ADDPTR daddu
#define SUBPTR dsubu
#define LDPTR  ld
#define STPTR  sd
#define LREG   ld
#define SREG   sd
#define MFC0   dmfc0
#define MTC0   dmtc0
#define SRL    dsrl
#define SLL    dsll
#else
#define ADDPTR addu
#define SUBPTR subu
#define LDPTR  lw
#define STPTR  sw
#define LREG   lw
#define SREG   sw
#define MFC0   mfc0
#define MTC0   mtc0
#define SRL    srl
#define SLL    sll
#endif

#ifdef __long64
/*
 * CFE is normally a 32-bit application.  It's still possible to build
 * a 64-bit version, but only for non-relocating cases.
 *
 * These macros are used to make the size of a pointer transparent (mostly).
 * So, when you load/store something on the stack, we use the right instruction
 * to save either 32 bits or all 64 bits.  Since CFE is mostly 32-bit nowadays,
 * these macros will mostly be the 32-bit variants.
 */
#define _VECT_	.dword
#define _LONG_	.dword
#define SR	sd
#define LR	ld
#define LA	dla
#define ADD     dadd
#define SUB     dsub
#define REGSIZE	8
#define BPWSIZE 3		/* bits per word size */
#define _TBLIDX(x) ((x)*REGSIZE)
#else
#define _VECT_	.word
#define _LONG_	.word
#define SR	sw
#define LR	lw
#define LA	la
#define ADD     add
#define SUB     sub
#define REGSIZE 4
#define BPWSIZE 2
#define _TBLIDX(x) ((x)*REGSIZE)
#endif


/*  *********************************************************************
    *  NORMAL_VECTOR(addr,vecname,vecdest)
    *  NORMAL_XVECTOR(addr,vecname,vecdest,code)
    *  
    *  Declare a trap or dispatch vector. There are two flavors,
    *  DECLARE_XVECTOR sets up an indentifying code in k0 before
    *  jumping to the dispatch routine.
    *  
    *  Input parameters: 
    *  	   addr - vector address
    *  	   vecname - for label at that address
    *  	   vecdest - destination (place vector jumps to)
    *  	   code - code to place in k0 before jumping
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */


#define NORMAL_VECTOR(addr,vecname,vecdest) \
       .globl vecname    ;                   \
       .org   addr       ;                   \
vecname: b    vecdest    ;                   \
       nop;

#define NORMAL_XVECTOR(addr,vecname,vecdest,code) \
       .globl vecname    ;                   \
       .org   addr       ;                   \
vecname: b    vecdest    ;                   \
	 li   k0,code    ;		     \
       nop;


/*  *********************************************************************
    *  Evil macros for bi-endian support.
    *  
    *  The magic here is in the instruction encoded as 0x10000014.
    *  
    *  This instruction in big-endian is:   "b .+0x54"
    *  this instruction in little-endian is: "bne zero,zero,.+0x44"
    *  
    *  So, depending on what the system endianness is, it will either
    *  branch to .+0x54 or not branch at all. 
    *  
    *  the instructions that follow are:
    *  
    *     0x10000014        "magic branch"  (either-endian)
    *     0x00000000        nop  (bds)      (either-endian)
    *     0xD0BF1A3C        lui k0,0xBFD0   (little-endian)
    *     0xxxxx5A27        addu k0,vector  (little-endian)
    *     0x08004003        jr k0           (little-endian)
    *     0x00000000        nop  (bds)      (little-endian)
    *  ... space up to offset 0x54
    *     .........         b vecaddr       (big-endian)
    *  
    *  The idea is that the big-endian firmware is first, from 0..1MB
    *  in the flash, and the little-endian firmware is second,
    *  from 1..2MB in the flash.  The little-endian firmware is
    *  set to load at BFD00000, so that its initial routines will
    *  work until relocation is completed.
    *  
    *  the instructions at the vectors will either jump to the 
    *  big-endian or little-endian code based on system endianness.
    *
    *  The ROM is built by compiling CFE twice, first with 
    *  CFG_BIENDIAN=1 and CFG_LITTLE=0 (big-endian) and again
    *  with CFG_BIENDIAN=1 and CFG_LITTLE=1.  The resulting
    *  cfe.bin files are located at 0xBFC00000 and 0xBFD00000
    *  for big and little-endian versions, respectively.
    * 
    *  More information about how this works can be found in the 
    *  CFE Manual.
    ********************************************************************* */

#define __SWAPW(x) ((((x) & 0xFF) << 8) | (((x) & 0xFF00) >> 8))

#define BIENDIAN_VECTOR(addr,vecname,vecdest) \
       .globl vecname    ;                   \
       .org   addr       ;                   \
vecname: .word 0x10000014  ;		     \
       .word 0		 ;		     \
       .word ((__SWAPW(BIENDIAN_LE_BASE >> 16)) << 16) | 0x1A3C ; \
       .word (0x00005A27 | (((addr) & 0xFF) << 24) | (((addr) & 0xFF00) << 8)) ; \
       .word 0x08004003 ;		    \
       .word 0          ;                   \
       .org  ((addr) + 0x54) ;		    \
        b    vecdest    ;                   \
       nop;

#define BIENDIAN_XVECTOR(addr,vecname,vecdest,code) \
       .globl vecname    ;                   \
       .org   addr       ;                   \
vecname: .word 0x10000014  ;		     \
       .word 0		 ;		     \
       .word ((__SWAPW(BIENDIAN_LE_BASE >> 16)) << 16) | 0x1A3C ; \
       .word (0x00005A27 | (((addr) & 0xFF) << 24) | (((addr) & 0xFF00) << 8)) ; \
       .word 0x08004003  ;		    \
       .word 0          ;                   \
       .org  ((addr) + 0x54) ;		    \
       b    vecdest      ;                  \
         li   k0,code    ;		    \
       nop;



/*  *********************************************************************
    *  Declare the right versions of DECLARE_VECTOR and 
    *  DECLARE_XVECTOR depending on how we're building stuff.
    *  Generally, we only use the biendian version if we're building
    *  as CFG_BIENDIAN=1 and we're doing the big-endian MIPS version.
    ********************************************************************* */

#if (CFG_BIENDIAN) && defined(__MIPSEB)
#define DECLARE_VECTOR BIENDIAN_VECTOR
#define DECLARE_XVECTOR BIENDIAN_XVECTOR
#else
#define DECLARE_VECTOR NORMAL_VECTOR
#define DECLARE_XVECTOR NORMAL_XVECTOR
#endif



/*  *********************************************************************
    *  LDADDR(reg,label)
    *  
    *  Load the address of a symbol via the GOT.  We use this in
    *  relocated (SVR4 PIC) mode, when we aren't sure that gp is loaded
    *  correctly or not.  We get our GP value from low memory
    *  (assume it's set up), make the reference, and restore GP to
    *  its old value.  Note: this macro uses the k0 register.
    *  
    *  Input parameters: 
    *  	   reg - register to load
    *  	   label - address (symbol) to load into register
    ********************************************************************* */

#if CFG_RELOC
#define LDADDR(reg,label)    \
        move    k0,gp ; \
	li	gp,PHYS_TO_K1(CFE_LOCORE_GLOBAL_GP) ; \
	LR	gp,0(gp) ; \
	la	reg,label ; \
	move	gp,k0
#else
#define LDADDR(reg,label)   \
        LA      reg,label
#endif
        


/*  *********************************************************************
    *  Subroutine Linkage Macros
    * 
    *  We deal with the differences between non-PIC and SVR4 PIC here.
    *
    *  The JAL_KSEG1 macro variant makes sure the target address
    *  is in KSEG1 for calling routines while running uncached,
    *  even if the target address was linked to a cached
    *  address (typical for init code).
    ********************************************************************* */


/*
 * Subroutine linkage varies among the relocation methods
 * we support.  In particular, SVR4 PIC will not allow
 * a direct branch or call to an external routine, so
 * we have these macros here for generating code that
 * the linker will be happy with.  Note that
 * 'k1' is used in the SVR4 PIC case.
 */

#if (CFG_RELOC)
#define JMP(x)  la t9,x ; j t9		/* use PIC linkage */
#define JAL(x)  la t9,x ; jalr t9
#else
#define JMP(x)  j x			/* Standard JAL ok */
#define JAL(x)  jal x
#endif

#define JAL_KSEG1(x) LA t9,x ; or t9,K1BASE ; jalr t9


/*  *********************************************************************
    *  SPIN_LOCK(lock,reg1,reg2)
    *  
    *  Acquire a spin lock.
    *  
    *  Input parameters: 
    *  	   lock - symbol (address) of lock to acquire
    *  	   reg1,reg2 - registers we can use to acquire lock
    *  	   
    *  Return value:
    *  	   nothing (lock acquired)
    ********************************************************************* */

#define SPIN_LOCK(lock,reg1,reg2)                 \
        LA      reg1,lock ;                       \
1:      ll	reg2,0(reg1) ;			  \
	bne	reg2,zero,1b ;			  \
	li	reg2,1	     ;			  \
	sc	reg2,0(reg1) ;			  \
	beq	reg2,zero,1b ;			  \
	nop

/*  *********************************************************************
    *  SPIN_UNLOCK(lock,reg1)
    *  
    *  Release a spin lock.
    *  
    *  Input parameters: 
    *  	   lock - symbol (address) of lock to release
    *  	   reg1 - a register we can use
    *  	   
    *  Return value:
    *  	   nothing (lock released)
    ********************************************************************* */


#define SPIN_UNLOCK(lock,reg1)			 \
	LA	reg1,lock ;			 \
	sw	zero,0(reg1)


/*  *********************************************************************
    *  SETCCAMODE(treg,mode)
    *  
    *  Set cacheability mode.  For some of the pass1 workarounds we
    *  do this alot, so here's a handy macro.
    *  
    *  Input parameters: 
    *  	   treg - temporary register we can use
    *  	   mode - new mode (K_CFG_K0COH_xxx)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

#define SETCCAMODE(treg,mode)                     \
		mfc0	treg,C0_CONFIG		; \
		srl	treg,treg,3		; \
		sll	treg,treg,3		; \
		or	treg,treg,mode          ; \
		mtc0	treg,C0_CONFIG		; \
		HAZARD


/*  *********************************************************************
    *  Declare variables
    ********************************************************************* */

#define DECLARE_LONG(x) \
                .global x ; \
x:              _LONG_  0




/*
 * end
 */

#endif

