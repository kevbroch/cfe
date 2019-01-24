/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU init module				File: init_ppc.S
    *
    *  This module contains the vectors and lowest-level CPU startup
    *  functions for CFE.
    *
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2004
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
#include "exception.h"

#include "bsp_config.h"
#include "cpu_config.h"

#ifdef _CFE_
#include "cfe_devfuncs.h"
#endif
#include "segtable.h"


/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#include "ppcmacros.h"

/*
 * Duplicates from cfe_iocb.h -- warning!
 */

#define CFE_CACHE_FLUSH_D	1
#define CFE_CACHE_INVAL_I	2
#define CFE_CACHE_INVAL_D	4
#define CFE_CACHE_INVAL_L2	8
#define CFE_CACHE_FLUSH_L2	16
#define CFE_CACHE_INVAL_RANGE	32
#define CFE_CACHE_FLUSH_RANGE	64

#define CFE_EPTSEAL 0x43464531


/*  *********************************************************************
    *  Other constants
    ********************************************************************* */

/*
 * This is the size of the stack, rounded to KByte boundaries.
 */

#ifndef CFG_STACK_SIZE
#error "CFG_STACK_SIZE not defined"
#else
#define STACK_SIZE	((CFG_STACK_SIZE+1023) & ~1023)
#endif




/*  *********************************************************************
    *  Names of registers used in this module
    ********************************************************************* */

#define REG_MEMSIZE r29
#define RELOCOFFSET r15

		.data

#include "initdata.h"		/* declare variables we use here */


		.extern	_fdata
		.extern	_edata
		.extern	_etext

/*  *********************************************************************
    *  uninitialized data
    ********************************************************************* */

		.data

		.comm	__junk,4

/*  *********************************************************************
    *  Exception Vectors
    ********************************************************************* */

		.text

/* 
 * This label is used for calculating our jump to DRAM
 */

		.org 0
_rom_start:
		.long 0x55aa55aa
		.long CFE_EPTSEAL
		.asciz "Common Firmware Environment, (c) 2004 Broadcom Corporation."

/*
 * Reset vector is special
 */

		.org EXC_OFF_RESET
		.globl vec_reset
		.globl _start
_start:
vec_reset:
		b	cpu_reset

/*
 * Other vectors follow
 */

DECLARE_VECTOR(EXC_OFF_MCHECK,_exc_entry)
DECLARE_VECTOR(EXC_OFF_DSI,_exc_entry)
DECLARE_VECTOR(EXC_OFF_ISI,_exc_entry)
DECLARE_VECTOR(EXC_OFF_INT,_exc_entry)
DECLARE_VECTOR(EXC_OFF_ALIGN,_exc_entry)
DECLARE_VECTOR(EXC_OFF_PROGRAM,_exc_entry)
DECLARE_VECTOR(EXC_OFF_FPUNVL,_exc_entry)
DECLARE_VECTOR(EXC_OFF_DECR,_exc_entry)
DECLARE_VECTOR(EXC_OFF_CINT,_exc_entry)
DECLARE_VECTOR(EXC_OFF_SYSCALL,_exc_entry)
DECLARE_VECTOR(EXC_OFF_TRACE,_exc_entry)
DECLARE_VECTOR(EXC_OFF_RSVD,_exc_entry)
DECLARE_VECTOR(EXC_OFF_ITLBMISS,_exc_entry)
DECLARE_VECTOR(EXC_OFF_DTLBMISS_LD,_exc_entry)
DECLARE_VECTOR(EXC_OFF_DTLBMISS_ST,_exc_entry)
DECLARE_VECTOR(EXC_OFF_SMI,_exc_entry)


/*  *********************************************************************
    *  CFE Entry Point (used by OS boot loaders and such)
    ********************************************************************* */

		.org (0x1500-8)		/* after exception vectors */
		.long CFE_EPTSEAL
		.long CFE_EPTSEAL
		b     cpu_apientry


#include "segtable.h"


/*  *********************************************************************
    *  LED messages
    ********************************************************************* */

#define LEDMSG_RELO	0x52454c4f	/* RELO - Start code relocation */
#define LEDMSG_L1CF	0x4c314346	/* L1CF - Flush L1 Cache */
#define LEDMSG_GOLO	0x474f4c4f	/* GOLO - Start running from RAM */
#define LEDMSG_GOT      0x474f5420	/* GOT  - Relocate GOT entries */
#define LEDMSG_DATA	0x44415441	/* DATA - Start data relocation */
#define LEDMSG_ZBSS	0x5A425353	/* ZBSS - Clear uninit data */
#define LEDMSG_INIT	0x494e4954	/* INIT - Set up variables */
#define LEDMSG_MAIN	0x4D41494E	/* MAIN - Jump to "C" code */

#define SETLEDS(x) LDCONST(r3,x) ; bl board_setleds

/*  *********************************************************************
    *  Global Offset Table (GOT)
    *  
    *  Addresses of stuff we want to use here.
    ********************************************************************* */

#if CFG_RELOC
BEGIN_GOT()
GOT_ENTRY(_GOT2_TABLE_)
GOT_ENTRY(_FIXUP_TABLE_)
GOT_ENTRY(_etext)
GOT_ENTRY(_fdata)
GOT_ENTRY(_edata)
GOT_ENTRY(_end)
GOT_ENTRY(_ftext)
GOT_ENTRY(_fbss)
GOT_ENTRY(_romdata)
GOT_ENTRY(mem_totalsize)
GOT_ENTRY(mem_textreloc)
GOT_ENTRY(mem_datareloc)
GOT_ENTRY(mem_bottomofmem)
GOT_ENTRY(segment_table)
GOT_ENTRY(mem_textbase)
GOT_ENTRY(mem_textsize)
GOT_ENTRY(mem_heapstart)
GOT_ENTRY(mem_topofmem)
GOT_ENTRY(cpu_prid)
GOT_ENTRY(cpu_apientry)
GOT_ENTRY(cfe_doxreq)
END_GOT()
#endif


/*  *********************************************************************
    *  Segment Table.
    *
    *  Addresses of data segments and of certain routines we're going
    *  to call from the init code.  It's actually more MIPS history
    *  than PowerPC related, but what the heck.
    ********************************************************************* */

               .data
	       .globl segment_table
segment_table:
		_LONG_	0			# [  0] End of text (R_SEG_ETEXT)
		_LONG_	0			# [  1] Beginning of data (R_SEG_FDATA)
		_LONG_	0			# [  2] End of data (R_SEG_EDATA)
		_LONG_	0			# [  3] End of BSS (R_SEG_END)
		_LONG_	0			# [  4] Beginning of text (R_SEG_FTEXT)
		_LONG_	0			# [  5] Beginning of BSS (R_SEG_FBSS)
		_LONG_	0			# [  6] Global Pointer (R_SEG_GP)
		_LONG_  0			# [  7] where to look for ROM data
		_LONG_  0			# [  8] End of reloc entries
		_LONG_	0			# [  9] R_SEG_APIENTRY

	       .text

	
/*  *********************************************************************
    *  CPU Startup Code
    ********************************************************************* */

cpu_reset:

	/*
	 * Do the following:
	 *
	 *  board_earlyinit
	 *  CPU Reset
	 *  Dram init query
	 *  DRAM init
	 *
	 *  Set up stack & heap
	 *  Start CFE
	 *
	 */

    /*
     * board_earlyinit is the first chance our board package gets to
     * initialize stuff.
     */

	bl	board_earlyinit

    /*
     * the CPU init code does some basic CPU setup.
     */

	bl	CPUCFG_CPUINIT


    /*
     * DRAM init routine turns on the DRAM controller.  Amount
     * of memory, in megabytes, is returned in r3
     */

	bl	CPUCFG_DRAMINIT

	mr	REG_MEMSIZE,r3


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#if CFG_RELOC

    /*
     * Need the GOT sooner or later...
     */

        LOAD_GOT(GOT_REG)

#define rsrc		r14
#define rdest		r13
#define rlen		r12

	SETLEDS(LEDMSG_RELO)

	LDADDR(rsrc,_ftext)		/* begin at the beginning of ROM */
	LDADDR(rlen,_end)
	sub    rlen,rlen,rsrc		/* _end-_ftext = length in bytes */

#if 1
	mr     rdest,REG_MEMSIZE
	slwi.  rdest,rdest,20		/* top memory address */
	sub    rdest,rdest,rlen		/* make room for code + data */
	LDCONST(r6,(CFG_HEAP_SIZE*1024+CFG_STACK_SIZE))
	sub	rdest,rdest,r6
	LDCONST(r6,0xFFFFFFE0)
	and	rdest,rdest,r6		/* round down to cache line */
#else
	/* Use this if you want a static relocation for debugging */
	LDCONST(rdest,0x10000)		/* relocate to here */
#endif


	mr	r10,rdest

	/*
	 * Compute distance we are moving the code by 
	 * subtracting destination from source, then
	 * apply the relocation to our GOT pointer.
	 */
	sub	RELOCOFFSET,rdest,rsrc
	add	GOT_REG,GOT_REG,RELOCOFFSET

	/* 
	 * Copy code from flash to DRAM
	 */

	cmplw   rdest,rsrc	/* don't copy if not relocating */
	beq	nocopy

	addi	r0,rlen,3
	srwi.	r0,r0,2

	mtctr	r0

	addi	r8,rsrc,-4	/* set up for lwzu/stwu instructions */
	addi	r7,rdest,-4
1:	lwzu	r0,4(r8)
	stwu	r0,4(r7)
	bdnz	1b

	/*
	 * Flush the DCache and invalidate the ICache 
	 */

	SETLEDS(LEDMSG_L1CF)

	li     r6,CPUCFG_CACHELINESIZE

	add	rlen,rdest,rlen
	subi	r0,r6,1
	andc	rdest,rdest,r0

	mr	rsrc,rdest

1:	cmplw	rsrc,rlen
	dcbst	0,rsrc
	add	rsrc,rsrc,r6
	blt	1b

	sync
	mr	rsrc,rdest

1:	cmplw	rsrc,rlen
	icbi	0,rsrc
	add	rsrc,rsrc,r6
	blt	1b

	sync
	isync

#undef rsrc
#undef rdest
#undef rlen

/*
 * Now, jump to the copy of ourselves in DRAM.
 */

nocopy:

	SETLEDS(LEDMSG_GOLO)


	addi	r0,r10,__GoRelo-_rom_start
	mtlr	r0
	blr

__GoRelo:


	SETLEDS(LEDMSG_GOT)

	/*
	 * Go through the GOT and apply our relocation offset
	 */
	li	r0,_GOT2_SIZE@sectoff@l
	la	r3,GOT(_GOT2_TABLE_)
	mtctr	r0
	addi	r3,r3,-4
1:	lwzu	r0,4(r3)
	add	r0,r0,RELOCOFFSET
	stw	r0,0(r3)
	bdnz	1b

	/*
         * Apply fixups to initialized structures, particularly
	 * pointers to code from data.
	 */
2:	li	r0,_FIXUP_SIZE@sectoff@l
	lwz	r3,GOT(_FIXUP_TABLE_)
	cmpwi	r0,0
	mtctr	r0
	addi	r3,r3,-4
	beq	4f
3:	lwzu	r4,4(r3)
	lwzux	r0,r4,RELOCOFFSET
	add	r0,r0,RELOCOFFSET
	stw	r10,0(r3)
	stw	r0,0(r4)
	bdnz	3b
4:

#else
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

        SETLEDS(LEDMSG_DATA)

	li	RELOCOFFSET,0


    /*
     * Copy data segment from ROM to RAM
     */

        LDADDR(r5,_fdata)
	LDADDR(r6,_edata)
	LDADDR(r7,_romdata)

1:	lwz	r1,0(r7)
	stw	r1,0(r5)
	addi	r7,r7,4
	addi	r5,r5,4
	cmplw	r5,r6
	ble	1b

#endif


    /*
     * Zero the BSS
     */

        SETLEDS(LEDMSG_ZBSS)

	LDADDR(r5,_fbss)
	LDADDR(r6,_end)
	li	r1,0

1:	stw	r1,0(r5)
	addi	r5,r5,4
	cmplw	r5,r6
	ble	1b


	LDADDR(r6,mem_totalsize)	/* store return value in memory */
	stw	REG_MEMSIZE,0(r6)

	SETLEDS(LEDMSG_INIT)


	LDADDR(r6,mem_textreloc)	/* mem_textreloc = 0 */
	stw	RELOCOFFSET,0(r6)
	LDADDR(r6,mem_datareloc)	/* mem_datareloc = 0 */
	stw	RELOCOFFSET,0(r6)

	LDADDR(r1,_ftext)		/* mem_textbase= &_ftext */
	LDADDR(r6,mem_textbase)
	stw	r1,0(r6)

	LDADDR(r2,_etext)		/* mem_textsize = &_etext - &_ftext */
	sub	r1,r2,r1
	LDADDR(r6,mem_textsize)
	stw	r1,0(r6)

#if CFG_RELOC	
	LDADDR(r1,_ftext)		/* mem_bottomofmem = &_ftext */
#else
	LDADDR(r1,_fdata)		/* mem_bottomofmem = &_fdata */
#endif
	LDADDR(r6,mem_bottomofmem)
	stw	r1,0(r6)

	LDADDR(r1,_end)			/* mem_heapstart = &_end */
	LDADDR(r6,mem_heapstart)
	stw	r1,0(r6)

	LDCONST(r2,(CFG_HEAP_SIZE*1024)+CFG_STACK_SIZE)
	add	r1,r1,r2
	LDADDR(r6,mem_topofmem)		/* mem_topofmem = &_end+heap+stack */
	stw	r1,0(r6)

	mfspr	r1,SPR_PVR		/* cpu_prid = Processor ID */
	LDADDR(r6,cpu_prid)
	stw	r1,0(r6)


   /*
    * Set up the segment_table (old habits die hard).  cfe_main prints
    * this stuff out.
    */


	LDADDR(r8,segment_table)

	LDADDR(r1,_etext)
	stw	r1,R_SEG_ETEXT(r8)

	LDADDR(r1,_fdata)
	stw	r1,R_SEG_FDATA(r8)

	LDADDR(r1,_edata)
	stw	r1,R_SEG_EDATA(r8)

	LDADDR(r1,_end)
	stw	r1,R_SEG_END(r8)

	LDADDR(r1,_fbss)
	stw	r1,R_SEG_FBSS(r8)

	LDADDR(r1,_romdata)
	stw	r1,R_SEG_ROMDATA(r8)

	LDADDR(r1,cpu_apientry)
	stw	r1,R_SEG_APIENTRY(r8)



    /*
     * Setup a stack and jump into the "C" code
     */

        SETLEDS(LEDMSG_MAIN)

	LDADDR(r1,_end)
	LDCONST(r2,(CFG_HEAP_SIZE*1024)+CFG_STACK_SIZE)
	add	r1,r1,r2
	LDCONST(r2,0xFFFFFFF0)
	and	r1,r1,r2
        
	b	cfe_main



#ifdef _CFE_
/*  *********************************************************************
    *  CFE_WARMSTART
    *  
    *  Restart the command interpreter
    *  
    *  Input parameters: 
    *      A0 - command status
    *  	   nothing (GP has already been set up for us)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(cfe_warmstart)

	/*
	 * Jump back into command interpreter 
	 */

END(cfe_warmstart)
#endif

/*  *********************************************************************
    *  CFE_FLUSHCACHE
    *  
    *  Perform certain cache operations
    *  
    *  Input parameters: 
    *  	   r3 - flags (CFE_CACHE_xxx flags, or zero for a default)
    *      r4,r5 - start/end of range for "range invalidate" operations
    *      (not used otherwise)
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(_cfe_flushcache)

	b	CPUCFG_CACHEOPS

END(_cfe_flushcache)


#if !CFG_PPCBOOT
/*  *********************************************************************
    *  CFE_LAUNCH
    *  
    *  Start the user program.  The program is passed a handle
    *  that must be passed back when calling the firmware.
    *
    *  Parameters passed to the called program are as follows:
    *
    *      r3 - CFE handle
    *      r4 - entry vector
    *      r5 - reserved, will be 0
    *      r6 - entrypoint signature.
    *  
    *  Input parameters: 
    *  	   r3 - entry vector
    *  	   
    *  Return value:
    *  	   does not return
    ********************************************************************* */

LEAF(cfe_launch)

	subi	r1,r1,8		/* save entry point on stack */
	stw	r3,0(r1)

	li	r3,CFE_CACHE_FLUSH_D
	bl	CPUCFG_CACHEOPS

	li	r3,CFE_CACHE_INVAL_I
	bl	CPUCFG_CACHEOPS

	LOAD_GOT(GOT_REG)
	li	r3,0
	LDADDR(r4,cpu_apientry)
	li	r5,0
	LDCONST(r6,CFE_EPTSEAL)
			
	lwz	r0,0(r1)
	mtlr	r0

	blr

END(cfe_launch)
#endif



/*  *********************************************************************
    *  CFE_LEDS
    *  
    *  Set the on-board LEDs.
    *  
    *  Input parameters: 
    *  	   a0 - LEDs
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(cfe_leds)

	b	board_setleds

END(cfe_leds)

/*  *********************************************************************
    *  cpu_apientry(handle,iocb)
    *
    *  API entry point for external apps.
    *  
    *  Input parameters: 
    *  	   r3 - firmware handle (used to determine the location of
    *  	        our relocated data)
    *  	   r4 - pointer to IOCB to execute
    *  	   
    *  Return value:
    *  	   r3 - return code, 0 if ok
    ********************************************************************* */


LEAF(cpu_apientry)

	/*
	 * Do the following:
	 *
	 *  Save registers
	 *  Get args from user stack
	 *  Call CFE
	 *  restore registers
	 */

	 subi	r1,r1,(20*4)
	 mflr	r0
	 stw	r0,0(r1)
	 stmw	r13,4(r1)

	 mr	r3,r4
	 LOAD_GOT(GOT_REG)
	 LDADDR(r4,cfe_doxreq)
	 mtlr	r4
	 blrl

	 lmw	r13,4(r1)
	 lwz	r0,0(r1)
	 addi	r1,r1,(20*4)
	 mtlr	r0
	 blr

END(cpu_apientry)



/*  *********************************************************************
    *  _GETTICKS()
    *  
    *  Read the COUNT register into v0
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   r3 - count register
    ********************************************************************* */

LEAF(_getticks)

	mfspr	r3,SPR_TBL

	blr

END(_getticks)



/*  *********************************************************************
    *  _GETSEGTBL()
    *  
    *  Return the address of the segment table.  We use this
    *  to display the startup messages.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *      address of table (r3)
    ********************************************************************* */


LEAF(_getsegtbl)

	mflr	r10

	SAVE_GOT(r11)
	LOAD_GOT(GOT_REG)
	LDADDR(r3,segment_table)
	RESTORE_GOT(r11)

	mtlr	r10
	blr

END(_getsegtbl)


/*  *********************************************************************
    *  _readmsr()
    *  
    *  Read the machine state register
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *      register value
    ********************************************************************* */
LEAF(_readmsr)

	mfmsr r3
	blr

END(_readmsr)

/*  *********************************************************************
    *  _writemsr()
    *  
    *  Write the machine state register
    *  
    *  Input parameters: 
    *      register value
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(_writemsr)

	mtmsr r3
	blr

END(_writemsr)

/*  *********************************************************************
    *  _wbflush()
    *  
    *  Flush the write buffer.  This is probably not necessary
    *  on SiByte CPUs, but we have it for completeness.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(_wbflush)

	eieio
	sync
	blr

END(_wbflush)




/*  *********************************************************************
    *  End
    ********************************************************************* */


