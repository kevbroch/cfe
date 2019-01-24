/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU init module				File: init_mips.S
    *
    *  This module contains the vectors and lowest-level CPU startup
    *  functions for CFE.
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
#include "exception.h"

#include "bsp_config.h"
#include "cpu_config.h"

#include "cfe_cache.h"

#ifdef _CFE_
#include "cfe_devfuncs.h"
#else

#if (CFG_BIENDIAN) && defined(__MIPSEB)
#define CFE_EPTSEAL_REV 0x31454643
#endif
#define CFE_EPTSEAL 0x43464531
#endif


#if CFG_VAPI		/* haul in SB1250-specfic stuff only for VAPI */
#include "sb1250_defs.h"
#ifdef _SB14XX_
#include "bcm1480_regs.h"
#include "bcm1480_scd.h"
#define CHIP_RESET_DEFEAT    bcm1480_reset_defeature
#define CHIP_MAILBOX_SET_CPU A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_SET_CPU)
#define CHIP_MAILBOX_CLR_CPU A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CLR_CPU)
#define CHIP_MAILBOX_CPU     A_BCM1480_IMR_REGISTER(0,R_BCM1480_IMR_MAILBOX_0_CPU)
#else
#include "sb1250_regs.h"
#include "sb1250_scd.h"
#define CHIP_RESET_DEFEAT    sb1250_reset_defeature
#define CHIP_MAILBOX_SET_CPU A_IMR_REGISTER(0,R_IMR_MAILBOX_SET_CPU)
#define CHIP_MAILBOX_CLR_CPU A_IMR_REGISTER(0,R_IMR_MAILBOX_CLR_CPU)
#define CHIP_MAILBOX_CPU     A_IMR_REGISTER(0,R_IMR_MAILBOX_CPU)
#endif
#endif

#include "elf_mips.h"

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#include "mipsmacros.h"

/*
 * Default names for some callouts
 */

#ifndef CPUCFG_EARLYINIT
#define CPUCFG_EARLYINIT board_earlyinit
#endif

#ifndef CPUCFG_DRAMINFO
#define CPUCFG_DRAMINFO board_draminfo
#endif


/*  *********************************************************************
    *  SETLEDS(a,b,c,d)
    *  SETLEDS1(a,b,c,d)
    *  
    *  Sets the on-board LED display (if present).  Two variants
    *  of this routine are provided.  If you're running KSEG1,
    *  call the SETLEDS1 variant, else call SETLEDS.
    *  
    *  Input parameters: 
    *  	   a,b,c,d - four ASCII characters (literal constants)
    *  	   
    *  Return value:
    *  	   a0,k1,ra trashed
    ********************************************************************* */


#define SETLEDS(a,b,c,d)                     \
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
       JAL(board_setleds) ;

#define SETLEDS1(a,b,c,d)                     \
       li     a0,(((a)<<24)|((b)<<16)|((c)<<8)|(d)) ;    \
       JAL_KSEG1(board_setleds) ;


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


/*
 * To make life easier reading this code, define "KSEGBASE" 
 * to either K0BASE or K1BASE depending on whether we're running
 * uncached.
 */

#if CFG_RUNFROMKSEG0
#define KSEGBASE	K0BASE
#else
#define KSEGBASE	K1BASE
#endif


/*  *********************************************************************
    *  Names of registers used in this module
    ********************************************************************* */

#define RELOCOFFSET	s8			/* $30 (fp) */
#define MEMTOP		t8			/* $24 (t8) */
#define TEXTBASE	s7			/* $23 (s7) */

		.sdata

#include "initdata.h"		/* declare variables we use here */

		.extern	_fdata
		.extern	_edata
		.extern	_etext

/*  *********************************************************************
    *  uninitialized data
    ********************************************************************* */

		.bss

		.comm	__junk,4

/*  *********************************************************************
    *  Exception Vectors
    ********************************************************************* */

		.text
	
		.set noreorder

/*
 * If we're building a bi-endian version, this is the base
 * address that we can expect to find the little-endian version
 * of the firmware.
 * 
 * Warning: If you change this, you must also change 
 * the linker script (arch/mips/common/src/cfe_ldscript.template)
 * and the mkflashimage program (hosttools/mkflashimage.c)
 */

#define BIENDIAN_LE_BASE	0xBFD00000

/*
 * Declare the actual vectors.  This expands to code that
 * must be at the very beginning of the text segment.
 */

DECLARE_VECTOR(0x0000,vec_reset,cpu_reset)
DECLARE_VECTOR(0x0200,vec_tlbfill,cpu_tlbfill)
DECLARE_XVECTOR(0x0280,vec_xtlbfill,cpu_xtlbfill,XTYPE_XTLBFILL)
DECLARE_VECTOR(0x0300,vec_cacheerr,cpu_cacheerr)
DECLARE_XVECTOR(0x0380,vec_exception,cpu_exception,XTYPE_EXCEPTION)
DECLARE_XVECTOR(0x0400,vec_interrupt,cpu_interrupt,XTYPE_INTERRUPT)
DECLARE_XVECTOR(0x0480,vec_ejtag,cpu_ejtag,XTYPE_EJTAG)


/*
 * New location of CFE seal.  Will eventually phase out the seal at
 * offset 0x508
 */
		.org	0x4E0
cfe_seal:	.word	CFE_EPTSEAL
		.word	CFE_EPTSEAL

#if (CFG_BIENDIAN) && defined(__MIPSEB)
		.org    0x4E8
cfe_seal_rev:	.word	CFE_EPTSEAL_REV
		.word	CFE_EPTSEAL_REV
#endif

		.set reorder

/*  *********************************************************************
    *  CFE Entry Point (used by OS boot loaders and such)
    ********************************************************************* */

                .set  noreorder

#if !defined(_ZIPSTART_)
DECLARE_VECTOR(0x0500,vec_apientry,cpu_apientry2)
#endif
#if !(CFG_BIENDIAN)
		.org	0x508
		.word   CFE_EPTSEAL
		.word	CFE_EPTSEAL
#endif


/*  *********************************************************************
    *  Verification APIs (if present)   [SB1250-specific]
    ********************************************************************* */

#if CFG_VAPI
#if (CFG_RELOC)
#error "CFG_VAPI is not compatible with relocatable code"
#endif
#include "vapi.h"
/*
 * Vector should be 16 bytes long
 */
#define VAPI_VECTOR(l,x) \
                .extern x ; \
                .org (l & 0xFFFF) ; \
                j       x ; \
                nop ;  \
		.word   VAPI_EPTSEAL ; \
		.word   VAPI_EPTSEAL

VAPI_VECTOR(VAPI_FUNC_EXIT,vapi_exit)
VAPI_VECTOR(VAPI_FUNC_DUMPGPRS,vapi_dumpgprs)
VAPI_VECTOR(VAPI_FUNC_SETLOG,vapi_setlog)
VAPI_VECTOR(VAPI_FUNC_LOGVALUE,vapi_logsingle)
VAPI_VECTOR(VAPI_FUNC_LOGDATA,vapi_logdata)
VAPI_VECTOR(VAPI_FUNC_LOGTRACE,vapi_logtrace)
VAPI_VECTOR(VAPI_FUNC_LOGSOC,vapi_savesoc)
VAPI_VECTOR(VAPI_FUNC_LOGGPRS,vapi_loggprs)
VAPI_VECTOR(VAPI_FUNC_DUMPSTRING,vapi_puts)
VAPI_VECTOR(VAPI_FUNC_SETLEDS,vapi_setleds)
VAPI_VECTOR(VAPI_FUNC_LOGFPRS,vapi_logfprs)
#endif


		.set   reorder


/*  *********************************************************************
    *  Segment Table.
    *
    *  Addresses of data segments and of certain routines we're going
    *  to call from KSEG1.  These are here mostly for the 
    *  PIC case, since we can't count on the 'la' instruction to
    *  do the expected thing (the assembler expands it into a macro
    *  for doing GP-relative stuff, and the code is NOT GP-relative.
    *  So, we (relocatably) get the offset of this table and then
    *  index within it.  
    *
    *  Pointer values in this segment will be relative to KSEG0 for 
    *  cached versions of CFE, so we need to OR in K1BASE in the
    *  case of calling to a uncached address.
    ********************************************************************* */


#include "segtable.h"

#if CFG_VAPI
		.org	0x600			# move past exception vectors
#else
		.org	0x580			# move past exception vectors
#endif

		.globl segment_table
segment_table:
		_LONG_	_etext			# [  0] End of text (R_SEG_ETEXT)
		_LONG_	_fdata			# [  1] Beginning of data (R_SEG_FDATA)
		_LONG_	_edata			# [  2] End of data (R_SEG_EDATA)
		_LONG_	_end			# [  3] End of BSS (R_SEG_END)
		_LONG_	_ftext			# [  4] Beginning of text (R_SEG_FTEXT)
		_LONG_	_fbss			# [  5] Beginning of BSS (R_SEG_FBSS)


/*  *********************************************************************
    *  CPU API entry (needs to be in this file for the SVR4 PIC build)
    *  Get our GP value from A0 first, then set it back.
    ********************************************************************* */

cpu_apientry2:
#if CFG_RELOC
		move	k0,gp
		move	gp,a0
		la	t9,cpu_apientry		# This implicitly uses GP
		move	gp,k0
		jr	t9
#else
		j	cpu_apientry
#endif
	
/*  *********************************************************************
    *  CPU Startup Code
    ********************************************************************* */

cpu_reset:

	/*
	 * SVR4 PIC: Load GP with our nonrelocated value.  Make
	 * sure it is in KSEG1 because the caches aren't hot yet.
	 */

#if CFG_RELOC
		lui	gp,%hi(_gp)
		addiu	gp,%lo(_gp)
		or	gp,gp,K1BASE
#else
		move	gp,zero			# start with no GP.
#endif


#if CFG_VAPI
	/*
	 * VAPI works by using the SCD to reset just the core.
	 * Look for a special signature in the mailbox register
	 * on CPU0 - if present, jump to the start of the diag.
	 * Of course, you need a real 1250 to do this.
	 */

		li	k0,PHYS_TO_K1(CHIP_MAILBOX_CPU)
		ld	k0,0(k0)
		dli	k1,VAPI_MAGIC_NUMBER_MC
		beq	k0,k1,vapi_runmc
		dli	k1,VAPI_MAGIC_NUMBER_UNC
		beq	k0,k1,vapi_rununc
		dli	k1,VAPI_MAGIC_NUMBER
		bne	k0,k1,vapi_skip

	/*
	 * The only CP0 init we do is to set K0 to cacheable
	 */

		mfc0	k0,C0_CONFIG		# get current CONFIG register
		srl	k0,k0,3			# strip out K0 bits
		sll	k0,k0,3			# k0 bits now zero
		or	k0,k0,K_CFG_K0COH_COHERENT # K0 is cacheable.
		mtc0	k0,C0_CONFIG

	/*
	 * Set any required defeature bits (for VAPI diagnostics only)
	 * they get cleared by the soft reset.
	 */

		jal	CHIP_RESET_DEFEAT	/* in sb1250_l1cache.S */

	/*
	 * Jump to the diagnostic.  Two variants, one for cached
	 * and one for uncached.
	 */

		li	k0,VAPI_DIAG_ENTRY
		j	k0

vapi_rununc:	li	k0,VAPI_DIAG_ENTRY_UNC
		j	k0

vapi_runmc:	li	k0,VAPI_DIAG_ENTRY_MC
		j	k0

vapi_skip:
#endif

	/*
	 * Test the CAUSE and STATUS registers for why we
         * are here.  Cold reset, Warm reset, and NMI all
         * use this vector.
         */

	 /* XXX */


	/*
	 * Test to see if we're on the secondary CPU.  If so,
	 * go do the initialization for that CPU.
	 */
	
#if (CFG_MULTI_CPUS)
		JAL_KSEG1(CPUCFG_ALTCPU_RESET)
		/* does not return if on CPU1 */
#endif

#------------------------------------------------------------------------------

	/*
	 * Do low-level board initialization.  This is our first
	 * chance to customize the startup sequence.
	 */

		JAL_KSEG1(CPUCFG_EARLYINIT)

		SETLEDS1('H','E','L','O')

		JAL_KSEG1(CPUCFG_CPUINIT)

	/*
	 * Run some diagnostics
	 */

#if 0
#if !CFG_MINIMAL_SIZE && !defined(_ZIPSTART_)
		SETLEDS1('T','S','T','1')

		JAL_KSEG1(CPUCFG_DIAG_TEST1)
#endif
#endif


#------------------------------------------------------------------------------
#if CFG_MULTI_CPUS
	/*
	 * Spin up secondary CPU core(s)
	 */

		JAL_KSEG1(CPUCFG_ALTCPU_START1)
#endif

	/*
	 * Now, switch from KSEG1 to KSEG0
	 */


#if CFG_RUNFROMKSEG0
		bal	cpu_kseg0_switch
#endif

	/*
	 * Now go back to a cacheable version of the GOT
	 */
#if CFG_RELOC
		lui	gp,%hi(_gp)
		addiu	gp,%lo(_gp)
#endif

#------------------------------------------------------------------------------
	/*
	 * Now running on cpu0 in K0SEG.
	 */

#ifdef CPUCFG_PREDRAMINIT
	      SETLEDS('P','D','R','I')
              JAL(CPUCFG_PREDRAMINIT)
#endif


#if CFG_INIT_DRAM
		SETLEDS('D','R','A','M')

		JAL(CPUCFG_DRAMINFO)

__DramInit:
		move   a0,v0			# pass these params
		JAL(CPUCFG_DRAMINIT)

		move   k0,v0			# Save in k0 for now
#else
		li	k0,(CFG_DRAM_SIZE * 1024)

#if (CFG_DRAM_SIZE == 0)
		JAL(CPUCFG_DRAMINFO)		# hack escape to board code
		b	have_ram		# assume non-DRAM memory
#endif
#endif

#------------------------------------------------------------------------------

#if (CFG_BOOTRAM || CFG_L2_RAM)
		b      have_ram			# No RAM is ok if using emulator RAM
#endif

		bne    k0,zero,have_ram

		SETLEDS('R','A','M','X')	# die here if no ram

die1:		b      die1

have_ram:

	 /*
	  * If this CPU supports 64-bit registers, set STATUS
	  * to allow 64-bit accesses.
	  */

#if CPUCFG_REGS64
		mfc0	t0,C0_SR
		or	t0,t0,M_SR_KX
		mtc0	t0,C0_SR
#endif

#------------------------------------------------------------------------------
	/*
	 * K0 contains the RAM size (and therefore the top of RAM 
	 * offset).  Start there, and subtract the amount of memory
	 * we expect to use.  If we have more than 256MB of
	 * physical memory, work backwards from the 256MB 
	 * boundary.
	 */  

__CalcMemTop:   li	MEMTOP,256		# 256MB boundary
		bgt	k0,MEMTOP,1f		# use 256MB if k0 is greater
		move	MEMTOP,k0		# otherwise keep top
1:		sll	MEMTOP,20		# make into byte amount


		li	RELOCOFFSET,0		# not relocating, no offset

	/*
	 * DRAM is now running, and we're alive in cacheable memory
	 * on cpu0 in K0SEG.  Set up GP.
	 */

#if !CFG_RELOC
		la	gp,_gp
		add	gp,RELOCOFFSET
#endif

	/*
	 * SVR4 PIC support: with the eventual demise of -membedded-pic,
	 * This is the way we do relocations now.  CFE as a shareable
	 * library, wahoo! 
	 */

#if CFG_RELOC

#define REG_REL		    s0
#define REG_RELSZ	    s1
#define REG_SYMTAB	    s2
#define REG_PLTGOT	    s3
#define REG_GOTNO	    s4
#define REG_SYMTABNO	    s5
#define REG_GOTSYM	    s6

__CopyCode:

		SETLEDS('C','O','P','Y')

	/*
	 * Calculate final location of CFE.  MEMTOP is our
	 * top-of-memory address.
	 */

		la	v0,_ftext
		la	v1,_end
		sub	v0,v1,v0		# v0 = size of text+init+bss
		add	v0,v0,(CFG_HEAP_SIZE*1024)+STACK_SIZE
		add	v0,v0,0xFFF
		and	v0,~0xFFF		# round down to 4K byte boundary

		subu	TEXTBASE,MEMTOP,v0	# Start at this physical location
#if CFG_RUNFROMKSEG0
		or	TEXTBASE,K0BASE
#else
		or	TEXTBASE,K1BASE
#endif

	/*
	 * TEXTBASE = desired target address
	 */

	/*
	 * Copy the code and data 
	 */

		la	v0,_ftext
		la	v1,_image_end
		move	s0,TEXTBASE
		# li	s0,IMAGE_TARGET

1:		lw	t0,0(v0)
		lw	t1,4(v0)
		lw	t2,8(v0)
		lw	t3,12(v0)
		lw	t4,16(v0)
		lw	t5,20(v0)
		lw	t6,24(v0)
		lw	t7,28(v0)
		sw	t0,0(s0)
		sw	t1,4(s0)
		sw	t2,8(s0)
		sw	t3,12(s0)
		sw	t4,16(s0)
		sw	t5,20(s0)
		sw	t6,24(s0)
		sw	t7,28(s0)
		addu	v0,32
		addu	s0,32
		blt	v0,v1,1b

__DoRelocs:
		move	RELOCOFFSET,TEXTBASE

		la	v0,_ftext
		subu	RELOCOFFSET,v0


	/*
	 * Process the DYNAMIC section, gathering interesting
	 * information into CPU registers.
	 */

		SETLEDS('R','E','L','O')

		la	v0,_DYNAMIC		

11:		lw	t0,0(v0)			# End of DYNAMIC table
		beq	t0,DT_NULL,22f

		bne	t0,DT_REL,1f			# Relocation section
		lw	REG_REL,4(v0)
		addu	REG_REL,RELOCOFFSET
		b	33f

1:		bne	t0,DT_RELSZ,1f			# Size of relocation section
		lw	REG_RELSZ,4(v0)
		b	33f

1:		bne	t0,DT_SYMTAB,1f			# Symbol table section
		lw	REG_SYMTAB,4(v0)
		addu	REG_SYMTAB,RELOCOFFSET
		b	33f

1:		bne	t0,DT_PLTGOT,1f			# GOT section
		lw	REG_PLTGOT,4(v0)
		addu	REG_PLTGOT,RELOCOFFSET
		b	33f

1:		bne	t0,DT_MIPS_LOCAL_GOTNO,1f	# Size of GOT section
		lw	REG_GOTNO,4(v0)
		b	33f

1:		bne	t0,DT_MIPS_SYMTABNO,1f		# Size of symbol table
		lw	REG_SYMTABNO,4(v0)
		b	33f

1:		bne	t0,DT_MIPS_GOTSYM,1f		# Number of GOT symbols
		lw	REG_GOTSYM,4(v0)
		b	33f

1:
33:		add	v0,8			# Advance to next entry
		b	11b			# go back for more.

	/*
	 * Relocate the local GOT entries.  Skip the first
	 * entry, or the first two entries if entry #2 has its
	 * high bit set.
	 */

__LocalGOT:
22:
		li	v0,1
		lw	t0,4(REG_PLTGOT)	# Skip first entry
		bge	t0,zero,1f              # and second entry if high bit set
		li	v0,2
		add	REG_PLTGOT,4
1:		add	REG_PLTGOT,4

	 /*
	  * Now, apply 'relocbase' to the local GOT entries.
	  */

1:		lw	t0,0(REG_PLTGOT)
		addu	t0,RELOCOFFSET		# Relocation base
		sw	t0,0(REG_PLTGOT)
		add	REG_PLTGOT,4
		add	v0,1
		blt	v0,REG_GOTNO,1b

	 /*
	  * Do the global GOT entries.
	  */

__GlobalGOT:
		sll	REG_GOTSYM,4		# mult by 16 for offset
		sll	REG_SYMTABNO,4		# do ending offset too.

		add	v0,REG_SYMTAB,REG_GOTSYM # SYMTAB points at symbol table
		add	v1,REG_SYMTAB,REG_SYMTABNO # v1 points at end of sym table

1:		lw	t0,R_ELF32SYM_ST_VALUE(v0)
		addu	t0,RELOCOFFSET			# Relocation base
		sw	t0,0(REG_PLTGOT)

		add	REG_PLTGOT,4		# next GOT entry
		add	v0,16			# next SYM entry
		blt	v0,v1,1b
		

	/*
	 * Do the symbol table.
	 */

__DoSymTab:
		add	v1,REG_REL,REG_RELSZ	    # V1 points at end of symbol table

1:		bge	v0,v1,33f

		lw	t0,R_REL_OFFSET(v0)         # Get symbol value
		addu	t0,RELOCOFFSET		    # Relocation base
		lw	t1,R_REL_INFO(v0)           # Get symbol type

		andi	t2,t1,M_REL_TYPE            # Ignore R_MIPS_NONE
		bne	t2,K_REL_TYPE_REL32,2f      # only take R_MIPS_R32

#define SUPPORT_OLD_BINUTILS
#ifdef SUPPORT_OLD_BINUTILS
	/*
	 * sbtools 2.7.x (binutils prior to 2.14) and earlier toolchains
	 * need the symbol value added in when relocating.  With new
	 * toolchains, which are more correct w.r.t. the SVR4 ABI, doing
	 * that is incorrect and * will cause the firmware to be
	 * relocated badly.
	 *
	 * We use a heuristic: if the relocated value with the symbol
	 * value added in is less than the relocated object base, we
	 * recalculate without the symbol value.  The theory is that no
	 * relocations will point outside the firmware.
	 */
		srl	t1,S_REL_SYM                # Symbol "info" is index
		sll	t1,4			    # back to symtab index
		add	t1,t1,REG_SYMTAB	    # t1 points at symtab entry

		lw	t2,R_ELF32SYM_ST_VALUE(t1)  # Symbol value
		addu	t2,RELOCOFFSET		    # Relocation base

		lw	t1,0(t0)		    # Read original word
		addu	t1,t2			    # Add in offset

		.set push
		.set noat
		sltu	AT, t1, TEXTBASE
		beqz	AT, 3f
		nop
		.set pop
#endif

		lw	t1,0(t0)		    # Read original word
		addu	t1,RELOCOFFSET		    # Add in relocation offset
#ifdef SUPPORT_OLD_BINUTILS
3:
#endif
		sw	t1,0(t0)		    # Write it back out

2:		add	v0,8                        # more punishment.
		b	1b

33:

#endif

#------------------------------------------------------------------------------
	/*
	 * Zero BSS
         */

		SETLEDS('Z','B','S','S')

		la	a0,segment_table
__ZeroBss:

		LR	v0,R_SEG_FBSS(a0)
		LR	v1,R_SEG_END(a0)
		ADD	v0,RELOCOFFSET		# Relocate to actual data segment
		ADD	v1,RELOCOFFSET

1:		SR	zero,0(v0)		# Zero one cacheline at a time
		SR	zero,(REGSIZE*1)(v0)
		SR	zero,(REGSIZE*2)(v0)
		SR	zero,(REGSIZE*3)(v0)
		add	v0,REGSIZE*4
		blt	v0,v1,1b

#------------------------------------------------------------------------------

	/*
	 * Copy initialized data (non-relocating case only, and
	 * not when we're using the PromICE memory as RAM)
         */

#if (!CFG_RELOC)
#if (CFG_BOOTRAM == 0)

		SETLEDS('D','A','T','A')

		la	a0,segment_table

__CopyData:
		LR	t1,R_SEG_ETEXT(a0)
		li	t0,15
		add	t1,t0
		not	t0
		and	t1,t0		# t1 = _etext rounded up to 16-byte boundary
			
		LR	t2,R_SEG_FDATA(a0)
		LR	t3,R_SEG_EDATA(a0)
		ADD	t2,RELOCOFFSET	# Relocate to actual data segment
		ADD	t3,RELOCOFFSET

1:		LR	t4,0(t1)	# read one cache line
		LR	t5,(REGSIZE*1)(t1)
		LR	t6,(REGSIZE*2)(t1)
		LR	t7,(REGSIZE*3)(t1)
		SR	t4,0(t2)	# write one cache line
		SR	t5,(REGSIZE*1)(t2)
		SR	t6,(REGSIZE*2)(t2)
		SR	t7,(REGSIZE*3)(t2)
		add	t1,(REGSIZE*4)
		add	t2,(REGSIZE*4)
		bltu	t2,t3,1b

#endif /* (CFG_BOOTRAM == 0) */
#endif /* (!CFG_RELOC) */


#------------------------------------------------------------------------------


#if (CFG_RELOC)
#if CFG_RUNFROMKSEG0
		SETLEDS('L','1','2','F')

		li	a0,CFE_CACHE_FLUSH_D | CFE_CACHE_FLUSH_L2
		JAL(CPUCFG_CACHEOPS)

		li	a0,CFE_CACHE_INVAL_I
		JAL(CPUCFG_CACHEOPS)
#endif /* CFG_RUNFROMKSEG0 */

__GoRelo:

		la	t0,1f
		addu	gp,RELOCOFFSET
		addu	t0,RELOCOFFSET
		jr	t0

1:
#endif /* CFG_RELOC */



	/*
         * Remember total amount of memory.  This is *still* in k0
	 * after all this time.  Hopefully.
	 */

__MemVars:
		SR	k0,mem_totalsize
		SR	RELOCOFFSET,mem_datareloc

		move	v0,zero

		la	a0,segment_table	# trashed by l2 cache flush
#if (CFG_RELOC)
		LR	v0,R_SEG_FTEXT(a0)
#else
		LR	v0,R_SEG_FDATA(a0)
		ADD	v0,RELOCOFFSET
#endif
		LR	v1,R_SEG_END(a0)
#if !CFG_RELOC
		ADD	v1,RELOCOFFSET
#endif

#if CFG_RELOC
		ADD	v1,31			# Realign _END so it 
		and	v1,~31			# is on a 64-bit boundary.
#endif

		SR	v0,mem_bottomofmem
		SR	v1,mem_heapstart

		add	v1,(CFG_HEAP_SIZE*1024)	# Otherwise
		add	v1,STACK_SIZE
		SR	v1,mem_topofmem

		SR	RELOCOFFSET,mem_textreloc


		LR	t1,R_SEG_FTEXT(a0)
		LR	t0,R_SEG_ETEXT(a0)
		sub	t0,t0,t1
		SR	t0,mem_textsize
		SR	t1,mem_textbase


#------------------------------------------------------------------------------

#if CFG_MULTI_CPUS
	/*
	 * Let secondary CPU(s) run their idle loops.  Set the 
	 * mailbox register to our relocation factor so we can read
	 * it out of the mailbox register and relocate GP properly.
	 */

		move	a0,RELOCOFFSET
		JAL(CPUCFG_ALTCPU_START2)
#endif

#ifdef _SB1250_PASS1_WORKAROUNDS_
	/*
	 * Okay, it's safe now to be coherent.  
	 * Flush the D cache to invalidate all the lines we have,
	 * then change the config register back.
	 */
		li	a0,CFE_CACHE_FLUSH_D
		JAL(CPUCFG_CACHEOPS)

		SETCCAMODE(v0,K_CFG_K0COH_COHERENT) /* cacheable coherent */
#endif
		
	/*
	 * Stash away some config register stuff
         */

		mfc0	v0,C0_PRID
		SR	v0,cpu_prid		


#------------------------------------------------------------------------------

	/*
	 * Set up the "C" stack and jump to the main routine.
         */

		SETLEDS('M','A','I','N')

		LR	sp,mem_heapstart
		ADD	sp,((CFG_HEAP_SIZE*1024)+STACK_SIZE - 8)
		li	a0,0			# call as "cfe_main(0,0)"
		li	a1,0

__LaunchMain:

		JAL(cfe_main)


	/*
	 * Terminate the simulator.
	 */

crash_sim:      li $2,1
                li $4,0
                syscall	0xCA
		b	cpu_reset



/*  *********************************************************************
    *  CPU_KSEG0_SWITCH
    *  
    *  Hack the return address so we will come back in KSEG0
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

cpu_kseg0_switch:

		and	ra,(K0SIZE-1)
		or	ra,K0BASE
		jr	ra




/*  *********************************************************************
    *  CFE_LAUNCH
    *  
    *  Start the user program.  The program is passed a handle
    *  that must be passed back when calling the firmware.
    *
    *  Parameters passed to the called program are as follows:
    *
    *      a0 - CFE handle
    *      a1 - entry vector
    *      a2 - reserved, will be 0
    *      a3 - entrypoint signature.
    *  
    *  Input parameters: 
    *  	   a0 - entry vector
    *  	   
    *  Return value:
    *  	   does not return
    ********************************************************************* */

LEAF(cfe_launch)

		sub	sp,8
		SR	a0,0(sp)


	/*
	 * Mask all interrupts.
	 */
		mfc0	v0,C0_SR		# Get current interrupt flag
		li	v1,M_SR_IE		# master interrupt control
		not	v1			# disable interrupts
		and	v0,v1			# SR now has IE=0
		mtc0	v0,C0_SR		# put back into CP0


	/*
	 * Flush the D-Cache, since the program we loaded is "data".
	 * Invalidate the I-Cache, so that addresses in the program
	 * region will miss and need to be filled from the data we 
	 * just flushed above.
	 */

		li	a0,CFE_CACHE_FLUSH_D|CFE_CACHE_INVAL_I
		JAL(CPUCFG_CACHEOPS)

		
	/*
	 * Set things up for launching the program.  Pass the
	 * handle in A0 - apps need to remember that and pass it
	 * back.
	 */

		j	RunProgram


	/*
 	 * This is a nice place to set a breakpoint.
	 */

RunProgram:

		la	a2,cpu_apientry # a2 = entry point
		move	t0,a0		# 
		move	a1,zero		# A1 = 0
		move	a0,gp		# A0 = handle (GOT pointer)
		li	a3,CFE_EPTSEAL  # A3 = entrypoint signature
		LR	t0,0(sp)	# entry point

		j	t0		# go for it.
END(cfe_launch)



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

		JMP(board_setleds)		# jump to BSP routine

END(cfe_leds)

/*  *********************************************************************
    *  TLB Fill Exeption Handler
    *
    *  In the SVR4 PIC case, get our GOT pointer from the locore area
    *  and use that.  Then, restore GP.  Otherwise, just load the
    *  address and go.
    ********************************************************************* */

cpu_tlbfill:	
		.set noat
#if CFG_RELOC
		LDADDR(k1,CPUCFG_TLBHANDLER)
#else
		la	k1,CPUCFG_TLBHANDLER
#endif
		j	k1			# Dispatch to handler

		.set at

/*  *********************************************************************
    *  XTLB Fill Exception Handler (shouldn't happen)
    ********************************************************************* */

cpu_xtlbfill:	
		LDADDR(k1,_exc_entry)
		j	k1

/*  *********************************************************************
    *  Cache Error Exception Handler
    ********************************************************************* */

cpu_cacheerr:

#if defined(_CSWARM_) || defined(_SWARM_) || defined(_BCM91120C_) || defined(_PTSWARM_) || defined(_BCM91250PT_) || defined(_BIGSUR_)
#define LED_CHAR0	(32+8*3)
#define LED_CHAR1	(32+8*2)
#define LED_CHAR2	(32+8*1)
#define LED_CHAR3	(32+8*0)
#if defined(_PTSWARM_) || defined(_BCM91250PT_)
		li    k0,0xBB0A0000	     /* address of LEDs */
#else
		li    k0,0xB00A0000	     /* address of LEDs */
#endif
		li    k1,'C'
		sb    k1,LED_CHAR0(k0)
		li    k1,'e'
		sb    k1,LED_CHAR1(k0)
		li    k1,'r'
		sb    k1,LED_CHAR2(k0)
		li    k1,'2'
		sb    k1,LED_CHAR3(k0)

		SETLEDS1('C','e','r','2')
#endif

cpu_cache_death:	b	cpu_cache_death



/*  *********************************************************************
    *  General Exception Handler (shouldn't happen)
    ********************************************************************* */

cpu_exception:
		JMP(_exc_entry)


/*  *********************************************************************
    *  General Interrupt Handler (shouldn't happen)
    ********************************************************************* */

cpu_interrupt:
		JMP(_exc_entry)


/*  *********************************************************************
    *  EJTAG Debug Exception Handler
    ********************************************************************* */

cpu_ejtag:
		.set push
		.set mips64
		deret
		.set pop
		j	cpu_reset



/*  *********************************************************************
    *  End
    ********************************************************************* */



