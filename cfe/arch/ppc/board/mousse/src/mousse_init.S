/*  *********************************************************************
    *  PPC Board Support Package
    *  
    *  Board-specific initialization		File: MOUSSE_INIT.S
    *
    *  This module contains the assembly-language part of the init
    *  code for this board support package.  The routine
    *  "board_earlyinit" lives here.
    *
    *  This board support package is for the MPC8245 Mousse board
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
#include "bsp_config.h"
#include "mousse.h"
#include "init_engine.h"
#include "mpc824x.h"


/*  *********************************************************************
    *  Global Offset Table
    ********************************************************************* */

#if CFG_RELOC
BEGIN_GOT()
GOT_ENTRY(board_draminittab)
END_GOT()
#endif

		.text

/*  *********************************************************************
    *  Macros
    ********************************************************************* */

#define DRAM_BASE	0
#define DRAM_SIZE	64	/* megabytes */


/*  *********************************************************************
    *  BOARD_EARLYINIT()
    *  
    *  Initialize board registers.  This is the earliest 
    *  time the BSP gets control.  This routine cannot assume that
    *  memory is operational, and therefore all code in this routine
    *  must run from registers only.  The lr register must not
    *  be modified, as it contains the return address.
    *
    *  This routine will be called from uncached space, before
    *  the caches are initialized.
    *
    *  Among other things, this is where the GPIO registers get 
    *  programmed to make on-board LEDs function, or other startup
    *  that has to be done before anything will work.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

LEAF(board_earlyinit)


	blr

END(board_earlyinit)


/*  *********************************************************************
    *  BOARD_DRAM_INIT
    *  
    *  Initialize DRAM controller.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   r3 - dram size in megabytes
    ********************************************************************* */

LEAF(board_dram_init)

	mflr	r29

	/* LOAD_GOT trashes LR, but we saved it above. */
	LOAD_GOT(GOT_REG)
	LDADDR(r6,board_draminittab)

	bl	init_engine

	li	r3,DRAM_SIZE

	mtlr	r29

	blr

END(board_dram_init)



/*  *********************************************************************
    *  BOARD_SETLEDS(x)
    *  
    *  Set LEDs for boot-time progress indication.  Not used if
    *  the board does not have progress LEDs.  This routine
    *  must not call any other routines.
    *  
    *  Input parameters: 
    *  	   r3 - LED value (8 bits per character, 4 characters)
    *  	   
    *  Return value:
    *  	   nothing
    *  
    *  Registers used:
    *  	   r3,r5,r6
    ********************************************************************* */

#define WAIT_SERIAL(base,tmp1,tmp2) \
88: lbz tmp1,5(base) ; \
   andi. tmp2,tmp1,0x40 ; \
   bc 12,2,88b

LEAF(board_setleds)

#if CFG_SERIAL_LEDS

	LDCONST(r5, 0xffe08080)

	li	r6, 0x83
	stb	r6, 3(r5)
	li	r6, 0x0a
	stb	r6, 0(r5)
	li	r6, 0x00
	stb	r6, 1(r5)
	li	r6, 0x03
	stb	r6, 3(r5)
	li	r6, 0x0b
	stb	r6, 4(r5)

	WAIT_SERIAL(r5,r6,r0)
	li	r0,0x5B		/* [ */
	stb	r0, 0(r5)
	sync

	rlwinm	r3,r3,8,0,31

	WAIT_SERIAL(r5,r6,r0)
	stb	r3, 0(r5)
	sync

	rlwinm	r3,r3,8,0,31

	WAIT_SERIAL(r5,r6,r0)
	stb	r3, 0(r5)
	sync

	rlwinm	r3,r3,8,0,31

	WAIT_SERIAL(r5,r6,r0)
	stb	r3, 0(r5)
	sync

	rlwinm	r3,r3,8,0,31

	WAIT_SERIAL(r5,r6,r0)
	stb	r3, 0(r5)
	sync

	WAIT_SERIAL(r5,r6,r0)
	li	r0,0x5D		/* ] */
	stb	r0, 0(r5)
	sync
#endif


	blr

END(board_setleds)



/*  *********************************************************************
    *  BOARD_OUTNUM
    *  
    *  Write a number to the serial port.
    *  
    *  Input parameters: 
    *  	   r3 - number to write
    *  	   
    *  Return value:
    *  	   nothing
    *
    *  Registers used:
    *      r5,r6,r4
    ********************************************************************* */

LEAF(board_outnum)

	LDCONST(r5, 0xffe08080)

	WAIT_SERIAL(r5,r6,r0)
	li	r0,0x5B		/* [ */
	stb	r0, 0(r5)
	sync

	rlwinm	r3,r3,4,0,31
	andi.   r4,r3,0x0F
	addi	r4,r4,0x30
	cmpwi	r4,0x39
	ble	1f
	addi	r4,r4,(0x41-0x3A)
1:	WAIT_SERIAL(r5,r6,r0)
	stb	r4,0(r5)
	sync
	
	rlwinm	r3,r3,4,0,31
	andi.   r4,r3,0x0F
	addi	r4,r4,0x30
	cmpwi	r4,0x39
	ble	1f
	addi	r4,r4,(0x41-0x3A)
1:	WAIT_SERIAL(r5,r6,r0)
	stb	r4,0(r5)
	sync
	
	rlwinm	r3,r3,4,0,31
	andi.   r4,r3,0x0F
	addi	r4,r4,0x30
	cmpwi	r4,0x39
	ble	1f
	addi	r4,r4,(0x41-0x3A)
1:	WAIT_SERIAL(r5,r6,r0)
	stb	r4,0(r5)
	sync
	
	rlwinm	r3,r3,4,0,31
	andi.   r4,r3,0x0F
	addi	r4,r4,0x30
	cmpwi	r4,0x39
	ble	1f
	addi	r4,r4,(0x41-0x3A)
1:	WAIT_SERIAL(r5,r6,r0)
	stb	r4,0(r5)
	sync
	
	rlwinm	r3,r3,4,0,31
	andi.   r4,r3,0x0F
	addi	r4,r4,0x30
	cmpwi	r4,0x39
	ble	1f
	addi	r4,r4,(0x41-0x3A)
1:	WAIT_SERIAL(r5,r6,r0)
	stb	r4,0(r5)
	sync
	
	rlwinm	r3,r3,4,0,31
	andi.   r4,r3,0x0F
	addi	r4,r4,0x30
	cmpwi	r4,0x39
	ble	1f
	addi	r4,r4,(0x41-0x3A)
1:	WAIT_SERIAL(r5,r6,r0)
	stb	r4,0(r5)
	sync
	
	rlwinm	r3,r3,4,0,31
	andi.   r4,r3,0x0F
	addi	r4,r4,0x30
	cmpwi	r4,0x39
	ble	1f
	addi	r4,r4,(0x41-0x3A)
1:	WAIT_SERIAL(r5,r6,r0)
	stb	r4,0(r5)
	sync
	
	rlwinm	r3,r3,4,0,31
	andi.   r4,r3,0x0F
	addi	r4,r4,0x30
	cmpwi	r4,0x39
	ble	1f
	addi	r4,r4,(0x41-0x3A)
1:	WAIT_SERIAL(r5,r6,r0)
	stb	r4,0(r5)
	sync
	
	WAIT_SERIAL(r5,r6,r0)
	li	r0,0x5D		/* ] */
	stb	r0, 0(r5)
	sync

	blr
END(board_outnum)



/*  *********************************************************************
    *  Initialization table for memory controller.  
    *
    *  This table is processsed by the "init_engine" to 
    *  orchestrate writes to PCI space and CPU MSRs.
    ********************************************************************* */

#ifndef CFG_DRAM_ECC
#define CFG_DRAM_ECC	0
#endif

#if CFG_DRAM_ECC
#define MCCR1_ECC_BITS M_MCCR1_PCKEN
#define MCCR2_ECC_BITS (M_MCCR2_WRPARCHK | M_MCCR2_INLPARNOECC | M_MCCR2_RMWPAR)
#define MCCR3_ECC_BITS 0
#define MCCR4_ECC_BITS M_MC4_BUFTYPE0
#else
#define MCCR1_ECC_BITS 0
#define MCCR2_ECC_BITS 0
#define MCCR3_ECC_BITS 0
#define MCCR4_ECC_BITS M_MCCR4_BUFTYPE1
#endif

#define MC_MCCR_BSTOPRE 0x79


board_draminittab:


	/*
	 * Initialize memory controller.
	 */

	IET_PCI32(MPC_MCCR1,V_MCCR1_ROMNAL(8) | V_MCCR1_ROMFAL(8) | M_MCCR1_SREN | 
	        MCCR1_ECC_BITS |
		V_MCCR1_BANKnROW(0,K_MCCR1_ROW10)  |
		V_MCCR1_BANKnROW(1,K_MCCR1_ROW10)  |
		V_MCCR1_BANKnROW(2,K_MCCR1_ROW10)  |
		V_MCCR1_BANKnROW(3,K_MCCR1_ROW10)  |
		V_MCCR1_BANKnROW(4,K_MCCR1_ROW10)  |
		V_MCCR1_BANKnROW(5,K_MCCR1_ROW10)  |
		V_MCCR1_BANKnROW(6,K_MCCR1_ROW10)  |
		V_MCCR1_BANKnROW(7,K_MCCR1_ROW10))


	IET_PCI32(MPC_MCCR2,V_MCCR2_TSWAIT(0) | V_MCCR2_ASRISE(8) | V_MCCR2_ASFALL(4) |
		MCCR2_ECC_BITS |
		V_MCCR2_REFINT(0xf5))

	IET_PCI32(MPC_MCCR3,V_MCCR3_BSTOPRE(((MC_MCCR_BSTOPRE >> 4) & 0x0F)) |
		V_MCCR3_REFREC(8) |
                MCCR3_ECC_BITS |
		V_MCCR3_RDLAT(4+CFG_DRAM_ECC))

	IET_PCI32(MPC_MCCR4,V_MCCR4_PRETOACT(3) | V_MCCR4_ACTOPRE(5) |
		V_MCCR4_ACTORW(3) |
                MCCR4_ECC_BITS |
		V_MCCR4_BSTOPRE01((MC_MCCR_BSTOPRE >> 8) & 3) |
		V_MCCR4_SDMODE(0x32) |
		V_MCCR4_BSTOPRE69(MC_MCCR_BSTOPRE & 0x0F))
#if CFG_DRAM_ECC
	IET_PCI32(MPC_ERREN1,xxx)
#endif
	IET_PCI32(MPC_MSAR1,0xC0804000)
	IET_PCI32(MPC_MSAR2,0xc0804000)

	IET_PCI32(MPC_XMSAR1,0x00000000)
	IET_PCI32(MPC_XMSAR2,0x01010101)

	IET_PCI32(MPC_MEAR1,0xffbf7f3f)
	IET_PCI32(MPC_MEAR2,0xffbf7f3f)

	IET_PCI32(MPC_XMEAR1,0x00000000)
	IET_PCI32(MPC_XMEAR2,0x01010101)

	IET_PCI8(MPC_ODCR,0x7f)

	IET_PCI8(MPC_MBEN,0x03)

	IET_PCI8(MPC_PGMODE,0x32)

	IET_DELAY(0x10)
	IET_PCI32X(MPC_MCCR1,0xFFFFFFFF,M_MCCR1_MEMGO)
	IET_DELAY(0x4000)

	/*
	 * Done with memory controller.
	 */

	IET_PCI32(MPC_EUMBBAR,A_MOUSSE_EUMBBAR_VAL)


	/*
	 * setup BATs
	 */

	/* BAT 0 : SDRAM */
	IET_IBAT(0,(DRAM_BASE | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (DRAM_BASE | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_MEMCOHERENCE))
	IET_DBAT(0,(DRAM_BASE | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (DRAM_BASE | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_MEMCOHERENCE))

        /* BAT 1 : DOC */
	IET_IBAT(1,(0x70000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0x70000000 | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_INHIBIT))
	IET_DBAT(1,(0x70000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0x70000000 | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_INHIBIT))

        /* BAT 2 : PCI */
	IET_IBAT(2,(0x80000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0x80000000 | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_INHIBIT))
	IET_DBAT(2,(0x80000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0x80000000 | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_INHIBIT))

        /* BAT 3 : EUMBAR, FLASH, PLD */
	IET_IBAT(3,(0xF0000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0xF0000000 | V_BATL_PP(K_BATL_PP_RDWR) | 0/*M_BATL_INHIBIT*/))
	IET_DBAT(3,(0xF0000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0xF0000000 | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_INHIBIT))

	IET_ENABBATS()

	IET_SPRX(SPR_HID0,0xffffffff,(M_HID0_DCE|M_HID0_DCFI))	
	IET_SPRX(SPR_HID0,~M_HID0_DCFI,M_HID0_DCE)	

	IET_EOT()


/*  *********************************************************************
    *  Misc functions
    ********************************************************************* */


	.globl read_hid0
read_hid0:

	mfspr r3,SPR_HID0
	blr

	.globl write_hid0
write_hid0:

	isync
	mtspr SPR_HID0,r3
	sync
	isync
	eieio
	blr

	.globl read_hid1
read_hid1:

	mfspr r3,SPR_HID1
	blr

	.globl read_msr
read_msr:

	mfmsr r3
	blr

	.globl write_msr
write_msr:

	isync
	mtmsr r3
	sync
	isync
	eieio
	blr

	.globl read_bats
read_bats:
	mfspr r0,SPR_IBAT0U
	stw    r0,0(r3)
	mfspr r0,SPR_IBAT0L
	stw    r0,4(r3)
	mfspr r0,SPR_IBAT1U
	stw    r0,8(r3)
	mfspr r0,SPR_IBAT1L
	stw    r0,12(r3)
	mfspr r0,SPR_IBAT2U
	stw    r0,16(r3)
	mfspr r0,SPR_IBAT2L
	stw    r0,20(r3)
	mfspr r0,SPR_IBAT3U
	stw    r0,24(r3)
	mfspr r0,SPR_IBAT3L
	stw    r0,28(r3)

	mfspr r0,SPR_DBAT0U
	stw    r0,32(r3)
	mfspr r0,SPR_DBAT0L
	stw    r0,36(r3)
	mfspr r0,SPR_DBAT1U
	stw    r0,40(r3)
	mfspr r0,SPR_DBAT1L
	stw    r0,44(r3)
	mfspr r0,SPR_DBAT2U
	stw    r0,48(r3)
	mfspr r0,SPR_DBAT2L
	stw    r0,52(r3)
	mfspr r0,SPR_DBAT3U
	stw    r0,56(r3)
	mfspr r0,SPR_DBAT3L
	stw    r0,60(r3)

	blr

	
