/*  *********************************************************************
    *  PPC Board Support Package
    *  
    *  Board-specific initialization		File: BMW_INIT.S
    *
    *  This module contains the assembly-language part of the init
    *  code for this board support package.  The routine
    *  "board_earlyinit" lives here.
    *
    *  This board support package is for the MPC8245 BMW board
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
#include "bmw.h"
#include "init_engine.h"
#include "mpc824x.h"
#include "sdram_spd.h"

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

#define PCICW32(addr, data) LDCONST(r3, (addr)); LDCONST(r4, (data)); bl pci_conf_write_32

#ifndef CFG_DRAM_ECC
#define CFG_DRAM_ECC	0
#endif

#if CFG_DRAM_ECC
#define MCCR1_ECC_BITS M_MCCR1_PCKEN
#define MCCR2_ECC_BITS (M_MCCR2_WRPARCHK | M_MCCR2_INLPARNOECC | M_MCCR2_RMWPAR)
#define MCCR3_ECC_BITS 0
#define MCCR4_ECC_BITS M_MCCR4_BUFTYPE0
#else
#define MCCR1_ECC_BITS 0
#define MCCR2_ECC_BITS 0
#define MCCR3_ECC_BITS 0
#define MCCR4_ECC_BITS M_MCCR4_BUFTYPE1
#endif

#define MC_MCCR_BSTOPRE 0x79

/* Default memory configuration

 This is the memory configuration that will be used if the SPD results
 in an error.

 Memory is specified in terms of the physical SDRAM architecture, such
 as the number of rows and columns of the memory devices, as well as
 the number of physical banks (chip selects). Due to the design of
 both the MPC8245 and the BMW subsystem the parameter limits are as
 follows:

      SDRAM memory rows: 12 - 13        (SPD parameter #3)
   SDRAM memory columns:  8 - 11        (SPD parameter #4)
   SDRAM physical banks:  1 - 2         (SPD parameter #5)

 The number of logical banks is fixed at 4.

 Configurations tested:

  Size   Rows   Columns  Banks
 ------ ------- ------- -------
  64MB       12       9       1
 128MB       12       9       2
 256MB       12      10       2

 */

#define SDRAM_ROWS      12
#define SDRAM_COLUMNS    9
#define SDRAM_BANKS      1

/*
  Calculate the size of a SDRAM physical bank in Megabytes. This size
  is num_logical_banks * memory_module_bit_width * 2^(rows+columns) bits.
  num_logical_banks is always 4 (2^2)
  memory_module_bit_width is always 64 (2^6)
  bits_per_byte is 8 (2^3)

  2^(rows+columns)*(2^2)*(2^6)/(2^20 * 2^3) = 2^(rows+columns-15)

*/

#define SDRAM_BANK_SIZE  (1<<(SDRAM_ROWS + SDRAM_COLUMNS - 15))



/* Set up MPC8245 memory configuration constants

   Memory controller parameters are encoded from sdram_spd

   Board info is encoded in 32 bits:

   00000000001111111111222222222233
   01234567890123456789012345678901
   --------------------------------
   ----------------slbrmmmm--------

   s    SPD Probe Result
         1 = SPD error
         0 = SPD success
   l    Logical banks
         0 = 2 logical banks
         1 = 4 logical banks
   b    Physical banks
         0 = 1 physical banks
         1 = 2 physical banks
   r    Device row addresses
         0 = 4096 row addresses
         1 = 8192 row addresses
   m    Physical bank size in 64MB units
         1 = 64MB
         2 = 128MB
         4 = 256MB
         4 = 512MB
 */

#define MC1_ROW13_BANKBITS 0xAAAA

#define MC2_REFINT_ROW12   0x3a5        /* Refresh Values are conservative */
#define MC2_REFINT_ROW13   0x1d2

#define MC_MSAR1_64   0x4000
#define MC_XSAR1_64   0x0000
#define MC_MEAR1_64   0x7f3f
#define MC_XEAR1_64   0x0000
#define BANK_SIZE_64      64

#define MC_MSAR1_128  0x8000
#define MC_XSAR1_128  0x0000
#define MC_MEAR1_128  0xff7f
#define MC_XEAR1_128  0x0000
#define BANK_SIZE_128    128

#define MC_MSAR1_256  0x0000
#define MC_XSAR1_256  0x0100
#define MC_MEAR1_256  0xffff
#define MC_XEAR1_256  0x0100
#define BANK_SIZE_256    256

#define MC_MSAR1_512  0x0000
#define MC_XSAR1_512  0x0200
#define MC_MEAR1_512  0xffff
#define MC_XEAR1_512  0x0301
#define BANK_SIZE_512    512

#if SDRAM_ROWS == 13
#define MC1_BANKBITS MC1_ROW13_BANKBITS
#define MEM_DEFAULT_ROW SPD_MEM_CONF_ROW
#elif SDRAM_ROWS == 12
#define MC1_BANKBITS 0x0000
#define MEM_DEFAULT_ROW 0
#else
#error "SDRAM_ROWS must be 12 or 13"
#endif /* SDRAM_ROWS */

#if SDRAM_BANKS == 2
#define MEM_DEFAULT_BANK SPD_MEM_CONF_BANK
#elif SDRAM_BANKS == 1
#define MEM_DEFAULT_BANK 0
#else
#error "SDRAM_BANKS must be 1 or 2"
#endif /* SDRAM_BANKS */

#define MEM_DEFAULT_SZ ((SDRAM_BANK_SIZE & 0x03C0)<<2)
#define MEM_CONF_DEFAULT \
  (SPD_MEM_CONF_LBNK|MEM_DEFAULT_ROW|MEM_DEFAULT_BANK|MEM_DEFAULT_SZ)

LEAF(board_dram_init)

	mflr	r29

	/* LOAD_GOT trashes LR, but we saved it above. */
	LOAD_GOT(GOT_REG)

       	/*
	 * Set up on-chip peripherals
	 */

 	PCICW32(MPC_EUMBBAR,A_BMW_EUMBBAR_VAL)


	/*
	 * Setup chip-select regions
	 */

	/* 0x84000000 */
	PCICW32(MPC_ERCR1,(M_ERCR12_EN | V_ERCR12_DBW(K_ERCR12_DBW_8BIT) | V_ERCR12_CTL(K_ERCR12_CTL_BASE)))
	PCICW32(MPC_ERCR2,(M_ERCR12_EN | V_ERCR12_DBW(K_ERCR12_DBW_8BIT) | V_ERCR12_CTL(K_ERCR12_CTL_BASE)))

	PCICW32(MPC_ERCR3,((A_BMW_XROM & M_ERCR34_SADDR) | V_ERCR34_SIZE(K_ERCR34_SIZE_64K)))
	PCICW32(MPC_ERCR4,((A_BMW_DOC & M_ERCR34_SADDR) | V_ERCR34_SIZE(K_ERCR34_SIZE_8K)))

        /*
         * Probe SDRAM for size
         */
#include "sdram_spd.s"

	cmpi	0, 0, r3, 0
        bgt     set_mem_config
set_mem_default:
        LDCONST(r3, (MEM_CONF_DEFAULT|SPD_MEM_CONF_ERR))
set_mem_config:
        mr r6, r3

      	/*
	 * Initialize memory controller.
	 */

        LDCONST(r3, MPC_MCCR1)
	LDCONST(r4, V_MCCR1_ROMNAL(8) | V_MCCR1_ROMFAL(11) | M_MCCR1_SREN |
	        MCCR1_ECC_BITS)

        /* Check for 8192 row device detected/configured */
        andi.   r7, r6, SPD_MEM_CONF_ROW
        beq     set_mem_row_4096

        /* 8192 rows */
set_mem_row_8192:
	ori	r4, r4, MC1_ROW13_BANKBITS
        bl pci_conf_write_32

        LDCONST(r3, MPC_MCCR2)
	LDCONST(r4, V_MCCR2_TSWAIT(0) | V_MCCR2_ASRISE(8) | V_MCCR2_ASFALL(4) |
		MCCR2_ECC_BITS |
		V_MCCR2_REFINT(MC2_REFINT_ROW13))
        bl pci_conf_write_32
        b set_mem_row_done

set_mem_row_4096:
        bl pci_conf_write_32

        LDCONST(r3, MPC_MCCR2)
	LDCONST(r4, V_MCCR2_TSWAIT(0) | V_MCCR2_ASRISE(8) | V_MCCR2_ASFALL(4) |
		MCCR2_ECC_BITS |
		V_MCCR2_REFINT(MC2_REFINT_ROW12))
        bl pci_conf_write_32

set_mem_row_done:
	PCICW32(MPC_MCCR3,V_MCCR3_BSTOPRE(((MC_MCCR_BSTOPRE >> 4) & 0x0F)) |
		V_MCCR3_REFREC(8))

	PCICW32(MPC_MCCR4,V_MCCR4_PRETOACT(3) | V_MCCR4_ACTOPRE(5) |
		V_MCCR4_ACTORW(3) |
		M_MCCR4_EXTROM |
		M_MCCR4_DBUS_SIZE |
                MCCR4_ECC_BITS |
		V_MCCR4_BSTOPRE01((MC_MCCR_BSTOPRE >> 8) & 3) |
		V_MCCR4_SDMODE(0x2) |
		V_MCCR4_SDMODE_CAS(0x022) |
		V_MCCR4_BSTOPRE69(MC_MCCR_BSTOPRE & 0x0F))
#if CFG_DRAM_ECC
	PCICW32(MPC_ERREN1,xxx)
#endif

        /* Set memory starting and ending addresses for banks 1 and 2

           Register usage for this section:

           r3:  scratch, arg1
           r4:  scratch, arg2
           r6:  SPD flags
           r7:  SPD Bank size flags
           r16: Bank size in MB - must not be modified by calls.

        */

        andi.   r7, r6, SPD_MEM_CONF_SZ

bank_size_512:

	cmpli	0, 0, r7, SPD_MEM_CONF_512
        bne     bank_size_256

        /* 512MB Bank */
	PCICW32(MPC_MSAR1,  MC_MSAR1_512)
	PCICW32(MPC_XMSAR1, MC_XSAR1_512)
	PCICW32(MPC_MEAR1,  MC_MEAR1_512)
	PCICW32(MPC_XMEAR1, MC_XEAR1_512)
        LDCONST(r16, BANK_SIZE_512)

        b       bank_size_done

bank_size_256:

	cmpli	0, 0, r7, SPD_MEM_CONF_256
        bne     bank_size_128

        /* 256MB Bank */
	PCICW32(MPC_MSAR1,  MC_MSAR1_256)
	PCICW32(MPC_XMSAR1, MC_XSAR1_256)
	PCICW32(MPC_MEAR1,  MC_MEAR1_256)
	PCICW32(MPC_XMEAR1, MC_XEAR1_256)
        LDCONST(r16, BANK_SIZE_256)

        b       bank_size_done

bank_size_128:

	cmpli	0, 0, r7, SPD_MEM_CONF_128
        bne     bank_size_64

        /* 128MB Bank */
	PCICW32(MPC_MSAR1,  MC_MSAR1_128)
	PCICW32(MPC_XMSAR1, MC_XSAR1_128)
	PCICW32(MPC_MEAR1,  MC_MEAR1_128)
	PCICW32(MPC_XMEAR1, MC_XEAR1_128)
        LDCONST(r16, BANK_SIZE_128)

        b       bank_size_done

bank_size_64:

        /* 64MB Bank */
	PCICW32(MPC_MSAR1,  MC_MSAR1_64)
	PCICW32(MPC_XMSAR1, MC_XSAR1_64)
	PCICW32(MPC_MEAR1,  MC_MEAR1_64)
	PCICW32(MPC_XMEAR1, MC_XEAR1_64)
        LDCONST(r16, BANK_SIZE_64)

bank_size_done:

        /* Physical banks 4-7 are not used */
	PCICW32(MPC_MSAR2,0xc0804000)
	PCICW32(MPC_XMSAR2,0x01010101)
	PCICW32(MPC_MEAR2,0xffbf7f3f)
	PCICW32(MPC_XMEAR2,0x01010101)

        /* Set the number of physical banks */
        li      r4, 0x01                /* Enable the first bank by default */
        andi.   r3, r6, SPD_MEM_CONF_BANK
        beq     set_mem_num_bank
	ori	r4, r4, 0x02		/* Enable the second banks, too */
        add     r16, r16, r16           /* total memory size is 2 * bank size */
set_mem_num_bank:
        LDCONST(r3, MPC_MBEN)
        bl      pci_conf_write_8

	LDADDR(r6,board_draminittab)
	bl	init_engine

        mr      r3, r16
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


LEAF(board_setleds)
#if 1
	LDCONST(r5,A_BMW_LED)


	rlwinm	r3,r3,8,0,31
	stb	r3,3(r5)
	sync

	rlwinm	r3,r3,8,0,31
	stb	r3,2(r5)
	sync

	rlwinm	r3,r3,8,0,31
	stb	r3,1(r5)
	sync

	rlwinm	r3,r3,8,0,31
	stb	r3,0(r5)
	sync

#endif

	blr

END(board_setleds)



/*  *********************************************************************
    *  Initialization table for memory controller.  
    *
    *  This table is processsed by the "init_engine" to 
    *  orchestrate writes to PCI space and CPU MSRs.
    ********************************************************************* */

#define DRAM_BASE	0

board_draminittab:

	IET_PCI8(MPC_ODCR,0x7f)

	IET_DELAY(0x10000)
	IET_PCI32X(MPC_MCCR1,0xFFFFFFFF,M_MCCR1_MEMGO)
	IET_DELAY(0x30000)

	/*
	 * Done with memory controller.
	 */

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

        /* BAT 2 : PCI, EUMBAR */
	IET_IBAT(2,(0x80000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0x80000000 | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_INHIBIT))
	IET_DBAT(2,(0x80000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0x80000000 | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_INHIBIT))

        /* BAT 3 : FLASH, PLD */
	IET_IBAT(3,(0xF0000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0xF0000000 | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_INHIBIT))
	IET_DBAT(3,(0xF0000000 | V_BATU_BL(K_BATU_BL_256M) | M_BATU_VS | M_BATU_VP),
		   (0xF0000000 | V_BATL_PP(K_BATL_PP_RDWR) | M_BATL_INHIBIT))

		 
	IET_ENABBATS()

	IET_ENABDCACHE()

	IET_EOT()


/*  *********************************************************************
    *  Random little routines to read/write MSRs
    ********************************************************************* */

	.globl read_hid1
read_hid1:

	mfspr r3,SPR_HID1
	blr

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

        .globl pci_write_32
pci_conf_write_32:

        LDCONST(r5,A_MPC_CONFIG_ADDR)
	stwbrx	r3,0,r5
	sync
	LDCONST(r5,A_MPC_CONFIG_DATA)
	stwbrx	r4,0,r5
	sync
        blr

        .globl pci_write_8
pci_conf_write_8:

        LDCONST(r5,A_MPC_CONFIG_ADDR)
	stwbrx	r3,0,r5
	sync
	LDCONST(r5,A_MPC_CONFIG_DATA)
	stb	r4,0(r5)
	sync
        blr
              
	.globl eumbbar_read
eumbbar_read:

        lis     r4, A_BMW_EUMBBAR_VAL>>16
        sync
        lwbrx   r3, r3, r4
        blr
        

	.globl eumbbar_write
eumbbar_write:

        lis     r5, A_BMW_EUMBBAR_VAL>>16
        stwbrx  r4, r3, r5
        sync
        blr
