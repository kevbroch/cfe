/*  *********************************************************************
    *  SB1250 Board Support Package
    *  
    *  Board-specific initialization		File: VCS_INIT.S
    *
    *  This module contains the assembly-language part of the init
    *  code for this board support package.  The routine
    *  "board_earlyinit" lives here.
    *
    *  This board support package is for operation in the Verilog
    *  simulator.
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
#include "bsp_config.h"
#include "cpu_config.h"
#include "mipsmacros.h"
#include "bcm1480_regs.h"
#include "bcm1480_scd.h"
#include "bcm1480_draminit.h"
#include "jedec.h"
#include "vcs1280.h"

		.text

/*  *********************************************************************
    *  Macros
    ********************************************************************* */



/*  *********************************************************************
    *  BOARD_EARLYINIT()
    *  
    *  Initialize board registers.  This is the earliest 
    *  time the BSP gets control.  This routine cannot assume that
    *  memory is operational, and therefore all code in this routine
    *  must run from registers only.  The $ra register must not
    *  be modified, as it contains the return address.
    *
    *  This routine will be called from uncached space, before
    *  the caches are initialized.  If you want to make
    *  subroutine calls from here, you must use the CALLKSEG1 macro.
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


		j	ra

END(board_earlyinit)


/*  *********************************************************************
    *  BOARD_DRAMINFO
    *  
    *  Return the address of the DRAM information table.  Note that
    *  the address must be obtained in a position-independent way,
    *  so avoid the use of the "la" instruction.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   v0 - DRAM info table, return 0 to use default table
    ********************************************************************* */

LEAF(board_draminfo)


	#
	# This board has soldered-down memory. (?)
	#

		la	v0,dramtab
		j	ra


	/*
	 * Regular DRAM for regular Bigsurs
	 */

dramtab:

	/*
	 * 64-bit channels. Channel 0: 256 MB
	 *
	 * Model's memory emulates Micron MT46V64M16TG-6
	 *
	 * 13 Rows
	 * 10 columns
	 * 2 banks
	 *
	 * tCK=6ns  rfsh=64Khz CASLat=2.5 tRAS=42 tRP=18.0 tRRD=12.0 tRCD=18.0 tRFC=n/a tRC=n/a
	 */

	DRAM_GLOBALS(CFG_DRAM_INTERLEAVE)	 
	
        DRAM_CHAN_CFG(MC_CHAN0, DRT10(6,0), DRT10(3,0), MC_64BIT_CHAN, JEDEC, CASCHECK, CFG_DRAM_CSINTERLEAVE, CFG_DRAM_ECC, 0)
	DRAM_CS_GEOM(MC_CS0, 13, 10, 2, 0)
	DRAM_CS_TIMING(DRT10(6,0), JEDEC_RFSH_64khz, JEDEC_CASLAT_25, 0,  42, DRT4(18,0), DRT4(12,0),  DRT4(18,0),  0, 0)
	DRAM_CS_GEOM(MC_CS2, 13, 10, 2, 0)
	DRAM_CS_TIMING(DRT10(6,0), JEDEC_RFSH_64khz, JEDEC_CASLAT_25, 0,  42, DRT4(18,0), DRT4(12,0),  DRT4(18,0),  0, 0)

        DRAM_CHAN_CFG(MC_CHAN1, DRT10(6,0), DRT10(3,0), MC_64BIT_CHAN, JEDEC, CASCHECK, CFG_DRAM_CSINTERLEAVE, CFG_DRAM_ECC, 0)
	DRAM_CS_GEOM(MC_CS0, 13, 10, 2, 0)
	DRAM_CS_TIMING(DRT10(6,0), JEDEC_RFSH_64khz, JEDEC_CASLAT_25, 0,  42, DRT4(18,0), DRT4(12,0),  DRT4(18,0),  0, 0)
	DRAM_CS_GEOM(MC_CS2, 13, 10, 2, 0)
	DRAM_CS_TIMING(DRT10(6,0), JEDEC_RFSH_64khz, JEDEC_CASLAT_25, 0,  42, DRT4(18,0), DRT4(12,0),  DRT4(18,0),  0, 0)

/*
        DRAM_CHAN_CFG(MC_CHAN1, DRT10(8,0), MC_64BIT_CHAN, JEDEC, CASCHECK, CFG_DRAM_CSINTERLEAVE, CFG_DRAM_ECC, 0)
	DRAM_CS_SPD(MC_CS0,0,CFG_DRAM_SMBUS_CHANNEL,CFG_DRAM_SMBUS_BASE)
	DRAM_CS_SPD(MC_CS2,0,CFG_DRAM_SMBUS_CHANNEL,CFG_DRAM_SMBUS_BASE+1)
*/
	DRAM_EOT


END(board_draminfo)



/*  *********************************************************************
    *  BOARD_SETLEDS(x)
    *  
    *  Set LEDs for boot-time progress indication.  Not used if
    *  the board does not have progress LEDs.  This routine
    *  must not call any other routines, since it may be invoked
    *  either from KSEG0 or KSEG1 and it may be invoked 
    *  whether or not the icache is operational.
    *  
    *  Input parameters: 
    *  	   a0 - LED value (8 bits per character, 4 characters)
    *  	   
    *  Return value:
    *  	   nothing
    *  
    *  Registers used:
    *  	   t0,t1,t2,t3
    ********************************************************************* */


LEAF(board_setleds)

		j	ra

END(board_setleds)

