/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  JEDEC DRAM constants			File: jedec.h
    *  
    *  Constants and macros mostly related to JEDEC serial-presence
    *  detect ROMs on DIMMs
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


#ifndef _JEDEC_H
#define _JEDEC_H

/*  *********************************************************************
    *  JEDEC constants (serial presence detect offsets and bit fields)
    ********************************************************************* */

#define JEDEC_SPD_MEMTYPE	2	/* memory type (sdram, ddr, etc.) */
#define JEDEC_SPD_ROWS		3	/* row bits */
#define JEDEC_SPD_COLS		4	/* column bits */
#define JEDEC_SPD_SIDES		5	/* number of sides */
#define JEDEC_SPD_WIDTH		6	/* and 7, bit width of device */	
#define JEDEC_SPD_BANKS		17	/* number of banks */
#define JEDEC_SPD_DENSITY	31	/* module bank density */
#define JEDEC_SPD_ECC_INFO	11

#define JEDEC_SPD_tCK25		9	/* tCK @ CAS 2.5 */
#define JEDEC_SPD_tCK20		23	/* tCK @ CAS 2.0 */
#define JEDEC_SPD_tCK10		25	/* tCK @ CAS 1.0 */
#define JEDEC_SPD_RFSH		12	/* refresh rate */
#define JEDEC_SPD_CASLATENCIES	18	/* CAS Latencies supported */
#define JEDEC_SPD_ATTRIBUTES	21	/* module attributes */
#define JEDEC_SPD_tRAS		30	/* tRAS */
#define JEDEC_SPD_tRP		27	/* tRP */
#define JEDEC_SPD_tRRD		28	/* tRRD */
#define JEDEC_SPD_tRCD		29	/* tRCD */
#define JEDEC_SPD_tRFC		42	/* tRFC */
#define JEDEC_SPD_tRC		41	/* tRC */
#define JEDEC_SPD_tRTP		38	/* tRTP */
#define JEDEC_SPD_DIMMTYPE	20	/* Dimm type DDR2 only */

#define JEDEC_SPD_SIZE		46	/* number of SPD bytes we will read
                                           (adequate for DDR SDRAM) */

#define JEDEC_MEMTYPE_DDRSDRAM	1
#define JEDEC_MEMTYPE_DDRSDRAM2	7
#define JEDEC_MEMTYPE_SDRAM	4
#define SPD_MEMTYPE_FCRAM	0x17	/* Use what SMART defines it as */
#define SPD_MEMTYPE_DDR2	0x08

#define	JEDEC_CASLAT_DDR2_6	0x40
#define	JEDEC_CASLAT_DDR2_5	0x20
#define JEDEC_CASLAT_DDR2_4	0x10
#define JEDEC_CASLAT_DDR2_3	0x08
#define JEDEC_CASLAT_DDR2_2	0x04

#define JEDEC_CASLAT_40		0x40
#define JEDEC_CASLAT_35		0x20
#define JEDEC_CASLAT_30		0x10
#define JEDEC_CASLAT_25		0x08
#define JEDEC_CASLAT_20		0x04
#define JEDEC_CASLAT_15		0x02
#define JEDEC_CASLAT_10		0x01

#define JEDEC_ATTRIB_REG	0x02

#define JEDEC_FLG_ECC_SUPPORT	0x02

#define JEDEC_RFSH_MASK		0x7F
#define JEDEC_RFSH_64khz	0
#define JEDEC_RFSH_256khz	1
#define JEDEC_RFSH_128khz	2
#define JEDEC_RFSH_32khz	3
#define JEDEC_RFSH_16khz	4
#define JEDEC_RFSH_8khz		5

#define JEDEC_DIMMTYPE_RDIMM	0x01
#define JEDEC_DIMMTYPE_UDIMM	0x02
#define JEDEC_DIMMTYPE_MRDIMM	0x10

#endif /* _JEDEC_H */
