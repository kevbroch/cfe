/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Processor-specific ELF definitions     	File: elf_mips.h
    *
    *  Stuff specific to ELF parsing on MIPS platforms.
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
    *  Section types
    ********************************************************************* */


#define DT_NULL		0
#define DT_REL		17
#define DT_RELSZ	18
#define DT_SYMTAB	6
#define DT_PLTGOT	3
#define DT_MIPS_LOCAL_GOTNO 0x7000000a
#define DT_MIPS_SYMTABNO    0x70000011
#define DT_MIPS_GOTSYM      0x70000013

/*  *********************************************************************
    *  Symbol types and values
    ********************************************************************* */

#define R_ELF32SYM_ST_NAME  0
#define R_ELF32SYM_ST_VALUE 4
#define R_ELF32SYM_ST_SIZE  8
#define R_ELF32SYM_ST_INFO  12
#define R_ELF32SYM_ST_OTHER 13
#define R_ELF32SYM_ST_SHNDX 14

#define R_REL_INFO	    4
#define R_REL_OFFSET	    0
#define M_REL_TYPE	    0xFF
#define K_REL_TYPE_REL32    3
#define S_REL_SYM	    8
