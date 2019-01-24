/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Test commands				File: ui_memtest.c
    *  
    *  Modified by :  Khin Zaw
    *
    *  a complete memory test
    *
    *  A simple memory test
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


#include "cfe.h"
#include "sbmips.h"

#include "ui_command.h"

#include "cfe_mem.h"
#include "lib_hssubr.h"
#include "exception.h"

#ifdef _BIGSUR_
#include "bcm1480_regs.h"
#include "bcm1480_scd.h"
#endif

static int ui_cmd_memorytest(ui_cmdline_t *cmd,int argc,char *argv[]);
#ifndef BCM47XX
static int ui_cmd_randmemtest(ui_cmdline_t *cmd,int argc,char *argv[]);
#endif

#ifndef _SB_MAKE64
#define _SB_MAKE64(x) ((uint64_t)(x))
#endif
#ifndef _SB_MAKEMASK
#define _SB_MAKEMASK(v,n) (_SB_MAKE64((_SB_MAKE64(1)<<(v))-1) << _SB_MAKE64(n))
#endif
#ifndef _SB_MAKEMASK1
#define _SB_MAKEMASK1(n) (_SB_MAKE64(1) << _SB_MAKE64(n))
#endif


int ui_init_memtestcmds(void);

int ui_init_memtestcmds(void)
{
    cmd_addcmd("memorytest",
	       ui_cmd_memorytest,
	       NULL,
	       "Tests all available memory",
	       "",
	       "-loop;Loop forever or until keypress|"
	       "-stoponerror;Stop if error occurs while looping|"
	       "-cca=*;Use specified cacheability attribute|"
	       "-arena=*;Test only specified arena index");

#ifndef BCM47XX
    cmd_addcmd("randmemtest",
	       ui_cmd_randmemtest,
	       NULL,
	       "Tests memory using random access pattern",
	       "randmemtest [num_mb]",
	       "-loop;Loop forever or until keypress|"
	       "-stoponerror;Stop if error occurs while looping|"
	       "-cca=*;Use specified cacheability attribute|"
	       "-ignoreerror;Ignore errors, don't print anything|"
	       "-arena=*;Test only specified arena index");
#endif /* BCM47XX */
    return 0;
}


/* extensive memory tests */

static void inline uacwrite(volatile uint64_t *srcadr,hsaddr_t dstadr) 
{
__asm __volatile ("ld $8, 0(%0) ; "
		  "ld $9, 8(%0) ; "  
		  "ld $10, 16(%0) ; " 
		  "ld $11, 24(%0) ; " 
		  "sync ; " 
		  ".align 4 ; "
		  "sd $8,  0(%1) ; " 
		  "sd $9,  8(%1) ; " 
		  "sd $10, 16(%1) ; " 
		  "sd $11, 24(%1) ; " 
		  "sync" :: "r"(srcadr),"r"(dstadr) : "$8","$9","$10","$11");
}

static void inline readline(hsaddr_t srcadr,volatile uint64_t *dstadr) 
{
__asm __volatile ("ld $8, 0(%0) ; "
		  "ld $9, 8(%0) ; "  
		  "ld $10, 16(%0) ; " 
		  "ld $11, 24(%0) ; " 
		  "sd $8,  0(%1) ; " 
		  "sd $9,  8(%1) ; " 
		  "sd $10, 16(%1) ; " 
		  "sd $11, 24(%1) ; " 
		   :: "r"(srcadr),"r"(dstadr) : "$8","$9","$10","$11");
}


#define TEST_DATA_LEN 4
#define CACHE_LINE_LEN 32

static int test_arena(hsaddr_t arena_start,hsaddr_t arena_size,int cca,int stoponerror);


static int ui_cmd_memorytest(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int arena;
    int error;
    int arena_type;
    uint64_t arena_start, arena_size;
    int forever;
    int passcnt;
    int stoponerr = 0;
    hsaddr_t phys_addr;
    hsaddr_t mem_base;
    int cca = K_CALG_UNCACHED_ACCEL;
    int arenanum = -1;
    char *x;
    int res;

    forever = cmd_sw_isset(cmd,"-loop");
    stoponerr = cmd_sw_isset(cmd,"-stoponerror");
    if (cmd_sw_value(cmd,"-cca",&x)) cca = atoi(x);
    if (cmd_sw_value(cmd,"-arena",&x)) arenanum = atoi(x);

    printf("Available memory arenas:\n");
    arena = 0;
    while (cfe_arena_enum(arena, &arena_type, &arena_start, &arena_size, FALSE) == 0) {
	phys_addr = arena_start;    /* actual physical address */
#if CPUCFG_REGS64
	mem_base = PHYS_TO_XKPHYS(cca, phys_addr); /* virtual address */
#else
	mem_base = PHYS_TO_K1(phys_addr); /* virtual address */
#endif
	xprintf("phys = %016llX, virt = %016llX, size = %016llX\n", phys_addr, mem_base, arena_size);
	arena++;
	}

    passcnt = 0;
    res = 0;

    printf("\nTesting memory.\n");
    do {

	passcnt++;
	if (forever) {
	    if (console_status()) break;
	    printf("***** Iteration %d *****\n",passcnt);
	    }

	arena = 0; 
	error = 0;

	while (cfe_arena_enum(arena, &arena_type, &arena_start, &arena_size, FALSE) == 0) {

	    /*
	     * See if we were asked to do just one part of memory.
	     */	       

	    if ((arenanum >= 0) && (arena != arenanum)) {
		arena++;
		continue;
		}

	    /*
	     * If we enumerated the locore area, avoid it.
	     */

	    if (arena_start == 0) {
		arena_start += CFE_LOCORE_GLOBAL_END;
		arena_size -= CFE_LOCORE_GLOBAL_END;
		}

	    res = test_arena(arena_start,arena_size,cca,stoponerr);

	    if (stoponerr && (res < 0)) {
		forever = 0;
		break;
		}
	    arena++;
	    }

	}  while (forever);

    return res;
}

static int test_arena(hsaddr_t arena_start,hsaddr_t arena_size,int cca,int stoponerr)
{

    static volatile uint64_t test_data[TEST_DATA_LEN];
    static volatile uint64_t check_data[TEST_DATA_LEN];
    int exitLoop;
    int error;
    hsaddr_t phys_addr;
    hsaddr_t offset;
    hsaddr_t mem_base;
    long i;
    hsaddr_t dst_adr, cache_dst_adr;
    long cda,tda;

    phys_addr = arena_start;    /* actual physical address */
#if CPUCFG_REGS64
    mem_base = PHYS_TO_XKPHYS(cca, phys_addr); /* virtual address */
#else
    mem_base = PHYS_TO_K1(phys_addr); /* virtual address */
#endif

    xprintf("\n");
    xprintf("Testing: phys = %016llX, virt = %016llX, size = %016llX\n", 
	    phys_addr, mem_base, arena_size);

int testno, patno ;

#define NUMTESTNO 12

typedef struct testinfo_s {
	char *testname;
	int num_patterns;
	uint64_t test_pattern[64];
} testinfo_t;

const testinfo_t test_patterns[] = {
		{"All Zeros",4,          {0,0,0,0}},
		{"All Ones",4,           {0xFFFFFFFFFFFFFFFFULL,0xFFFFFFFFFFFFFFFFULL,0xFFFFFFFFFFFFFFFFULL,0xFFFFFFFFFFFFFFFFULL}} ,
		{"Alternate Zeros/Ones",4,           {0ULL,0xFFFFFFFFFFFFFFFFULL,0ULL,0xFFFFFFFFFFFFFFFFULL}} ,
		{"Checker Board",4,      {0xAAAAAAAAAAAAAAAAULL,0x5555555555555555ULL,0xAAAAAAAAAAAAAAAAULL,0x5555555555555555ULL}} ,
		{"Walking 1 byte(8)",8,  {0x0101010101010101ULL,0x0202020202020202ULL,0x0404040404040404ULL,0x0808080808080808ULL,
					  0x1010101010101010ULL,0x2020202020202020ULL,0x4040404040404040ULL,0x8080808080808080ULL}} ,
		{"Walking 1 word(16)",16,{0x0001000100010001ULL,0x0002000200020002ULL,0x0004000400040004ULL,0x0008000800080008ULL,
				          0x0010001000100010ULL,0x0020002000200020ULL,0x0040004000400040ULL,0x0080008000800080ULL,
					  0x0100010001000100ULL,0x0200020002000200ULL,0x0400040004000400ULL,0x0800080008000800ULL,
					  0x1000100010001000ULL,0x2000200020002000ULL,0x4000400040004000ULL,0x8000800080008000ULL}} ,
		{"Walking 0 byte(8)",8,  {0xFEFEFEFEFEFEFEFEULL,0xFDFDFDFDFDFDFDFDULL,0xFBFBFBFBFBFBFBFBULL,0xF7F7F7F7F7F7F7F7ULL,
				          0xEFEFEFEFEFEFEFEFULL,0xDFDFDFDFDFDFDFDFULL,0xBFBFBFBFBFBFBFBFULL,0x7F7F7F7F7F7F7F7FULL}} ,
		{"Walking 0 word(16)",16,{0xFFFEFFFEFFFEFFFEULL,0xFFFDFFFDFFFDFFFDULL,0xFFFBFFFBFFFBFFFBULL,0xFFF7FFF7FFF7FFF7ULL,
					  0xFFEFFFEFFFEFFFEFULL,0xFFDFFFDFFFDFFFDFULL,0xFFBFFFBFFFBFFFBFULL,0xFF7FFF7FFF7FFF7FULL,
					  0xFEFFFEFFFEFFFEFFULL,0xFDFFFDFFFDFFFDFFULL,0xFBFFFBFFFBFFFBFFULL,0xF7FFF7FFF7FFF7FFULL,
					  0xEFFFEFFFEFFFEFFFULL,0xDFFFDFFFDFFFDFFFULL,0xBFFFBFFFBFFFBFFFULL,0x7FFF7FFF7FFF7FFFULL}},
		{"Walking ECC8 0 ",8,    {0x0000004000808082ULL,0x0000004000808084ULL,0x0000004000808090ULL,0x0000004000800001ULL,   // fe fd fb f7
					  0x0000004000008001ULL,0x0000000000808001ULL,0x0000000000000080ULL,0x0000004000808081ULL}} ,// ef df bf 7f
		{"Walking ECC8 1 ",8,    {0x0000004000808014ULL,0x0000004000808012ULL,0x0000004000808006ULL,0x0000004000800097ULL,   // 01 02 04 08
				          0x0000004000008097ULL,0x0000000000808097ULL,0x0000000000000016ULL,0x0000004000808017ULL}} ,// 10 20 40 80
	        {"Crosstalk 1",64,       {0x0001000100010001ULL,0xFFFFFFFFFFFFFFFFULL,0x0001000100010001ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x0002000200020002ULL,0xFFFFFFFFFFFFFFFFULL,0x0002000200020002ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x0004000400040004ULL,0xFFFFFFFFFFFFFFFFULL,0x0004000400040004ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x0008000800080008ULL,0xFFFFFFFFFFFFFFFFULL,0x0008000800080008ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x0010001000100010ULL,0xFFFFFFFFFFFFFFFFULL,0x0010001000100010ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x0020002000200020ULL,0xFFFFFFFFFFFFFFFFULL,0x0020002000200020ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x0040004000400040ULL,0xFFFFFFFFFFFFFFFFULL,0x0040004000400040ULL,0xFFFFFFFFFFFFFFFFULL,
					  0x0080008000800080ULL,0xFFFFFFFFFFFFFFFFULL,0x0080008000800080ULL,0xFFFFFFFFFFFFFFFFULL,
					  0x0100010001000100ULL,0xFFFFFFFFFFFFFFFFULL,0x0100010001000100ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x0200020002000200ULL,0xFFFFFFFFFFFFFFFFULL,0x0200020002000200ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x0400040004000400ULL,0xFFFFFFFFFFFFFFFFULL,0x0400040004000400ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x0800080008000800ULL,0xFFFFFFFFFFFFFFFFULL,0x0800080008000800ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x1000100010001000ULL,0xFFFFFFFFFFFFFFFFULL,0x1000100010001000ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x2000200020002000ULL,0xFFFFFFFFFFFFFFFFULL,0x2000200020002000ULL,0xFFFFFFFFFFFFFFFFULL,
				          0x4000400040004000ULL,0xFFFFFFFFFFFFFFFFULL,0x4000400040004000ULL,0xFFFFFFFFFFFFFFFFULL,
					  0x8000800080008000ULL,0xFFFFFFFFFFFFFFFFULL,0x8000800080008000ULL,0xFFFFFFFFFFFFFFFFULL }} ,
		{"Crosstalk 0",64,       {0xFFFEFFFEFFFEFFFEULL,0,0xFFFEFFFEFFFEFFFEULL,0,
					  0xFFFDFFFDFFFDFFFDULL,0,0xFFFDFFFDFFFDFFFDULL,0,
					  0xFFFBFFFBFFFBFFFBULL,0,0xFFFBFFFBFFFBFFFBULL,0,
					  0xFFF7FFF7FFF7FFF7ULL,0,0xFFF7FFF7FFF7FFF7ULL,0,
					  0xFFEFFFEFFFEFFFEFULL,0,0xFFEFFFEFFFEFFFEFULL,0,
					  0xFFDFFFDFFFDFFFDFULL,0,0xFFDFFFDFFFDFFFDFULL,0,
					  0xFFBFFFBFFFBFFFBFULL,0,0xFFBFFFBFFFBFFFBFULL,0,
					  0xFF7FFF7FFF7FFF7FULL,0,0xFF7FFF7FFF7FFF7FULL,0,
					  0xFEFFFEFFFEFFFEFFULL,0,0xFEFFFEFFFEFFFEFFULL,0,
					  0xFDFFFDFFFDFFFDFFULL,0,0xFDFFFDFFFDFFFDFFULL,0,
					  0xFBFFFBFFFBFFFBFFULL,0,0xFBFFFBFFFBFFFBFFULL,0,
					  0xF7FFF7FFF7FFF7FFULL,0,0xF7FFF7FFF7FFF7FFULL,0,
					  0xEFFFEFFFEFFFEFFFULL,0,0xEFFFEFFFEFFFEFFFULL,0,
					  0xDFFFDFFFDFFFDFFFULL,0,0xDFFFDFFFDFFFDFFFULL,0,
					  0xBFFFBFFFBFFFBFFFULL,0,0xBFFFBFFFBFFFBFFFULL,0,
					  0x7FFF7FFF7FFF7FFFULL,0,0x7FFF7FFF7FFF7FFFULL,0}} 
} ;
// address test
    xprintf("Address test :Writing: [address|5555][~][aaaa|address][~] ");
    exitLoop = 0;

    for (offset = 0; (offset < arena_size); offset += CACHE_LINE_LEN) {
	dst_adr = (mem_base+offset);
	test_data[0] = ((uint64_t)dst_adr<<32)|0x55555555;
	test_data[1] = ~test_data[0];
	test_data[2] = 0xaaaaaaaa00000000ULL|(dst_adr & 0xffffffff);
	test_data[3] = ~test_data[2];
	uacwrite(test_data, dst_adr);
        if ((dst_adr>>28)&((dst_adr&0xfffffff)==0)) xprintf(".");
	}
    
    xprintf("Reading: ");

    error = 0;
    for (offset = 0; (offset < arena_size); offset += CACHE_LINE_LEN) {
	dst_adr = (mem_base+offset);
	test_data[0] = ((uint64_t)dst_adr<<32)|0x55555555;
	test_data[1] = ~test_data[0];
	test_data[2] = 0xaaaaaaaa00000000ULL|(dst_adr & 0xffffffff);
	test_data[3] = ~test_data[2];
	cache_dst_adr = (mem_base+offset);
	readline(cache_dst_adr,check_data);
        if ((dst_adr>>28)&((dst_adr&0xfffffff)==0)) xprintf(".");
	for (i = 0; i < TEST_DATA_LEN; i++) {
	    cda = check_data[i];
	    tda = test_data[i];
	    if (cda != tda) {
		xprintf("mem[%016llX] %016llX != %016llX\n",
			mem_base+offset+(i*8),cda,tda);
		error++;
		}	
	    }
	if (error) break;
    }
    xprintf("Done!\n");

    if (error) return -1;
    // data test
    for (testno = 0 ; testno < NUMTESTNO ; testno++ ) {

    xprintf("Data Test %d : writing %s. ",testno,test_patterns[testno].testname);

    patno = 0 ;
    for (offset = 0; (offset < arena_size); offset += CACHE_LINE_LEN) {
	dst_adr = (mem_base+offset);
            test_data[0] = test_patterns[testno].test_pattern[patno];
            test_data[1] = test_patterns[testno].test_pattern[patno+1];
            test_data[2] = test_patterns[testno].test_pattern[patno+2];
            test_data[3] = test_patterns[testno].test_pattern[patno+3];
	    uacwrite(test_data, dst_adr);
	    //if (testno==6) xprintf("W[%016llx]=%016llx,%016llx %016llx,%016llx\n",dst_adr,test_data[0],test_data[1],test_data[2],test_data[3]);
  	    if (patno+4 < test_patterns[testno].num_patterns) patno+=4;
	    else patno = 0 ;
	    }
    

    xprintf("Reading and checking :");

    error = 0;
    patno=0 ;
    for (offset = 0; (offset < arena_size); offset += CACHE_LINE_LEN) {
	dst_adr = (mem_base+offset);
	cache_dst_adr = (mem_base+offset);
            test_data[0] = test_patterns[testno].test_pattern[patno];
            test_data[1] = test_patterns[testno].test_pattern[patno+1];
            test_data[2] = test_patterns[testno].test_pattern[patno+2];
            test_data[3] = test_patterns[testno].test_pattern[patno+3];
	    //if (testno==6) xprintf("R[%016llx]=%016llx,%016llx %016llx,%016llx\n",dst_adr,test_data[0],test_data[1],test_data[2],test_data[3]);
  	if(patno+4 < test_patterns[testno].num_patterns) patno+=4;
	else patno = 0 ;

	readline(cache_dst_adr,check_data);
	for (i = 0; i < TEST_DATA_LEN; i++) {
	    cda = check_data[i];
	    tda = test_data[i];
	    if (cda != tda) {
		xprintf("mem[%016llX] %016llX != %016llX\n",
			mem_base+offset+(i*8), cda, tda);
		error++;
		if (error > 25) return -1;
	    }
	if (error) break;
	}
    }
    xprintf(" Done!\n");
// end of patterns
    }
//
    return error ? -1 : 0;
}

#define LFSR_M	22
#define LFSR_TAP 21
//#define LFSR_PERIOD 0x3fff80
#define LFSR_PERIOD 0x3ffff

static inline unsigned long nextval(unsigned long regval)
{
    unsigned long newbit;

    newbit = (regval & 1) ^ ((regval & (1<<(LFSR_M - LFSR_TAP))) != 0);

    regval = (regval >> 1) | (newbit << (LFSR_M-1));

    return regval;
}


#ifndef BCM47XX
#define START_PHYS 0x1000
static int randmemtest(int num_mb,int high_mb,int cca,int stoponerr,int ignoreerr)
{
#if CPUCFG_REGS64
    hsaddr_t membase = PHYS_TO_XKPHYS(cca,START_PHYS);
#else
    hsaddr_t membase = PHYS_TO_K1(START_PHYS); /* UNCACHED virtual address */
#endif
    unsigned long lfsr;
    uint64_t idx;
    uint64_t maxidx;
    uint64_t highlfsr;
    int error = 0;
    register uint64_t pattern0,pattern1,pattern2,pattern3,x;

    highlfsr = (high_mb << 15);
    if (highlfsr == 0) highlfsr = 0x3fff80;

    maxidx = (uint64_t) ((num_mb << 15)-1);

    printf("Highest physaddr is %016llX\n",highlfsr<<5);
    printf("Writing (memory base %016llX)\n",membase);
    lfsr = (1<<LFSR_M)-1;
    for (idx = 0; idx < maxidx; idx++) {
	pattern0 = 0x5555555555555555ULL ^ (lfsr | (lfsr << LFSR_M)); pattern1 = ~pattern0;
	pattern3 = 0xaaaaaaaaaaaaaaaaULL ^ (idx | (idx << LFSR_M)) ; pattern2 = ~pattern3;

	if (lfsr < highlfsr) {
	    __asm __volatile ( " .align 4 ; "
			       " sd %0,0(%4) ; "
			       " sd %1,8(%4) ; "
			       " sd %2,16(%4) ; "
			       " sd %3,24(%4) ; "
			       : : "r"(pattern0),"r"(pattern1),"r"(pattern2),"r"(pattern3),
			       "r"(membase+(hsaddr_t)(lfsr << 5)));
	    }
	lfsr = nextval(lfsr);
	}

    printf("Reading.\n");
    lfsr = (1<<LFSR_M)-1;
    for (idx = 0; idx < maxidx; idx++) {

	if (lfsr < highlfsr) { /* stay within 128MB */
	    hsaddr_t addr = (membase + (hsaddr_t)(lfsr<<5));
	    __asm __volatile ( " ld %0,0(%4) ; "
			       " ld %1,8(%4) ; "
			       " ld %2,16(%4) ; "
			       " ld %3,24(%4)"
			       : "=r"(pattern0),"=r"(pattern1),"=r"(pattern2),"=r"(pattern3) : "r"(addr));

	    if (!ignoreerr) {
	    x = 0x5555555555555555ULL ^ (lfsr | (lfsr << LFSR_M));
	    if (pattern0 != x) {
		printf("mem[%016llX] %016llX should be %016llX (%016llX)\n",addr,pattern0,x,pattern0^x);
		error++;
		if (console_status()) break;
		}

	    x = ~x;
	    if (pattern1 != x) {
		printf("mem[%016llX] %016llX should be %016llX (%016llX)\n",addr+8,pattern1,x,pattern1^x);
		error++;
		if (console_status()) break;
		}

	    x = 0xaaaaaaaaaaaaaaaaULL ^ (idx | (idx << LFSR_M));
	    if (pattern3 != x) {
		printf("mem[%016llX] %016llX should be %016llX (%016llX)\n",addr+24,pattern3,x,pattern3^x);
		error++;
		if (console_status()) break;
		}

	    x = ~x;
	    if (pattern2 != x) {
		printf("mem[%016llX] %016llX should be %016llX (%016llX)\n",addr+16,pattern2,x,pattern2^x);
		error++;
		if (console_status()) break;
		}
		}

	    if (error && stoponerr) break;
	    }
	lfsr = nextval(lfsr);
	}
    return error;
}

static int ui_cmd_randmemtest(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int stoponerr = 1;
    int ignoreerr = 0;
    char *x;
    int cca = K_CALG_UNCACHED_ACCEL;
    int forever;
    int error = 0;
    int res;
    int num_mb = 128;
    int high_mb = 128;

    ignoreerr = cmd_sw_isset(cmd,"-ignoreerror");
    stoponerr = cmd_sw_isset(cmd,"-stoponerror");
    if (cmd_sw_value(cmd,"-cca",&x)) cca = atoi(x);
    forever = cmd_sw_isset(cmd,"-loop");

    if ((x = cmd_getarg(cmd,0))) num_mb = atoi(x);
    if (num_mb > 128) num_mb = 128;

    if ((x = cmd_getarg(cmd,1))) high_mb = atoi(x);
    if (high_mb > 128) high_mb = 128;

    do {
#ifdef _BIGSUR_
	SBWRITECSR(A_BUS_L2_ERRORS,0);
	SBWRITECSR(A_BUS_MEM_IO_ERRORS,0);
#endif
	res = randmemtest(num_mb,high_mb,cca,stoponerr,ignoreerr);
	if (res != 0) error = -1;
	if ((res != 0) && stoponerr) break;
#ifdef _BIGSUR_
	do {
	    uint64_t l2reg,memreg;
	    l2reg = SBREADCSR(A_BUS_L2_ERRORS);
	    memreg = SBREADCSR(A_BUS_MEM_IO_ERRORS);
	    printf("L2: CorrECC=%d BadECC=%d   Mem: CorrECC=%d BadECC=%d\n",
		   (int)G_SCD_L2ECC_CORR_D(l2reg),(int)G_SCD_L2ECC_BAD_D(l2reg),
		   (int)G_SCD_MEM_ECC_CORR(memreg),(int)G_SCD_MEM_ECC_BAD(memreg));
	    } while (0);
#endif
	} while (forever && !console_status());

    return error;
}
#endif /* !BCM47XX */

