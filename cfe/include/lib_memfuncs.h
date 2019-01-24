
#ifndef _LIB_HSSUBR_H
#include "lib_hssubr.h"
#endif

#define MEM_BYTE     1
#define MEM_HALFWORD 2
#define MEM_WORD     3
#define MEM_QUADWORD 4

int mem_peek(void*, hsaddr_t, int);
int mem_poke(hsaddr_t, uint64_t, int);
