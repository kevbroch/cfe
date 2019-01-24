/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  "catch" and "throw" stuff		File: lib_try.c
    *
    *  Try/Except routines - a crude exception catcher that we can
    *  use to deal with bus errors and other stuff.
    *  
    *  Author:  Binh Vo, Mitch Lichtenberg
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

#include "lib_types.h"
#include "lib_queue.h"
#include "lib_malloc.h"
#include "lib_setjmp.h"
#include "lib_try.h"

exc_handler_t exc_handler;


/*  *********************************************************************
    *  exc_initialize_block()
    *
    *  Set up the exception handler.  Allow exceptions to be caught. 
    *  Allocate memory for jmpbuf and store it away.
    *
    *  Returns NULL if error in memory allocation.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   jmpbuf_t structure, or NULL if no memory
    ********************************************************************* */
jmpbuf_t *exc_initialize_block(void)
{
    jmpbuf_t *jmpbuf_local;

    exc_handler.catch_exc = 1;
  
    /* Create the jmpbuf_t object */
    jmpbuf_local = (jmpbuf_t *) KMALLOC((sizeof(jmpbuf_t)),0);

    if (jmpbuf_local == NULL) {
	return NULL;
	}

    q_enqueue( &(exc_handler.jmpbuf_stack), &((*jmpbuf_local).stack));

    return jmpbuf_local;
}

/*  *********************************************************************
    *  exc_cleanup_block(dq_jmpbuf)
    *  
    *  Remove dq_jmpbuf from the exception handler stack and free
    *  the memory.
    *  
    *  Input parameters: 
    *  	   dq_jmpbuf - block to deallocate
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void exc_cleanup_block(jmpbuf_t *dq_jmpbuf)
{
    int count;

    if (dq_jmpbuf == NULL) {
	return;
	}
  
    count = q_count( &(exc_handler.jmpbuf_stack));

    if( count > 0 ) {
	q_dequeue( &(*dq_jmpbuf).stack );
	KFREE(dq_jmpbuf);
	}
}

/*  *********************************************************************
    *  exc_cleanup_handler(dq_jmpbuf,chain_exc)
    *  
    *  Clean a block, then chain to the next exception if required.
    *  
    *  Input parameters: 
    *  	   dq_jmpbuf - current exception
    *  	   chain_exc - true if we should chain to the next handler
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void exc_cleanup_handler(jmpbuf_t *dq_jmpbuf, int chain_exc)
{
    exc_cleanup_block(dq_jmpbuf);

    if( chain_exc == EXC_CHAIN_EXC ) {
	/*Go to next exception on stack */
	exc_longjmp_handler();
	}
}



/*  *********************************************************************
    *  exc_longjmp_handler()
    *  
    *  This routine long jumps to the exception handler on the top
    *  of the exception stack.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
void exc_longjmp_handler(void)
{
    int count;   
    jmpbuf_t *jmpbuf_local;

    count = q_count( &(exc_handler.jmpbuf_stack));

    if( count > 0 ) {
	jmpbuf_local = (jmpbuf_t *) q_getlast(&(exc_handler.jmpbuf_stack));

	lib_longjmp( (*jmpbuf_local).jmpbuf, -1);
	} 
}
