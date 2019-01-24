/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Exception Handler			File: exchandler.c       
    *  
    *  This is the "C" part of the exception handler and the
    *  associated setup routines.  We call these routines from
    *  the assembly-language exception handler.
    *  
    *  Author:  Mitch Lichtenberg
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001
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
#include "exception.h"
#include "exchandler.h"


/*  *********************************************************************
    *  Globals 
    ********************************************************************* */

exc_handler_t exc_handler;


/*  *********************************************************************
    *  cfe_exception(code,info)
    *  
    *  Exception handler.  This routine is called when any CPU 
    *  exception that is handled by the assembly-language
    *  vectors is reached.  The usual thing to do here is just to
    *  reboot.
    *  
    *  Input parameters: 
    *  	   code - exception type
    *  	   info - exception stack frame
    *  	   
    *  Return value:
    *  	   usually reboots
    ********************************************************************* */

void cfe_exception(int code,uint32_t *info)
{
    SETLEDS("EXC!");
    
    if(exc_handler.catch_exc == 1)
      {
	/*Deal with exception without restarting CFE.*/
	
	/*Reset flag*/
	exc_handler.catch_exc = 0;

	exc_longjmp_handler();       
      }
    

    xprintf("**Exception %d: EIP=%08X\n",code,info[XGR_EIP]);
    xprintf("\n");
    xprintf("        EAX = %08X  EBX = %08X\n",info[XGR_EAX],info[XGR_EBX]);
    xprintf("        ECX = %08X  EDX = %08X\n",info[XGR_ECX],info[XGR_EDX]);
    xprintf("        ESI = %08X  EDI = %08X\n",info[XGR_ESI],info[XGR_EDI]);
    xprintf("        ESP = %08X  EBP = %08X\n",info[XGR_ESP],info[XGR_EBP]);

    xprintf("\n");
    _exc_restart();
}


/*  *********************************************************************
    *  cfe_setup_exceptions()
    *  
    *  Set up the exception handlers.  
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */
void cfe_setup_exceptions(void)
{
#if 0
    _exc_setvector(XTYPE_TLBFILL,  (void *) cfe_exception);
    _exc_setvector(XTYPE_XTLBFILL, (void *) _exc_crash_sim);
    _exc_setvector(XTYPE_CACHEERR, (void *) _exc_cache_crash_sim);
    _exc_setvector(XTYPE_EXCEPTION,(void *) cfe_exception);
    _exc_setvector(XTYPE_INTERRUPT,(void *) _exc_crash_sim);
    _exc_setvector(XTYPE_EJTAG,    (void *) _exc_crash_sim);
#endif

    exc_handler.catch_exc = 0;
    q_init( &(exc_handler.jmpbuf_stack));
}

 









