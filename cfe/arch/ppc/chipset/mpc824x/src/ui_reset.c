/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  RESET command				File: ui_reset.c
    *  
    *  Commands to reset the CPU
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
#include "lib_physio.h"

#include "ui_command.h"

/*  *********************************************************************
    *  Configuration
    ********************************************************************* */

/*  *********************************************************************
    *  prototypes
    ********************************************************************* */

int ui_init_resetcmds(void);
static int ui_cmd_reset(ui_cmdline_t *cmd,int argc,char *argv[]);


/*  *********************************************************************
    *  Data
    ********************************************************************* */


/*  *********************************************************************
    *  ui_init_resetcmds()
    *  
    *  Add RESET-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */


int ui_init_resetcmds(void)
{
    cmd_addcmd("reset",
	       ui_cmd_reset,
	       NULL,
	       "Reset the system.",
	       "reset [-yes] -cpu|-sysreset",
	       "-yes;Don't ask for confirmation|"
	       "-cpu;Reset the CPU|"
	       "-sysreset;Full system reset");



    return 0;
}





/*  *********************************************************************
    *  ui_cmd_reset(cmd,argc,argv)
    *  
    *  RESET command.
    *  
    *  Input parameters: 
    *  	   cmd - command structure
    *  	   argc,argv - parameters
    *  	   
    *  Return value:
    *  	   -1 if error occured.  Does not return otherwise
    ********************************************************************* */

static int ui_cmd_reset(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint8_t data = 0;
    int confirm = 1;
    char str[50];

    if (cmd_sw_isset(cmd,"-yes")) confirm = 0;

    if (cmd_sw_isset(cmd,"-sysreset")) data |= 0xFF;

    if (cmd_sw_isset(cmd,"-cpu")) data |= 0xFF;

    if (data == 0) {		/* no changes to reset pins were specified */
	return ui_showusage(cmd);
	}

    if (confirm) {
	console_readline("Are you sure you want to reset? ",str,sizeof(str));
	if ((str[0] != 'Y') && (str[0] != 'y')) return -1;
	}

#ifdef A_BMW_PLD
    phys_write8(A_BMW_PLD,data);
    phys_write8(A_BMW_PLD,0);
    for (;;);
#else
    printf("PLD on this board does not support self-reset\n");
#endif

    /* should not return */

    return -1;
}


