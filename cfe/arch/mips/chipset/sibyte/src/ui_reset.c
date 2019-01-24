/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  RESET command				File: ui_reset.c
    *  
    *  Commands to reset the CPU
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

#include "ui_command.h"

#include "sbmips.h"
#include "sb1250_regs.h"
#include "sb1250_scd.h"
#include "bcm1480_scd.h"

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
	       "reset [-yes] -softreset|-cpu|-unicpu1|-unicpu0|-ext|-sysreset",
	       "-yes;Don't ask for confirmation|"
	       "-softreset;Soft reset of the entire chip|"
	       "-cpu;Reset the CPUs|"
#if (!SIBYTE_HDR_FEATURE_CHIP(1480))
	       "-unicpu1;Reset to uniprocessor using CPU1|"
	       "-unicpu0;Reset to uniprocessor using CPU0|"
#endif
	       "-ext;Reset to external devices only|"
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
#if SIBYTE_HDR_FEATURE_1250_112x
    uint64_t data;
    uint64_t olddata;
    int confirm = 1;
    int extreset = 0;
    char str[50];

    data = SBREADCSR(A_SCD_SYSTEM_CFG) & ~M_SYS_SB_SOFTRES;
    olddata = data;

    if (cmd_sw_isset(cmd,"-yes")) confirm = 0;

    if (cmd_sw_isset(cmd,"-softreset")) data |= M_SYS_SB_SOFTRES;

    if (cmd_sw_isset(cmd,"-unicpu0")) data |= M_SYS_UNICPU0;
    else if (cmd_sw_isset(cmd,"-unicpu1")) data |= M_SYS_UNICPU1;

    if (cmd_sw_isset(cmd,"-sysreset")) data |= M_SYS_SYSTEM_RESET;

    if (cmd_sw_isset(cmd,"-cpu")) data |= (M_SYS_CPU_RESET_0 | M_SYS_CPU_RESET_1);

    if (cmd_sw_isset(cmd,"-ext")) {
        data |= M_SYS_EXT_RESET;
        extreset = 1;
    }

    if (data == olddata) {		/* no changes to reset pins were specified */
	return ui_showusage(cmd);
	}

    if (confirm) {
	console_readline("Are you sure you want to reset? ",str,sizeof(str));
	if ((str[0] != 'Y') && (str[0] != 'y')) return -1;
	}

    SBWRITECSR(A_SCD_SYSTEM_CFG,data);
#elif  SIBYTE_HDR_FEATURE_CHIP(1480)
    uint64_t data;
    uint64_t olddata;
    int confirm = 1;
    char str[50];
    int extreset = 0;

    data = SBREADCSR(A_SCD_SYSTEM_CFG) & ~M_BCM1480_SYS_SB_SOFTRES;
    olddata = data;

    if (cmd_sw_isset(cmd,"-yes")) confirm = 0;

    if (cmd_sw_isset(cmd,"-softreset")) data |= M_BCM1480_SYS_SB_SOFTRES;

    if (cmd_sw_isset(cmd,"-sysreset")) data |= M_BCM1480_SYS_SYSTEM_RESET;

    if (cmd_sw_isset(cmd,"-cpu")) data |= (M_BCM1480_SYS_CPU_RESET_0 | M_BCM1480_SYS_CPU_RESET_1 | M_BCM1480_SYS_CPU_RESET_2 | M_BCM1480_SYS_CPU_RESET_3);

    if (data == olddata) {		/* no changes to reset pins were specified */
	return ui_showusage(cmd);
	}

    if (confirm) {
	console_readline("Are you sure you want to reset? ",str,sizeof(str));
	if ((str[0] != 'Y') && (str[0] != 'y')) return -1;
	}

    /*
     * Workaround for BCM1480 S0 erratum SOC-111:
     *
     * When writing to the system config register, do two dummy writes to
     * another SCD CSR first (RO CSRs are OK, we choose the system revision
     * register in this case since it's easy).
     *
     * This makes the assumption that when this code is run, other processors
     * will **NOT** be writing any SCD registers.  (If other processors do,
     * this workaround may not be successful.)
     *
     * This workaround doesn't need to be conditional on the chip rev
     * actually in use; it will cause on harm on revisions in which this
     * erratum is fixed.
     */
#if _BCM1480_PASS1_WORKAROUNDS_
    SBWRITECSR(A_SCD_SYSTEM_REVISION,data);
    SBWRITECSR(A_SCD_SYSTEM_REVISION,data);
#endif

    SBWRITECSR(A_SCD_SYSTEM_CFG,data);
#endif

    /* should not return unless ext reset only */

    if (extreset) {
        cfe_usleep(1000);
        SBWRITECSR(A_SCD_SYSTEM_CFG,olddata);
        return 0;
    }

    return -1;
}


