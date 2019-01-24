/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    * HSP UI Commands			  File:    ui_hspcmds.c
    *  
    * This file contains BCM1480 HSP console command extensions 
    * 
    * Code was leveraged from 'bcm1480_pci_machdep.c' and 'ui_pmcmd.c' 
    *
    *  Author:  Kean Hurley
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001,2002,2003,2004,2005,2006
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


#ifndef DOXYGEN_IGNORE_EXTERNAL
//----------------------------------------------------------
/**

@file ui_hspcmds.c

    This file contains BCM1480 HSP console command extensions 

 @todo

 @author Kean Hurley

    Copyright 2006 Broadcom, Inc.

    This computer program is CONFIDENTIAL and a TRADE SECRET of BROADCOM.
    The receipt or possession of this program does not convey
    rights to reproduce or disclose its contents, or to manufacture, use,
    or sell anything that it may describe, in whole or in part, without the
    specific written consent of BROADCOM. Any reproduction of
    this program without the express written consent of BROADCOM is a
    violation of the copyright laws and may subject you to criminal
    prosecution.


 <b>History:</b>

 @verbatim

 Engineer       Date        PR     Description
 ---------------------------------------------
 Kean Hurley    06/05/06           Creation  

 @endverbatim
*/
//----------------------------------------------------------
#endif // DOXYGEN_IGNORE_EXTERNAL

/***********************************************************
*
* Included files
*
***********************************************************/
#include "cfe.h"
#include "sbmips.h"
#include "ui_command.h"

#include "lib_physio.h"

#include "pcivar.h"
#include "pci_internal.h"
#include "pcireg.h"
#include "ldtreg.h"

#include "bcm1480_regs.h"
#include "bcm1480_hr.h"
#include "bcm1480_ht.h"
#include "bcm1480_hsp.h"
#include "bcm1480_scd.h"

#include "ui_bitfields.h"

#include "bcm1480_hsp_utils.h"

/***********************************************************
*
* Global variables
*
***********************************************************/

/***********************************************************
*
* Defined values, enumerated types, macros, typedefs
* internal to this file
*
***********************************************************/

#define WRITECSR(x,y) phys_write64(x,y)
#define READCSR(x) phys_read64(x)


static bitfield_t spi4_txintstat[] = {
    {M_BCM1480_HSP_TX_INT_TSTATTIMEOUT,"TSTAT_Timeout"},
    {M_BCM1480_HSP_TX_INT_DIP2RXERR,"DIP2_Error"},
    {M_BCM1480_HSP_TX_INT_SPI4RESET,"Port_Reset"},
    {0,NULL}};

static bitfield_t spi4_rxintstat[] = {
    {M_BCM1480_HSP_RX_INT_PERVCERR,"per_vc_err"},
    {M_BCM1480_HSP_RX_INT_EOPABORT,"eop_abort"},
    {M_BCM1480_HSP_RX_INT_SPI4PROTOERR,"spi4_proto_err"},
    {M_BCM1480_HSP_RX_INT_ESTOREOVERFLOW,"estore_ovflo"},
    {M_BCM1480_HSP_RX_INT_ALPHATRAINERR,"alpha_train_err"},
    {M_BCM1480_HSP_RX_INT_DIP4ERROR,"dip4_err"},
    {M_BCM1480_HSP_RX_INT_HRERROR,"hr_err"},
    {M_BCM1480_HSP_RX_INT_INTOVERFLOW,"intrnl_ovflo"},
    {0,NULL}};

static bitfield_t spi4_txcalibration[] = {
    {M_BCM1480_HSP_CAL_STARTCAL2,"StartCal|NotStartCal"},
    {M_BCM1480_HSP_CAL_PDTEST,"PDTest"},
    {M_BCM1480_HSP_CAL_CALFIN,"Cal_Finish|Cal_Inprog"},
    {M_BCM1480_HSP_CAL_S100M66M,"TestTXREF/2|TestTXREF/1"},
    {M_BCM1480_HSP_CAL_NO_CALIB,"No_Calib|Auto_Calib"},
    {M_BCM1480_HSP_CAL_BMODE,"BMode"},
    {M_BCM1480_HSP_CAL_CALSETP,"calsetp"},
    {M_BCM1480_HSP_CAL_CALSETN,"calsetn"},
    {M_BCM1480_HSP_CAL_CALPSTAT,"calpstat"},
    {M_BCM1480_HSP_CAL_CALNSTAT,"calnstat"},
    {0,NULL}};

static bitfield_t htb_status[] = {
    {PCI_STATUS_PARITY_ERROR,"PCI_data_error(parity/crc)"},
    {PCI_STATUS_SYSTEM_ERROR,"PCI_system_error"},
    {PCI_STATUS_MASTER_ABORT,"PCI_master_abort"},
    {PCI_STATUS_MASTER_TARGET_ABORT,"PCI_target_abort(sent)"},
    {PCI_STATUS_TARGET_TARGET_ABORT,"PCI_target_abort(rcvd)"},
    {0,NULL}};

static bitfield_t htb_linkctrl[] = {
    {M_BCM1480_HTB_LINKCTRL_CRCFLEN, "CRC_sync_flood_enable"},
    {M_BCM1480_HTB_LINKCTRL_LINKFAIL,"Link_fail"},
    {M_BCM1480_HTB_LINKCTRL_CRCERR,"CRC_error"},
    {0,NULL}};

static bitfield_t htb_linkfreqerr[] = {
    {M_BCM1480_HTB_LINKFQERR_PROTERR,"Protocol_error"},
    {M_BCM1480_HTB_LINKFQERR_OVFLERR,"Overflow_error"},
    {M_BCM1480_HTB_LINKFQERR_EOCERR, "EOC_error"},
    {0,NULL}};

static bitfield_t htb_errhndl[] = {
    {M_BCM1480_HTB_ERRHNDL_PROFLEN, "Protocol_error_sync_flood_enable"},
    {M_BCM1480_HTB_LINKFQERR_OVFLERR, "Overflow_error_sync_flood_enable"},
    {M_BCM1480_HTB_ERRHNDL_CHNFAIL,"Chain_failed_(sync_flood)_error"},
    {M_BCM1480_HTB_ERRHNDL_RSPERR, "Response_error"},
    {0,NULL}};



/***********************************************************
*
* Static variables
*
***********************************************************/
// Documented


/***********************************************************
*
* Function prototypes of static functions
*
***********************************************************/
#ifndef DOXYGEN_IGNORE_ALWAYS
static uint32_t hsp_ht_log_errors(uint32_t port);
static uint32_t hsp_spi4_log_errors(uint32_t port);
#endif

int ui_init_hspcmds(void);

/***********************************************************
*
* Functions
*
***********************************************************/


static int ui_cmd_ht_show_errors(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    uint32_t port = 0;

    if ((x = cmd_getarg(cmd,0))) 
    {
        port = atoi(x);
    }

    if (!HSP_IS_PORT_HT_MODE(port))
    {
        printf("WARNING Port %d is NOT in HT mode.\n",port);
    }  


    if ( cmd_sw_isset(cmd,"-reset") != NULL )
    {
        hsp_ht_reset_errors(port);
    }


    if ( cmd_sw_value(cmd,"-sync_flood",&x))
    {
        hsp_ht_enable_sync_flood_on_errors( port, atoi(x));
    }

    
    hsp_ht_log_errors( port );

    return(0);
}  



static int ui_cmd_spi4_show_errors(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    uint32_t port = 0;

    if ((x = cmd_getarg(cmd,0))) 
    {
        port = atoi(x);
    }

    if (!HSP_IS_PORT_SPI4_MODE(port))
    {
        printf("WARNING Port %d is NOT in SPI4 mode.\n",port);
    }  

    if ( cmd_sw_isset(cmd,"-reset") != NULL )
    {
        hsp_spi4_reset_errors(port);
    }


    hsp_spi4_log_errors( port );

    return(0);
}  


static int ui_cmd_hsp_txram(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    hsaddr_t port = 0,ctlreg,datareg;
    int low,high,idx;

    low = 0; high = 65535;

    if ((x = cmd_getarg(cmd,0))) port = atoi(x);
    else ui_showusage(cmd);

    if ((x = cmd_getarg(cmd,1))) low = xtoi(x);
    if ((x = cmd_getarg(cmd,2))) high = xtoi(x);

    ctlreg = A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_RAM_READCTL);
    datareg = A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_RAM_READWINDOW);

    for (idx = low; idx <= high; idx++) {
	WRITECSR(ctlreg,V_BCM1480_HSP_TXVIS_RAM_ADDR(idx) | 
		     V_BCM1480_HSP_TXVIS_RAM(K_BCM1480_HSP_TXVIS_RAM_DRAM_128_151));
	printf("%04X:  %016llX   ",idx,READCSR(datareg));

	WRITECSR(ctlreg,V_BCM1480_HSP_TXVIS_RAM_ADDR(idx) | 
		     V_BCM1480_HSP_TXVIS_RAM(K_BCM1480_HSP_TXVIS_RAM_DRAM_64_127));
	printf("%016llX ",READCSR(datareg));

	WRITECSR(ctlreg,V_BCM1480_HSP_TXVIS_RAM_ADDR(idx) | 
		     V_BCM1480_HSP_TXVIS_RAM(K_BCM1480_HSP_TXVIS_RAM_DRAM_0_63));
	printf("%016llX\n",READCSR(datareg));     
	if (console_status()) break;
	}

    return 0;
    
}

static int ui_cmd_hsp_rxram(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    hsaddr_t port = 0,ctlreg,datareg;
    int low,high,idx;

    low = 0; high = 65535;

    if ((x = cmd_getarg(cmd,0))) port = atoi(x);
    else return ui_showusage(cmd);

    if ((x = cmd_getarg(cmd,1))) low = xtoi(x);
    if ((x = cmd_getarg(cmd,2))) high = xtoi(x);

    ctlreg = A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_RAM_READCTL);
    datareg = A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_RAM_READWINDOW);

    for (idx = low; idx <= high; idx++) {
	WRITECSR(ctlreg,V_BCM1480_HSP_TXVIS_RAM_ADDR(idx) | 
		     V_BCM1480_HSP_TXVIS_RAM(K_BCM1480_HSP_RXVIS_RAM_DRAM_128_151));
	printf("%04X:  %016llX   ",idx,READCSR(datareg));

	WRITECSR(ctlreg,V_BCM1480_HSP_TXVIS_RAM_ADDR(idx) | 
		     V_BCM1480_HSP_TXVIS_RAM(K_BCM1480_HSP_RXVIS_RAM_DRAM_64_127));
	printf("%016llX ",READCSR(datareg));

	WRITECSR(ctlreg,V_BCM1480_HSP_TXVIS_RAM_ADDR(idx) | 
		     V_BCM1480_HSP_TXVIS_RAM(K_BCM1480_HSP_RXVIS_RAM_DRAM_0_63));

	printf("%016llX\n",READCSR(datareg));
	if (console_status()) break;
	}

    return 0;
    
}

static int ui_cmd_hsp_txvis(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint64_t port,reg;
    uint64_t idx = 0;
    uint64_t val,val2,val3;
    hsaddr_t ctlreg,datareg;

    if (!cmd_getarg(cmd,1)) return ui_showusage(cmd);
    port = atoi(cmd_getarg(cmd,0));
    reg = atoi(cmd_getarg(cmd,1));

    ctlreg = A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_RF_READCTL);
    datareg = A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_RF_READWINDOW);

    printf("Dumping visibility of TX port %d register file #%d\n",(int)port,(int)reg);

    switch (reg) {
	default:
	    for (idx = 0; idx < 24; idx++) {
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_TXRFVIS_RAM(reg) |
			     V_BCM1480_HSP_TXRFVIS_RAM_ADDR(idx));
		printf("Regfile[%02X] = %016llX\n",idx,READCSR(datareg));
		}

	    break;
	case 0:
	    for (idx = 0; idx < 24; idx++) {
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_TXRFVIS_RAM(reg+0) |
			     V_BCM1480_HSP_TXRFVIS_RAM_ADDR(idx));
		val = READCSR(datareg);
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_TXRFVIS_RAM(reg+1) |
			     V_BCM1480_HSP_TXRFVIS_RAM_ADDR(idx));
		val2 = READCSR(datareg);
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_TXRFVIS_RAM(reg+8) |
			     V_BCM1480_HSP_TXRFVIS_RAM_ADDR(idx));
		val3 = READCSR(datareg);
		printf("Chan %2d   Head:%04X  Tail:%04X  Phit:%04X\n",idx,
		       (uint32_t)val,(uint32_t)val2,(uint32_t)val3);
		}
	    break;
	    

	case 2: case 5:
	    for (idx = 0; idx < 24; idx++) {
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_TXRFVIS_RAM(reg+0) |
			     V_BCM1480_HSP_TXRFVIS_RAM_ADDR(idx));
		val = READCSR(datareg);
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_TXRFVIS_RAM(reg+1) |
			     V_BCM1480_HSP_TXRFVIS_RAM_ADDR(idx));
		val2 = READCSR(datareg);
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_TXRFVIS_RAM(reg+2) |
			     V_BCM1480_HSP_TXRFVIS_RAM_ADDR(idx));
		val3 = READCSR(datareg);
		printf("Chan %2d   Meta:%08llX %s %s BC:%2d %s IVC:%2d Data:%02llX_%016llX\n",idx,val3,
		       (val3 & 1) ? "SOP" : "---",
		       (val3 & 2) ? "EOP" : "---",
		       (int)((val3 >> 2) & 15),
		       (val3 & 64) ? "ERR" : "OK ",
		       (int)(val3 >> 7),
		       val2,val);
		}
	    break;
	    
	case 9:
	    for (idx = 0; idx < 24; idx++) {
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_TXRFVIS_RAM(reg) |
			     V_BCM1480_HSP_TXRFVIS_RAM_ADDR(idx));
		val = READCSR(datareg);
		printf("Chan %2d   Push:%d   Pop:%d\n",idx,
		       (int)(val & 0x0F),(int)((val >> 4) & 0x0F));
		}
	    break;

	case 10:
	    WRITECSR(ctlreg,
			 V_BCM1480_HSP_TXRFVIS_RAM(reg) |
			 V_BCM1480_HSP_TXRFVIS_RAM_ADDR(0));
	    val = READCSR(datareg);
	    printf("Ready to pick:  %06llx\n",((val >> 32) & 0xFFFFFF));
	    printf("IVC OK:         %06llx\n",(val & 0xFFFFFF));
	    break;	    

	}

    return 0;
}

static int ui_cmd_hsp_rxvis(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint64_t port,reg;
    uint64_t idx;
    uint64_t val,val2,val3;
    hsaddr_t ctlreg,datareg;

    if (!cmd_getarg(cmd,1)) return ui_showusage(cmd);
    port = atoi(cmd_getarg(cmd,0));
    reg = atoi(cmd_getarg(cmd,1));

    ctlreg = A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_RF_READCTL);
    datareg = A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_RF_READWINDOW);

    printf("Dumping visibility of RX port %d register file #%d\n",(int)port,(int)reg);


    switch (reg) {
	case 0:
	    for (idx = 0; idx < 24; idx++) {
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_RXRFVIS_RAM(reg+0) |
			     V_BCM1480_HSP_RXRFVIS_RAM_ADDR(idx));
		val = READCSR(datareg);
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_RXRFVIS_RAM(reg+1) |
			     V_BCM1480_HSP_RXRFVIS_RAM_ADDR(idx));
		val2 = READCSR(datareg);
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_RXRFVIS_RAM(reg+2) |
			     V_BCM1480_HSP_RXRFVIS_RAM_ADDR(idx));
		val3 = READCSR(datareg);
		printf("Chan %2d   Meta:%016llX  Data:%016llX_%016llX\n",idx,
		       val3,val,val2);
		}
	    break;
	case 5:
	    for (idx = 0; idx < 24; idx++) {
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_RXRFVIS_RAM(reg+0) |
			     V_BCM1480_HSP_RXRFVIS_RAM_ADDR(idx));
		val = READCSR(datareg);
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_RXRFVIS_RAM(reg+1) |
			     V_BCM1480_HSP_RXRFVIS_RAM_ADDR(idx));
		val2 = READCSR(datareg);
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_RXRFVIS_RAM(reg+3) |
			     V_BCM1480_HSP_RXRFVIS_RAM_ADDR(idx));
		val3 = READCSR(datareg);
		printf("Chan %2d   Head:%04X  Tail:%04X  Phit:%04X\n",idx,
		       (uint32_t)val,(uint32_t)val2,(uint32_t)val3);
		}
	    break;
	default:
	    for (idx = 0; idx < 32; idx++) {
		WRITECSR(ctlreg,
			     V_BCM1480_HSP_RXRFVIS_RAM(reg) |
			     V_BCM1480_HSP_RXRFVIS_RAM_ADDR(idx));
		printf("Regfile[%02X] = %016llX\n",idx,READCSR(datareg));
		}
	}

    return 0;
}



static int ui_cmd_spi4_show(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int port = 0;
    char *x;
    uint64_t reg;

    if ((x = cmd_getarg(cmd,0))) port = atoi(x);

    if (!HSP_IS_PORT_SPI4_MODE(port))
    {
        printf("WARNING Port %d is NOT in SPI4 mode.\n",port);
    }  

    printf("** Port %d **\n",port);
    reg = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_PORT_INT_STATUS));
    printf("TX SPI-4 Interrupt Status:   %016llX",reg);
    printf(" [ %s]\n",showfields(spi4_txintstat,reg));

    reg = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_CALIBRATION));
    printf("TX SPI-4 Calibration:        %016llX",reg);
    printf(" [ %s]\n",showfields(spi4_txcalibration,reg));

    reg = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_PORT_INT_STATUS));
    printf("RX SPI-4 Interrupt Status:   %016llX",reg);
    printf(" [ %s]\n",showfields(spi4_rxintstat,reg));

    reg = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_CALIBRATION));
    printf("RX SPI-4 Calibration:        %016llX",reg);
    printf(" [ %s]\n",showfields(spi4_txcalibration,reg));	/* same fields as TX */
	   
    return 0;
}


/**
    uint32_t hsp_ht_log_errors(uint32_t port)
*/
uint32_t hsp_ht_log_errors(uint32_t port)
{
    pcireg_t reg;

    if (!HSP_IS_PORT_HT_MODE(port))
    {
        printf("WARNING Port %d is NOT in HT mode.\n",port);
    }  

    printf("** Port %d HT Error Status - Summary( %s ) **\n", port,
           hsp_ht_check_for_errors(port) ? "Error" : "No Errors" );

    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), PCI_COMMAND_STATUS_REG); 
    printf("HT/PCI Primary Status:   %08X",reg);
    printf(" [ %s]\n",showfields(htb_status, (uint64_t) reg));

    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), PPB_IO_STATUS_REG);
    printf("HT/PCI Secondary Status: %08X",reg);
    printf(" [ %s]\n",showfields(htb_status, (uint64_t) reg));

    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKCTRL);
    printf("HT Link Ctrl / Status:   %08X",reg);
    printf(" [ %s]\n",showfields(htb_linkctrl, (uint64_t) reg));

    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_LINKFREQERR);
    printf("HT Link Error:           %08X",reg);
    printf(" [ %s]\n",showfields(htb_linkfreqerr, (uint64_t) reg));

    reg = pci_conf_read32(BCM1480_EXTx_BRIDGE(port), R_BCM1480_HTB_ERRHNDL);
    printf("HT Error Handling :      %08X",reg);
    printf(" [ %s]\n",showfields(htb_errhndl, (uint64_t) reg));

    return (0);
}


/**
    uint32_t hsp_spi4_log_errors(uint32_t port)
*/
uint32_t hsp_spi4_log_errors(uint32_t port)
{
    uint64_t reg;

    if (!HSP_IS_PORT_SPI4_MODE(port))
    {
        printf("WARNING Port %d is NOT in SPI4 mode.\n",port);
    }  

    printf("** Port %d SPI4 Error Status - Summary( %s ) **\n", port,
           hsp_spi4_check_for_errors(port) ? "Error" : "No Errors" );

    reg = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_TX_SPI4_PORT_INT_STATUS));
    printf("TX SPI-4 Interrupt Status:   %016llX",reg);
    printf(" [ %s]\n",showfields(spi4_txintstat,reg));

    reg = READCSR(A_BCM1480_HSP_REGISTER(port,R_BCM1480_HSP_RX_SPI4_PORT_INT_STATUS));
    printf("RX SPI-4 Interrupt Status:   %016llX",reg);
    printf(" [ %s]\n",showfields(spi4_rxintstat,reg));
    return (0);
}




/*  *********************************************************************
    *  ui_init_hspcmds()
    *  
    *  Add HSP-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */

int ui_init_hspcmds(void)
{

    cmd_addcmd("hsp txvis",	
	       ui_cmd_hsp_txvis,
	       NULL,
	       "Display TX RF Visibility",
	       "hsp txvis <port> (reg)",
	       "");

    cmd_addcmd("hsp rxvis",	
	       ui_cmd_hsp_rxvis,
	       NULL,
	       "Display RX RF Visibility",
	       "hsp rxvis <port> (reg)",
	       "");

    cmd_addcmd("hsp txram",	
	       ui_cmd_hsp_txram,
	       NULL,
	       "Display TX buffer RAM",
	       "hsp txram <port>",
	       "");

    cmd_addcmd("hsp rxram",	
	       ui_cmd_hsp_rxram,
	       NULL,
	       "Display RX buffer RAM",
	       "hsp rxram <port> [low] [high]",
	       "");

    cmd_addcmd("hsp ht errors",
	       ui_cmd_ht_show_errors,
	       NULL,
	       "Display/Reset HT Error Registers",
	       "hsp ht errors <port> \n",
           "-sync_flood=*;Enable (1) / Disable (0) on all HT errors (CRC, Protocol, Overflow)|"
	       "-reset; Reset latched errors");

    cmd_addcmd("hsp spi4 errors",
	       ui_cmd_spi4_show_errors,
	       NULL,
	       "Display/Reset SPI4 Error Registers",
	       "hsp spi4 errors <port> \n",
	       "-reset; Reset latched errors");

    cmd_addcmd("hsp spi4 show",
	       ui_cmd_spi4_show,
	       NULL,
	       "Display TX/RX registers",
	       "hsp spi4 show 0|1|2",
	       "");

    return 0;
}





/*  *********************************************************************
    *  Bit Wrangling
    ********************************************************************* */

char *showfields(bitfield_t *fields,uint64_t val)
{
    int left,right;
    int idx;
    static char buffer[512];
    char fieldname[128];
    char *p = buffer;
    char *x,*setstr,*clearstr;

    *p = '\0';
    while (fields->name) {

	/* Determine left and right edge of bit field */
	right = -1; left = -1;
	for (idx = 0; idx < 64; idx++) {
	    if ((right < 0) && (fields->field & (1LL << idx))) right = idx;
	    if ((left < 0) && (fields->field & (0x8000000000000000LL >> idx))) left = (63-idx);
	    }

	/* If same, it's a one bit field */
	if (right == left) {
	    strcpy(fieldname,fields->name);
	    x = strchr(fieldname,'|');
	    if (x) {
		*x++ = '\0'; setstr = fieldname; clearstr = x;
		}
	    else {
		setstr = fieldname; clearstr = NULL;
		}
	 
	    if (fields->field & val) p += sprintf(p,"%s ",setstr);
	    else {
		if (clearstr) p += sprintf(p,"%s ",clearstr);
		}
	    }
	else {
	    p += sprintf(p,"%s=%u ",fields->name,
			 (int)((fields->field & val) >> right));
	    }

	fields++;
	}

    return buffer;
}
