/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  CPU Configuration file			File:  bcm1480_hsp_utils.h
    *  
    *  This file contains defintions for initalization, configuration,
    *  and inspection routines for the BCM1480 High Speed Port Interface.

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

#ifndef _BCM1480_HSP_UTILS_H_
#define _BCM1480_HSP_UTILS_H_

/***********************************************************
*
* Included files
*
***********************************************************/

#include "cfe.h"


/***********************************************************
*
* Defined values
*
***********************************************************/

/**********************************************************
*
* Enumerated types
*
***********************************************************/

/***********************************************************
*
* Macro definitions
*
***********************************************************/

//--------------------------------------------------
/**
 Check bitmap of HT mode ports

 @ingroup HSP
*/
//--------------------------------------------------
#define HSP_PORT_MODE_HT(map,port) (((map) & (1 << (port))) != 0)


//--------------------------------------------------
/**
 Check registers if HSP is HT mode 

 @ingroup HSP
*/
//--------------------------------------------------
#define HSP_IS_PORT_HT_MODE(port)                                                                     \
	((G_BCM1480_HSP_LINK_MODE(READCSR(A_BCM1480_HSP_REGISTER((port), R_BCM1480_HSP_RX_SPI4_CFG_0))) == \
                            K_BCM1480_HSP_LINK_MODE_HT) ? 1 : 0)

//--------------------------------------------------
/**
 Check registers if HSP is SPI4 mode 

 @ingroup HSP
*/
//--------------------------------------------------
#define HSP_IS_PORT_SPI4_MODE(port)                                                                   \
	((G_BCM1480_HSP_LINK_MODE(READCSR(A_BCM1480_HSP_REGISTER((port), R_BCM1480_HSP_RX_SPI4_CFG_0))) == \
                            K_BCM1480_HSP_LINK_MODE_SPI4) ? 1 : 0)


/***********************************************************
*
* Typedefs
*
***********************************************************/

/***********************************************************
*
* Structure definitions
*
***********************************************************/

/***********************************************************
*
* External declarations of global variables
*
***********************************************************/

/***********************************************************
*
* Function prototypes
*
***********************************************************/


//--------------------------------------------------
/**

 Function called to allocate RX / TX RAM buffers
 for HT I/O and CCNuma

 @inputs
    @param port         - HSP port for operation

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    none

 @ingroup HSP

*/
//--------------------------------------------------
void hsp_ht_ram_alloc ( uint32_t port );


//--------------------------------------------------
/**

 Function called to set PHY buffer counters for
 for HT I/O and CCNuma

 @inputs
    @param port         - HSP port for operation

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    none

 @ingroup HSP

*/
//--------------------------------------------------
void hsp_ht_flow_alloc ( uint32_t port );

/**

 Function called to allocated Rx/TX buffers and
 set PHY buffer counters for Packet Manager 

    @param port         - HSP port for operation
    @param num_active   - NUmber of active channels (8 or 16)

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    0                   - Operation completed successfully
    <other>             - Operation failure , error

 @ingroup HSP

*/
//--------------------------------------------------
void hsp_pm_ram_n_flow_alloc( uint32_t port_idx, uint32_t num_active );

/**

 Function called to allocated RX/TX buffers and
 set PHY buffer counters for all port configured at
 HT.   Set for HT I/O and CCNuma  

    @param port         - HSP port for operation
    @param num_active   - NUmber of active channels (8 or 16)

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    <bitmap>            - HSP bitmap of HT configured ports

 @ingroup HSP

*/
//--------------------------------------------------
uint32_t hsp_ht_init ( void );


/**

  This routine issues a HT LINK_RESET which is a required part of the sequence
  establish credit on the HT for all channels.  The Link Reset functionality
  is abstracted through the PCI configuration space for the HT port

 @param port         - HSP port for operation

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    None

 @ingroup HSP

*/
//--------------------------------------------------
void hsp_ht_port_link_reset( uint32_t port );


/**

  This routine initialize a SPI4 port.

 @param port            - HSP port for operation
 @param mhz             - HSP port frequency (200..600)
 @param active_vc_chan  - Number of active VC channels being used (1..16)
 @param rstat           - RSTAT signal (for loopback)

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    None

 @ingroup HSP

*/
//--------------------------------------------------
void hsp_spi4_init_port(uint32_t port,
                        uint32_t mhz,
                        uint32_t active_vc_channels,
                        uint32_t rstat);


//--------------------------------------------------
/**

 Function called to reset / clear all error event 
 status bits related to a HSP configured as as HT link

 @inputs
    @param port         - HSP port for operation

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    0                   - Operation completed successfully
    <other>             - Operation failure , error

 @ingroup HSP

*/
//--------------------------------------------------
uint32_t hsp_ht_reset_errors(uint32_t port);


//--------------------------------------------------
/**

 Function called to enable/disable hardward to generate
 a HT sync-flood when a detectable link level error occurs
 CRC, etc,

 @inputs
    @param port         - HSP port for operation
    @param b_enable     - enable (1) or disable(0) sync-flood

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    0                   - Operation completed successfully
    <other>             - Operation failure , error

 @ingroup HSP

*/
//--------------------------------------------------
uint32_t hsp_ht_enable_sync_flood_on_errors(uint32_t port, 
                                            uint32_t enable);


//--------------------------------------------------
/**

 Function called to check all error status registers for
 an error condition on a HSP-HT port.

 @inputs
    @param port         - HSP port for operation

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    0                   - No error exists on port
    <other>             - Error exists on port

 @ingroup HSP

*/
//--------------------------------------------------
uint32_t hsp_ht_check_for_errors(uint32_t port);


//--------------------------------------------------
/**

 Function called to reset / clear all error event 
 status bits related to a HSP configured as as HT link

 @inputs
    @param port         - HSP port for operation

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    0                   - Operation completed successfully
    <other>             - Operation failure , error

 @ingroup HSP

*/
//--------------------------------------------------
uint32_t hsp_spi4_reset_errors(uint32_t port);

//--------------------------------------------------
/**

 Function called to check all error status registers for
 an error condition on a HSP-HT port.

 @inputs
    @param port         - HSP port for operation

 @inouts
    @param 

 @outputs
    @param

 @special
    None

 @retval
    0                   - No error exists on port
    <other>             - Error exists on port

 @ingroup HSP

*/
//--------------------------------------------------
uint32_t hsp_spi4_check_for_errors(uint32_t port);



#endif /* _BCM1480_HSP_UTILS_H_ */


