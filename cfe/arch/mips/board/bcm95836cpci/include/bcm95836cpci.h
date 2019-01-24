/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  BCM95836CPCI board-specific definitions       File: bcm95836cpci.h
    *  
    *********************************************************************  
    *
    *  Copyright 2003
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

#ifndef __BCM95836CPCI_H
#define __BCM95836CPCI_H
/* Macros */

#define KB 1024
#define MB (1024*1024)


/*
 * Put board-specific stuff here
 */

/* Definitions for the BCM95836CPCI platform. */

#define BCM95836_CPCI_RESET_ADDR            (MIPS33_EXTIF_REGION + 0x0000)
#define BCM95836_CPCI_BOARDID_ADDR          (MIPS33_EXTIF_REGION + 0x4000)
#define BCM95836_CPCI_LED_ADDR              (MIPS33_EXTIF_REGION + 0xC000)
#define BCM95836_CPCI_NVRAM_ADDR            (MIPS33_EXTIF_REGION + 0xE000)
#define BCM95836_CPCI_NVRAM_SIZE            0x1FF0      /* 8K NVRAM + TOD */

/* Internal NS16550-compatible dual UART */
#define BCM95836_COM1   BCM5836_UART0       /* console, DB-9 */
#define BCM95836_COM2   BCM5836_UART1       /* header */

#endif /* ! __BCM95836CPCI_H */
