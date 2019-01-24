/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Crypto IOCTL definitions			File: cfe_crypto.h
    *  
    *  IOCTL function numbers and I/O data structures for communicating
    *  with BlueSteel/Broadcom BCM582x crypto engines.
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

#ifndef _CFE_CRYPTO_H_
#define _CFE_CRYPTO_H_

#define _DD_MAKEMASK1(n) (1 << (n))
#define _DD_MAKEMASK(v,n) ((((1)<<(v))-1) << (n))
#define _DD_MAKEVALUE(v,n) ((v) << (n))
#define _DD_GETVALUE(v,n,m) (((v) & (m)) >> (n))

/*  *********************************************************************
    *  Crypto stuff
    ********************************************************************* */

#define IOCTL_CRYPTO_GETINFO	0	/* return crypto_info_t */
#define IOCTL_CRYPTO_CMD_1	1	/* run a 582x MCR1 command */
#define IOCTL_CRYPTO_CMD_2	2	/* run a 582x MCR2 command */

/*
   The various implementations share a common command structure but
   differ somewhat in the supported commands and features.  For now,
   we use the corresponding chip identifiers to indicate capabilities
   instead of defining a feature mask.
*/
  
typedef enum {
    BCM5820,
    BCM5821,
    BCM5822,
    BCM5823,
    OCP80B
} bs_chip_type;

typedef struct crypto_info_s {
    bs_chip_type  chip;
} crypto_info_t;


/*
   The CMD IOCTL's expect pointers to records describing the crypto
   operation to be carried out.  Such records are passed directly to
   the corresponding device (see chip_type) and are limited to its
   capabilities, which vary somewhat among the chip types.

    Formats of 582x Command Record fields follow.
*/

/* Master Command Record Header Format */

#define S_MCR_NUM_PACKETS        0
#define M_MCR_NUM_PACKETS        _DD_MAKEMASK(16,S_MCR_NUM_PACKETS)
#define V_MCR_NUM_PACKETS(x)     _DD_MAKEVALUE(x,S_MCR_NUM_PACKETS)
#define G_MCR_NUM_PACKETS(x)     _DD_GETVALUE(x,S_MCR_NUM_PACKETS,M_MCR_NUM_PACKETS)

/* Input flags */

#define M_MCR_SUPPRESS_INTR      _DD_MAKEMASK1(31)

/* Output flags */

#define M_MCR_DONE               _DD_MAKEMASK1(16)
#define M_MCR_ERROR              _DD_MAKEMASK1(17)

#define S_MCR_ERROR_CODE         24
#define M_MCR_ERROR_CODE         _DD_MAKEMASK(8,S_MCR_ERROR_CODE)
#define V_MCR_ERROR_CODE(x)      _DD_MAKEVALUE(x,S_MCR_ERROR_CODE)
#define G_MCR_ERROR_CODE(x)      _DD_GETVALUE(x,S_MCR_ERROR_CODE,M_MCR_ERROR_CODE)
#define K_MCR_ERROR_OK           0
#define K_MCR_ERROR_UNKNOWN_OP   1
#define K_MCR_ERROR_DSA_SHORT    2
#define K_MCR_ERROR_PKI_SHORT    3
#define K_MCR_ERROR_PKO_SHORT    4                    /* Not 5820 */
#define K_MCR_ERROR_CHAIN_SHORT  5                    /* Not 5820 */
#define K_MCR_ERROR_FIFO         6                    /* Not 5820 */

/* In both cases, the header word is followed by an array of N PD entries:
     commandContext[0]
     dataBuffer[0]
     pktLen[0]
     outputBuffer[0]
     ...
     commandContext[N-1]
     dataBuffer[N-1]
     pktLen[N-1]
     outputBuffer[N-1]
*/

#define MCR_WORDS(n)  (1+8*(n))
#define MCR_BYTES(n)  ((1+8*(n))*4)


/* Descriptions of PDs to be added. */

#endif /* _CFE_CRYPTO_H_ */
