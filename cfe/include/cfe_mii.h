/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Common MII definitions			File: cfe_mii.h
    *  
    *  Driver common data structures for MII devices.   The
    *  higher-level (device-specific) MII drivers talk to this, and
    *  via these structures we end up at platform-specific MII 
    *  handlers.
    *  
    *********************************************************************  
    *
    *  Copyright 2004
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


typedef struct cfe_mii_channel_s cfe_mii_channel_t;
typedef struct cfe_mii_s cfe_mii_t;

struct cfe_mii_s {
    /* Attach channel */
    cfe_mii_channel_t * (*attach)(cfe_mii_t *,uint64_t probe_a,uint64_t probe_b);
    /* Initialize channel */
    int (*init)(cfe_mii_channel_t *chan);
    /* Get default address */
    int (*default_addr)(cfe_mii_channel_t *chan);
    /* Write data to device */
    void (*write)(cfe_mii_channel_t *chan,int addr,int regidx,unsigned int regval);
    /* Read data from device */
    unsigned int (*read)(cfe_mii_channel_t *chan,int addr,int regidx);
};

struct cfe_mii_channel_s {
    cfe_mii_t *ops;
    void *softc;
};

#define MII_CHANNELS_MAX	4

extern cfe_mii_channel_t *cfe_mii_channels[MII_CHANNELS_MAX];

#define MII_CHANNEL(chanidx) cfe_mii_channels[chanidx]
#define MII_INIT(chan) (chan)->ops->init(chan)
#define MII_DEFAULT_ADDR(chan) (chan)->ops->default_addr(chan)
#define MII_WRITE(chan,addr,regidx,regval) (chan)->ops->write(chan,addr,regidx,regval)
#define MII_READ(chan,addr,regidx) (chan)->ops->read(chan,addr,regidx)


int cfe_add_mii(cfe_mii_t *ops,uint64_t probe_a,uint64_t probe_b);


