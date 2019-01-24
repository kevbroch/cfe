/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Common SMBus definitions			File: cfe_smbus.h
    *  
    *  Driver common data structures for SMBus devices.   The
    *  higher-level (device-specific) SMBus drivers talk to this, and
    *  via these structures we end up at platofrm-specific SMbus 
    *  handlers.
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


typedef struct cfe_smbus_channel_s cfe_smbus_channel_t;
typedef struct cfe_smbus_s cfe_smbus_t;

struct cfe_smbus_s {
    /* Attach channel */
    cfe_smbus_channel_t * (*attach)(cfe_smbus_t *,uint64_t probe_a,uint64_t probe_b);
    /* initialize channel */
    int (*init)(cfe_smbus_channel_t *chan);
    /* Write 'n' bytes to device (stop condition after write) */
    int (*write)(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t *buf,int len);
    /* Read 'n' bytes from device (stop condition after each read) */
    int (*read)(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t *buf,int len);
    /* Transaction: Write 1 byte and read 'n' bytes (no stop between write and read) */
    int (*xact)(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t cmd,uint8_t *buf,int len);
    /* Quick command */
    int (*qcmd)(cfe_smbus_channel_t *chan,uint8_t slave,int rw);
    /* Extended mode Read command */
    int (*xread)(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t cmd,uint8_t numcmd,uint8_t numdat, int *buf);
    /* Extended mode Write command */
    int (*xwrite)(cfe_smbus_channel_t *chan,uint8_t slave,uint8_t cmd,uint8_t numcmd,uint8_t numdat, int *buf);
};

#define SMBUS_QCMD_R	0
#define SMBUS_QCMD_W	1

struct cfe_smbus_channel_s {
    cfe_smbus_t *ops;
    void *softc;
};

#define SMBUS_CHANNELS_MAX	4

extern cfe_smbus_channel_t *cfe_smbus_channels[SMBUS_CHANNELS_MAX];

#define SMBUS_CHANNEL(chanidx) cfe_smbus_channels[chanidx]
#define SMBUS_INIT(chan) (chan)->ops->init(chan)
#define SMBUS_READ(chan,slave,buf,len) (chan)->ops->read(chan,slave,buf,len)
#define SMBUS_WRITE(chan,slave,buf,len) (chan)->ops->write(chan,slave,buf,len)
#define SMBUS_XACT(chan,slave,cmd,buf,len) (chan)->ops->xact(chan,slave,cmd,buf,len)
#define SMBUS_QCMD(chan,slave,rw) (chan)->ops->qcmd(chan,slave,rw);
#define SMBUS_XREAD(chan,slave,cmd,ncmd,ndat,buf) (chan)->ops->xread(chan,slave,cmd,ncmd,ndat,buf)
#define SMBUS_XWRITE(chan,slave,cmd,ncmd,ndat,buf) (chan)->ops->xwrite(chan,slave,cmd,ncmd,ndat,buf)

int cfe_add_smbus(cfe_smbus_t *ops,uint64_t probe_a,uint64_t probe_b);


