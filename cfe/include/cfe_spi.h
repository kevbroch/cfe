/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Common SPI definitions			File: cfe_spi.h
    *  
    *  Driver common data structures for SPI devices.   The
    *  higher-level (device-specific) SPI drivers talk to this, and
    *  via these structures we end up at platform-specific SPI 
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


typedef struct cfe_spi_channel_s cfe_spi_channel_t;
typedef struct cfe_spi_s cfe_spi_t;

struct cfe_spi_s {
    /* Attach channel */
    cfe_spi_channel_t * (*attach)(cfe_spi_t *,uint64_t probe_a,uint64_t probe_b);
    /* Initialize channel */
    int (*init)(cfe_spi_channel_t *chan);
    /* Enable device (assert chip select) */
    int (*enable)(cfe_spi_channel_t *chan,uint8_t slave);
    /* Disable device (deassert chip select) */
    int (*disable)(cfe_spi_channel_t *chan,uint8_t slave);
    /* Write 'n' bytes to device */
    int (*write)(cfe_spi_channel_t *chan,uint8_t *buf,int len);
    /* Read 'n' bytes from device */
    int (*read)(cfe_spi_channel_t *chan,uint8_t *buf,int len,uint8_t data_out);
};

struct cfe_spi_channel_s {
    cfe_spi_t *ops;
    void *softc;
};

#define SPI_CHANNELS_MAX	4

extern cfe_spi_channel_t *cfe_spi_channels[SPI_CHANNELS_MAX];

#define SPI_CHANNEL(chanidx) cfe_spi_channels[chanidx]
#define SPI_INIT(chan) (chan)->ops->init(chan)
#define SPI_ENABLE(chan,slave) (chan)->ops->enable(chan,slave)
#define SPI_DISABLE(chan,slave) (chan)->ops->disable(chan,slave)
#define SPI_WRITE(chan,buf,len) (chan)->ops->write(chan,buf,len)
#define SPI_READ(chan,buf,len,data_out) (chan)->ops->read(chan,buf,len,data_out)


int cfe_add_spi(cfe_spi_t *ops,uint64_t probe_a,uint64_t probe_b);


