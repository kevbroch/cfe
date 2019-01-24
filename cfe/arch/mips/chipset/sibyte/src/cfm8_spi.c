/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  BCM91125CFM8 SPI driver                          File: cfm8_spi.c
    *  
    *  Basic operations for BCM91125CFM8 SPI interface.
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

#include "cfe.h"

#include "cfe_spi.h"

/*  *********************************************************************
    *  Types
    ********************************************************************* */

typedef struct cfm8_spi_softc_s {
    unsigned long base;         /* physical base address */
    volatile uint8_t *addr;     /* logical base address (uncached) */
    uint8_t data_out;           /* current write value */
    uint8_t data_in;            /* last read value */
} cfm8_spi_softc_t;

/*  *********************************************************************
    *  Access macros
    ********************************************************************* */

#define SPI_DELAY(s,c) \
do { \
    uint8_t _i,_t; \
    for (_i = 0; _i < c; _i++) { \
        _t = *(s)->addr; \
        if (_t); \
    }  \
} while (0)

#define SPI_DATA_SCK            0x01 /* Slave Clock bit */
#define SPI_DATA_SS             0x02 /* Slave Select bit */
#define SPI_DATA_MOSI           0x04 /* Master-Out Slave-In bit */
#define SPI_DATA_MISO           0x08 /* Master-In Slave-Out bit */

#define SPI_ASSERT_CS(s)        (s)->data_out = SPI_DATA_SS | SPI_DATA_SCK
#define SPI_DEASSERT_CS(s)      (s)->data_out = SPI_DATA_SCK
#define SPI_CLK_INIT(s)         (s)->data_out = SPI_DATA_SCK
#define SPI_CLK_TOGGLE(s)       (s)->data_out ^= SPI_DATA_SCK
#define SPI_CLK_DELAY(s)        SPI_DELAY(s,4)

#define SPI_DATA_GET(s)         ((s)->data_in & SPI_DATA_MISO) ? 1 : 0
#define SPI_DATA_SET(s,v) \
    do { if (v) { \
            (s)->data_out |= SPI_DATA_MOSI; \
        } else { \
            (s)->data_out &= ~SPI_DATA_MOSI; \
        } \
    } while (0)

#define SPI_DATA_IN(s)          (s)->data_in = *(s)->addr
#define SPI_DATA_OUT(s)         *(s)->addr = (s)->data_out

/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */

static cfe_spi_channel_t *cfm8_spi_attach(cfe_spi_t *ops,uint64_t probe_a,uint64_t probe_b);
static int cfm8_spi_init(cfe_spi_channel_t *chan);
static int cfm8_spi_enable(cfe_spi_channel_t *chan,uint8_t slave);
static int cfm8_spi_disable(cfe_spi_channel_t *chan,uint8_t slave);
static int cfm8_spi_read(cfe_spi_channel_t *chan,uint8_t *buf,int len,uint8_t data_out);
static int cfm8_spi_write(cfe_spi_channel_t *chan,uint8_t *buf,int len);

/*  *********************************************************************
    *  SPI operations
    ********************************************************************* */

cfe_spi_t cfm8_spi = {
    cfm8_spi_attach,
    cfm8_spi_init,
    cfm8_spi_enable,
    cfm8_spi_disable,
    cfm8_spi_write,
    cfm8_spi_read
};


static cfe_spi_channel_t *cfm8_spi_attach(cfe_spi_t *ops,uint64_t probe_a,uint64_t probe_b)
{
    cfe_spi_channel_t *chan;	
    cfm8_spi_softc_t *softc;

    chan = KMALLOC(sizeof(cfe_spi_channel_t)+sizeof(cfm8_spi_softc_t),0);

    if (!chan) return NULL;

    chan->ops = ops;
    chan->softc = (void *) (chan+1);

    softc = chan->softc;
    softc->base = probe_a;
    softc->addr = (volatile uint8_t *)PHYS_TO_K1(softc->base);

    return chan;
}

static int cfm8_spi_init(cfe_spi_channel_t *chan)
{
    return 0;
}

static int cfm8_spi_enable(cfe_spi_channel_t *chan,uint8_t slave)
{
    cfm8_spi_softc_t *softc = chan->softc;

    /* Set up initial clock state before asserting chip select */
    SPI_CLK_INIT(softc);
    SPI_DATA_OUT(softc);
    SPI_CLK_DELAY(softc);

    /* Assert chip select */
    SPI_ASSERT_CS(softc);
    SPI_DATA_OUT(softc);
    SPI_CLK_DELAY(softc);

    return 0;
}

static int cfm8_spi_disable(cfe_spi_channel_t *chan,uint8_t slave)
{
    cfm8_spi_softc_t *softc = chan->softc;

    /* Deassert chip select */
    SPI_DEASSERT_CS(softc);
    SPI_DATA_OUT(softc);
    SPI_CLK_DELAY(softc);

    /* Extra delay is solely for debug purposes */
    SPI_CLK_DELAY(softc);
    SPI_CLK_DELAY(softc);

    return 0;
}

static int cfm8_spi_read(cfe_spi_channel_t *chan,uint8_t *buf,int len,uint8_t data_out)
{
    cfm8_spi_softc_t *softc = chan->softc;
    int i;
    uint8_t mask;

    for (i = 0; i < len; i++) {
        buf[i] = 0;
        for (mask = 0x80; mask; mask >>= 1) {
            /* Dummy read to even up duty-cycle */
            SPI_DATA_IN(softc);
            /* Output zeros when reading */
            SPI_DATA_SET(softc,(mask & data_out));
            SPI_CLK_TOGGLE(softc);
            /* Input data is sampled on falling edge */
	    SPI_DATA_OUT(softc);
            /* Allow data to stabilize */
            SPI_CLK_DELAY(softc);
            SPI_DATA_IN(softc);
            if (SPI_DATA_GET(softc)) {
  		buf[i] |= mask;
            }
            SPI_CLK_TOGGLE(softc);
	    SPI_DATA_OUT(softc);
            SPI_CLK_DELAY(softc);
        }
    }
    return 0;
}

static int cfm8_spi_write(cfe_spi_channel_t *chan,uint8_t *buf,int len)
{
    cfm8_spi_softc_t *softc = chan->softc;
    int i;
    uint8_t mask;

    for (i = 0; i < len; i++) {
        for (mask = 0x80; mask; mask >>= 1) {
            /* Dummy read to even up duty-cycle */
            SPI_DATA_IN(softc);
            /* Set up data on falling edge to allow it to stabilize */
            SPI_DATA_SET(softc,mask & buf[i]);
            SPI_CLK_TOGGLE(softc);
            /* Input data is sampled on falling edge */
	    SPI_DATA_OUT(softc);
            /* Allow data to stabilize */
            SPI_CLK_DELAY(softc);
            /* Dummy read to even up duty-cycle */
            SPI_DATA_IN(softc);
            /* Output data is sampled on rising edge */
            SPI_CLK_TOGGLE(softc);
	    SPI_DATA_OUT(softc);
            SPI_CLK_DELAY(softc);
        }
    }
    return 0;
}
