/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  SB1250 MII driver			File: sb1250_mii.c
    *  
    *  Basic operations for BCM1250 MII managment interface.
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
#include "sbmips.h"

#include "sb1250_defs.h"
#include "sb1250_regs.h"
#include "sb1250_mac.h"

#include "mii.h"

#include "cfe_mii.h"


#define PHY_READCSR(t) (*((volatile uint64_t *) (t)))
#define PHY_WRITECSR(t,v) *((volatile uint64_t *) (t)) = (v)

#define M_MAC_MDIO_DIR_OUTPUT	0		/* for clarity */

#ifdef __long64
typedef volatile uint64_t phy_port_t;
typedef uint64_t phy_physaddr_t;
#define PHY_PORT(x) PHYS_TO_K1(x)
#else
typedef volatile uint32_t phy_port_t;
typedef uint32_t phy_physaddr_t;
#define PHY_PORT(x) PHYS_TO_K1(x)
#endif

typedef struct sb1250_mii_softc_s {
    unsigned long base;
    int default_phyaddr;
    phy_port_t sbe_mdio;
} sb1250_mii_softc_t;


/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */

static cfe_mii_channel_t *sb1250_mii_attach(cfe_mii_t *ops,uint64_t probe_a,uint64_t probe_b);
static int sb1250_mii_init(cfe_mii_channel_t *chan);
static int sb1250_mii_default_addr(cfe_mii_channel_t *chan);
static void sb1250_mii_write(cfe_mii_channel_t *chan,int addr,int regidx,unsigned int regval);
static unsigned int sb1250_mii_read(cfe_mii_channel_t *chan,int addr,int regidx);


/*  *********************************************************************
    *  MII operations
    ********************************************************************* */

cfe_mii_t sb1250_mii = {
    sb1250_mii_attach,
    sb1250_mii_init,
    sb1250_mii_default_addr,
    sb1250_mii_write,
    sb1250_mii_read
};


static cfe_mii_channel_t *sb1250_mii_attach(cfe_mii_t *ops,uint64_t probe_a,uint64_t probe_b)
{
    cfe_mii_channel_t *chan;	
    sb1250_mii_softc_t *softc;

    chan = KMALLOC(sizeof(cfe_mii_channel_t)+sizeof(sb1250_mii_softc_t),0);

    if (!chan) return NULL;

    chan->ops = ops;
    chan->softc = (void *) (chan+1);

    softc = chan->softc;
    softc->base = probe_a;
    softc->default_phyaddr = probe_b;
    softc->sbe_mdio = PHY_PORT(softc->base+R_MAC_MDIO);

    return chan;
}

static int sb1250_mii_init(cfe_mii_channel_t *chan)
{
    return 0;
}

static int sb1250_mii_default_addr(cfe_mii_channel_t *chan)
{
    sb1250_mii_softc_t *s = chan->softc;

    return s->default_phyaddr;
}

static void sb1250_mii_sync(sb1250_mii_softc_t *s)
{
    int cnt;
    uint64_t bits;
    int mac_mdio_genc; /*genc bit needs to be saved*/

    mac_mdio_genc = PHY_READCSR(s->sbe_mdio) & M_MAC_GENC;

    bits = M_MAC_MDIO_DIR_OUTPUT | M_MAC_MDIO_OUT;

    PHY_WRITECSR(s->sbe_mdio,bits | mac_mdio_genc);

    for (cnt = 0; cnt < 32; cnt++) {
	PHY_WRITECSR(s->sbe_mdio,bits | M_MAC_MDC | mac_mdio_genc);
	PHY_WRITECSR(s->sbe_mdio,bits | mac_mdio_genc);
	}

}

static void sb1250_mii_senddata(sb1250_mii_softc_t *s,unsigned int data, int bitcnt)
{
    int i;
    uint64_t bits;
    unsigned int curmask;
    int mac_mdio_genc;

    mac_mdio_genc = PHY_READCSR(s->sbe_mdio) & M_MAC_GENC;

    bits = M_MAC_MDIO_DIR_OUTPUT;
    PHY_WRITECSR(s->sbe_mdio,bits | mac_mdio_genc);

    curmask = 1 << (bitcnt - 1);

    for (i = 0; i < bitcnt; i++) {
	if (data & curmask) bits |= M_MAC_MDIO_OUT;
	else bits &= ~M_MAC_MDIO_OUT;
	PHY_WRITECSR(s->sbe_mdio,bits | mac_mdio_genc);
	PHY_WRITECSR(s->sbe_mdio,bits | M_MAC_MDC | mac_mdio_genc);
	PHY_WRITECSR(s->sbe_mdio,bits | mac_mdio_genc);
	curmask >>= 1;
	}
}

static unsigned int sb1250_mii_read(cfe_mii_channel_t *chan,int phyaddr,int regidx)
{
    sb1250_mii_softc_t *s = chan->softc;
    int idx;
    int error;
    int regval;
    int mac_mdio_genc;

    if (phyaddr < 0) phyaddr = s->default_phyaddr;

    /*
     * Synchronize ourselves so that the PHY knows the next
     * thing coming down is a command
     */

    sb1250_mii_sync(s);

    /*
     * Send the data to the PHY.  The sequence is
     * a "start" command (2 bits)
     * a "read" command (2 bits)
     * the PHY addr (5 bits)
     * the register index (5 bits)
     */

    sb1250_mii_senddata(s,MII_COMMAND_START, 2);
    sb1250_mii_senddata(s,MII_COMMAND_READ, 2);
    sb1250_mii_senddata(s,phyaddr, 5);
    sb1250_mii_senddata(s,regidx, 5);

    mac_mdio_genc = PHY_READCSR(s->sbe_mdio) & M_MAC_GENC;

    /* 
     * Switch the port around without a clock transition.
     */
    PHY_WRITECSR(s->sbe_mdio,M_MAC_MDIO_DIR_INPUT | mac_mdio_genc);

    /*
     * Send out a clock pulse to signal we want the status
     */

    PHY_WRITECSR(s->sbe_mdio,M_MAC_MDIO_DIR_INPUT | M_MAC_MDC | mac_mdio_genc);
    PHY_WRITECSR(s->sbe_mdio,M_MAC_MDIO_DIR_INPUT | mac_mdio_genc);

    /* 
     * If an error occured, the PHY will signal '1' back
     */
    error = PHY_READCSR(s->sbe_mdio) & M_MAC_MDIO_IN;

    /* 
     * Issue an 'idle' clock pulse, but keep the direction
     * the same.
     */
    PHY_WRITECSR(s->sbe_mdio,M_MAC_MDIO_DIR_INPUT | M_MAC_MDC | mac_mdio_genc);
    PHY_WRITECSR(s->sbe_mdio,M_MAC_MDIO_DIR_INPUT | mac_mdio_genc);

    regval = 0;

    for (idx = 0; idx < 16; idx++) {
	regval <<= 1;

	if (error == 0) {
	    if (PHY_READCSR(s->sbe_mdio) & M_MAC_MDIO_IN) regval |= 1;
	    }

	PHY_WRITECSR(s->sbe_mdio,M_MAC_MDIO_DIR_INPUT | M_MAC_MDC | mac_mdio_genc);
	PHY_WRITECSR(s->sbe_mdio,M_MAC_MDIO_DIR_INPUT | mac_mdio_genc);
	}

    /* Switch back to output */
    PHY_WRITECSR(s->sbe_mdio,M_MAC_MDIO_DIR_OUTPUT | mac_mdio_genc);

    if (error == 0) return regval;
    return 0;
}

static void sb1250_mii_write(cfe_mii_channel_t *chan,int phyaddr,int regidx,
                             unsigned int regval)
{
    sb1250_mii_softc_t *s = chan->softc;
    int mac_mdio_genc;

    if (phyaddr < 0) phyaddr = s->default_phyaddr;

    sb1250_mii_sync(s);

    sb1250_mii_senddata(s,MII_COMMAND_START,2);
    sb1250_mii_senddata(s,MII_COMMAND_WRITE,2);
    sb1250_mii_senddata(s,phyaddr, 5);
    sb1250_mii_senddata(s,regidx, 5);
    sb1250_mii_senddata(s,MII_COMMAND_ACK,2);
    sb1250_mii_senddata(s,regval,16);

    mac_mdio_genc = PHY_READCSR(s->sbe_mdio) & M_MAC_GENC;

    PHY_WRITECSR(s->sbe_mdio,M_MAC_MDIO_DIR_OUTPUT | mac_mdio_genc);
}
