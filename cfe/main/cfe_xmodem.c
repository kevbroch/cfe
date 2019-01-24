/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  "Xmodem" file system			File: cfe_xmodem.c
    *  
    *  This "file system" only works with serial devices, and 
    *  implements the venerable XMODEM protocol to receive files.
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


/*  *********************************************************************
    *  WARNING!  This is very much a work-in-progress.  Beware!
    ********************************************************************* */

#include "cfe.h"
#include "cfe_fileops.h"

/*  *********************************************************************
    *  XMODEM protocol and state machine constants
    ********************************************************************* */

/* Buffer is big enough for a 1K packet plus packet # and 2-byte CRC */
#define XMODEM_BUFSIZE	(1024 + 4)

/* Protocol states */
#define XMS_SYNC	0
#define XMS_SYNCWAIT	1
#define XMS_RX		2
#define XMS_DONE	3
#define XMS_WAIT	4
#define XMS_ERROR	5
#define XMS_RETRY	6

#define XMODEM_TIMEOUT	(CFE_HZ)		/* Timeout for sync */
#define XMODEM_RETRIES	16			/* Number of retransmissions */


/* Special XMODEM control characters */
#define XMC_SOH		0x01
#define XMC_STX		0x02
#define XMC_EOT		0x04
#define XMC_ACK		0x06
#define XMC_NAK		0x15
#define XMC_CAN		0x18
#define XMC_SYNC	'C'

#define HEADERSIZE	2			/* header is blk # + complement */
#define CRCSIZE		2			/* CRC follows the user data */

/*  *********************************************************************
    *  XMODEM context
    ********************************************************************* */

/*
 * File system context - describes overall file system info,
 * such as the handle to the underlying device.
 */

typedef struct xmodem_fsctx_s {
    int xmodem_dev;
    int xmodem_isconsole;
    int xmodem_refcnt;
} xmodem_fsctx_t;

/*
 * File context - describes an open file on the file system.
 */


typedef struct xmodem_file_s {
    xmodem_fsctx_t *xmodem_fsctx;
    int xmodem_fileoffset;
    int xmodem_blkoffset;
    int xmodem_eof;
    int xmodem_error;

    /* XMODEM control variables */
    int xmodem_state;
    uint8_t xmodem_sync;	/* char we use for sync */
    uint8_t xmodem_buffer[XMODEM_BUFSIZE];
    uint8_t xmodem_curblk;
    int xmodem_tries;
    int xmodem_blksize;
    int xmodem_crcmode;
    cfe_timer_t xmodem_timer;
} xmodem_file_t;

/*  *********************************************************************
    *  Prototypes
    ********************************************************************* */

static int xmodem_fileop_init(void **fsctx,void *devicename);
static int xmodem_fileop_open(void **ref,void *fsctx,char *filename,int mode);
static int xmodem_fileop_read(void *ref,hsaddr_t buf,int len);
static int xmodem_fileop_write(void *ref,hsaddr_t buf,int len);
static int xmodem_fileop_seek(void *ref,int offset,int how);
static void xmodem_fileop_close(void *ref);
static void xmodem_fileop_uninit(void *fsctx);


/*  *********************************************************************
    *  XMODEM CRC
    ********************************************************************* */
 

static uint16_t calc_crc(uint8_t *buf,int len)
{
    int i;
    uint16_t crc = 0;

    while (len > 0) {
	crc = crc ^ (((uint16_t) *buf) << 8);
	for (i = 0; i < 8; i++) {
	    if (crc & 0x8000) crc = crc << 1 ^ 0x1021;
	    else crc = crc << 1;
	    }
	len--;
	buf++;
	}

    return crc;
}


/*  *********************************************************************
    *  XMODEM protocol
    ********************************************************************* */


/*  *********************************************************************
    *  xmodem_rxbuf(xmf,chptr,len,timeout)
    *  
    *  Receive 'n' characters from remote host, with a timeout
    *  if we do not get them all.
    *  
    *  Input parameters: 
    *  	   xmf - XMODEM state
    *  	   chptr - pointer to receive buffer
    *  	   len - number of characters to receive
    *  	   timeout - timeout value in CFE ticks
    *  	   
    *  Return value:
    *  	   -1: timeout occured
    *  	   else number of characters received
    ********************************************************************* */

static int xmodem_rxbuf(xmodem_file_t *xmf,uint8_t *chptr,int len,int timeout)
{
    cfe_timer_t timer;
    int savelen = len;
    int res;

    TIMER_SET(timer,timeout);

    while (!TIMER_EXPIRED(timer)) {

	if (len == 0) break;
	/* note: we assume cfe_read calls POLL() internally to advance time */
	res = cfe_read(xmf->xmodem_fsctx->xmodem_dev,PTR2HSADDR(chptr),len);

	/* nothing received, wait for more */
	if (res == 0) {
	    POLL();
	    continue;
	    }
	if (res < 0) return -1;		/* some sort of device error */

	/* if we got anything, reset timer */
	TIMER_SET(timer,timeout);

	/* try for less next time */
	len -= res;
	chptr += res;
	}

    if (len != 0) return -1;		/* we did not get it all */
    return savelen;			/* otherwise, we got it all. */

}


/*  *********************************************************************
    *  xmodem_waitidle(xmf)
    *  
    *  Wait for line to become idle-- toss characters until we see
    *  one second of silence.
    *  
    *  Input parameters: 
    *  	   xmf - XMODEM state
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void xmodem_waitidle(xmodem_file_t *xmf)
{
    uint8_t b;

    while (xmodem_rxbuf(xmf,&b,1,CFE_HZ) >= 0) ;	/* wait till timeout */

}


/*  *********************************************************************
    *  xmodem_send1(xmf,c)
    *  
    *  Send a control character, such as an ACK or NAK
    *  
    *  Input parameters: 
    *  	   xmf - XMODEM state
    *  	   c - character to send
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void xmodem_send1(xmodem_file_t *xmf,uint8_t c)
{
    int dev;

    dev = xmf->xmodem_fsctx->xmodem_dev;

    cfe_write(dev,PTR2HSADDR(&c),1);
}



/*  *********************************************************************
    *  xmodem_run(xmf)
    *  
    *  Main XMODEM state machine.  We call this periodically to advance
    *  the XMODEM protocol through its states.  The xmodem_state field
    *  of the state data structure is monitored for changes, and
    *  while in any of the transfer states this routine gets called
    *  over and over to process data.
    *  
    *  Input parameters: 
    *  	   xmf - XMODEM state
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void xmodem_run(xmodem_file_t *xmf)
{
    uint8_t syncbuf[1];
    uint16_t crc,pktcrc;

    switch (xmf->xmodem_state) {
	case XMS_SYNC:
	    xmodem_send1(xmf,xmf->xmodem_sync);
	    xmf->xmodem_state = XMS_SYNCWAIT;
	    TIMER_SET(xmf->xmodem_timer,XMODEM_TIMEOUT);
	    break;

	case XMS_SYNCWAIT:
	    if (xmodem_rxbuf(xmf,syncbuf,1,XMODEM_TIMEOUT) < 0) {
		xmf->xmodem_tries++;
		if (xmf->xmodem_tries < XMODEM_RETRIES) {
		    xmf->xmodem_state = XMS_SYNC;
		    break;
		    }
		else {
		    xmf->xmodem_state = XMS_ERROR;
		    break;
		    }
		}

	    switch (syncbuf[0]) {
		case XMC_SOH:
		    xmf->xmodem_blksize = 128;
		    xmf->xmodem_state = XMS_RX;
		    xmf->xmodem_tries = 0;
		    break;

		case XMC_STX:
		    xmf->xmodem_blksize = 1024;
		    xmf->xmodem_state = XMS_RX;
		    xmf->xmodem_tries = 0;
		    break;

		case XMC_EOT:
		    xmodem_waitidle(xmf);
		    xmodem_send1(xmf,XMC_ACK);
		    xmf->xmodem_state = XMS_DONE;
		    xmf->xmodem_tries = 0;
		    break;

		case XMC_CAN:
		    /*
		     * If two CAN (Ctrl-X) chars received, go to ERROR state
		     */
		    if (xmodem_rxbuf(xmf,syncbuf,1,XMODEM_TIMEOUT) < 0) {
			xmf->xmodem_state = XMS_SYNC;
			break;
			}
		    if (syncbuf[0] != XMC_CAN) {
			xmf->xmodem_state = XMS_SYNC;
			break;
			}
		    xmf->xmodem_state = XMS_ERROR;
		    break;

		default:
		    /* Ignore bad sync characters, wait until line is idle and stay in SYNCWAIT */
		    xmodem_waitidle(xmf);
		    break;
		}
	    break;

	case XMS_RX:
	    /* 
	     * receive block plus 2 chars at the front with blk# and complement and one checksum byte 
	     * or two CRC bytes
	     */

	    if (xmodem_rxbuf(xmf,xmf->xmodem_buffer,xmf->xmodem_blksize+(HEADERSIZE+CRCSIZE),XMODEM_TIMEOUT) < 0) {
		xmf->xmodem_state = XMS_RETRY;
		break;
		}

	    /* Check the block number */
	    if (xmf->xmodem_buffer[0] != (uint8_t)(~xmf->xmodem_buffer[1])) {
		xmf->xmodem_state = XMS_RETRY;
		break;
		}


	    /* Check the checksum or CRC */

	    if (xmf->xmodem_crcmode) {
		crc = calc_crc(&(xmf->xmodem_buffer[HEADERSIZE]),xmf->xmodem_blksize);
		pktcrc = (((uint16_t)xmf->xmodem_buffer[HEADERSIZE+xmf->xmodem_blksize]) << 8) |
		    (uint16_t)(xmf->xmodem_buffer[HEADERSIZE+xmf->xmodem_blksize+1]);


		if (crc != pktcrc) {
		    xmf->xmodem_state = XMS_RETRY;
		    break;
		    }
		}
	    else {
#if 0
		/* XXX do regular checksum someday */
		for (idx = 0; idx < xmf->xmodem_blksize; idx++) {
		    csum  += xmf->xmodem_buffer[2+idx];
		    }
#endif

		}

	    /* 
	     * If the other side lost our ack, it might send the same
	     * block again.  Just ack it again if we get a repeat block.
	     */

	    if (xmf->xmodem_buffer[0] == (xmf->xmodem_curblk-1)) {
		xmodem_send1(xmf,XMC_ACK);
		xmf->xmodem_state = XMS_SYNCWAIT;
		break;
		}

	    /* 
	     * Otherwise, we want exactly the right block in sequence.
	     */

	    if (xmf->xmodem_buffer[0] != xmf->xmodem_curblk) {
		printf("incorrect block, want %02X got %02X\n",
		       xmf->xmodem_curblk,xmf->xmodem_buffer[0]);
		xmf->xmodem_state = XMS_RETRY;
		break;
		}

	    xmf->xmodem_curblk++;

	    /* 
	     * go to WAIT state until CFE requests the data.  We'll exit this
	     * state when CFE has fetched all the data in our buffer, then
	     * we'll send an ACK.
	     */
	    xmf->xmodem_state = XMS_WAIT;

	    break;

	    /* DONE, WAIT, and ERROR are all terminal states */
	case XMS_DONE:
	    break;

	case XMS_WAIT:
	    break;

	case XMS_ERROR:
	    break;

	case XMS_RETRY:
	    xmodem_waitidle(xmf);
	    xmf->xmodem_tries++;
	    if (xmf->xmodem_tries >= XMODEM_RETRIES) {
		xmf->xmodem_state = XMS_ERROR;
		xmodem_send1(xmf,XMC_CAN);
		xmodem_send1(xmf,XMC_CAN);
		break;
		}
	    xmodem_send1(xmf,XMC_NAK);
	    xmf->xmodem_state = XMS_SYNCWAIT;
	    break;
	}
}


/*  *********************************************************************
    *  xmodem_continue(xmf)
    *  
    *  This routine is called by upper levels when we're done 
    *  processing a recieved block and it's time to request another
    *  one.  When xmodem_run puts the connection in a WAIT state,
    *  that means it's time for the application to process data.  
    *  When the app is done, we come here and the protocol acks the
    *  data and requests more.
    *  
    *  Input parameters: 
    *  	   xmf - XMODEM state
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void xmodem_continue(xmodem_file_t *xmf)
{
    xmodem_send1(xmf,XMC_ACK);
    xmf->xmodem_state = XMS_SYNCWAIT;
    TIMER_SET(xmf->xmodem_timer,XMODEM_TIMEOUT);
}


/*  *********************************************************************
    *  xmodem_cancel(xmf)
    *  
    *  This routine is called when we want to terminate a recieve
    *  file operation early.  Not many XMODEM senders will actually
    *  listen to this, but we try anyway.
    *  
    *  Input parameters: 
    *  	   xmf - XMODEM state
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void xmodem_cancel(xmodem_file_t *xmf)
{
    /* wait till line is idle */
    xmodem_waitidle(xmf);

    /* send at least two CAN characters to the other side */
    xmodem_send1(xmf,XMC_CAN);
    xmodem_send1(xmf,XMC_CAN);
    xmodem_send1(xmf,XMC_CAN);
}


/*  *********************************************************************
    *  xmodem_readmore(xmf)
    *  
    *  This routine is called by the filesystem hooks (see below)
    *  to request more data from the other side.  It's basically
    *  a helper routine to call xmodem_run and xmodem_continue
    *  at appropriate times.
    *  
    *  Input parameters: 
    *  	   xmf - XMODEM state
    *  	   
    *  Return value:
    *  	   1: at EOF
    *  	   -1: some error state
    *  	   0: still receiving
    ********************************************************************* */

static int xmodem_readmore(xmodem_file_t *xmf)
{
    if (xmf->xmodem_eof) return 1;	/* already at EOF */
    if (xmf->xmodem_error) return -1;	/* in error state */

    /*
     * If we were waiting before, send an ack to get the next chunk
     */

    if (xmf->xmodem_state == XMS_WAIT) {
	xmodem_continue(xmf);
	}

    /*
     * Run protocol engine until we get something.
     */

    for (;;) {

	xmodem_run(xmf);

	if (xmf->xmodem_state == XMS_WAIT) {
	    xmf->xmodem_blkoffset = 0;
	    return 0;	/* OK */
	    }

	if (xmf->xmodem_state == XMS_DONE) {
	    xmf->xmodem_blkoffset = 0;
	    xmf->xmodem_blksize = 0;
	    xmf->xmodem_eof = 1;
	    return 1;
	    }

	if (xmf->xmodem_state == XMS_ERROR) {
	    xmf->xmodem_blkoffset = 0;
	    xmf->xmodem_blksize = 0;
	    xmf->xmodem_error = 1;
	    return -1;
	    }
	}

}

/*  *********************************************************************
    *  RAW fileio dispatch table
    ********************************************************************* */

const fileio_dispatch_t xmodem_fileops = {
    "xmodem",
    0,
    xmodem_fileop_init,
    xmodem_fileop_open,
    xmodem_fileop_read,
    xmodem_fileop_write,
    xmodem_fileop_seek,
    xmodem_fileop_close,
    xmodem_fileop_uninit
};

static int xmodem_fileop_init(void **newfsctx,void *dev)
{
    xmodem_fsctx_t *fsctx;
    char *devicename = (char *) dev;

    *newfsctx = NULL;

    fsctx = KMALLOC(sizeof(xmodem_fsctx_t),0);
    if (!fsctx) {
	return CFE_ERR_NOMEM;
	}

    if (strcmp(devicename,console_name) == 0) {	
	fsctx->xmodem_dev = console_handle;
	fsctx->xmodem_isconsole = TRUE;
	}
    else {
	fsctx->xmodem_dev = cfe_open(devicename);
	fsctx->xmodem_isconsole = FALSE;
	}

    fsctx->xmodem_refcnt = 0;

    if (fsctx->xmodem_dev >= 0) {
	*newfsctx = fsctx;
	return 0;
	}

    KFREE(fsctx);

    return CFE_ERR_FILENOTFOUND;
}

static int xmodem_fileop_open(void **ref,void *fsctx_arg,char *filename,int mode)
{
    xmodem_fsctx_t *fsctx;
    xmodem_file_t *file;

    if (mode != FILE_MODE_READ) return CFE_ERR_UNSUPPORTED;

    fsctx = (xmodem_fsctx_t *) fsctx_arg;

    file = KMALLOC(sizeof(xmodem_file_t),0);
    if (!file) {
	return CFE_ERR_NOMEM;
	}

    memset(file,0,sizeof(xmodem_file_t));
    file->xmodem_state = XMS_SYNC;
    file->xmodem_fsctx = fsctx;
    file->xmodem_sync = XMC_SYNC;
    file->xmodem_curblk = 1;
    file->xmodem_crcmode = 1;

    fsctx->xmodem_refcnt++;

    xprintf("Ready to receive XMODEM/CRC data.  Type two Ctrl-X characters to cancel\n");

    *ref = file;
    return 0;
}

static int xmodem_fileop_read(void *ref,hsaddr_t buf,int len)
{
    xmodem_file_t *xmf = (xmodem_file_t *) ref;
    int copied = 0;
    int amtcopy;
    int res;
    
    if (xmf->xmodem_error) return CFE_ERR_IOERR;

    if (xmf->xmodem_blksize == 0) {
	res = xmodem_readmore(xmf);
	if (res < 0) return CFE_ERR_IOERR;
	if (res == 1) return 0;		/* EOF */
	}


    while (len) {
	if (xmf->xmodem_blkoffset >= xmf->xmodem_blksize) break;
	amtcopy = len;

	if (amtcopy > (xmf->xmodem_blksize-xmf->xmodem_blkoffset)) {
	    amtcopy = (xmf->xmodem_blksize-xmf->xmodem_blkoffset);
	    }

	if (buf) {
	    hs_memcpy_to_hs(buf,&(xmf->xmodem_buffer[xmf->xmodem_blkoffset+HEADERSIZE]),amtcopy);
	    buf += amtcopy;
	    }

	xmf->xmodem_blkoffset += amtcopy;
	len -= amtcopy;
	xmf->xmodem_fileoffset += amtcopy;
	copied += amtcopy;

	if (xmf->xmodem_blkoffset >= xmf->xmodem_blksize) {
	    res = xmodem_readmore(xmf);
	    if (res != 0) break;
	    }
	}

    return copied;

}

static int xmodem_fileop_write(void *ref,hsaddr_t buf,int len)
{
    return CFE_ERR_UNSUPPORTED;
}

static int xmodem_fileop_seek(void *ref,int offset,int how)
{
    xmodem_file_t *file = (xmodem_file_t *) ref;
    int delta;
    int startloc;
    int res;

    switch (how) {
	case FILE_SEEK_BEGINNING:
	    startloc = file->xmodem_fileoffset;
	    break;
	case FILE_SEEK_CURRENT:
	    startloc = 0;
	    break;
	default:
	    startloc = 0;
	    break;
	}

    delta = offset - startloc;
    if (delta < 0) {
	/* xprintf("Warning: negative seek on xmodem file attempted\n"); */
	return CFE_ERR_UNSUPPORTED;
	}
    res = xmodem_fileop_read(ref,NULL,delta);
    if (res < 0) return res;

    return file->xmodem_fileoffset;
}


static void xmodem_fileop_close(void *ref)
{
    xmodem_file_t *file = (xmodem_file_t *) ref;

    /*
     * If we were not done receiving a file, send a CANCEL 
     * XXX should we drain the rest of the file out?  It appears that
     * XXX some XMODEM senders do not honor the CAN command, so 
     * XXX both ends will look stuck if you don't need all the file,
     * XXX as is the case when loading ELF files.
     */

    if ((file->xmodem_state != XMS_ERROR) && (file->xmodem_state != XMS_DONE)) {
	xmodem_cancel(file);
	}

    file->xmodem_fsctx->xmodem_refcnt--;

    KFREE(file);
}

static void xmodem_fileop_uninit(void *fsctx_arg)
{
    xmodem_fsctx_t *fsctx = (xmodem_fsctx_t *) fsctx_arg;

    if (fsctx->xmodem_refcnt) {
	return;
	}

    if (fsctx->xmodem_isconsole == FALSE) {
	cfe_close(fsctx->xmodem_dev);
	}

    KFREE(fsctx);
}
