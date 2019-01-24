/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  M-Systems Block Device Driver for
    *  Binary Partition region of DiskOnChip (TM) Devices
    *  File: dev_msys.c
    *  
    *  To create a binary partition, we use the M-Systems Linux utility
    *  "dformat" for low-level (INFTL/NFTL) format the device for a
    *  BDTL (Block Device translation Layer) Partition as well as a
    *  user-defined binary partition of various sizes. Hereis an
    *  example of creating a device with a name of BIPO of size 1M with
    *  initial 800K kernel data image file vmlinux.img
    *
    *  dformat -win:1f006000 -bdkL0:1M -bdkN0:BIPO -bdkF0:vmlinux.img
    *
    *  Notes:
    *  
    *  1) DFORMAT must be run AT LEAST ONCE, in order to create the
    *     binary partition area. Note also that the filename must be
    *     provided to write the binary partition.
    *  
    *  2) All kernel image files must be 32K page aligned!
    *     To workaround DiskOnChip (TM) binary partition loader page
    *     write issue. In order for DOC to compute ECC/EDC, you need
    *     to always write a page (32K) of data (minimum). If the
    *     image is not page aligned, we simply write N bytes of zeros
    *     at the end so that the DOC ASIC controller will correctly
    *     compute the ECC syndrome bytes and all will be good.
    *     See mkflashimage for more details.
    *  
    *  3) Note that the CFE loader will never read these bytes,
    *     we just put them there to keep the DOC asic controller happy.
    *   
    *   Author:  Mitch Lichtenberg (mpl@broadcom.com)
    *   Author:  James Dougherty (jfd@broadcom.com)
    *  
    *********************************************************************  
    *
    *  Copyright 2000,2001
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
    *     and retain this copyright notice and list of conditions as 
    *     they appear in the source file.
    *  
    *  2) No right is granted to use any trade name, trademark, or 
    *     logo of Broadcom Corporation. Neither the "Broadcom 
    *     Corporation" name nor any trademark or logo of Broadcom 
    *     Corporation may be used to endorse or promote products 
    *     derived from this software without the prior written 
    *     permission of Broadcom Corporation.
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

#include "dev_msys.h"
#include "dev_flash.h"
#include "dev_newflash.h"

/*  *********************************************************************
    *  Forward declarations
    ********************************************************************* */
static FLStatus bdkdrv_print_status(const FLStatus bStat);


static void bdkdrv_probe(cfe_driver_t *drv,
			 unsigned long probe_a, unsigned long probe_b, 
			 void *probe_ptr);

static int bdkdrv_open(cfe_devctx_t *ctx);
static int bdkdrv_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int bdkdrv_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat);
static int bdkdrv_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int bdkdrv_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer);
static int bdkdrv_close(cfe_devctx_t *ctx);

/*  *********************************************************************
    *  Device dispatch
    ********************************************************************* */

const static cfe_devdisp_t bdkdrv_dispatch = {
    bdkdrv_open,
    bdkdrv_read,
    bdkdrv_inpstat,
    bdkdrv_write,
    bdkdrv_ioctl,
    bdkdrv_close,	
    NULL,
    NULL
};

const cfe_driver_t bdkdrv = {
    "M-Systems Binary Partition",
    "doc",
    CFE_DEV_OTHER,
    &bdkdrv_dispatch,
    bdkdrv_probe
};

typedef struct bdkdrv_s {
    newflash_probe_t bdkdrv_probe;	/* data from probe */
    flash_info_t bdkdrv_info;   /* data from flash info */
    int bdkdrv_devsize;		/* size reported by driver */
    int bdkdrv_initialized;	/* true if we've probed already */
    int bdkdrv_unlocked;        
    FLStatus          status;    
    unsigned long     base, size;
    unsigned char     sign[SIGNATURE_LEN] ;
    unsigned long     real_part_size ;
    unsigned long     image_size ;
    unsigned long     unit_size ;
    unsigned long     start_unit;
    unsigned long     num_blocks;
    unsigned char     chksum;
} bdkdrv_t;


/*
 * Output message for FLStatus from M-Systems BDK software.
 */
FLStatus
bdkdrv_print_status(const FLStatus bStat)
{
	printf("Error: FLStatus - ") ;

	switch(bStat){

	case flOK:
		printf("OK\n") ;
		break ;
	case flBadFunction:
		printf("BadFunction\n") ;
		break ;
	case flFileNotFound:
		printf("FileNotFound\n") ;
		break ;
	case flPathNotFound:
		printf("PathNotFound\n") ;
		break ;
	case flTooManyOpenFiles:
		printf("TooManyOpenFiles\n") ;
		break ;
	case flNoWriteAccess:
		printf("NoWriteAccess\n") ;
		break ;
	case flBadFileHandle:
		printf("BadFileHandle\n") ;
		break ;
	case flDriveNotAvailable:
		printf("DriveNotAvailable\n") ;
		break ;
	case flNonFATformat:
		printf("NonFATformat\n") ;
		break ;
	case flFormatNotSupported:
		printf("FormatNotSupported\n") ;
		break ;
	case flNoMoreFiles:
		printf("NoMoreFiles\n") ;
		break ;
	case flWriteProtect:
		printf("WriteProtect\n") ;
		break ;
	case flBadDriveHandle:
		printf("BadDriveHandle\n") ;
		break ;
	case flDriveNotReady:
		printf("DriveNotReady\n") ;
		break ;
	case flUnknownCmd:
		printf("UnknownCmd\n") ;
		break ;
	case flBadFormat:
		printf("BadFormat\n") ;
		break ;
	case flBadLength:
		printf("BadLength\n") ;
		break ;
	case flDataError:
		printf("DataError\n") ;
		break ;
	case flUnknownMedia:
		printf("UnknownMedia\n") ;
		break ;
	case flSectorNotFound:
		printf("SectorNotFound\n") ;
		break ;
	case flOutOfPaper:
		printf("OutOfPaper\n") ;
		break ;
	case flWriteFault:
		printf("WriteFault\n") ;
		break ;
	case flReadFault:
		printf("ReadFault\n") ;
		break ;
	case flGeneralFailure:
		printf("GeneralFailure\n") ;
		break ;
	case flDiskChange:
		printf("DiskChange\n") ;
		break ;
	case flVppFailure:
		printf("VppFailure\n") ;
		break ;
	case flBadParameter:
		printf("BadParameter\n") ;
		break ;
	case flNoSpaceInVolume:
		printf("NoSpaceInVolume\n") ;
		break ;
	case flInvalidFATchain:
		printf("InvalidFATchain\n") ;
		break ;
	case flRootDirectoryFull:
		printf("RootDirectoryFull\n") ;
		break ;
	case flNotMounted:
		printf("NotMounted\n") ;
		break ;
	case flPathIsRootDirectory:
		printf("PathIsRootDirectory\n") ;
		break ;
	case flNotADirectory:
		printf("NotADirectory\n") ;
		break ;
	case flDirectoryNotEmpty:
		printf("DirectoryNotEmpty\n") ;
		break ;
	case flFileIsADirectory:
		printf("FileIsADirectory\n") ;
		break ;
	case flAdapterNotFound:
		printf("AdapterNotFound\n") ;
		break ;
	case flFormattingError:
		printf("FormattingError\n") ;
		break ;
	case flNotEnoughMemory:
		printf("NotEnoughMemory\n") ;
		break ;
	case flVolumeTooSmall:
		printf("VolumeTooSmall\n") ;
		break ;
	case flBufferingError:
		printf("BufferingError\n") ;
		break ;
	case flFileAlreadyExists:
		printf("FileAlreadyExists\n") ;
		break ;
	case flIncomplete:
		printf("Incomplete\n") ;
		break ;
	case flTimedOut:
		printf("TimedOut\n") ;
		break ;
	case flTooManyComponents:
		printf("TooManyComponents\n") ;
		break ;
	case flTooManyDrives:
		printf("TooManyDrives\n") ;
		break ;
	case flTooManyBinaryPartitions:
		printf("TooManyBinaryPartitions\n") ;
		break ;
	case flPartitionNotFound:
		printf("PartitionNotFound\n") ;
		break ;
	case flFeatureNotSupported:
		printf("FeatureNotSupported\n") ;
		break ;
	case flWrongVersion:
		printf("WrongVersion\n") ;
		break ;
	case flTooManyBadBlocks:
		printf("TooManyBadBlocks\n") ;
		break ;
	case flNotProtected:
		printf("NotProtected\n") ;
		break ;
	case flUnchangeableProtection:
		printf("UnchangableProection\n") ;
		break ;
	case flBadBBT:
		printf("BadBBT\n") ;
		break ;
	case flInterleaveError:
		printf("InterlreavError\n") ;
		break ;
	case flWrongKey:
		printf("WrongKey\n") ;
		break ;
	case flHWProtection:
		printf("HWProtection\n") ;
		break ;
	case flBadDownload:
		printf("BadDownload\n") ;
		break ;
	default:
		printf("Unknown status\n") ;
		break ;
	}

	return (flOK) ;
}


/*  *********************************************************************
    *  bdkdrv_probe(drv,probe_a,probe_b,probe_ptr)
    *  
    *  Device probe routine.  Attach the flash device to
    *  CFE's device table.
    *  
    *  Input parameters: 
    *  	   drv - driver descriptor
    *  	   probe_a - physical address of flash
    *  	   probe_b - size of flash (bytes)
    *  	   probe_ptr - unused
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static void bdkdrv_probe(cfe_driver_t *drv,
			 unsigned long probe_a, unsigned long probe_b, 
			 void *probe_ptr)
{
    bdkdrv_t *softc;
    char descr[80];
    newflash_probe_t *probe = (newflash_probe_t *) probe_ptr;

    /* 
     * probe_a is the flash base address
     * probe_b is the size of the flash
     * probe_ptr is unused.
     */

    softc = (bdkdrv_t *) KMALLOC(sizeof(bdkdrv_t),0);
    if (softc) {
	memset(softc,0,sizeof(bdkdrv_t));

	softc->base = probe_a;
	softc->size = probe_b;

	softc->bdkdrv_initialized = 0;

	/* Init BDK API */
	bdkInit();

	/* Find devices, setup driver metadata state */
	softc->status = bdkFindDiskOnChip( &softc->base, &softc->size );

	if( softc->status != flOK ) {
	    printf("ERROR: DiskOnChip not found!\n");
	    bdkdrv_print_status( softc->status ) ;
	    bdkExit();
	}
	
	/* Get signature of device and check offset of binary partition */
	tffscpy((void *)softc->sign,
		(void *)DEFAULTSIGNATURE,BDK_SIGNATURE_NAME);
	softc->sign[BDK_SIGNATURE_NAME] = '\0';

	softc->bdkdrv_info.flash_type = FLASH_TYPE_FLASH;
	softc->bdkdrv_info.flash_base = softc->base;
	softc->bdkdrv_info.flash_size = softc->size;
	softc->bdkdrv_info.flash_type = FLASH_TYPE_FLASH;
	softc->bdkdrv_info.flash_flags = 0;
	
	xsprintf(descr,"%s at %08X size %dMB",
		 drv->drv_description,
		 softc->base,
		 softc->size/(1024*1024));


	if (probe) {
	    /* 
	     * Passed probe structure, do fancy stuff 
	     */
	    memcpy(&(softc->bdkdrv_probe),probe,sizeof(newflash_probe_t));
	    if (softc->bdkdrv_probe.flash_nchips == 0) {
		softc->bdkdrv_probe.flash_nchips = 1;
	    }
	}

	
	cfe_attach(drv,softc,NULL,descr);
    }

}


/*  *********************************************************************
    *  bdkdrv_open(ctx)
    *  
    *  Called when the flash device is opened.
    *  
    *  Input parameters: 
    *  	   ctx - device context
    *  	   
    *  Return value:
    *  	   0 if ok else error code
    ********************************************************************* */

static int bdkdrv_open(cfe_devctx_t *ctx)
{
    bdkdrv_t *softc = ctx->dev_softc;

    /*
     * do initialization
     */

    if (!softc->bdkdrv_initialized) {
	/* Get Boot information */
	softc->start_unit = 0;
	softc->status = bdkGetBootPartitionInfo( softc->start_unit,
						 &softc->real_part_size,
						 &softc->image_size,
						 &softc->unit_size,
						 softc->sign );
	if( softc->status != flOK ) {
	    printf("Partition with Sign: %s not found\n", softc->sign);
	    bdkdrv_print_status( softc->status ) ;
	    bdkExit();
	    return -1;
	}
	printf("Partition: [%s], Size=%ld,"
	       "UnitSize=%ld\n\t%ld Units,ImageSize=%d (%x)\n",
	       softc->sign, softc->real_part_size,
	       softc->unit_size, softc->real_part_size / softc->unit_size,
	       (int)softc->image_size, (int)softc->image_size);

	softc->num_blocks = softc->real_part_size / softc->unit_size;

	softc->bdkdrv_info.flash_size = softc->image_size;
	softc->bdkdrv_info.flash_type = FLASH_TYPE_FLASH;

#if 0
	softc->bdkdrv_probe.flash_nsectors =
	    softc->image_size / softc->unit_size;
#else
	softc->bdkdrv_probe.flash_nsectors = softc->image_size;
#endif
	softc->bdkdrv_probe.flash_size = softc->image_size;
	printf("nblocks=%d, nsectors=%d size=%d\n",
	       softc->num_blocks, softc->bdkdrv_probe.flash_nsectors,
	       softc->bdkdrv_probe.flash_size);
	
	/* Chip Now Ready for I/O */
	softc->bdkdrv_initialized = TRUE;
    }

    return 0;
}


/*  *********************************************************************
    *  bdkdrv_read(ctx,buffer)
    *  
    *  Read data from the flash device.    The flash device is 
    *  considered to be like a disk (you need to specify the offset).
    *  
    *  Input parameters: 
    *  	   ctx - device context
    *  	   buffer - buffer descriptor
    *  	   
    *  Return value:
    *  	   0 if ok, else error code
    ********************************************************************* */

static int bdkdrv_read(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
    bdkdrv_t *softc = ctx->dev_softc;
    unsigned char *bptr;
#if 0
    unsigned char *tmp;
#endif
    int offset;
    int blen, ret, startUnit = 0;

    bptr = buffer->buf_ptr;
    blen = buffer->buf_length;
    offset = (int) buffer->buf_offset;

    if (!(softc->bdkdrv_unlocked)) {
	if ((offset + blen) > softc->real_part_size) {
	    blen = softc->real_part_size - offset;
	} 
    }
    printf("bdkdrv_read: bptr=0x%x blen=%d offset=%d\n",
	   (unsigned)bptr, blen, offset);
#if 0
    tmp = (unsigned char*)KMALLOC(softc->unit_size,0);
    
    startUnit = offset / softc->unit_size; /* e.g. 512/32K */

    ret = bdkCopyBootAreaInit( startUnit,
			       softc->image_size, softc->sign );    

    /* Read data from binary partition */
    ret = bdkCopyBootAreaBlock((void*)tmp,
			       blen,
			       &softc->chksum);

    memcpy(bptr, tmp + offset, blen);
    KFREE(tmp);
#else
    /* Read data from binary partition */
    ret = bdkCopyBootArea((void*)bptr,
			  startUnit,
			  softc->image_size,
			  &softc->chksum,
			  softc->sign);
#endif    
    if ( ret == flOK ) {
	printf("DOC read %d bytes OK with checksum 0x%x\n",
	       (int)blen, (int)softc->chksum);
    } else {
	bdkdrv_print_status( ret );
	return -1;
    }

    buffer->buf_retlen = blen;

    return 0;
}

/*  *********************************************************************
    *  bdkdrv_inpstat(ctx,inpstat)
    *  
    *  Return "input status".  For flash devices, we always return true.
    *  
    *  Input parameters: 
    *  	   ctx - device context
    *  	   inpstat - input status structure
    *  	   
    *  Return value:
    *  	   0 if ok, else error code
    ********************************************************************* */

static int bdkdrv_inpstat(cfe_devctx_t *ctx,iocb_inpstat_t *inpstat)
{
    /* bdkdrv_t *softc = ctx->dev_softc; */

    inpstat->inp_status = 1;
    return 0;
}




/*  *********************************************************************
    *  bdkdrv_write(ctx,buffer)
    *  
    *  Write data to the flash device.    The flash device is 
    *  considered to be like a disk (you need to specify the offset).
    *  
    *  Input parameters: 
    *  	   ctx - device context
    *  	   buffer - buffer descriptor
    *  	   
    *  Return value:
    *  	   0 if ok, else error code
    ********************************************************************* */

static int bdkdrv_write(cfe_devctx_t *ctx,iocb_buffer_t *buffer)
{
#if 0
    bdkdrv_t *softc = ctx->dev_softc;
    unsigned char *bptr;
    int offset;
    int blen;
    int res;

    bptr = buffer->buf_ptr;
    blen = buffer->buf_length;
    offset = (int) buffer->buf_offset;

    if (!(softc->bdkdrv_unlocked)) {
	if ((offset + blen) > softc->bdkdrv_devsize) {
	    blen = softc->bdkdrv_devsize - offset;
	    }
	}

    res = FLASHOP_WRITE_BLOCK(softc,offset,bptr,blen);

    buffer->buf_retlen = res;

    /* XXX flush the cache here? */

    return (res == blen) ? 0 : CFE_ERR_IOERR;
#else
    printf("bdkdrv_write:\n");
    return 0;
#endif    
}

/*  *********************************************************************
    *  bdkdrv_ioctl(ctx,buffer)
    *  
    *  Handle special IOCTL functions for the flash.  Flash devices
    *  support NVRAM information, sector and chip erase, and a
    *  special IOCTL for updating the running copy of CFE.
    *  
    *  Input parameters: 
    *  	   ctx - device context
    *  	   buffer - descriptor for IOCTL parameters
    *  	   
    *  Return value:
    *  	   0 if ok else error
    ********************************************************************* */
static int bdkdrv_ioctl(cfe_devctx_t *ctx,iocb_buffer_t *buffer) 
{
    bdkdrv_t *softc = ctx->dev_softc;
    /*	
     * If using flash to store environment, only the last sector
     * is used for environment stuff.
     */
    printf("bdkdrv_ioctl: command=%d\n",
	   (int)buffer->buf_ioctlcmd);
    switch ((int)buffer->buf_ioctlcmd) {

	case IOCTL_NVRAM_ERASE:
	    return CFE_ERR_UNSUPPORTED;

	case IOCTL_NVRAM_GETINFO:
	    return CFE_ERR_UNSUPPORTED;

	case IOCTL_FLASH_ERASE_SECTOR:
	    return 0;

	case IOCTL_FLASH_ERASE_ALL:
	    return CFE_ERR_UNSUPPORTED;

	case IOCTL_FLASH_WRITE_ALL:
	    return CFE_ERR_UNSUPPORTED;

	case IOCTL_FLASH_GETINFO:
	    printf("bdkdrv_ioctl: getflashinfo:size=%d\n",
		   softc->bdkdrv_info.flash_size);
	    memcpy(buffer->buf_ptr,&(softc->bdkdrv_info),sizeof(flash_info_t));
	    return 0;

	case IOCTL_FLASH_GETSECTORS:
	    printf("bdkdrv_ioctl: nsectors=%d\n",
		   softc->bdkdrv_probe.flash_nsectors);

	    return softc->bdkdrv_probe.flash_nsectors;

	case IOCTL_FLASH_ERASE_RANGE:
	    return 0;

	case IOCTL_NVRAM_UNLOCK:
	    softc->bdkdrv_unlocked = TRUE;
	    break;

	default:
	    return CFE_ERR_UNSUPPORTED;
	}

    return CFE_ERR_UNSUPPORTED;
}


/*  *********************************************************************
    *  bdkdrv_close(ctx)
    *  
    *  Close the flash device.
    *  
    *  Input parameters: 
    *  	   ctx - device context
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */
static int bdkdrv_close(cfe_devctx_t *ctx)
{
    /* bdkdrv_t *softc = ctx->dev_softc; */

    /* XXX Invalidate the cache */
    return 0;
}


