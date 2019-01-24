/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Main CFE include file			File: cfe.h
    *  
    *  This file is the first file included by most CFE modules.  It
    *  brings in definitions for the most commonly used internal
    *  routines and data and also makes sure the right configuration
    *  files get included in the correct order.   If you need to
    *  bring in additional definitions, put them _after_ this file.
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

#ifndef _CFE_H
#define _CFE_H

/*  *********************************************************************
    *  Version number
    ********************************************************************* */

#ifdef CFE_VER_PFX
#define CFE_VER_PREFIX	CFE_VER_PFX
#else
#define CFE_VER_PREFIX  ""
#endif
#define CFE_VER_MAJOR	CFE_VER_MAJ
#define CFE_VER_MINOR	CFE_VER_MIN
#define CFE_VER_BUILD   CFE_VER_ECO

/*  *********************************************************************
    *  Global include files (stuff we always use)
    ********************************************************************* */

#include "cpu_config.h"		/* CPU configuration, needed for stuff below */
#include "lib_types.h"		/* basic types */
#include "lib_queue.h"		/* queues, lists */
#include "lib_string.h"		/* string mangling */
#include "lib_malloc.h"		/* memory allocator */
#include "lib_printf.h"		/* printf and sprintf, etc. */
#include "lib_hssubr.h"		/* hyperspace routines for 64-bit archs */

/*  *********************************************************************
    *  Common CFE functions used by most things
    ********************************************************************* */

#include "cfe_iocb.h"		/* External APIs */
#include "cfe_device.h"		/* Device structures */
#include "cfe_ioctl.h"		/* IOCTL commands to devices */
#include "cfe_error.h"		/* master error code list */
#include "cfe_console.h"	/* console input/output */
#include "cfe_timer.h"		/* timer/polling routines */
#include "cfe_devfuncs.h"	/* internally callable CFE API */
#include "cfe_cache.h"		/* Cache-related stuff */

/*  *********************************************************************
    *  Board and CPU specific include files
    ********************************************************************* */

#include "bsp_config.h"		/* board configuration */
#include "endian.h"		/* which way endian */
#include "addrspace.h"		/* Address space conversions */
#include "cfe_main.h"		/* top-level board stuff */

#endif
