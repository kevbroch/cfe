/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  NVRAM Environment commands		     File: ui_partition.c
    *  
    *  User interface for environment variables
    *  
    *  Author:  Morten Larsen
    *  
    *********************************************************************  
    *
    *  Copyright 2005
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
#include "ptable.h"

/*  *********************************************************************
    *  Data
    ********************************************************************* */

static ptable_drv_t *ptable_drv;

/*  *********************************************************************
    *  Functions
    ********************************************************************* */

int
ptable_init(ptable_drv_t *ptbl_drv)
{
    ptable_drv = ptbl_drv;

    return 0;
}

uint32_t
ptable_check(ptable_t *ptbl)
{
    int i;
    uint32_t *p32 = (uint32_t*)ptbl;
    uint32_t sum = 0;

    for (i = 0; i < sizeof(ptable_t)/4; i++) {
        sum ^= p32[i];
        }

    return sum;
}

int
ptable_read(ptable_t *ptbl)
{
    int res;

    if (ptable_drv && ptable_drv->pt_read) {
        if ((res = (ptable_drv->pt_read)(ptbl)) < 0) {
            return res;
            }

        if (ptbl->magic != PTABLE_MAGIC) {
            return CFE_ERR_INVBOOTBLOCK;
        }

        if (ptable_check(ptbl) != 0) {
            return CFE_ERR_BBCHECKSUM;
        }

        return 0;
        }

    return -1;
}

int
ptable_write(ptable_t *ptbl)
{
    if (ptable_drv && ptable_drv->pt_write) {
        ptbl->magic = PTABLE_MAGIC;
        ptbl->chksum = 0;
        ptbl->chksum = ptable_check(ptbl);
        return (ptable_drv->pt_write)(ptbl);
        }

    return -1;
}

