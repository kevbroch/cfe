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
#include "ui_command.h"

static ptable_t ptable;

int ui_init_ptablecmds(void);
static int ui_cmd_ptable_show(ui_cmdline_t *cmd, int argc, char *argv[]);
static int ui_cmd_ptable_add(ui_cmdline_t *cmd, int argc, char *argv[]);
static int ui_cmd_ptable_del(ui_cmdline_t *cmd, int argc, char *argv[]);
static int ui_cmd_ptable_clear(ui_cmdline_t *cmd, int argc, char *argv[]);
static int ui_cmd_ptable_save(ui_cmdline_t *cmd, int argc, char *argv[]);


static int ui_cmd_ptable_show(ui_cmdline_t *cmd, int argc, char *argv[])
{
    int i;

    xprintf("Partition Name          Offset          Size\n");
    xprintf("--------------------------------------------\n");

    for (i = 0; i < PTABLE_MAX_PARTITIONS && ptable.part[i].size; i++) {
        xprintf("%-16s    0x%8x    0x%8x\n", ptable.part[i].name, 
                ptable.part[i].offset, ptable.part[i].size);
    }

    return 0;
}

static int ui_cmd_ptable_add(ui_cmdline_t *cmd, int argc, char *argv[])
{
    int i, j, offset, size, overlap;
    char *name;

    if (argc == 3) {
        name = cmd_getarg(cmd, 0);
        offset = atoi(cmd_getarg(cmd, 1));
        size = atoi(cmd_getarg(cmd, 2));
    }
    else {
	return ui_showusage(cmd);
    }

    if (ptable.part[PTABLE_MAX_PARTITIONS-1].size != 0) {
        return ui_showerror(CFE_ERR_INV_PARAM, 
                            "Partition table is full");
    }

    for (i = 0; i < PTABLE_MAX_PARTITIONS; i++) {
        if (strcmp(ptable.part[i].name, name) == 0) {
            return ui_showerror(CFE_ERR_INV_PARAM, 
                                "Partition name already exists");
        }
    }

    if (size == 0) {
	return ui_showerror(CFE_ERR_INV_PARAM, 
                            "Size cannot be zero");
    }

    for (i = 0, overlap = 0; i < PTABLE_MAX_PARTITIONS; i++) {
        if (offset < ptable.part[i].offset) {
            if (offset+size > ptable.part[i].offset) {
                overlap = 1;
            }
            break;
        }
        if (ptable.part[i].offset + ptable.part[i].size > offset) {
            overlap = 1;
            break;
        }
        if (ptable.part[i].size == 0) {
            break;
        }
    }

    if (overlap) {
        return ui_showerror(CFE_ERR_INV_PARAM, 
                            "Partition overlaps existing partition");
    }

    for (j = PTABLE_MAX_PARTITIONS-1; j >= i+1; j--) {
        ptable.part[j] = ptable.part[j-1];
    }

    strncpy(ptable.part[i].name, name, sizeof(ptable.part[i].name)-1);
    ptable.part[i].offset = offset;
    ptable.part[i].size = size;

    return 0;
}

static int ui_cmd_ptable_del(ui_cmdline_t *cmd, int argc, char *argv[])
{
    int i, j;
    char *name;

    if (argc == 1) {
        name = cmd_getarg(cmd, 0);
    }
    else {
	return ui_showusage(cmd);
    }

    for (i = 0; i < PTABLE_MAX_PARTITIONS; i++) {
        if (strcmp(ptable.part[i].name, name) == 0) {
            for (j = i; j < PTABLE_MAX_PARTITIONS-1; j++) {
                ptable.part[j] = ptable.part[j+1];
            }
            return 0;
        }
    }

    return ui_showerror(CFE_ERR_INV_PARAM, 
                        "Partition not found");
}

static int ui_cmd_ptable_clear(ui_cmdline_t *cmd, int argc, char *argv[])
{
    memset(&ptable, 0, sizeof(ptable));

    return 0;
}

static int ui_cmd_ptable_save(ui_cmdline_t *cmd, int argc, char *argv[])
{
    int ret;

    if ((ret = ptable_write(&ptable)) != 0) {
        return ret;
    }

    return 0;
}


int ui_init_ptablecmds(void)
{
    int ret;

    cmd_addcmd("ptable show",
	       ui_cmd_ptable_show,
	       NULL,
	       "Show flash partition table.",
	       "ptable show\n\n"
	       "Show flash partition table.",
	       "");

    cmd_addcmd("ptable add",
	       ui_cmd_ptable_add,
	       NULL,
	       "Delete flash partition.",
	       "ptable add <name> <offset> <size>\n\n"
	       "Add partition to flash partition table.",
	       "");

    cmd_addcmd("ptable del",
	       ui_cmd_ptable_del,
	       NULL,
	       "Delete flash partition table.",
	       "ptable del <name>\n\n"
	       "Remove partition from flash partition table.",
	       "");

    cmd_addcmd("ptable clear",
	       ui_cmd_ptable_clear,
	       NULL,
	       "Clear flash partition table.",
	       "ptable show\n\n"
	       "Remove all partitions from flash partition table.",
	       "");

    cmd_addcmd("ptable save",
	       ui_cmd_ptable_save,
	       NULL,
	       "Save flash partition table.",
	       "ptable save\n\n"
	       "Save flash partition table.\n"
	       "NOTE: You must reboot the system for changes to take effect.",
	       "");

    if ((ret = ptable_read(&ptable)) != 0) {
        memset(&ptable, 0, sizeof(ptable));
        return ret;
    }

    return 0;
}
