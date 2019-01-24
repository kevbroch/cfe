/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  NVRAM Environment commands		     File: ui_nvramcmds.c
    *  
    *  User interface for environment variables
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


#include "cfe.h"
#include "env_subr.h"
#include "bcmnvram.h"
#include "ui_command.h"

static nvram_import_t nvram_import_default[] = {
    { "LINUX_CMDLINE",  "kernel_args",  NULL },
    { NULL, NULL }
};

nvram_import_t *nvram_import_data = nvram_import_default;

int ui_init_nvramcmds(void);
static int ui_cmd_nvram_get(ui_cmdline_t *cmd, int argc, char *argv[]);
static int ui_cmd_nvram_set(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_nvram_unset(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_nvram_show(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_nvram_commit(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_nvram_erase(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_nvram_import(ui_cmdline_t *cmd,int argc,char *argv[]);

static int ui_cmd_nvram_get(ui_cmdline_t *cmd, int argc, char *argv[])
{
    const char *name;
    const char *value;

    name = cmd_getarg(cmd, 0);

    if (!name) {
	return ui_showusage(cmd);
	}

    value = nvram_get(name);
    if (value) {
	printf("%s\n", value);
	}
    else {
	return ui_showerror(-1, "Could not get nvram variable '%s'", name);
	}

    return 0;
}


static int ui_cmd_nvram_set(ui_cmdline_t *cmd, int argc, char *argv[])
{
    char *name, *eq, *value;
    int res;
   
    if (argc == 2) {
        name = cmd_getarg(cmd, 0);
	eq = "=";
        value = cmd_getarg(cmd, 1);
	}
    else if (argc == 3) {
        name = cmd_getarg(cmd, 0);
	eq = cmd_getarg(cmd, 1);
        value = cmd_getarg(cmd, 2);
	}
    else {
	return ui_showusage(cmd);
	}
    if (!name || !value || !eq || strcmp(eq, "=") != 0) {
	return ui_showusage(cmd);
	}

    res = nvram_set(name, value);
    if (res != 0) {
	return ui_showerror(res, "Could not set nvram variable '%s'", name);
	}

    return 0;
}


static int ui_cmd_nvram_unset(ui_cmdline_t *cmd, int argc, char *argv[])
{
    char *name;
    int res;

    name = cmd_getarg(cmd,0);

    if (!name) {
	return ui_showusage(cmd);
	}

    res = nvram_unset(name);
    if (res != 0) {
	return ui_showerror(res, "Could not delete nvram variable '%s'", name);
	}

    return 0;
}


static int nvram_show_one(const char *tuple)
{
    xprintf("  %s\n", tuple);
    return 1;
}

static int ui_cmd_nvram_show(ui_cmdline_t *cmd, int argc, char *argv[])
{
    nvram_enum(&nvram_show_one);
    return 0;
}


static int ui_cmd_nvram_commit(ui_cmdline_t *cmd, int argc, char *argv[])
{
    int nvram_handle;
    int res;

    nvram_handle = cfe_open("flash1.nvram");
    if (nvram_handle < 0)
	return CFE_ERR_DEVNOTFOUND;

    res = nvram_commit(nvram_handle);
    cfe_close(nvram_handle);

    return res;
}


static int ui_cmd_nvram_erase(ui_cmdline_t *cmd, int argc, char *argv[])
{
    int nvram_handle;
    int res;

    nvram_handle = cfe_open("flash0.nvram");
    if (nvram_handle < 0)
	return CFE_ERR_DEVNOTFOUND;

    /* XXX Write NVRAM_SPACE of 0xFF ? */
    res = -1;

    return res;
}


static int ui_cmd_nvram_import(ui_cmdline_t *cmd, int argc, char *argv[])
{
    char *value;
    nvram_import_t *e2n = nvram_import_data;

    while (e2n && e2n->envname) {
        if ((value = env_getenv(e2n->envname)) != NULL) {
            nvram_set(e2n->nvname, value);
        }
        else if (nvram_get(e2n->nvname) == NULL && e2n->def != NULL) {
            nvram_set(e2n->nvname, e2n->def);
        }
        e2n++;
    }

    return 0;
}


int ui_init_nvramcmds(void)
{
    /* Note: HND code has a single command "nvram" with subcommand
       selected by the first argument.  Better? */

    cmd_addcmd("nvram get",
	       ui_cmd_nvram_get,
	       NULL,
	       "Get the value of an nvram variable.",
	       "nvram get <variable>\n\n"
	       "Get the current value of an nvram variable.",
	       "");

    cmd_addcmd("nvram set",
	       ui_cmd_nvram_set,
	       NULL,
	       "Set the value of an nvram variable.",
	       "nvram set <variable>=<value>\n\n"
	       "Set the value of an nvram variable.\n"
	       "(This session only until commit)",
	       "");

    cmd_addcmd("nvram unset",
	       ui_cmd_nvram_unset,
	       NULL,
	       "Delete an nvram variable.",
	       "nvram unsetenv <variable>\n\n"
	       "Delete an nvram variable and its value from memory.\n"
	       "(This session only until commit)",
	       "");

    cmd_addcmd("nvram show",
	       ui_cmd_nvram_show,
	       NULL,
	       "Show all nvram variables.",
	       "nvram show\n\n"
	       "Display the names and values of all defined nvram variables.",
	       "");

    cmd_addcmd("nvram commit",
	       ui_cmd_nvram_commit,
	       NULL,
	       "Commit nvram variable bindings.",
	       "nvram commit\n\n"
	       "Commit the current nvram variables and values to "
	       "non-volatile memory.",
	       "");

    cmd_addcmd("nvram erase",
	       ui_cmd_nvram_erase,
	       NULL,
	       "Delete all nvram variables.",
	       "nvram erase\n\n"
	       "This command deletes all nvram variables from both memory "
	       "and non-volatile memory.",
	       "");

    cmd_addcmd("nvram import",
	       ui_cmd_nvram_import,
	       NULL,
	       "Import nvram variables from standard environment.",
	       "nvram import\n\n"
	       "This command converts standard environment variables "
	       "into the corresponding nvram variables.",
	       "");

    return 0;
}
