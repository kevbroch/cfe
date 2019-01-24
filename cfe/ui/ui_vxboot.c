/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  VxWorks Boot Commands			File: ui_vxboot.c
    *  
    *  Commands useful to people booting VxWorks
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

#include "ui_command.h"

#include "cfe_fileops.h"
#include "cfe_loader.h"
#include "cfe_bootblock.h"
#include "cfe_boot.h"

#include "env_subr.h"

#include "net_ebuf.h"
#include "net_ether.h"
#include "net_enet.h"
#include "net_ip.h"
#include "net_api.h"

#include "addrspace.h"

typedef struct vxflash_s {
    char *device;
    char *filesys;
} vxflash_t;

static vxflash_t vxflash_default[] = {
    { "flash3.vxworks", "fat" },
    { "flash3.fatfs",   "fat" },
    { "flash1.user",    "fat" },
    { "flash0.os",      "fat" },
    { NULL,NULL }
};

/* Magic address (usually in NVRAM) where to store MAC address */
char *vxboot_mac_addr;
char *vxboot_mac_env = "ETH0_HWADDR";

/*
 * XXX should probably go somewhere else 
 */

#define BOOT_LINE_ADDRS_PPC	0x4200
#define BOOT_LINE_ADDRS_MIPS	0x700

int ui_init_vxbootcmds(void);
static int ui_cmd_vxboot_M(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_vxboot_p(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_vxboot_c(ui_cmdline_t *cmd,int argc,char *argv[]);
static int ui_cmd_vxboot_go(ui_cmdline_t *cmd,int argc,char *argv[]);

extern cfe_loadargs_t cfe_loadargs;

typedef struct vxkey_s {
    char *name;
    char *description;
    int num;
} vxkey_t;

typedef struct vxval_s {
    char *name;
    char *val;
} vxval_t;

#define MAXVXKEYS 16
typedef struct vxbootline_s {
    char *bootline;		/* strdup of entire bootline */
    vxval_t vals[MAXVXKEYS+1];	/* Stuff we parsed */
} vxbootline_t;


static char *vxboot_envname = "vxboot";

static vxkey_t validvxkeys[] = {
    /* These keys go in the first token of the boot line */
    {"!bd","boot device"},
    {"!un","unit number"},
    {"!pn","processor number"},
    {"!hn","host name"},
    {"!fn","file name"},

    /* These keys go on the rest of the boot line */
    {"e","inet on ethernet (e)"},
    {"h","host inet (h)"},
    {"g","gateway inet (g)"},
    {"u","user (u)"},
    {"pw","ftp password (pw)"},
    {"f","flags (f)"},
    {"tn","target name (tn)"},
    {"s","startup script (s)"},
    {"o","other (o)"},
    {NULL,NULL,0}};


#define VXFS_FLASH	0
#define VXFS_ETH	1

static vxkey_t *vxboot_findkey(char *name)
{
    vxkey_t *key = validvxkeys;

    while (key->description) {
	if (key->name && strcmp(name,key->name) == 0) return key;
	key++;
	}

    return NULL;
}

static vxval_t *vxboot_findval(vxbootline_t *info,char *name)
{
    vxval_t *val = info->vals;

    while (val->name) {
	if (strcmp(name,val->name) == 0) return val;
	val++;
	}

    return NULL;
}

static char *vxboot_getval(vxbootline_t *info,char *name,char *def)
{
    vxval_t *val = vxboot_findval(info,name);
    if (!val) return def;
    return val->val;
}

static int vxboot_init(vxbootline_t *info,char *str)
{
    memset(info,0,sizeof(vxbootline_t));
    info->bootline = lib_strdup(str);

    if (!info->bootline) return -1;
    return 0;
}

static void vxboot_free(vxbootline_t *info)
{
    vxval_t *val;

    if (info->bootline) {
	KFREE(info->bootline);
	}

    val = info->vals;
    while (val->name) {
	if (val->val) KFREE(val->val);
	val++;
	}
    memset(info,0,sizeof(vxbootline_t));
}

static int vxboot_addkey(vxbootline_t *info,char *name,char *val)
{
    int idx;

    for (idx = 0; idx < MAXVXKEYS; idx++) {
	if (!info->vals[idx].name) break;
	if (strcmp(info->vals[idx].name,name) == 0) break;
	}
    if (idx == MAXVXKEYS) return -1;	/* no more room */

    info->vals[idx].name = name;
    if (info->vals[idx].val) KFREE(info->vals[idx].val);
    info->vals[idx].val = lib_strdup(val);
    return 0;
}


static int vxboot_parse(vxbootline_t *info)
{
    char *x,*y;
    char *ptr;
    char *tok;

    ptr = info->bootline;		/* start hacking it up */

    tok = lib_gettoken(&ptr);		/* first token is the boot file */

    if (!tok) return 0;			/* an empty booline is ok */

    /*
     * Format of the first token is as follows:
     *
     *	   bootdev(unit,proc)host:filename
     */


    x = tok;

    /* find end of boot device name */
    while (*tok && (*tok != '(')) tok++;
    if (!*tok) goto error;
    *tok++ = '\0';

    vxboot_addkey(info,"!bd",x);		/* add boot device */

    x = strchr(tok,')');
    if (!x) goto error;		/* no close parens */
    *x++ = '\0';

    /* Okay, 'tok' now points at the stuff between the parens. */
    if ((y = strchr(tok,','))) {
	*y++ = '\0';
	vxboot_addkey(info,"!un",tok);
	vxboot_addkey(info,"!pn",y);
	}
    else {
	vxboot_addkey(info,"!un",tok);
	vxboot_addkey(info,"!pn","0");
	}
    tok = x;

    /* 'tok' now points at the stuff just beyond the right paren */

    if ((x = strchr(tok,':'))) {
	*x++ = '\0';
	vxboot_addkey(info,"!hn",tok);
	vxboot_addkey(info,"!fn",x);
	}
    else {
	goto error;
	}

    /* Now parse the rest of the bootline params */

    while ((tok = gettoken(&ptr))) {
	if ((x = strchr(tok,'='))) {
	    *x++ = '\0';
	    if (vxboot_addkey(info,tok,x) < 0) break;
	    }
	}

    return 0;

error:
    return -1;
}


static void vxboot_show(vxbootline_t *info)
{
    vxval_t *val;
    vxkey_t *key;
    char buf[50];

    val = info->vals;

    while (val->name) {
	key = vxboot_findkey(val->name);
	if (key) {
	    printf("%-20s : %s\n",key->description,val->val);
	    }
	else {
	    sprintf(buf,"value '%s'",val->name);
	    printf("%20s : %s\n",buf,val->val);
	    }
	val++;
	}
}

static int vxboot_genbootline(vxbootline_t *info)
{
    char buffer[1000];
    char *p;
    char *unk = "unknown";
    vxval_t *val;

    /* Regenerate boot line */
    if (info->bootline) KFREE(info->bootline);
    info->bootline = NULL;

    p = buffer;
    p += sprintf(p,"%s(%s,%s)%s:%s",
		 vxboot_getval(info,"!bd",unk),
		 vxboot_getval(info,"!un",unk),
		 vxboot_getval(info,"!pn",unk),
		 vxboot_getval(info,"!hn",unk),
		 vxboot_getval(info,"!fn",unk));

    val = info->vals;

    while (val->name) {
	if ((val->name[0] != '!') && (val->val[0])) {	/* names starting with ! are special */
	    p += sprintf(p," %s=%s",val->name,val->val);
	    }
	val++;
	}

    info->bootline = lib_strdup(buffer);
    
    return 0;
}

static int vxboot_net_config(vxbootline_t *info)
{
    uint8_t netmask[IP_ADDR_LEN];
    char *p, *ipstr, tmp;
    char netcmd[128];
    int i;

    if ((p = vxboot_getval(info,"e",NULL)) == NULL) {
        return -1;
        }
    if ((ipstr = strdup(p)) == NULL) {
        return -1;
        }

    /* Extract network mask */
    memset(netmask,0,sizeof(netmask));
    if ((p = strchr(ipstr,':')) != NULL) {
        *p++ = 0;
        if (strlen(p) == 8) {
            for (i = 0; i < IP_ADDR_LEN; i++) {
                tmp = p[2];
                p[2] = 0;
                netmask[i] = lib_xtoi(p);
                p[2] = tmp;
                p += 2;
                }
            }
        }

    /* Create command line */
    sprintf(netcmd,"ifconfig eth0 -addr=%s -mask=%d.%d.%d.%d",ipstr,
            netmask[0],netmask[1],netmask[2],netmask[3]);
    if ((p = vxboot_getval(info,"g",NULL)) != NULL) {
        sprintf(&netcmd[strlen(netcmd)]," -gw=%s",p);
        }

    /* Run through standard command line parser */
    ui_docommand(netcmd);

    KFREE(ipstr);
    return 0;
}

static int ui_cmd_vxboot_M(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    uint8_t hwaddr[ENET_ADDR_LEN];

    if ((x = cmd_getarg(cmd,0)) != NULL) {
        if (strlen(x) == 17 && enet_parse_hwaddr(x,hwaddr) == 0) {
            env_setenv(vxboot_mac_env,x,ENV_FLG_NORMAL);
            }
        else {
            printf("MAC address syntax error\n");
            return -1;
            }
        }
    else if ((x = env_getenv(vxboot_mac_env)) != NULL) {
	printf("MAC address: %s\n",x);
        }
    else {
	printf("MAC address not configured\n");
        }

    return 0;
}

static int ui_cmd_vxboot_p(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    vxbootline_t info;

    x = env_getenv(vxboot_envname);
    if (!x) x = "";

    vxboot_init(&info,x);
    if (vxboot_parse(&info) < 0) {
	printf("Warning: Current boot line does not appear to be valid\n");
	}
    vxboot_show(&info);
    vxboot_free(&info);    

    return 0;
}

static int ui_cmd_vxboot_c(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    vxbootline_t info;
    vxval_t *val;
    int keyidx = 0;
    int editing = 1;
    char buffer[500];
    char prompt[80];

    x = env_getenv(vxboot_envname);
    if (!x) x = "";

    vxboot_init(&info,x);
    if (vxboot_parse(&info) < 0) {
	printf("Warning: Current boot line does not appear to be valid\n");
	}

    printf("Type '-' to move to previous field, or press ENTER to accept each value\n");

    while (editing) {
	if (validvxkeys[keyidx].name == NULL) break;
        if (keyidx < 0) {
            vxboot_free(&info);    
            return 0;
            }

	/* Emulate perculiar VxBoot behavior */
	if (strcmp(validvxkeys[keyidx].name,"!un") == 0) {
	    keyidx++;
	    continue;
	    }

	sprintf(prompt,"%20s : ",validvxkeys[keyidx].description);

	buffer[0] = '\0';
	if ((val = vxboot_findval(&info,validvxkeys[keyidx].name))) {
	    strcpy(buffer,val->val);
	    }

	/* Emulate perculiar VxBoot behavior */
	if (strcmp(validvxkeys[keyidx].name,"!bd") == 0 &&
            memcmp(buffer,"flash",5) != 0) {
	    strcat(buffer,vxboot_getval(&info,"!un","0"));
	    }

	console_readline_default(prompt,buffer,sizeof(buffer));
	if (buffer[0] && buffer[strlen(buffer)-1] == '-') {
	    keyidx--;

            /* Emulate perculiar VxBoot behavior */
            if (keyidx >= 0 && strcmp(validvxkeys[keyidx].name,"!un") == 0) {
                keyidx--;
            }

	    continue;
	    }

        /* Emulate perculiar VxBoot behavior */
        if (strcmp(validvxkeys[keyidx].name,"!bd") == 0 && buffer[0]) {
            x = &buffer[strlen(buffer)-1];
            if (*x < '0' || *x > '9') {
                vxboot_addkey(&info,"!un","0");
                }
            else {
                vxboot_addkey(&info,"!un",x);
                *x = 0;
                }
            }

        vxboot_addkey(&info,validvxkeys[keyidx].name,buffer);
        keyidx++;
        }


    vxboot_genbootline(&info);

    env_setenv(vxboot_envname,info.bootline,ENV_FLG_NORMAL);

    vxboot_free(&info);    

    env_save();

    return 0;
}


static int ui_cmd_vxboot_go(ui_cmdline_t *cmd,int argc,char *argv[])
{
    char *x;
    char *f;
    char *p0, *p1;
    vxbootline_t info;
    char filename[256];
    char devname[32];
    cfe_loadargs_t *la = &cfe_loadargs;
    vxflash_t *vxflash;
    fileio_ctx_t *fsctx;
    int res;

    x = env_getenv(vxboot_envname);
    if (!x) {
	printf("VxWorks boot line has not been set.  Use the 'c' command to create a boot line\n");
	return -1;
	}

    vxboot_init(&info,x);
    if (vxboot_parse(&info) < 0) {
	printf("Error: not a valid VxWorks boot line: %s.\n Use the 'c' command to edit the boot line\n",x);
	}

    /* Copy boot line to the magic place */
    p0 = NULL;
    if (strcmp(CPUCFG_ARCHNAME, "PPC") == 0) {
        p0 = (char *) KERNADDR(BOOT_LINE_ADDRS_PPC);
        strcpy(p0,x);
        } 
    else if (strcmp(CPUCFG_ARCHNAME, "MIPS") == 0) {
        p0 = (char *) KERNADDR(BOOT_LINE_ADDRS_MIPS);
        strcpy(p0,x);
        } 
    else {
	printf("Error: unsupported architecture for VxWorks\n");
	}
    /* Add MAC addres to 'other' */
    if (p0 != NULL) {
        if ((p1 = env_getenv(vxboot_mac_env)) != NULL && strlen(p1) == 17) {
            if (vxboot_findval(&info,"o") == NULL) {
                strcat(p0," o=");
                }
            strcat(p0,";mac=");
            strcat(p0,p1);
            /* MAC address separator must be colon (:) */
            p1 = &p0[strlen(p0)-17];
            p1[2] = p1[5] = p1[8] = p1[11] = p1[14] = ':';
            }
	}
    /* Also store MAC in magic place if provided */
    if (vxboot_mac_addr != NULL) {
        if ((p1 = env_getenv(vxboot_mac_env)) != NULL) {
            enet_parse_hwaddr(p1,(uint8_t *)vxboot_mac_addr);
            }
        }

    /* Load the image. */
    la->la_filesys = NULL;
    la->la_loader = "elf";
    la->la_options = NULL;
    la->la_flags = LOADFLG_NOISY | LOADFLG_EXECUTE;

    x = vxboot_getval(&info,"!bd",NULL);
    if (!x) goto error;
    if (strcmp(x,"sbe") == 0 || strcmp(x,"bc") == 0 || memcmp(x,"et", 2) == 0) {
	la->la_device = "eth0";
	la->la_filesys = "tftp";
	f = vxboot_getval(&info,"h",NULL);
	if (!f) f = vxboot_getval(&info,"!hn","");
	sprintf(filename,"%s:%s",f,vxboot_getval(&info,"!fn",""));
	}
    else if (strcmp(x,"flash") == 0) {
        vxflash = &vxflash_default[0];
        while (vxflash->device) {
            la->la_device = vxflash->device;
            la->la_filesys = vxflash->filesys;
            if (fs_init(la->la_filesys,&fsctx,la->la_device) == CFE_OK) {
                fs_uninit(fsctx);
                break;
                }
            vxflash++;
            }
	sprintf(filename,"%s",vxboot_getval(&info,"!fn",""));
	}
    else if (memcmp(x,"flash",5) == 0) {
        strncpy(devname,x,sizeof(devname)-1);
        devname[sizeof(devname)-1] = 0;
	la->la_device = devname;
	la->la_filesys = "fat";
	sprintf(filename,"%s",vxboot_getval(&info,"!fn",""));
	}

    if (vxboot_getval(&info,"e",NULL) != NULL) {
        vxboot_net_config(&info);
        }
    
    la->la_filename = filename;
#if CFG_ZLIB
    if (strlen(filename) > 3 && 
        strcmp(&filename[strlen(filename)-3], ".gz") == 0) {
        printf("Assuming compressed image\n");
        la->la_flags |= LOADFLG_COMPRESSED;
	}
#endif
    vxboot_free(&info);    

    /* Call the loader. */
    /* Run the image. */
    res = cfe_boot(la->la_loader,la);

    if (res < 0) ui_showerror(res,"Could not load image %s",filename);

    /* Should not return here. */

    return res;

error:
    printf("Incorrect or invalid fields in VxWorks boot line\n");
    vxboot_free(&info);    
    return -1;
}





int ui_init_vxbootcmds(void)
{
    cmd_addcmd("M",
	       ui_cmd_vxboot_M,
	       NULL,
	       "M [xx:xx:xx:xx:xx:xx]\n\n"
               "Show or set Ethernet MAC address",
	       "This command reads or writes the environment variable 'ETH0_HWADDR',\n"
	       "and is provided for VxWorks boot loader compatibility.",
	       "");

    cmd_addcmd("p",
	       ui_cmd_vxboot_p,
	       NULL,
	       "Parse and display VxWorks boot string",
	       "This command reads the environent variable 'vxboot', if defined, and\n"
	       "breaks out the fields for easy display.  You can change the vxworks boot\n"
	       "string with the 'c' command.",
	       "");

    cmd_addcmd("c",
	       ui_cmd_vxboot_c,
	       NULL,
	       "Change the VxWorks boot string",
	       "This command changes the environent variable 'vxboot', prompting for\n"
	       "the fields.  The 'vxboot' string is committed to NVRAM.",
	       "");

    cmd_addcmd("@",
	       ui_cmd_vxboot_go,
	       NULL,
	       "Boot VxWorks",
	       "This command copies the 'vxboot' string to the architecture-dependent area\n"
	       "and transfers control to VxWorks.",
	       "");

    return 0;
}


