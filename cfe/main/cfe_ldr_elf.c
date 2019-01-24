/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  ELF Program Loader			File: cfe_ldr_elf.c
    *  
    *  This program parses ELF executables and loads them into memory.
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
#include "cfe_mem.h"

#include "cfe_loader.h"
#include "cfe_fileops.h"
#include "elf.h"

#include "cfe_boot.h"


/*  *********************************************************************
    *  Externs
    ********************************************************************* */

static int cfe_elfload(cfe_loadargs_t *la);

const cfe_loader_t elfloader = {
    "elf",
    cfe_elfload,
    0};

/*  *********************************************************************
    *  readprogsegment(fsctx,ref,addr,size)
    *  
    *  Read a program segment, generally corresponding to one
    *  section of the file.
    *  
    *  Input parameters: 
    *  	   fsctx - file I/O dispatch
    *  	   ref - reference data for open file handle
    *  	   addr - target virtual address
    *  	   size - size of region to read
    *  	   
    *  Return value:
    *  	   Number of bytes copied or <0 if error occured
    ********************************************************************* */

static int readprogsegment(fileio_ctx_t *fsctx,void *ref,
			   hsaddr_t addr,int size,int flags)
{
    int res;

#if CPUCFG_REGS64
    if (flags & LOADFLG_NOISY) xprintf("0x%016llx/%d ",addr,size);
#else
    if (flags & LOADFLG_NOISY) xprintf("0x%x/%d ",addr,size);
#endif

    if (!cfe_arena_loadcheck(addr,size)) {
	return CFE_ERR_BADADDR;
	}

    res = fs_read(fsctx,ref,addr,size);

    if (res < 0) return CFE_ERR_IOERR;
    if (res != size) return CFE_ERR_BADELFFMT;
    
    return size;
}


/*  *********************************************************************
    *  readclearbss(addr,size)
    *  
    *  Process a BSS section, zeroing memory corresponding to
    *  the BSS.
    *  
    *  Input parameters: 
    *  	   addr - address to zero
    *  	   size - length of area to zero
    *  	   
    *  Return value:
    *  	   number of zeroed bytes or <0 if error occured
    ********************************************************************* */

static int readclearbss(hsaddr_t addr,int size,int flags)
{

#if CPUCFG_REGS64
    if (flags & LOADFLG_NOISY) xprintf("0x%016llx/%d ",addr,size);
#else
    if (flags & LOADFLG_NOISY) xprintf("0x%x/%d ",addr,size);
#endif

    if (!cfe_arena_loadcheck(addr,size)) {
	return CFE_ERR_BADADDR;
	}

    if (size > 0) hs_memset(addr,0,size);
    return size;
}



/*  *********************************************************************
    *  elfload_internal_32(fsctx,ref,entrypt,flags)
    *  
    *  Read a 32-bit ELF file.
    *  
    *  Input parameters: 
    *  	   ops - file I/O dispatch
    *  	   ref - open file handle
    *  	   entrypt - filled in with entry vector
    *      flags - generic boot flags
    *      ep - ELF32 file header.
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else error code
    ********************************************************************* */
static int elfload_internal_32(fileio_ctx_t *fsctx,void *ref,
			       hsaddr_t *entrypt,int flags,
			       Elf32_Ehdr *ep)
{
    Elf32_Phdr *phtab = 0;
    unsigned int nbytes;
    int i;
    int res;

    /*
     * Read the rest of the private header.   We only loaded enough before to
     * deterine what type of executable it was.
     */

    if (fs_read(fsctx,ref,PTR2HSADDR((((uint8_t *) ep)+EI_MINHDR)),
		(sizeof(Elf32_Ehdr)-EI_MINHDR)) != (sizeof(Elf32_Ehdr)-EI_MINHDR)) {
	return CFE_ERR_IOERR;
	}

    /* Is there a program header? */
    if (ep->e_phoff == 0 || ep->e_phnum == 0 ||
	ep->e_phentsize != sizeof(Elf32_Phdr)) {
	return CFE_ERR_BADELFFMT;
	}

    /* Load program header */
    nbytes = ep->e_phnum * sizeof(Elf32_Phdr);
    phtab = (Elf32_Phdr *) KMALLOC(nbytes,0);
    if (!phtab) {
	return CFE_ERR_NOMEM;
	}

    if (fs_seek(fsctx,ref,ep->e_phoff,FILE_SEEK_BEGINNING) != ep->e_phoff || 
	fs_read(fsctx,ref,PTR2HSADDR(phtab),nbytes) != nbytes) {
	KFREE(phtab);
	return CFE_ERR_IOERR;
	}


    /* 
     * Load program segments.  Do this in file offset order
     * so that we don't have to seek backwards, difficult to do
     * on TFTP streams.
     */

    while (1) {
	Elf32_Off lowest_offset = ~0;
	Elf32_Phdr *ph = 0;

	/* find nearest loadable segment */
	for (i = 0; i < ep->e_phnum; i++)
	    if ((phtab[i].p_type == PT_LOAD) && (phtab[i].p_offset < lowest_offset)) {
		ph = &phtab[i];
		lowest_offset = ph->p_offset;
		}
	if (!ph) {
	    break;		/* none found, finished */
	    }

	/* load the segment */
	if (ph->p_filesz) {
	    if (fs_seek(fsctx,ref,ph->p_offset,FILE_SEEK_BEGINNING) != ph->p_offset) {
		KFREE(phtab);
		return CFE_ERR_BADELFFMT;
		}
	    res = readprogsegment(fsctx,ref,
				  PTR2HSADDR(ph->p_vaddr), 
				  ph->p_filesz,flags);
	    if (res != ph->p_filesz) {
		KFREE(phtab);
		return res;
		}
	    }

	if (ph->p_filesz < ph->p_memsz) {
	    res = readclearbss(PTR2HSADDR(ph->p_vaddr + ph->p_filesz), 
			       ph->p_memsz - ph->p_filesz,flags);
	    if (res < 0) {
		KFREE(phtab);
		return res;
		}
	    }

	ph->p_type = PT_NULL;
	}

    KFREE(phtab);

    *entrypt = (intptr_t)(signed)ep->e_entry;		/* return entry point */

    return 0;
}


#if CPUCFG_REGS64
/*  *********************************************************************
    *  elfload_internal_64(fsctx,ref,entrypt,flags)
    *  
    *  Read a 64-bit ELF file.
    *  
    *  Input parameters: 
    *  	   ops - file I/O dispatch
    *  	   ref - open file handle
    *  	   entrypt - filled in with entry vector
    *      flags - generic boot flags
    *      ep - ELF64 file header.
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else error code
    ********************************************************************* */
static int elfload_internal_64(fileio_ctx_t *fsctx,void *ref,
			       hsaddr_t *entrypt,int flags,
			       Elf64_Ehdr *ep)
{
    Elf64_Phdr *phtab = 0;
    unsigned int nbytes;
    int i;
    int res;

    /*
     * Read the rest of the private header.   We only loaded enough before to
     * deterine what type of executable it was.
     */

    if (fs_read(fsctx,ref,PTR2HSADDR(((uint8_t *) ep)+EI_MINHDR),
		(sizeof(Elf64_Ehdr)-EI_MINHDR)) != (sizeof(Elf64_Ehdr)-EI_MINHDR)) {
	return CFE_ERR_IOERR;
	}

    /* Is there a program header? */
    if (ep->e_phoff == 0 || ep->e_phnum == 0 ||
	ep->e_phentsize != sizeof(Elf64_Phdr)) {
	return CFE_ERR_BADELFFMT;
	}

    /* Load program header */
    nbytes = ep->e_phnum * sizeof(Elf64_Phdr);
    phtab = (Elf64_Phdr *) KMALLOC(nbytes,0);
    if (!phtab) {
	return CFE_ERR_NOMEM;
	}

    if (fs_seek(fsctx,ref,ep->e_phoff,FILE_SEEK_BEGINNING) != ep->e_phoff || 
	fs_read(fsctx,ref,PTR2HSADDR(phtab),nbytes) != nbytes) {
	KFREE(phtab);
	return CFE_ERR_IOERR;
	}


    /* 
     * Load program segments.  Do this in file offset order
     * so that we don't have to seek backwards, difficult to do
     * on TFTP streams.
     */

    while (1) {
	Elf64_Off lowest_offset = ~0;
	Elf64_Phdr *ph = 0;

	/* find nearest loadable segment */
	for (i = 0; i < ep->e_phnum; i++)
	    if ((phtab[i].p_type == PT_LOAD) && (phtab[i].p_offset < lowest_offset)) {
		ph = &phtab[i];
		lowest_offset = ph->p_offset;
		}
	if (!ph) {
	    break;		/* none found, finished */
	    }

	/* load the segment */
	if (ph->p_filesz) {
	    if (fs_seek(fsctx,ref,ph->p_offset,FILE_SEEK_BEGINNING) != ph->p_offset) {
		KFREE(phtab);
		return CFE_ERR_BADELFFMT;
		}
	    /*
	     * Don't use PTR2HSADDR(x) here, since it is only for converting 32-bit pointers
	     * to 64-bit pointers.  We're already guaranteed to be 64 bits here
	     * and below where readclearbss() is called.
	     */
	    res = readprogsegment(fsctx,ref,
				  (hsaddr_t)ph->p_vaddr, 
				  ph->p_filesz,flags);
	    if (res != ph->p_filesz) {
		KFREE(phtab);
		return res;
		}
	    }

	if (ph->p_filesz < ph->p_memsz) {
	    res = readclearbss((hsaddr_t)ph->p_vaddr + ph->p_filesz, 
			       ph->p_memsz - ph->p_filesz,flags);
	    if (res < 0) {
		KFREE(phtab);
		return res;
		}
	    }

	ph->p_type = PT_NULL;
	}

    KFREE(phtab);

    *entrypt = (hsaddr_t)ep->e_entry;		/* return entry point */

    return 0;
}
#endif

/*  *********************************************************************
    *  elfload_internal(ops,ref,entrypt,flags)
    *  
    *  Read an ELF file (main routine)
    *  
    *  Input parameters: 
    *  	   ops - file I/O dispatch
    *  	   ref - open file handle
    *  	   entrypt - filled in with entry vector
    *      flags - generic boot flags
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else error code
    ********************************************************************* */

static int elfload_internal(fileio_ctx_t *fsctx,void *ref,
			    hsaddr_t *entrypt,int flags)
{
    int res;
    Elf32_Ehdr *ep;
    uint8_t ehdr[ELF_EHDRSIZE];

    /*
     * Read enough of the ELF header to determine what type of
     * ELF file this is.
     */

    ep = (Elf32_Ehdr *)(&ehdr[0]);

    if (fs_read(fsctx,ref,PTR2HSADDR(ep),EI_MINHDR) != EI_MINHDR) {
	return CFE_ERR_IOERR;
	}

    /* check header validity */
    if (ep->e_ident[EI_MAG0] != ELFMAG0 ||
	ep->e_ident[EI_MAG1] != ELFMAG1 ||
	ep->e_ident[EI_MAG2] != ELFMAG2 ||
	ep->e_ident[EI_MAG3] != ELFMAG3) {
	return CFE_ERR_NOTELF;
	}

#if (ENDIAN_BIG)
    if (ep->e_ident[EI_DATA] != ELFDATA2MSB) return CFE_ERR_WRONGENDIAN;	/* big endian */
#endif
#if (ENDIAN_LITTLE)
    if (ep->e_ident[EI_DATA] != ELFDATA2LSB) return CFE_ERR_WRONGENDIAN;	/* little endian */
#endif

    if (ep->e_ident[EI_VERSION] != EV_CURRENT) return CFE_ERR_BADELFVERS;

    if (ep->e_machine != CPUCFG_ELFTYPE) return CFE_ERR_NOTMIPS;

    switch (ep->e_ident[EI_CLASS]) {
	case ELFCLASS32:
	    res = elfload_internal_32(fsctx,ref,entrypt,flags,ep);
	    break;
#if CPUCFG_REGS64
	case ELFCLASS64:
	    res = elfload_internal_64(fsctx,ref,entrypt,flags,(Elf64_Ehdr *) ep);
	    break;
#endif
	default:
	    res = CFE_ERR_NOT32BIT;
	}
	
    return res;
}



/*  *********************************************************************
    *  cfe_elfload(ops,file,flags)
    *  
    *  Read an ELF file (main entry point)
    *  
    *  Input parameters: 
    *  	   ops - fileio dispatch
    *  	   file - name of file to read
    *  	   ept - where to put entry point
    *      flags - load flags
    *  	   
    *  Return value:
    *  	   0 if ok, else error code
    ********************************************************************* */

static int cfe_elfload(cfe_loadargs_t *la)
{
    fileio_ctx_t *fsctx;
    void *ref;
    int res;

    /*
     * Look up the file system type and get a context
     */


    res = fs_init(la->la_filesys,&fsctx,la->la_device);
    if (res != 0) {
	return res;
	}

    /*
     * Turn on compression if we're doing that.
     */

    if (la->la_flags & LOADFLG_COMPRESSED) {
	res = fs_hook(fsctx,"z");
	if (res != 0) {
	    return res;
	    }
	}

    /*
     * Open the remote file.  If the remote file is our console,
     * turn off noise since it can interrupt serial port transfers.
     */

    if (strcmp(la->la_filename,console_name) == 0) la->la_flags &= ~LOADFLG_NOISY;

    res = fs_open(fsctx,&ref,la->la_filename,FILE_MODE_READ);
    if (res != 0) {
	fs_uninit(fsctx);
	return CFE_ERR_FILENOTFOUND;
	}

    /*
     * Load the image.
     */

    la->la_entrypt = 0;
    res = elfload_internal(fsctx,ref,&(la->la_entrypt),la->la_flags);

    /*
     * All done, release resources
     */

    fs_close(fsctx,ref);
    fs_uninit(fsctx);

    return res;
}


