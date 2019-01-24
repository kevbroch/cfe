/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  ELF Program Loader			File: zipstart_load.c
    *  
    *  This program parses ELF executables and loads them into memory.
    *  
    *  Author:  Mitch Lichtenberg (mpl@broadcom.com)
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
#include "elf.h"


/*  *********************************************************************
    *  Types
    ********************************************************************* */

int zipstart_elfload(void *file,int flags,long *entrypt);

extern void puthex(uint32_t x);
extern void putdec(uint32_t x);
extern void putstr(const char *x);

extern void *zs_malloc(int size);
extern void zs_free(void *);
#define KMALLOC(a,b) zs_malloc(a)
#define KFREE(a) zs_free(a)

typedef struct fileio_ctx_s {
    uint8_t *beginning;
    uint8_t *ptr;
} fileio_ctx_t;

#define FILE_SEEK_BEGINNING 0

static int fs_init(fileio_ctx_t *fsctx,uint8_t *file)
{
    fsctx->beginning = file;
    fsctx->ptr = file;
    return 0;
}

static int fs_seek(fileio_ctx_t *fsctx,int off,int how)
{
    fsctx->ptr = fsctx->beginning + off;

    return off;
}

static int fs_read(fileio_ctx_t *fsctx,uint8_t *dest,int cnt) 
{
    int c = cnt;
    uint8_t *sptr = fsctx->ptr;

    while (c > 0) {
	*dest++ = *sptr++;
	c--;
	}

    fsctx->ptr += cnt;
    return cnt;
}


/*  *********************************************************************
    *  readprogsegment(fsctx,addr,size)
    *  
    *  Read a program segment, generally corresponding to one
    *  section of the file.
    *  
    *  Input parameters: 
    *  	   fsctx - file I/O dispatch
    *  	   addr - target virtual address
    *  	   size - size of region to read
    *  	   
    *  Return value:
    *  	   Number of bytes copied or <0 if error occured
    ********************************************************************* */

static int readprogsegment(fileio_ctx_t *fsctx,
			   void *addr,int size,int flags)
{
    int res;

    puthex((uint32_t)addr);
    putstr("/");
    putdec(size);
    putstr(" ");

    res = fs_read(fsctx,addr,size);

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

static int readclearbss(void *addr,int size,int flags)
{

    puthex((uint32_t)addr);
    putstr("/");
    putdec(size);
    putstr(" ");

    if (size > 0) {
	uint8_t *dptr = addr;
	int c = size;

	while (c > 0) {
	    *dptr++ = 0;
	    c--;
	    }
	}
    return size;
}


/*  *********************************************************************
    *  elfgetshdr(ops,ep)
    *  
    *  Get a section header from the ELF file
    *  
    *  Input parameters: 
    *  	   ops - file I/O dispatch
    *  	   ep - extended header info
    *  	   
    *  Return value:
    *  	   copy of section header (malloc'd) or NULL if no memory
    ********************************************************************* */

static Elf32_Shdr *elfgetshdr(fileio_ctx_t *fsctx,Elf32_Ehdr *ep)
{
    Elf32_Shdr *shtab;
    unsigned size = ep->e_shnum * sizeof(Elf32_Shdr);

    shtab = (Elf32_Shdr *) KMALLOC(size,0);
    if (!shtab) {
	return NULL;
	}

    if (fs_seek(fsctx,ep->e_shoff,FILE_SEEK_BEGINNING) != ep->e_shoff ||
	fs_read(fsctx,(uint8_t *)shtab,size) != size) {
	KFREE(shtab);
	return NULL;
	}

    return (shtab);
}

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

static int elfload_internal(fileio_ctx_t *fsctx,
			    long *entrypt,int flags)
{
    Elf32_Ehdr *ep;
    Elf32_Phdr *phtab = 0;
    Elf32_Shdr *shtab = 0;
    unsigned int nbytes;
    int i;
    int res;
    Elf32_Ehdr ehdr;

    ep = &ehdr;
    if (fs_read(fsctx,(uint8_t *) ep,sizeof(*ep)) != sizeof(*ep)) {
	return CFE_ERR_IOERR;
	}

    /* check header validity */
    if (ep->e_ident[EI_MAG0] != ELFMAG0 ||
	ep->e_ident[EI_MAG1] != ELFMAG1 ||
	ep->e_ident[EI_MAG2] != ELFMAG2 ||
	ep->e_ident[EI_MAG3] != ELFMAG3) {
	return CFE_ERR_NOTELF;
	}

    if (ep->e_ident[EI_CLASS] != ELFCLASS32) return CFE_ERR_NOT32BIT;

#ifdef __MIPSEB
    if (ep->e_ident[EI_DATA] != ELFDATA2MSB) return CFE_ERR_WRONGENDIAN;	/* big endian */
#endif
#ifdef __MIPSEL
    if (ep->e_ident[EI_DATA] != ELFDATA2LSB) return CFE_ERR_WRONGENDIAN;	/* little endian */
#endif

    if (ep->e_ident[EI_VERSION] != EV_CURRENT) return CFE_ERR_BADELFVERS;
    if (ep->e_machine != EM_MIPS) return CFE_ERR_NOTMIPS;
	
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

    if (fs_seek(fsctx,ep->e_phoff,FILE_SEEK_BEGINNING) != ep->e_phoff || 
	fs_read(fsctx,(uint8_t *)phtab,nbytes) != nbytes) {
	KFREE(phtab);
	return CFE_ERR_IOERR;
	}

    /*
     * From now on we've got no guarantee about the file order, 
     * even where the section header is.  Hopefully most linkers
     * will put the section header after the program header, when
     * they know that the executable is not demand paged.  We assume
     * that the symbol and string tables always follow the program 
     * segments.
     */

    /* read section table (if before first program segment) */
    if (ep->e_shoff < phtab[0].p_offset) {
	shtab = elfgetshdr(fsctx,ep);
	}

    /* load program segments */
    /* We cope with a badly sorted program header, as produced by 
     * older versions of the GNU linker, by loading the segments
     * in file offset order, not in program header order. */

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
	    if (fs_seek(fsctx,ph->p_offset,FILE_SEEK_BEGINNING) != ph->p_offset) {
		if (shtab) KFREE(shtab);
		KFREE(phtab);
		return CFE_ERR_BADELFFMT;
		}
	    res = readprogsegment(fsctx,
				  (void *)(intptr_t)(signed)ph->p_vaddr, 
				  ph->p_filesz,flags);
	    if (res != ph->p_filesz) {
		if (shtab) KFREE(shtab);
		KFREE(phtab);
		return res;
		}
	    }

	if (ph->p_filesz < ph->p_memsz) {
	    res = readclearbss((void *)(intptr_t)(signed)ph->p_vaddr + ph->p_filesz, 
			       ph->p_memsz - ph->p_filesz,flags);
	    if (res < 0) {
		if (shtab) KFREE(shtab);
		KFREE(phtab);
		return res;
		}
	    }

	ph->p_type = PT_NULL; /* remove from consideration */
	}

    KFREE(phtab);

    *entrypt = (intptr_t)(signed)ep->e_entry;		/* return entry point */
    return 0;
}



/*  *********************************************************************
    *  zipstart_elfload(file,flags)
    *  
    *  Read an ELF file (main entry point)
    *  
    *  Input parameters: 
    *  	   file - pointer to ELF image to load
    *      flags - load flags
    *  	   
    *  Return value:
    *  	   0 if ok, else error code
    ********************************************************************* */

int zipstart_elfload(void *file,int flags,long *entrypt)
{
    int res;
    fileio_ctx_t fsctx;

    fs_init(&fsctx,file);

#if 0
    /*
     * Turn on compression if we're doing that.
     */

    if (la->la_flags & LOADFLG_COMPRESSED) {
	res = fs_hook(&fsctx,"z");
	if (res != 0) {
	    return res;
	    }
	}
#endif

    /*
     * Load the image.
     */

    res = elfload_internal(&fsctx,entrypt,flags);

    return res;
}


