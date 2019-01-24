/*
 * This program will take an image and make a 32K page aligned file
 * from the given source file so that we can write the EDC syndrom
 * bytes to the DiskOnChip device using the M-Systems BDK.
 *
 * $Id: docprep.c,v 1.1 2003/05/23 17:19:27 mpl Exp $
 *
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define KBYTE    1024
#define BOUNDARY (32*KBYTE)


int
main(int argc, char *argv[])
{
    int fh;    
    unsigned char* image;
    unsigned char* padding;
    
    int totalsize = 0;
    int pa, npages, padbytes;


    if(argc != 3){

	printf("Usage: %s <infile> <outfile>\n",
	       argv[0]);
	exit(1);
	    
    }

    fh = open(argv[1],O_RDONLY);
    if (fh < 0) {
	perror(argv[1]);
    }
    
    totalsize = lseek(fh,0L,SEEK_END);
    lseek(fh,0L,SEEK_SET);

    image = (unsigned char*)malloc(totalsize);
    if (image == NULL) {
	perror("malloc");
	exit(1);
    }

    if (read(fh,image, totalsize) != totalsize) {
	perror("read");
	exit(1);
    }

    close(fh);

    /*
     * Now write the output file
     */

    fh = open(argv[2],O_RDWR |O_CREAT |O_TRUNC,
	      S_IREAD|S_IWRITE|S_IRGRP|S_IWGRP|S_IROTH);
    if  (fh < 0) {
	perror(argv[2]);
	exit(1);
    }

    if (write(fh,image,totalsize) != totalsize) {
	perror(argv[2]);
	exit(1);
    }

    /* Workaround DiskOnChip (TM) binary partition loader page
     * write issue. In order for DOC to compute ECC/EDC, you need
     * to always write a page (32K) of data (minimum). If the
     * image is not page aligned, we simply write N bytes of zeros
     * at the end so that the DOC ASIC controller will correctly
     * compute the ECC syndrome bytes and all will be good.
     * Note that the PPCBoot loader will never read these bytes,
     * we just put them there to keep the DOC asic controller happy.
     * 
     */
        
    pa = (totalsize % BOUNDARY) == 0; /* True if 32K aligned */
    npages = (totalsize / BOUNDARY) + (totalsize % BOUNDARY > 0);
    padbytes = (npages*BOUNDARY - totalsize);

    if(!pa){
	padding = (char*)malloc(padbytes);
	memset(padding, 0x0, padbytes);
	printf("Total Size: %d bytes (%d bytes aligned [%x]),"
	       "%d bytes pad)\n",
	       totalsize + padbytes,
	       npages*BOUNDARY,
	       npages*BOUNDARY,
	       padbytes);
	
	if (write(fh, padding, padbytes) != padbytes) {
	    perror(argv[2]);
	    exit(2);
	}
	
	free(padding);
	totalsize += padbytes;
    }	
    
    printf("Wrote %d bytes to %s\n",totalsize, argv[2]);
    
    close(fh);

    exit(0);
}

