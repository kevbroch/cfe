/* 
 * $Id: sdram_spd.h,v 1.1.1.1 2005/06/08 15:37:02 mpl Exp $
 * $Copyright: (c) 2005 Broadcom Corp.
 * All Rights Reserved.$
 *
 * File:        sdram_spd.h
 * Purpose:     SPD/Memory Controller interface constants
 */

#ifndef   _SDRAM_SPD_H_
#define   _SDRAM_SPD_H_

#define SPD_MEM_CONF_ERR  0x8000
#define SPD_MEM_CONF_LBNK 0x4000
#define SPD_MEM_CONF_BANK 0x2000
#define SPD_MEM_CONF_ROW  0x1000
#define SPD_MEM_CONF_SZ   0x0f00
#define SPD_MEM_CONF_64   0x0100
#define SPD_MEM_CONF_128  0x0200
#define SPD_MEM_CONF_256  0x0400
#define SPD_MEM_CONF_512  0x0800

#endif /* _SDRAM_SPD_H_ */
