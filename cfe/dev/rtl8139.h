/*
 * Register and bit definitions for the Realtek RTL 8139C Single Chip
 *   Fast Ethernet Controller with integrated PHY.
 * Reference:
 *   Realtek 3.3V Single Chip Fast EThernet Controller
 *     with Power Management RTL8139C(L), Rev 1.4
 *   Realtek Semiconductor Corporation, Hsinchu, Taiwan, 2002/01/10.
 */
#ifndef _RTL8139_H_
#define _RTL8139_H_

#define _DD_MAKEMASK1(n) (1 << (n))
#define _DD_MAKEMASK(v,n) ((((1)<<(v))-1) << (n))
#define _DD_MAKEVALUE(v,n) ((v) << (n))
#define _DD_GETVALUE(v,n,m) (((v) & (m)) >> (n))

/* Register definitions generally use Realtek nomenclature.  Registers
   are 4 bytes and R/W except as indicated. */

#define K_PCI_VENDOR_REALTEK  0x10EC
#define K_PCI_ID_RTL8139      0x8139


/* PCI Configuration Register offsets (8139 specific, Section 8) */

#define CAP_PM                0x50          /* Power Management */
#define CAP_VPD               0x60          /* Vital Product Data */


/* RTL8139 Operation Register offsets (Section 6) */

#define R_IDR0                0x0000        /* 1 byte,  R/W */
#define R_IDR1                0x0001
#define R_IDR2                0x0002
#define R_IDR3                0x0003
#define R_IDR4                0x0004
#define R_IDR5                0x0005
#define R_IDR(i)              (R_IDR0 + (i))

#define R_MAR0                0x0008        /* 1 byte,  R/W */
#define R_MAR1                0x0009
#define R_MAR2                0x000A
#define R_MAR3                0x000B
#define R_MAR4                0x000C
#define R_MAR5                0x000D
#define R_MAR6                0x000E
#define R_MAR7                0x000F
#define R_MAR(i)              (R_MAR0 + (i))

#define N_TSD                 4             /* tx_descr(n) = (TSDn, TSADn) */

#define R_TSD0                0x0010
#define R_TSD1                0x0014
#define R_TSD2                0x0018
#define R_TSD3                0x001C
#define R_TSD(i)              (R_TSD0 + 4*(i))

#define R_TSAD0               0x0020
#define R_TSAD1               0x0024
#define R_TSAD2               0x0028
#define R_TSAD3               0x002C
#define R_TSAD(i)             (R_TSAD0 + 4*(i))

#define R_RBSTART             0x0030
#define R_ERBCR               0x0034        /* 2 bytes, R */
#define R_ERSR                0x0036        /* 1 byte,  R */
#define R_CR                  0x0037        /* 1 byte,  R/W */
#define R_CAPR                0x0038        /* 2 bytes, R/W */
#define R_CBR                 0x003A        /* 2 bytes, R */

#define R_IMR                 0x003C        /* 2 bytes, R/W */
#define R_ISR                 0x003E        /* 2 bytes, R/W */

#define R_TCR                 0x0040
#define R_RCR                 0x0044
#define R_TCTR                0x0048
#define R_MPC                 0x004C

#define R_9346CR              0x0050        /* 1 byte,  R/W */
#define R_CONFIG0             0x0051        /* 1 byte,  R/W */
#define R_CONFIG1             0x0052        /* 1 byte,  R/W */
#define R_TIMERINT            0x0054
#define R_MSR                 0x0058        /* 1 byte,  R/W */
#define R_CONFIG3             0x0059        /* 1 byte,  R/W */
#define R_CONFIG4             0x005A        /* 1 byte,  R/W */

#define R_MULINT              0x005C        /* 2 bytes, R/W */
#define R_REVID               0x005E        /* 1 byte,  R */

#define R_TSDS                0x0060        /* 2 bytes, R */

/* MII Control Registers (all 2 bytes) */
#define R_BMCR                0x0062
#define R_BMSR                0x0064
#define R_ANAR                0x0066
#define R_ANLPAR              0x0068
#define R_ANER                0x006A
#define R_DIS                 0x006C
#define R_FCSC                0x006E
#define R_NWAYTR              0x0070
#define R_REC                 0x0072
#define R_CSCR                0x0074

#define R_PHY1_PARM           0x0078
#define R_TW_PARM             0x007C
#define R_PHY2_PARM           0x0080        /* 1 byte,  R/W */

/* Wake-On-LAN Registers at 0x0084-0x00D3 (not supported) */

#define R_FLASH               0x00D4
#define R_CONFIG5             0x00D8        /* 1 byte,  R/W */

/* CardBus Registers at 0x00F0-0x00FC (not supported) */


/* 0x0010-0x001C  TSDn: Transmit Status Registers (6.2) */

#define S_TS_SIZE               0
#define M_TS_SIZE               _DD_MAKEMASK(13,S_TS_SIZE)
#define V_TS_SIZE(x)            _DD_MAKEVALUE(x,S_TS_SIZE)
#define G_TS_SIZE(x)            _DD_GETVALUE(x,S_TS_SIZE,M_TS_SIZE)

#define M_TS_OWN                _DD_MAKEMASK1(13)
#define M_TS_TUN                _DD_MAKEMASK1(14)
#define M_TS_TOK                _DD_MAKEMASK1(15)

#define S_TS_ERTXTH             16
#define M_TS_ERTXTH             _DD_MAKEMASK(6,S_TS_ERTXTH)
#define V_TS_ERTXTH(x)          _DD_MAKEVALUE(x,S_TS_ERTXTH)
#define G_TS_ERTXTH(x)          _DD_GETVALUE(x,S_TS_ERTXTH,M_TS_ERTXTH)
#define ENCODE_TXTH(n)          (((n)>>5)-1)
#define DECODE_TXTH(v)          (((v)+1)<<5)

#define S_TS_NCC                24
#define M_TS_NCC                _DD_MAKEMASK(4,S_TS_NCC)
#define V_TS_NCC(x)             _DD_MAKEVALUE(x,S_TS_NCC)
#define G_TS_NCC(x)             _DD_GETVALUE(x,S_TS_NCC,M_TS_NCC)

#define M_TS_CDH                _DD_MAKEMASK1(28)
#define M_TS_OWC                _DD_MAKEMASK1(29)
#define M_TS_TABT               _DD_MAKEMASK1(30)
#define M_TS_CRS                _DD_MAKEMASK1(31)


/* 0x0036  ERSR: Early Rx Status Register (6.3) */

#define M_ERSR_EROK             _DD_MAKEMASK1(0)
#define M_ERSR_EROVW            _DD_MAKEMASK1(1)
#define M_ERSR_ERBAD            _DD_MAKEMASK1(2)
#define M_ERSR_ERGOOD           _DD_MAKEMASK1(3)


/* 0x0037  CR: Command Register (6.4) */

#define M_CR_BUFE               _DD_MAKEMASK1(0)
#define M_CR_TE                 _DD_MAKEMASK1(2)
#define M_CR_RE                 _DD_MAKEMASK1(3)
#define M_CR_RST                _DD_MAKEMASK1(4)


/* 0x003C  IMR: Interrupt Mask Register (6.5) */
/* 0x003E  ISR: Interrupt Status Register (6.6) */

#define M_INT_ROK               _DD_MAKEMASK1(0)
#define M_INT_RER               _DD_MAKEMASK1(1)
#define M_INT_TOK               _DD_MAKEMASK1(2)
#define M_INT_TER               _DD_MAKEMASK1(3)
#define M_INT_RXOVW             _DD_MAKEMASK1(4)
#define M_INT_PUN_LINKCHG       _DD_MAKEMASK1(5)
#define M_INT_FOVW              _DD_MAKEMASK1(6)
#define M_INT_LENCHG            _DD_MAKEMASK1(13)
#define M_INT_TIMEOUT           _DD_MAKEMASK1(14)
#define M_INT_SERR              _DD_MAKEMASK1(15)


/* 0x0040  TCR: Transmit Configuration Register (6.7) */

#define M_TCR_CLRABT            _DD_MAKEMASK1(0)

#define S_TCR_TXRR              4
#define M_TCR_TXRR              _DD_MAKEMASK(4,S_TCR_TXRR)
#define V_TCR_TXRR(x)           _DD_MAKEVALUE(x,S_TCR_TXRR)      
#define G_TCR_TXRR(x)           _DD_GETVALUE(x,S_TCR_TXRR,M_TCR_TXRR)

#define S_TCR_MXDMA             8
#define M_TCR_MXDMA             _DD_MAKEMASK(3,S_TCR_MXDMA)
#define V_TCR_MXDMA(x)          _DD_MAKEVALUE(x,S_TCR_MXDMA)      
#define G_TCR_MXDMA(x)          _DD_GETVALUE(x,S_TCR_MXDMA,M_TCR_MXDMA)

#define M_TCR_CRC               _DD_MAKEMASK1(16)

#define S_TCR_LBK               17
#define M_TCR_LBK               _DD_MAKEMASK(2,S_TCR_LBK)
#define V_TCR_LBK(x)            _DD_MAKEVALUE(x,S_TCR_LBK)      
#define G_TCR_LBK(x)            _DD_GETVALUE(x,S_TCR_LBK,M_TCR_LBK)
#define K_LBK_OFF               0x0
#define K_LBK_ON                0x3

#define M_TCR_8139G             _DD_MAKEMASK1(23)

#define S_TCR_IFG               24
#define M_TCR_IFG               _DD_MAKEMASK(2,S_TCR_IFG)
#define V_TCR_IFG(x)            _DD_MAKEVALUE(x,S_TCR_IFG)      
#define G_TCR_IFG(x)            _DD_GETVALUE(x,S_TCR_IFG,M_TCR_IFG)
#define K_802_3_IFG             0x3

#define S_TCR_HWVERID           26
#define M_TCR_HWVERID           _DD_MAKEMASK(5,S_TCR_HWVERID)
#define V_TCR_HWVERID(x)        _DD_MAKEVALUE(x,S_TCR_HWVERID)      
#define G_TCR_HWVERID(x)        _DD_GETVALUE(x,S_TCR_HWVERID,M_TCR_HWVERID)
#define K_VER_8139              0x18
#define K_VER_8139A             0x1C
#define K_VER_8139B             0x1E
#define K_VER_8130              0x1F
#define K_VER_8139C             0x1D


/* 0x0044  RCR: Receive Configuration Register (6.8) */

#define M_RCR_AAP               _DD_MAKEMASK1(0)
#define M_RCR_APM               _DD_MAKEMASK1(1)
#define M_RCR_AM                _DD_MAKEMASK1(2)
#define M_RCR_AB                _DD_MAKEMASK1(3)
#define M_RCR_AR                _DD_MAKEMASK1(4)
#define M_RCR_AER               _DD_MAKEMASK1(5)
#define M_RCR_9356SEL           _DD_MAKEMASK1(6)
#define M_RCR_WRAP              _DD_MAKEMASK1(7)

#define S_RCR_MXDMA             8
#define M_RCR_MXDMA             _DD_MAKEMASK(3,S_RCR_MXDMA)
#define V_RCR_MXDMA(x)          _DD_MAKEVALUE(x,S_RCR_MXDMA)      
#define G_RCR_MXDMA(x)          _DD_GETVALUE(x,S_RCR_MXDMA,M_RCR_MXDMA)

#define S_RCR_RBLEN             11
#define M_RCR_RBLEN             _DD_MAKEMASK(2,S_RCR_RBLEN)
#define V_RCR_RBLEN(x)          _DD_MAKEVALUE(x,S_RCR_RBLEN)      
#define G_RCR_RBLEN(x)          _DD_GETVALUE(x,S_RCR_RBLEN,M_RCR_RBLEN)
#define K_RBLEN_8K              0x0
#define K_RBLEN_16K             0x1
#define K_RBLEN_32K             0x2
#define K_RBLEN_64K             0x3

#define S_RCR_RXFTH             13
#define M_RCR_RXFTH             _DD_MAKEMASK(3,S_RCR_RXFTH)
#define V_RCR_RXFTH(x)          _DD_MAKEVALUE(x,S_RCR_RXFTH)      
#define G_RCR_RXFTH(x)          _DD_GETVALUE(x,S_RCR_RXFTH,M_RCR_RXFTH)
#define K_RXFTH_16              0x0
#define K_RXFTH_32              0x1
#define K_RXFTH_64              0x2
#define K_RXFTH_128             0x3
#define K_RXFTH_256             0x4
#define K_RXFTH_512             0x5
#define K_RXFTH_1024            0x6
#define K_RXFTH_SF              0x7

#define M_RCR_RER8              _DD_MAKEMASK1(16)
#define M_RCR_MULERINT          _DD_MAKEMASK1(17)

#define S_RCR_ERTH              24
#define M_RCR_ERTH              _DD_MAKEMASK(4,S_RCR_ERTH)
#define V_RCR_ERTH(x)           _DD_MAKEVALUE(x,S_RCR_ERTH)      
#define G_RCR_ERTH(x)           _DD_GETVALUE(x,S_RCR_ERTH,M_RCR_ERTH)
#define K_ERTH_NONE             0


/* Maximum DMA encodings for TCR_MXDMA and RCR_MXDMA */

#define K_MXDMA_16              0x0
#define K_MXDMA_32              0x1
#define K_MXDMA_64              0x2
#define K_MXDMA_128             0x3
#define K_MXDMA_256             0x4
#define K_MXDMA_512             0x5
#define K_MXDMA_1024            0x6
#define K_MXDMA_2048            0x7        /* TCR_MXDMA */
#define K_MXDMA_UNLIMITED       0x7        /* RCR_MXDMA */


/* 0x0050  9346CR: 93C46 Command Register (6.9) */

#define M_EECR_EEDO             _DD_MAKEMASK1(0)
#define M_EECR_EEDI             _DD_MAKEMASK1(1)
#define M_EECR_EESK             _DD_MAKEMASK1(2)
#define M_EECR_EECS             _DD_MAKEMASK1(3)

#define S_EECR_EEM              6
#define M_EECR_EEM              _DD_MAKEMASK(2,S_EECR_EEM)
#define V_EECR_EEM(x)           _DD_MAKEVALUE(x,S_EECR_EEM)      
#define G_EECR_EEM(x)           _DD_GETVALUE(x,S_EECR_EEM,M_EECR_EEM)
#define K_EEM_NORMAL            0x0
#define K_EEM_AUTOLOAD          0x1
#define K_EEM_PROGRAM           0x2
#define K_EEM_CONFIG            0x3


/* 0x0051  CONFIG0: Configuration Register 0 (6.10) */

#define S_CFG0_BS              0
#define M_CFG0_BS              _DD_MAKEMASK(3,S_CFG0_BS)
#define V_CFG0_BS(x)           _DD_MAKEVALUE(x,S_CFG0_BS)      
#define G_CFG0_BS(x)           _DD_GETVALUE(x,S_CFG0_BS,M_CFG0_BS)
#define K_BS_NONE              0x0

#define S_CFG0_PL              3
#define M_CFG0_PL              _DD_MAKEMASK(2,S_CFG0_PL)
#define V_CFG0_PL(x)           _DD_MAKEVALUE(x,S_CFG0_PL)      
#define G_CFG0_PL(x)           _DD_GETVALUE(x,S_CFG0_PL,M_CFG0_PL)

#define M_CFG0_T10             _DD_MAKEMASK1(5)
#define M_CFG0_PCS             _DD_MAKEMASK1(6)
#define M_CFG0_SCR             _DD_MAKEMASK1(7)


/* 0x0052  CONFIG1: Configuration Register 1 (6.11) */

#define M_CFG1_PMEN            _DD_MAKEMASK1(0)
#define M_CFG1_VDP             _DD_MAKEMASK1(1)
#define M_CFG1_IOMAP           _DD_MAKEMASK1(2)
#define M_CFG1_MEMMAP          _DD_MAKEMASK1(3)
#define M_CFG1_LWACT           _DD_MAKEMASK1(4)
#define M_CFG1_DVRLOAD         _DD_MAKEMASK1(5)

#define S_CFG1_LEDS             6
#define M_CFG1_LEDS             _DD_MAKEMASK(2,S_CFG1_LEDS)
#define V_CFG1_LEDS(x)          _DD_MAKEVALUE(x,S_CFG1_LEDS)      
#define G_CFG1_LEDS(x)          _DD_GETVALUE(x,S_CFG1_LEDS,M_CFG1_LEDS)


/* 0x0058  MSR: Media Status Register (6.12) */

#define M_MSR_RXPF              _DD_MAKEMASK1(0)
#define M_MSR_TXPF              _DD_MAKEMASK1(1)
#define M_MSR_LINKB             _DD_MAKEMASK1(2)
#define M_MSR_SPEED_10          _DD_MAKEMASK1(3)
#define M_MSR_AUX_STATUS        _DD_MAKEMASK1(4)
#define M_MSR_RXFCE             _DD_MAKEMASK1(6)
#define M_MSR_TXFCE             _DD_MAKEMASK1(7)


/* 0x0059  CONFIG3: Configuration Register 3 (6.13) */

#define M_CFG3_FBTB_EN          _DD_MAKEMASK1(0)
#define M_CFG3_FUNCREG_EN       _DD_MAKEMASK1(1)
#define M_CFG3_CLKRUN_EN        _DD_MAKEMASK1(2)
#define M_CFG3_CARDB_EN         _DD_MAKEMASK1(3)
#define M_CFG3_LINKUP           _DD_MAKEMASK1(4)
#define M_CFG3_MAGIC            _DD_MAKEMASK1(5)
#define M_CFG3_PARM_EN          _DD_MAKEMASK1(6)
#define M_CFG3_GNTSEL           _DD_MAKEMASK1(7)


/* 0x005A  CONFIG4: Configuration Register 4 (6.14) */

#define M_CFG4_PBWAKEUP         _DD_MAKEMASK1(0)
#define M_CFG4_LWPTN            _DD_MAKEMASK1(2)
#define M_CFG4_LWPME            _DD_MAKEMASK1(4)
#define M_CFG4_LONGWF           _DD_MAKEMASK1(5)
#define M_CFG4_ANAOFF           _DD_MAKEMASK1(6)
#define M_CFG4_RXFIFOAUTOCLR    _DD_MAKEMASK1(7)


/* 0x005C MULINT: Multiple Interrupt Select Register (6.15) */

#define S_MULINT_MISR           0
#define M_MULINT_MISR           _DD_MAKEMASK(12,S_MULINT_MISR)
#define V_MULINT_MISR(x)        _DD_MAKEVALUE(x,S_MULINT_MISR)      
#define G_MULINT_MISR(x)        _DD_GETVALUE(x,S_MULINT_MISR,M_MULINT_MISR)


/* 0x0060 TSDS (TSAD): Transmit Status of All Descriptors Register (6.17) */

#define M_TSDS_OWN(i)           _DD_MAKEMASK1(0+(i))
#define M_TSDS_TABT(i)          _DD_MAKEMASK1(4+(i))
#define M_TSDS_TUN(i)           _DD_MAKEMASK1(8+(i))
#define M_TSDS_TOK(i)           _DD_MAKEMASK1(12+(i))


/* 0x0070 NWAYTR: NWay Test Register (6.25) */

#define M_NWAYTR_FLAGLSC        _DD_MAKEMASK1(0)
#define M_NWAYTR_FLAGPDF        _DD_MAKEMASK1(1)
#define M_NWAYTR_FLAGABD        _DD_MAKEMASK1(2)
#define M_NWAYTR_ENNWLE         _DD_MAKEMASK1(3)
#define M_NWAYTR_NWLPBK         _DD_MAKEMASK1(7)


/* 0x0074 CSCR: CS Configuration Register (6.27) */

#define M_CSCR_PASS_SCR         _DD_MAKEMASK1(0)
#define M_CSCR_CONN_STATUS_EN   _DD_MAKEMASK1(2)
#define M_CSCR_CON_STATUS       _DD_MAKEMASK1(3)
#define M_CSCR_F_CONNECT        _DD_MAKEMASK1(5)
#define M_CSCR_F_LINK_100       _DD_MAKEMASK1(6)
#define M_CSCR_JBEN             _DD_MAKEMASK1(7)
#define M_CSCR_HEART_BEAT       _DD_MAKEMASK1(8)
#define M_CSCR_LD               _DD_MAKEMASK1(9)
#define M_CSCR_TESTFUN          _DD_MAKEMASK1(15)


/* 0x00D4 FLASH: Flash Memory Read/Write Register */

#define S_FLASH_MA              0
#define M_FLASH_MA              _DD_MAKEMASK(16,S_FLASH_MA)
#define V_FLASH_MA(x)           _DD_MAKEVALUE(x,S_FLASH_MA)      
#define G_FLASH_MA(x)           _DD_GETVALUE(x,S_FLASH_MA,M_FLASH_MA)

#define M_FLASH_SWRWEN          _DD_MAKEMASK1(17)
#define M_FLASH_WEB             _DD_MAKEMASK1(18)
#define M_FLASH_OEB             _DD_MAKEMASK1(19)
#define M_FLASH_ROMCSB          _DD_MAKEMASK1(20)

#define S_FLASH_MD              24
#define M_FLASH_MD              _DD_MAKEMASK(8,S_FLASH_MD)
#define V_FLASH_MD(x)           _DD_MAKEVALUE(x,S_FLASH_MD)      
#define G_FLASH_MD(x)           _DD_GETVALUE(x,S_FLASH_MD,M_FLASH_MD)


/* 0x00D8 CONFIG5: Configuration Register 5 */

#define M_CFG5_PME_STS          _DD_MAKEMASK1(0)
#define M_CFG5_LANWAKE          _DD_MAKEMASK1(1)
#define M_CFG5_LDPS             _DD_MAKEMASK1(2)
#define M_CFG5_FIFOADDRPTR      _DD_MAKEMASK1(3)
#define M_CFG5_UWF              _DD_MAKEMASK1(4)
#define M_CFG5_MWF              _DD_MAKEMASK1(5)
#define M_CFG5_BWF              _DD_MAKEMASK1(6)


/* The 8139 does not use a descriptor ring.  There is a contiguous
   ring buffer for received packets, and each packet is preceded by a
   (little endian) status word with the following format. */

#define M_RS_ROK                _DD_MAKEMASK1(0)
#define M_RS_FAE                _DD_MAKEMASK1(1)
#define M_RS_CRC                _DD_MAKEMASK1(2)
#define M_RS_LONG               _DD_MAKEMASK1(3)
#define M_RS_RUNT               _DD_MAKEMASK1(4)
#define M_RS_ISE                _DD_MAKEMASK1(5)
#define M_RS_BAR                _DD_MAKEMASK1(13)
#define M_RS_PAM                _DD_MAKEMASK1(14)
#define M_RS_MAR                _DD_MAKEMASK1(15)
#define S_RS_LEN                16
#define M_RS_LEN                _DD_MAKEMASK(16,S_RS_LEN)
#define V_RS_LEN(x)             _DD_MAKEVALUE(x,S_RS_LEN)      
#define G_RS_LEN(x)             _DD_GETVALUE(x,S_RS_LEN,M_RS_LEN)

#endif /* _RTL8139_H_ */
