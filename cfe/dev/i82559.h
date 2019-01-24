/*
 * Register and bit definitions for the Intel 8255x 10/100 Ethernet
 *   Controller Family
 * Reference:
 *   Intel 8255x 10/100 Mbps Ethernet Controller Family Open Source
 *     Software Developer Manual, Revision 1.0.
 *   Intel, January 2003
 */
#ifndef _I82559_H_
#define _I82559_H_

#define _DD_MAKEMASK1(n) (1 << (n))
#define _DD_MAKEMASK(v,n) ((((1)<<(v))-1) << (n))
#define _DD_MAKEVALUE(v,n) ((v) << (n))
#define _DD_GETVALUE(v,n,m) (((v) & (m)) >> (n))


/* PCI Vendor and Device Ids */

#define K_PCI_VENDOR_INTEL    0x8086
#define K_PCI_ID_I82557       0x1229    /* also I82558, some I82559 */
#define K_PCI_ID_I82559ER     0x1209
#define K_PCI_ID_INBUSINESS   0x1030


/*  i8255x Control and Status Register offsets.

    Note: Several of the CSRs are intended to be accessed as bytes or
    half words to avoid side effects on other registers packed in the
    same word.  The offsets and bit fields of such CSRs are defined
    relative to their byte/half word addresses.
*/
    
#define R_SCB_STAT            0x00       /* 8 bits */
#define R_SCB_STATACK         0x01       /* 8 bits */
#define R_SCB_CMD             0x02       /* 8 bits */
#define R_SCB_IC              0x03       /* 8 bits */
#define R_SCB_PTR             0x04
#define R_PORT                0x08
#define R_EEPROM_CTL          0x0E       /* 16 bits */
#define R_MDI_CTL             0x10
#define R_RXBC                0x14

/* Following in i82558 and later */
#define R_ERX_INT             0x18       /* 8 bits */
#define R_FC_THRESH           0x19       /* 8 bits */
#define R_FC_CMD              0x20       /* 8 bits */
#define R_PMDR                0x21       /* 8 bits */

/* Following in i82559 and later */
#define R_GEN_CTL             0x1C       /* 8 bits */
#define R_GEN_STAT            0x1D       /* 8 bits */
#define R_FUNC_EVENT          0x30
#define R_FUNC_MASK           0x34
#define R_FUNC_STATE          0x38
#define R_FORCE_EVENT         0x3C


/* 0x00  SCB: SCB Status Register (8 bits) */

#define S_SCB_RUS               2
#define M_SCB_RUS               _DD_MAKEMASK(4,S_SCB_RUS)
#define V_SCB_RUS(x)            _DD_MAKEVALUE(x,S_SCB_RUS)
#define G_SCB_RUS(x)            _DD_GETVALUE(x,S_SCB_RUS,M_SCB_RUS)
#define K_RUS_IDLE              0x0
#define K_RUS_SUSPENDED         0x1
#define K_RUS_BLOCKED           0x2
#define K_RUS_READY             0x4

#define S_SCB_CUS               6
#define M_SCB_CUS               _DD_MAKEMASK(2,S_SCB_CUS)
#define V_SCB_CUS(x)            _DD_MAKEVALUE(x,S_SCB_CUS)
#define G_SCB_CUS(x)            _DD_GETVALUE(x,S_SCB_CUS,M_SCB_CUS)
#define K_CUS_IDLE              0x0
#define K_CUS_SUSPENDED         0x1
#define K_CUS_LPQ               0x2
#define K_CUS_HPQ               0x3


/* 0x01  SCB: SCB STAT/ACK Register (8 bits, R/C) */

#define M_SCB_FCP               _DD_MAKEMASK1(0)   /* not i82557 */
#define M_SCB_SWI               _DD_MAKEMASK1(2)
#define M_SCB_MDI               _DD_MAKEMASK1(3)
#define M_SCB_RNR               _DD_MAKEMASK1(4)
#define M_SCB_CNA               _DD_MAKEMASK1(5)   /* CNA or CI */
#define M_SCB_FR                _DD_MAKEMASK1(6)   /* FR  or ER */
#define M_SCB_CX                _DD_MAKEMASK1(7)   /* also TNO, i82557 */


/* 0x02  SCB: SCB Command Byte Register (8 bits) */

#define S_SCB_RUC               0
#define M_SCB_RUC               _DD_MAKEMASK(3,S_SCB_RUC)
#define V_SCB_RUC(x)            _DD_MAKEVALUE(x,S_SCB_RUC)
#define G_SCB_RUC(x)            _DD_GETVALUE(x,S_SCB_RUC,M_SCB_RUC)
#define K_RUC_NOP               0x0
#define K_RUC_START             0x1
#define K_RUC_RESUME            0x2
#define K_RUC_REDIRECT          0x3   /* not i82557 */
#define K_RUC_ABORT             0x4
#define K_RUC_HDS               0x5
#define K_RUC_BASE              0x6

#define S_SCB_CUC               4
#define M_SCB_CUC               _DD_MAKEMASK(4,S_SCB_CUC)
#define V_SCB_CUC(x)            _DD_MAKEVALUE(x,S_SCB_CUC)
#define G_SCB_CUC(x)            _DD_GETVALUE(x,S_SCB_CUC,M_SCB_CUC)
#define K_CUC_NOP               0x0
#define K_CUC_START             0x1
#define K_CUC_RESUME            0x2
#define K_CUC_CTR_ADDR          0x4
#define K_CUC_CTR_DUMP          0x5
#define K_CUC_BASE              0x6
#define K_CUC_CTR_DUMP_RESET    0x7
#define K_CUC_STATIC_RESUME     0xA   /* not i82557 */

#define K_CMD_ACCEPTED          0x00


/* 0x03  SCB: SCB Interrupt Control Byte Register (8 bits) */

#define M_SCB_M                 _DD_MAKEMASK1(0)
#define M_SCB_SI                _DD_MAKEMASK1(1)
#define M_SCB_FCP_M             _DD_MAKEMASK1(2)   /* not i82557 */
#define M_SCB_ER_M              _DD_MAKEMASK1(3)   /* not i82557 */
#define M_SCB_RNR_M             _DD_MAKEMASK1(4)   /* not i82557 */
#define M_SCB_CNA_M             _DD_MAKEMASK1(5)   /* not i82557 */
#define M_SCB_FR_M              _DD_MAKEMASK1(6)   /* not i82557 */
#define M_SCB_CX_M              _DD_MAKEMASK1(7)   /* not i82557 */


/* 0x04  SCB: SCB General Pointer Register */


/* 0x08  PORT: Port Interface Register */

#define S_PORT_FUNC              0
#define M_PORT_FUNC              _DD_MAKEMASK(4,S_PORT_FUNC)
#define V_PORT_FUNC(x)           _DD_MAKEVALUE(x,S_PORT_FUNC)
#define G_PORT_FUNC(x)           _DD_GETVALUE(x,S_PORT_FUNC,M_PORT_FUNC)
#define K_PORT_FUNC_SWRESET      0x0
#define K_PORT_FUNC_SELFTEST     0x1
#define K_PORT_FUNC_SELRESET     0x2
#define K_PORT_FUNC_DUMP         0x3
#define K_PORT_FUNC_DUMP_WKUP    0x7

#define S_PORT_ADDR              4
#define M_PORT_ADDR              _DD_MAKEMASK(28,S_PORT_ADDR)
#define V_PORT_ADDR(x)           _DD_MAKEVALUE(x,S_PORT_ADDR)
#define G_PORT_ADDR(x)           _DD_GETVALUE(x,S_PORT_ADDR,M_PORT_ADDR)


/* 0x0E  PROM: EEPROM Control Register (16 bits) */

#define M_PROM_EESK             _DD_MAKEMASK1(0)
#define M_PROM_EECS             _DD_MAKEMASK1(1)
#define M_PROM_EEDI             _DD_MAKEMASK1(2)
#define M_PROM_EEDO             _DD_MAKEMASK1(3)


/* 0x10  MDI: MDI Control Register */

#define S_MDI_DATA              0
#define M_MDI_DATA              _DD_MAKEMASK(16,S_MDI_DATA)
#define V_MDI_DATA(x)           _DD_MAKEVALUE(x,S_MDI_DATA)
#define G_MDI_DATA(x)           _DD_GETVALUE(x,S_MDI_DATA,M_MDI_DATA)

#define S_MDI_REGADD            16
#define M_MDI_REGADD            _DD_MAKEMASK(5,S_MDI_REGADD)
#define V_MDI_REGADD(x)         _DD_MAKEVALUE(x,S_MDI_REGADD)
#define G_MDI_REGADD(x)         _DD_GETVALUE(x,S_MDI_REGADD,M_MDI_DATA)

#define S_MDI_PHYADD            21
#define M_MDI_PHYADD            _DD_MAKEMASK(5,S_MDI_PHYADD)
#define V_MDI_PHYADD(x)         _DD_MAKEVALUE(x,S_MDI_PHYADD)
#define G_MDI_PHYADD(x)         _DD_GETVALUE(x,S_MDI_PHYADD,M_MDI_PHYADD)

#define S_MDI_OP                26
#define M_MDI_OP                _DD_MAKEMASK(2,S_MDI_OP)
#define V_MDI_OP(x)             _DD_MAKEVALUE(x,S_MDI_OP)
#define G_MDI_OP(x)             _DD_GETVALUE(x,S_MDI_OP,M_MDI_OP)
#define K_MDI_OP_WRITE          0x1
#define K_MDI_OP_READ           0x2

#define M_MDI_R                _DD_MAKEMASK1(28)
#define M_MDI_IE               _DD_MAKEMASK1(29)


/* 0x14  RXBC: RX DMA Byte Count Register */


/* 0x18  FLOW: Flow Control Threshold and Power Management Driver Register */


/* 0x1C  General Control Register (8 bits, not i82557, i82558) */

#define M_GCTL_NOCLKRUN         _DD_MAKEMASK1(0)
#define M_GCTL_DEEPPWRDN        _DD_MAKEMASK1(1)


/* 0x1D  General Status Register (8 bits, RO, not i82557, i82558) */

#define M_GSTAT_LINKUP          _DD_MAKEMASK1(0)
#define M_GSTAT_100             _DD_MAKEMASK1(1)
#define M_GSTAT_FDX             _DD_MAKEMASK1(2)


/* i8255x-specific Constants (recognized by hardware) */

#define NULL_LINK               0xFFFFFFFF


/* i8255x Action Command Block (CB) formats. */

/* CB Halfword 0: Status */

#define M_CB0_OK               _DD_MAKEMASK1(13)
#define M_CB0_C                _DD_MAKEMASK1(15)

/* CB Halfword 1: Command */

#define S_CB0_CMD              0
#define M_CB0_CMD              _DD_MAKEMASK(3,S_CB0_CMD)
#define V_CB0_CMD(x)           _DD_MAKEVALUE(x,S_CB0_CMD)
#define G_CB0_CMD(x)           _DD_GETVALUE(x,S_CB0_CMD,M_CB0_CMD)
#define K_CB_NOP               0x0
#define K_CB_ADDRSETUP         0x1
#define K_CB_CONFIGURE         0x2
#define K_CB_MCASTSETUP        0x3
#define K_CB_TRANSMIT          0x4
#define K_CB_LOADUCODE         0x5
#define K_CB_DUMP              0x6
#define K_CB_DIAGNOSE          0x7
#define K_CB_IPTRANSMIT        0x9    /* i82550, i82551 only */

#define M_CB0_I                _DD_MAKEMASK1(13)
#define M_CB0_S                _DD_MAKEMASK1(14)
#define M_CB0_EL               _DD_MAKEMASK1(15)

/* CB Word 1: Link Address */

/* CB Word 2-n: command dependent extensions */

#define CB_HDR_WORDS           2
#define CB_HDR_BYTES           8

/* Transmit Control Block (TCB) extension format */

/* TCB Word 0: Additional Command fields */

#define M_TCB0_SF              _DD_MAKEMASK1(3)
#define M_TCB0_NC              _DD_MAKEMASK1(4)

#define S_TCB0_CID             8
#define M_TCB0_CID             _DD_MAKEMASK(5,S_TCB0_CID)
#define V_TCB0_CID(x)          _DD_MAKEVALUE(x,S_TCB0_CID)
#define G_TCB0_CID(x)          _DD_GETVALUE(x,S_TCB0_CID,M_TCB0_CID)


/* TCB Word 2: Transmit Buffer Descriptor (TBD) Array Address */

/* TCB Word 3: Transmit Command */

#define S_TCB3_COUNT           0
#define M_TCB3_COUNT           _DD_MAKEMASK(14,S_TCB3_COUNT)
#define V_TCB3_COUNT(x)        _DD_MAKEVALUE(x,S_TCB3_COUNT)
#define G_TCB3_COUNT(x)        _DD_GETVALUE(x,S_TCB3_COUNT,M_TCB3_COUNT)

#define M_TCB3_EOF             _DD_MAKEMASK1(15)

#define S_TCB3_THRESH          16
#define M_TCB3_THRESH          _DD_MAKEMASK(8,S_TCB3_THRESH)
#define V_TCB3_THRESH(x)       _DD_MAKEVALUE(x,S_TCB3_THRESH)
#define G_TCB3_THRESH(x)       _DD_GETVALUE(x,S_TCB3_THRESH,M_TCB3_THRESH)

#define S_TCB3_TBDNUM          24
#define M_TCB3_TBDNUM          _DD_MAKEMASK(8,S_TCB3_TBDNUM)
#define V_TCB3_TBDNUM(x)       _DD_MAKEVALUE(x,S_TCB3_TBDNUM)
#define G_TCB3_TBDNUM(x)       _DD_GETVALUE(x,S_TCB3_TBDNUM,M_TCB3_TBDNUM)


/* i8255x Transmit Buffer Descriptor (TBD) formats. */

/* TBD Word 0: Buffer Address */

#define K_TBD0_NV              0x00000000

/* TBD Word 1: Buffer Size */

#define S_TBD1_COUNT           0
#define M_TBD1_COUNT           _DD_MAKEMASK(14,S_TBD1_COUNT)
#define V_TBD1_COUNT(x)        _DD_MAKEVALUE(x,S_TBD1_COUNT)
#define G_TBD1_COUNT(x)        _DD_GETVALUE(x,S_TBD1_COUNT,M_TBD1_COUNT)

#define M_TBD1_EL              _DD_MAKEMASK1(16)    /* not i82557 */


/* i8255x Receive Frame Descriptor (RFD) formats. */

/* RFD Halfword 0: Status */

#define S_RFD0_STATUS         0
#define M_RFD0_STATUS         _DD_MAKEMASK(13,S_RFD0_STATUS)
#define V_RFD0_STATUS(x)      _DD_MAKEVALUE(x,S_RFD0_STATUS)
#define G_RFD0_STATUS(x)      _DD_GETVALUE(x,S_RFD0_STATUS,M_RFD0_STATUS)

/* Bit assignments within RFD0_STATUS */
#define M_STAT_RXCOL          _DD_MAKEMASK1(0)
#define M_STAT_IAMATCH        _DD_MAKEMASK1(1)
#define M_STAT_NOMATCH        _DD_MAKEMASK1(2)
#define M_STAT_RXERR          _DD_MAKEMASK1(4)
#define M_STAT_TYPELEN        _DD_MAKEMASK1(5)
#define M_STAT_RUNTERR        _DD_MAKEMASK1(7)
#define M_STAT_DMAOVRRUN      _DD_MAKEMASK1(8)
#define M_STAT_BUFOVRRUN      _DD_MAKEMASK1(9)
#define M_STAT_ALIGNERR       _DD_MAKEMASK1(10)
#define M_STAT_CRCERR         _DD_MAKEMASK1(11)

#define M_RFD0_ERRS           ((M_STAT_CRCERR | M_STAT_ALIGNERR | \
                                M_STAT_BUFOVRRUN | M_STAT_DMAOVRRUN | \
                                M_STAT_RUNTERR | M_STAT_RXERR \
                               ) << S_RFD0_STATUS)

#define M_RFD0_OK             _DD_MAKEMASK1(13)
#define M_RFD0_C              _DD_MAKEMASK1(15)

/* RFD Halfword 1: Command */

#define M_RFD0_SF             _DD_MAKEMASK1(3)
#define M_RFD0_H              _DD_MAKEMASK1(4)
#define M_RFD0_S              _DD_MAKEMASK1(14)
#define M_RFD0_EL             _DD_MAKEMASK1(15)

/* RFD Word 1: Link Address */

/* RFD Word 2: Reserved */

/* RFD Word 3: Count and Size */

#define S_RFD3_COUNT          0
#define M_RFD3_COUNT          _DD_MAKEMASK(14,S_RFD3_COUNT)
#define V_RFD3_COUNT(x)       _DD_MAKEVALUE(x,S_RFD3_COUNT)
#define G_RFD3_COUNT(x)       _DD_GETVALUE(x,S_RFD3_COUNT,M_RFD3_COUNT)

#define M_RFD3_F              _DD_MAKEMASK1(14)
#define M_RFD3_EOF            _DD_MAKEMASK1(15)

#define S_RFD3_SIZE           16
#define M_RFD3_SIZE           _DD_MAKEMASK(14,S_RFD3_SIZE)
#define V_RFD3_SIZE(x)        _DD_MAKEVALUE(x,S_RFD3_SIZE)
#define G_RFD3_SIZE(x)        _DD_GETVALUE(x,S_RFD3_SIZE,M_RFD3_SIZE)


/* Offsets (words) of Statistical Counters */

#define STAT_TX_GOOD_FRAMES    0
#define STAT_TX_MAX_COL        1
#define STAT_TX_LATE_COL       2
#define STAT_TX_UNDERRUN       3
#define STAT_TX_LOST_CRS       4
#define STAT_TX_DEFERRED       5
#define STAT_TX_1_COL          6
#define STAT_TX_MULT_COL       7
#define STAT_TX_TOTAL_COL      8

#define STAT_RX_GOOD_FRAMES    9
#define STAT_RX_CRC_ERRS       10
#define STAT_RX_ALGN_ERRS      11
#define STAT_RX_RSRC_ERRS      12
#define STAT_RX_OVERRUN        13
#define STAT_RX_CDT            14
#define STAT_RX_RUNT_FRAMES    15

#define STAT_MARKER_557        16

/* Following only for i82558 and later */
#define STAT_FC_TX_PAUSE       16
#define STAT_FC_RX_PAUSE       17
#define STAT_FC_RX_UNSUPPORTED 18

#define STAT_MARKER_558        19

/* Following only for i82559 and later */
#define STAT_TX_TCO_FRAMES     19
#define STAT_RX_TCO_FRAMES     20

#define STAT_MARKER_559        24

/* Codes written in the STAT_MARKER slots */

#define K_STAT_COMPLETE        0xA005
#define K_STAT_ZERO_COMPLETE   0xA007

#endif /* _I82559_H_ */
