/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  ccnuma commands				File: ui_ccncmds.c
    *  
    *  Commands and test stuff to check out ccnuma (evolving)
    *  
    *********************************************************************  
    *
    *  Copyright 2004,2005
    *  Broadcom Corporation. All rights reserved.
    *  
    *  BROADCOM PROPRIETARY AND CONFIDENTIAL
    *  
    *  This software is furnished under license and may be used and 
    *  copied only in accordance with the license.
    ********************************************************************* */


#include "cfe.h"
#include "ui_command.h"

#include "lib_physio.h"
#include "bcm1480_regs.h"
#include "bcm1480_hsp.h"
#include "bcm1480_rld.h"
#include "bcm1480_scd.h"

extern void ht_ccn_enable(unsigned int node, unsigned int port, int enable);


/*  *********************************************************************
    *  Constants 
    ********************************************************************* */

typedef unsigned int uint_t;


/*  *********************************************************************
    *  prototypes
    ********************************************************************* */

int ui_init_ccncmds(void);


/*  *********************************************************************
    *  RLD stuff
    ********************************************************************* */

#define RLD_LINES 2048

#define M_BCM1480_RLD_FIELD_PATTERN  (M_BCM1480_RLD_FIELD_TAG          | \
                                      M_BCM1480_RLD_FIELD_VALID        | \
                                      M_BCM1480_RLD_FIELD_MODIFIED     | \
                                      M_BCM1480_RLD_FIELD_NODE_VECTOR)

static void
fill_rld(int node, uint64_t pattern, uint8_t ecc, int new_ecc, int check_ecc)
{
    unsigned int index, way;
    uint64_t common_val;
    uint64_t prefix = (uint64_t)node << 36;

    common_val = ((pattern & M_BCM1480_RLD_FIELD_PATTERN) |
		  V_BCM1480_RLD_FIELD_ECC_BITS(ecc));
    if (new_ecc) common_val |= M_BCM1480_RLD_FIELD_NEW_ECC;
    if (check_ecc) common_val |= M_BCM1480_RLD_FIELD_CHECK_ECC;

    for (index = 0; index < RLD_LINES; index++) {
	for (way = 0; way < 8; way++) {
	    if (way != 3 && way != 7) {    /* unimplemented ways */
		phys_write64(prefix | A_BCM1480_NC_RLD_FIELD,
			     (common_val |
			      V_BCM1480_RLD_FIELD_INDEX(index) | 
			      V_BCM1480_RLD_FIELD_WAY_SELECT(way)));
		phys_write64(prefix | A_BCM1480_NC_RLD_TRIGGER, 0);
		}
	    }
	}
}

static int
check_rld_pattern(uint64_t pattern, uint64_t rdtrigger)
{
    uint64_t tagin = G_BCM1480_RLD_FIELD_TAG(pattern);
    int validin = (pattern & M_BCM1480_RLD_FIELD_VALID) != 0;
    int modifiedin  = (pattern & M_BCM1480_RLD_FIELD_MODIFIED) != 0;
    uint64_t vectorin  = G_BCM1480_RLD_FIELD_NODE_VECTOR(pattern);
    uint64_t eccin  = G_BCM1480_RLD_FIELD_ECC_BITS(pattern);

    uint64_t tagout = G_BCM1480_RLD_TRIGGER_TAG(rdtrigger);
    int validout = (rdtrigger & M_BCM1480_RLD_TRIGGER_VALID) != 0;
    int modifiedout = (rdtrigger & M_BCM1480_RLD_TRIGGER_MODIFIED) != 0;
    uint64_t vectorout = G_BCM1480_RLD_TRIGGER_NODE_VECTOR(rdtrigger);
    uint64_t eccout = G_BCM1480_RLD_TRIGGER_ECC_BITS(rdtrigger);

    return ((tagin != tagout) || (validin != validout) ||
	    (vectorin !=vectorout) || (modifiedin!=modifiedout) ||
	    (eccin != eccout)) ? -1 : 0;
}

static void
log_rld_error(uint_t index, uint_t way, uint64_t pattern, uint64_t rdtrigger)
{
    uint64_t tagin = G_BCM1480_RLD_FIELD_TAG(pattern);
    int validin = (pattern & M_BCM1480_RLD_FIELD_VALID) != 0;
    int modifiedin  = (pattern & M_BCM1480_RLD_FIELD_MODIFIED) != 0;
    uint64_t vectorin  = G_BCM1480_RLD_FIELD_NODE_VECTOR(pattern);
    uint64_t eccin  = G_BCM1480_RLD_FIELD_ECC_BITS(pattern);

    uint64_t tagout = G_BCM1480_RLD_TRIGGER_TAG(rdtrigger);
    int validout = (rdtrigger & M_BCM1480_RLD_TRIGGER_VALID) != 0;
    int modifiedout = (rdtrigger & M_BCM1480_RLD_TRIGGER_MODIFIED) != 0;
    uint64_t vectorout = G_BCM1480_RLD_TRIGGER_NODE_VECTOR(rdtrigger);
    uint64_t eccout = G_BCM1480_RLD_TRIGGER_ECC_BITS(rdtrigger);

    printf("  failed comparison, index %d, way %d:\n", index, way);
    printf("   pattern=%16llx, rdtrigger=%16llx\n", pattern, rdtrigger);
    if (tagin != tagout)
	printf("   tagin=%x, tagout=%x\n", tagin, tagout);
    if (validin != validout)
	printf("   validin=%x, validout=%x\n", validin, validout);
    if (vectorin != vectorout)
	printf("   vectorin=%x, vectorout=%x\n", vectorin, vectorout);
    if (modifiedin != modifiedout)
	printf("   modifiedin=%x, modifiedout=%x\n", modifiedin, modifiedout);
    if (eccin != eccout)
	printf("   eccin=%x, eccout=%x\n", eccin, eccout);
}

static int
check_rld(int node, uint64_t pattern, uint8_t ecc, int new_ecc, int check_ecc)
{
    uint_t index, way;
    uint64_t common_val;
    uint64_t prefix = (uint64_t)node << 36;
    uint64_t rdtrigger;
    int errors;

    common_val = ((pattern & M_BCM1480_RLD_FIELD_PATTERN) |
		  V_BCM1480_RLD_FIELD_ECC_BITS(ecc));
    if (new_ecc) common_val |= M_BCM1480_RLD_FIELD_NEW_ECC;
    if (check_ecc) common_val |= M_BCM1480_RLD_FIELD_CHECK_ECC;

    errors = 0;

    for (index = 0; index < RLD_LINES; index++) {
	for (way = 0; way < 8; way++) {
	    if (way != 3 && way != 7) {    /* unimplemented ways */
		phys_write64(prefix | A_BCM1480_NC_RLD_FIELD,
			     (common_val |
			      V_BCM1480_RLD_FIELD_INDEX(index) | 
			      V_BCM1480_RLD_FIELD_WAY_SELECT(way)));
		rdtrigger = phys_read64(prefix | A_BCM1480_NC_RLD_TRIGGER);
		if (check_rld_pattern(pattern, rdtrigger) != 0) {
		    if (errors < 10)
			log_rld_error(index, way, pattern, rdtrigger);
		    else if (errors == 10)
			printf("  ...\n");
		    errors++;
		    }
	    }
	}
    }

    if (errors != 0)
	printf(" %d errors for pattern %16llx\n", errors, pattern);
    return errors;
}


#if 0  /* Still useful? */
/*  *********************************************************************
    *  HSP commands
    ********************************************************************* */

#define TX_RAM_READCTRL(port)                                            \
    A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_RAM_READCTL)
#define TX_RAM_READWINDOW(port)                                          \
    A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_RAM_READWINDOW)

static uint64_t
tx_ram_read(uint_t node, uint_t port, uint_t file, uint_t addr)
{
    uint64_t prefix = ((uint64_t)node) << 36;

    phys_write64(prefix | TX_RAM_READCTRL(port),
		 (V_BCM1480_HSP_TXVIS_RAM(file) |
		  V_BCM1480_HSP_TXVIS_RAM_ADDR(addr)));
    return phys_read64(prefix | TX_RAM_READWINDOW(port));
}

static int
ui_cmd_ccn_txram(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint_t node, port;
    uint_t low, high, idx;
    char *x;

    low = 0; high = 65535;

    node = 0;
    port = 0;
    if (cmd_sw_value(cmd, "-n", &x)) {
	node = atoi(x);
	node &= 0xF;
	}

    if ((x = cmd_getarg(cmd,0))) low = xtoi(x);
    if ((x = cmd_getarg(cmd,1))) high = xtoi(x);

    for (idx = low; idx <= high; idx++) {
	printf("%04X:  ", idx);
	printf("%016llX   ",
	       tx_ram_read(node, port,
			   K_BCM1480_HSP_TXVIS_RAM_DRAM_128_151, idx));
	printf("%016llX ",
	       tx_ram_read(node, port,
			   K_BCM1480_HSP_TXVIS_RAM_DRAM_64_127, idx));
	printf("%016llX\n",
	       tx_ram_read(node, port,
			   K_BCM1480_HSP_TXVIS_RAM_DRAM_0_63, idx));     
	if (console_status()) break;
	}

    return 0;
}


#define RX_RAM_READCTRL(port)                                            \
    A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_RAM_READCTL)
#define RX_RAM_READWINDOW(port)                                          \
    A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_RAM_READWINDOW)

static uint64_t
rx_ram_read(uint_t node, uint_t port, uint_t file, uint_t addr)
{
    uint64_t prefix = ((uint64_t)node) << 36;

    phys_write64(prefix | RX_RAM_READCTRL(port),
		 (V_BCM1480_HSP_RXVIS_RAM(file) |
		  V_BCM1480_HSP_RXVIS_RAM_ADDR(addr)));
    return phys_read64(prefix | RX_RAM_READWINDOW(port));
}

static int
ui_cmd_ccn_rxram(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint_t node, port;
    uint_t low, high, idx;
    char *x;

    low = 0; high = 65535;

    node = 0;
    port = 0;
    if (cmd_sw_value(cmd, "-n", &x)) {
	node = atoi(x);
	node &= 0xF;
	}

    if ((x = cmd_getarg(cmd,0))) low = xtoi(x);
    if ((x = cmd_getarg(cmd,1))) high = xtoi(x);

    for (idx = low; idx <= high; idx++) {
	printf("%04X:  ", idx);
	printf("%016llX   ",
	       rx_ram_read(node, port,
			   K_BCM1480_HSP_RXVIS_RAM_DRAM_128_151, idx));
	printf("%016llX ",
	       rx_ram_read(node, port,
			   K_BCM1480_HSP_RXVIS_RAM_DRAM_64_127, idx));
	printf("%016llX\n",
	       rx_ram_read(node, port,
			   K_BCM1480_HSP_RXVIS_RAM_DRAM_0_63, idx));
	if (console_status()) break;
	}

    return 0;
    
}


#define TX_RF_READCTRL(port)                                            \
    A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_RF_READCTL)
#define TX_RF_READWINDOW(port)                                          \
    A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_TX_RF_READWINDOW)

static uint64_t
tx_rf_read(uint_t node, uint_t port, uint_t file, uint_t addr)
{
    uint64_t prefix = ((uint64_t)node) << 36;

    phys_write64(prefix | TX_RF_READCTRL(port),
		 (V_BCM1480_HSP_TXRFVIS_RAM(file) |
		  V_BCM1480_HSP_TXRFVIS_RAM_ADDR(addr)));
    return phys_read64(prefix | TX_RF_READWINDOW(port));
}

static int
ui_cmd_ccn_txvis(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint_t node, port, reg;
    uint_t idx;
    uint64_t val,val2,val3;
    char *x;

    if (cmd_getarg(cmd,0))
	reg = atoi(cmd_getarg(cmd,0));
    else
	reg = 0;   /* Default to {head, tail, phitcnt} */

    node = 0;
    if (cmd_sw_value(cmd, "-n", &x)) {
	node = atoi(x);
	node &= 0xF;
	}

    port = 0;
    if (cmd_sw_value(cmd, "-p", &x)) {
	port = atoi(x);
	}
    if (port >= 3) return ui_showusage(cmd);

    printf("node %d: TX port %d register file #%d\n", node, port, reg);

    switch (reg) {
	default:
	    for (idx = 0; idx < 24; idx++) {
		printf("Regfile[%02X] = %016llX\n",
		       idx, tx_rf_read(node, port, reg, idx));
		}

	    break;
	case 0:
	    for (idx = 0; idx < 24; idx++) {
		val =  tx_rf_read(node, port, 0, idx);
		val2 = tx_rf_read(node, port, 1, idx);
		val3 = tx_rf_read(node, port, 8, idx);
		printf("Chan %2d   Head:%04X  Tail:%04X  Phit:%04X\n", idx,
		       (uint32_t)val, (uint32_t)val2, (uint32_t)val3);
		}
	    break;
	    

	case 2: case 5:
	    for (idx = 0; idx < 24; idx++) {
		val =  tx_rf_read(node, port, reg+0, idx);
		val2 = tx_rf_read(node, port, reg+1, idx);
		val3 = tx_rf_read(node, port, reg+2, idx);
		printf("Chan %2d   Meta:%08llX %s %s BC:%2d %s IVC:%2d Data:%02llX_%016llX\n",
		       idx, val3,
		       (val3 & 1) ? "SOP" : "---",
		       (val3 & 2) ? "EOP" : "---",
		       (int)((val3 >> 2) & 15),
		       (val3 & 64) ? "ERR" : "OK ",
		       (int)(val3 >> 7),
		       val2, val);
		}
	    break;
	    
	case 9:
	    for (idx = 0; idx < 24; idx++) {
		val = tx_rf_read(node, port, reg, idx);
		printf("Chan %2d   Push:%d   Pop:%d\n", idx,
		       (int)(val & 0x0F), (int)((val >> 4) & 0x0F));
		}
	    break;

	case 10:
	    val = tx_rf_read(node, port, reg, 0);
	    printf("Ready to pick:  %06llx\n",((val >> 32) & 0xFFFFFF));
	    printf("IVC OK:         %06llx\n",(val & 0xFFFFFF));
	    break;	    
	}

    return 0;
}


#define RX_RF_READCTRL(port)                                            \
    A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_RF_READCTL)
#define RX_RF_READWINDOW(port)                                          \
    A_BCM1480_HSP_REGISTER(port, R_BCM1480_HSP_RX_RF_READWINDOW)

static uint64_t
rx_rf_read(uint_t node, uint_t port, uint_t file, uint_t addr)
{
    uint64_t prefix = ((uint64_t)node) << 36;

    phys_write64(prefix | RX_RF_READCTRL(port),
		 (V_BCM1480_HSP_RXRFVIS_RAM(file) |
		  V_BCM1480_HSP_RXRFVIS_RAM_ADDR(addr)));
    return phys_read64(prefix | RX_RF_READWINDOW(port));
}

static int
ui_cmd_ccn_rxvis(ui_cmdline_t *cmd,int argc,char *argv[])
{
    uint_t node, port, reg;
    uint_t idx;
    uint64_t val, val2, val3;
    char *x;

    if (cmd_getarg(cmd,0))
	reg = atoi(cmd_getarg(cmd,0));
    else
	reg = 5;   /* Default to {head, tail, phitcnt} */

    node = 0;
    if (cmd_sw_value(cmd, "-n", &x)) {
	node = atoi(x);
	node &= 0xF;
	}

    port = 0;
    if (cmd_sw_value(cmd, "-p", &x)) {
	port = atoi(x);
	}
    if (port >= 3) return ui_showusage(cmd);

    printf("node %d: RX port %d register file #%d\n", node, port, reg);

    switch (reg) {
	case 0:
	    for (idx = 0; idx < 24; idx++) {
		val =  rx_rf_read(node, port, 0, idx);
		val2 = rx_rf_read(node, port, 1, idx);
		val3 = rx_rf_read(node, port, 2, idx);
		printf("Chan %2d   Meta:%016llX  Data:%016llX_%016llX\n", idx,
		       val3, val, val2);
		}
	    break;
	case 5:
	    for (idx = 0; idx < 27; idx++) {
		val =  rx_rf_read(node, port, 5, idx);
		val2 = rx_rf_read(node, port, 6, idx);
		val3 = rx_rf_read(node, port, 8, idx);
		printf("Chan %2d   Head:%04X  Tail:%04X  Phit:%04X\n",idx,
		       (uint32_t)val, (uint32_t)val2, (uint32_t)val3);
		}
	    break;
	default:
	    for (idx = 0; idx < 32; idx++) {
		val = rx_rf_read(node, port, reg, idx);
		printf("Regfile[%02X] = %016llX\n", idx, val);
		}
	}

    return 0;
}
#endif /* useful? */


/*  *********************************************************************
    *  ccnuma management commands
    ********************************************************************* */
    
static int
ui_cmd_testrld(ui_cmdline_t *cmd, int argc, char *argv[])
{
    static const uint64_t patterns[4] = {
	0xFFFFFFFFFFFFFFFFULL, 0xAAAAAAAAAAAAAAAAULL,
	0x5555555555555555ULL, 0x0000000000000000ULL
    };
    /* Note that the final pattern is all 0's, to leave RLD entries invalid. */
    unsigned int node;
    char *x;
    int errors;
    int i;

    node = 0;
    if (argc > 0) {
	x = cmd_getarg(cmd, 0);
	node = atoi(x);
	if (node < 4 || node > 14) {
	    printf("Invalid node number %d\n", node);
	    return CFE_ERR;
	    }

	}

    errors = 0;
    if (0)
      printf ("%16s -> %16s\n", "write field", "read trigger");

    for (i=0; i<4; ++i) {
	fill_rld(node, patterns[i], ((patterns[i] >> 4) & 0x7f), 0, 0);
	errors +=
	  check_rld(node, patterns[i], ((patterns[i] >> 4) & 0x7f), 0, 0);
	}

    printf(" %d total errors\n", errors);
    return (errors != 0 ? -1 : 0);
}

/* The following command can be issued multiple times to enable all
   the relevant ports for a particular node. */
static int
ui_cmd_ccn_enable(ui_cmdline_t *cmd,int argc,char *argv[])
{
    unsigned int node;
    unsigned int port;
    uint64_t localcfg;
    char *x;

    localcfg = phys_read64(A_SCD_SYSTEM_CFG);
    if (argc == 0) {
	node = G_BCM1480_SYS_NODEID(localcfg);
	}
    else {
	x = cmd_getarg(cmd, 0);
	node = atoi(x);
	if (node < 4 || node > 14) {
	    printf("Invalid node number %d\n", node);
	    return CFE_ERR;
	    }
	}

    port = 0;
    if (cmd_sw_value(cmd, "-port", &x)) {
	port = atoi(x);
	if (port > 2) {
	    printf("Invalid port number %d\n", port);
	    return CFE_ERR;
	    }
	}

    if (node == G_BCM1480_SYS_NODEID(localcfg)) {
	if ((localcfg & M_BCM1480_SYS_CCNUMA_EN) == 0) {
	    /* Fill the node's rld with invalid entries. */
	    fill_rld(0, 0, 0, 0, 0);

	    /* Enable local ccnuma. */
	    localcfg |= M_BCM1480_SYS_CCNUMA_EN;
	    phys_write64(A_SCD_SYSTEM_CFG, localcfg);
	    }
	}
    else {
	uint64_t prefix;
	uint64_t syscfg;

	prefix = (uint64_t)node << 36;
	syscfg = phys_read64(prefix | A_SCD_SYSTEM_CFG);
	if ((syscfg & M_BCM1480_SYS_CCNUMA_EN) == 0) {
	    /* Fill the node's rld with invalid entries. */
	    fill_rld(node, 0, 0, 0, 0);

	    /* Enable ccnuma for the node. */
	    syscfg |= M_BCM1480_SYS_CCNUMA_EN;
	    phys_write64(prefix | A_SCD_SYSTEM_CFG, syscfg);
	    }
	}
			
    /* Turn on ccnuma for the port. */
    ht_ccn_enable(node, port, 1);

    return 0;
}


extern int cmd_ccn_test(ui_cmdline_t *cmd,int argc,char *argv[]);

static int
ui_cmd_ccn_test(ui_cmdline_t *cmd,int argc,char *argv[])
{
    return cmd_ccn_test(cmd,argc,argv);
}


/*  *********************************************************************
    *  ui_init_ccncmds()
    *  
    *  Add ccnuma-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */

int
ui_init_ccncmds(void)
{
#if 0  /* Still useful? */
    cmd_addcmd("ccn txvis",	
	       ui_cmd_ccn_txvis,
	       NULL,
	       "Display TX RF Visibility",
	       "ccn txvis [regset]",
	       "-n=*;Specify node number|"
	       "-p=*;Specify port number");

    cmd_addcmd("ccn rxvis",	
	       ui_cmd_ccn_rxvis,
	       NULL,
	       "Display RX RF Visibility",
	       "ccn rxvis [regset]",
	       "-n=*;Specify node number|"
	       "-p=*;Specify port number");

    cmd_addcmd("ccn txram",	
	       ui_cmd_ccn_txram,
	       NULL,
	       "Display TX buffer RAM",
	       "ccn txram",
	       "-n=*;Specify node number");

    cmd_addcmd("ccn rxram",	
	       ui_cmd_ccn_rxram,
	       NULL,
	       "Display RX buffer RAM",
	       "ccn rxram",
	       "-n=*;Specify node number");
#endif /* useful? */

    cmd_addcmd("ccn enable",
	       ui_cmd_ccn_enable,
	       NULL,
	       "Enable ccnuma",
	       "ccn enable [node]",
	       "-port=*;Specify port number");

    cmd_addcmd("ccn test",
	       ui_cmd_ccn_test,
	       NULL,
	       "Start ccnuma test on this node",
	       "ccn test",
	       "");

    cmd_addcmd("test rld",
	       ui_cmd_testrld,
	       NULL,
	       "Check ECC for the RLD",
	       "test rld [node]",
	       "");

    return 0;
}
