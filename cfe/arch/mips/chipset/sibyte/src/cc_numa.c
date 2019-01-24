/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  ccnuma test commands:		File: cc_numa.c
    *
    *  Adapted to CFE from a stand-alone test program by Manu Gulati.
    *  
    *********************************************************************  
    *
    *  Copyright 2005
    *  Broadcom Corporation. All rights reserved.
    *  
    *  BROADCOM PROPRIETARY AND CONFIDENTIAL
    *  
    *  This software is furnished under license and may be used and 
    *  copied only in accordance with the license.
    ********************************************************************* */

#include "cfe.h"
#include "sbmips.h"
#include "bcm1480_regs.h"
#include "bcm1480_scd.h"
#include "ui_command.h"
int cmd_ccn_test(ui_cmdline_t *cmd,int argc,char *argv[]);

typedef	uint64_t		u64;
typedef uint32_t		u32;
typedef	uint8_t			u8;
typedef	unsigned int		uint;

#define NODE_ADDR(node,addr)	(((u64)((node) & 0xF) << 36) | (u64)(addr))

#define MAX_NODES	16		/* Only NodeID's 4..14 are usable. */
#define	LAST_NODEID	MAX_NODES	/* All NodeIDs less than this */

/* The following definitions assume that the regions beginning at
   physical address 0x01000000 (16M) are not used by CFE and are
   available as scratch memory on all nodes. */

#define	INIT_DONE_ADDR		0x01000000
#define STRIPE_DONE_ADDR	0x01000100
#if 0
#define	STRIPE_START_ADDR	0x01008000
#define	STRIPE_END_ADDR		0x0100FFFF
#else
#define	STRIPE_START_ADDR	0x01100000
#define	STRIPE_END_ADDR		0x011FFFFF
#endif

static u32	node_signature[LAST_NODEID];
static u8	verbose = 1;
static u8	my_nodeid;
static u32	my_signature;
static u8	total_nodes;	/* total number of nodes in this CCNuma experiment */

static u8	node_valid[LAST_NODEID];
static u8	node_alive[LAST_NODEID];
static u8	my_seq_num;	/* valid nodes with lower node id's */

static u64	exp_stripe_data = 0;

static void
st_u64_noncc(u64 paddr, u64 val)
{
    u64 vaddr = PHYS_TO_XKPHYS_UNCACHED(paddr);

    asm volatile ("sd %0, 0(%1)" : : "r"(val), "r"(vaddr));
    if (verbose > 1)
	printf ("Stored uncached u64 %llx to addr %llx\n", val, vaddr);
}

static u64
ld_u64_noncc(u64 paddr)
{
    u64 vaddr = PHYS_TO_XKPHYS_UNCACHED(paddr);
    u64 val;

    asm volatile ("ld %0, 0(%1)" : "=r"(val) : "r"(vaddr));
    if (verbose > 1)
	printf ("Loaded uncached u64 %llx from addr %llx\n", val, vaddr);
    return val;
}

static void
st_u64_cc(u64 paddr, u64 val)
{
    u64 vaddr = PHYS_TO_XKSEG_CACHED(paddr);

    asm volatile ("sd %0, 0(%1)" : : "r"(val), "r"(vaddr));
    if (verbose > 1)
	printf ("Stored coherent u64 %llx to addr %llx\n", val, vaddr);
}

static u64
ld_u64_cc(u64 paddr)
{
    u64 vaddr = PHYS_TO_XKSEG_CACHED(paddr);
    u64 val;

    asm volatile ("ld %0, 0(%1)" : "=r"(val) : "r"(vaddr));
    if (verbose > 1)
	printf ("Loaded coherent u64 %llx from addr %llx\n", val, vaddr);
    return val;
}

static void
st_u8_cc(u64 paddr, u8 val)
{
    u64 vaddr = PHYS_TO_XKSEG_CACHED(paddr);

    asm volatile ("sb %0, 0(%1)" : : "r"(val), "r"(vaddr));
    if (verbose > 1)
	printf ("Stored coherent u8 %x to addr %llx\n", val, vaddr);
}

#if 0 /* not currently used */
static u8
ld_u8_cc(u64 paddr)
{
    u64 vaddr = PHYS_TO_XKSEG_CACHED(paddr);
    u8 val;

    asm volatile ("lbu %0, 0(%1)" : "=r"(val) : "r"(vaddr));
    if (verbose > 1)
	printf ("Loaded coherent u8 %x from addr %llx\n", val, vaddr);
  return val;
}
#endif


static u8
get_my_nodeid (void)
{
    unsigned int node;
    u64 localcfg;

    localcfg = ld_u64_noncc(A_SCD_SYSTEM_CFG);
    node = G_BCM1480_SYS_NODEID(localcfg);
    return node;
}

static char
do_cc_init(u8 nodeid)
{
    /* This version assumes that initialization has already been done. */
    return 1;
}

static void
init_homemem_shared (u8 nodeid)
{
    u64 addr;
    u64 startaddr = NODE_ADDR(nodeid, STRIPE_START_ADDR);
    u64 endaddr   = NODE_ADDR(nodeid, STRIPE_END_ADDR);
    if (verbose)
	printf ("\nInitializing shared memory on local node:\n");
    for (addr=startaddr; addr<endaddr; addr=addr+8) {
#if 0
	st_u64_noncc(addr, 0);
#else
	st_u64_cc(addr, 0);
#endif
	}
    if (verbose)
	printf ("Done Initializing shared memory on local node\n");
}

static char
poll_location(u64 addr, u64 val)
{
    uint timout_cnt, msg_cnt;
    char not_found = 0;

    timout_cnt = msg_cnt = 0;
    while (ld_u64_noncc(addr) != val) {
	++timout_cnt;
	if (verbose && ((timout_cnt & 0x000FFFFF) == 0x000FFFFF)) {
	    if (msg_cnt == 0)
		printf ("Waiting for addr %llx to be %llx", addr, val);
	    if (msg_cnt % 64 == 0) printf("\n");
	    printf(".");
	    msg_cnt++;
	    }
	if (timout_cnt == 0x03FFFFFF) {
	    not_found = 1;
	    break;
	    }
	}
    if (verbose && (msg_cnt != 0)) printf("\n");
    return not_found;
}


static void
fill_bytes (u8 nodeid)
{
    int  i;
    uint bytecnt;
    u8  data;
    u8  mynodeid;
    int j;
    u8  node[MAX_NODES];
    u8  nextnode;
    u64 addr;

    node[0] = mynodeid = nodeid;
    nextnode = 1;
    for (i = (mynodeid+1)%LAST_NODEID; i!=mynodeid; i = (i+1)%LAST_NODEID) {
	/* Note that cachable operations to nodes that failed to
	   initialize will generally cause bus errors, so exclude them
	   here. */
	if (node_valid[i] && node_alive[i]) {
	    node[nextnode] = i;
	    nextnode++;
	    }
	}

    data = ((nodeid & 0xF) << 4) | nodeid;
    bytecnt = (STRIPE_END_ADDR - STRIPE_START_ADDR + 1) & 0xFFFFFFFF;

    for (i=0; i<bytecnt; i+=8) {
	for (j=1; j<nextnode; j++) {
	    /* Note that STRIPE_END_ADDR is last byte, not last word. */
#ifdef ENDIAN_BIG
	    if (j % 2 == 0)
		addr = STRIPE_START_ADDR + i + 7-my_seq_num;
	    else
		addr = STRIPE_END_ADDR - i - my_seq_num;
#else
	    if (j % 2 == 0)
		addr = STRIPE_START_ADDR + i + my_seq_num;
	    else
		addr = STRIPE_END_ADDR - i - 7-my_seq_num;
#endif
	    st_u8_cc (NODE_ADDR(node[j], addr), data);
	    }
	}

    mynodeid  &= 0xF;    /* should be redundant */
    for (j=nextnode-1; j >0; j--) {
	addr = NODE_ADDR(node[j], STRIPE_DONE_ADDR) | (mynodeid << 4);
	if (verbose)
	    printf("Signaling fill_bytes done to node %d\n", node[j]);
	st_u64_noncc (addr, my_signature);
	}
}

static int
check_mem (u8 nodeid)
{
    int	 i;
    u64	 addr;
    uint bytecnt;
    u64  rdval;
    int  errors;

    bytecnt = (STRIPE_END_ADDR - STRIPE_START_ADDR + 1) & 0xFFFFFFFF;
    errors = 0;
    for (i=0; i<bytecnt; i+=8) {
	addr = NODE_ADDR(nodeid, STRIPE_START_ADDR) | (u64)i;
	rdval = ld_u64_cc (addr);
	if (rdval != exp_stripe_data) {
	    if (errors < 32)
		printf(" mismatch, addr %llx: expected %16llx, read %16llx\n",
		       addr, exp_stripe_data, rdval);
	    else if (errors == 32)
		printf (" ...\n");
	    errors++;
	    }
	}
    return errors;
}


static int
ccn_test(void)
{
    u8	remnode;	/* temp. var for remote node	*/
    u64	doneaddr;
    uint i;
    u32 pattern;
    u8	my_fill_byte;

    my_nodeid = get_my_nodeid();

    my_seq_num  = 0;
    total_nodes = 0;
    exp_stripe_data = 0;

    for (i=0; i<LAST_NODEID; i++) {
	if (node_valid[i]) {
	    if (my_nodeid == i) {
		my_seq_num = total_nodes;
		}
	    else {
		my_fill_byte = ((i & 0xF) << 4) | (i & 0xF);
		exp_stripe_data |= ((u64) my_fill_byte << (8*total_nodes));
		}
	    ++total_nodes;
	    }
	}

    if (verbose) {
	printf ("System has %d nodes. My nodeid is %d\n",
		total_nodes, my_nodeid);
	printf ("Expected final pattern is %llx\n", exp_stripe_data);
	}
    if (total_nodes > 8) {
    	printf ("ERROR: total_nodes(%d) must be <= 8\n", total_nodes);
	return -1;
	}

    if (!do_cc_init(my_nodeid)) {
	printf("Initialization failed\n");
	return -1;
	}

    for (i=0; i<LAST_NODEID; ++i) {
	if (node_valid[i]) {
	    pattern = ((i & 0xF) << 4) | (((i & 0xF) ^ 0xF) << 0);
	    node_signature[i] = ((pattern << 24) | (pattern << 16) |
				 (pattern <<  8) | (pattern <<  0));
	    }
	}
    /* Intent of the above loop is to accomplish:
     * node_signature[4]  = 0x4b4b4b4b;
     * ...
     * node_signature[8]  = 0x87878787;
     * ...
     * node_signature[14] = 0xe1e1e1e1;
     */

    my_signature = node_signature[my_nodeid];
    if (verbose)
	printf ("Initialization complete. My signature=%x\n", my_signature);

    /* Initialize memory to be used for CC testing before the memory
     * barrier is executed */
    init_homemem_shared(my_nodeid);

    /* Let other nodes in the system know we succesfully initialized */
    if (verbose)
	printf ("\nSignaling other nodes that my init is complete:\n");
    for (remnode=0; remnode<LAST_NODEID; ++remnode) {
	if ((remnode!=my_nodeid) && node_valid[remnode]) {
	    doneaddr = (NODE_ADDR(remnode, INIT_DONE_ADDR) |
			((u64) (my_nodeid & 0xF) << 4));
	    st_u64_noncc(doneaddr, (u64) my_signature);
	    }
	}
    if (verbose)
	printf ("Done Signaling other nodes that my init is complete\n");

    /* check if other nodes in the system signaled to us that they're
       done initializing */
    if (verbose)
	printf ("\nChecking to see if other nodes init's are complete:\n");
    for (remnode=0; remnode<LAST_NODEID; ++remnode) {
	if ((remnode!=my_nodeid) && node_valid[remnode]) {
	    node_alive[remnode] = 1;
	    doneaddr = (NODE_ADDR(my_nodeid, INIT_DONE_ADDR) |
			((u64) (remnode & 0xF) << 4));
	    if (poll_location (doneaddr, (u64) node_signature[remnode])) {
		printf ("NOTE: Timed out waiting for data %x at addr %llx for node %d\n",
			node_signature[remnode], doneaddr, remnode);
		node_alive[remnode] = 0;
		}
	    }
	}
    if (verbose)
	printf ("Done checking to see if other nodes init's are complete\n");

    /* Ready to do CC testing now */

    /* Striping test: in this, each node inserts its nodeID in certain
     * bytes of each cache line. Each node only inserts its nodeID in
     * every eighth byte of the cache line.  At the end, the home node
     * will check that the data is correct.  */
    if (verbose)
	printf("\nFilling bytes with my nodeid:\n");
    fill_bytes(my_nodeid);
    if (verbose)
	printf("Done Filling bytes with my nodeid\n");

    /* Check to see that all nodes are done filling their nodeid	*/
    if (verbose)
	printf ("\nChecking to see if other nodes filling is complete:\n");
    for (remnode=0; remnode<LAST_NODEID; ++remnode) {
	if ((remnode!=my_nodeid) && node_valid[remnode]) {
	    doneaddr = NODE_ADDR(my_nodeid, STRIPE_DONE_ADDR) | ((remnode & 0xF) << 4);
	    if (poll_location (doneaddr, (u64) node_signature[remnode]))
		printf ("NOTE: Timed out waiting for data %x at addr %llx for node %d\n",
		node_signature[remnode], doneaddr, remnode);
	    }
	}
    if (verbose)
	printf ("Done checking to see if other nodes filling is complete\n");

    if (verbose)
	printf("\nChecking local memory for correct data:\n");
    if (check_mem(my_nodeid)) {
	printf ("Memory check failed. Quitting.\n");
	return -1;
	}
    else printf ("TEST PASSED! IT'S A MIRACLE!!\n");

    /* reset handshake areas to allow rerun. */
    for (remnode=0; remnode<LAST_NODEID; ++remnode) {
	if ((remnode!=my_nodeid) && node_valid[remnode]) {
	    doneaddr = (NODE_ADDR(my_nodeid, INIT_DONE_ADDR) |
			((u64) (remnode & 0xF) << 4));
	    st_u64_noncc(doneaddr, 0);
	    }
	}
    for (remnode=0; remnode<LAST_NODEID; ++remnode) {
	if ((remnode!=my_nodeid) && node_valid[remnode]) {
	    doneaddr = (NODE_ADDR(my_nodeid, STRIPE_DONE_ADDR) |
			((remnode & 0xF) << 4));
	    st_u64_noncc(doneaddr, 0);
	    }
	}

   return 0;
}


static void
set_default_nodes(void)
{
    /* Default active nodes (4 and 5 for now) */
    node_valid[4] = node_valid[5] = 1;
}
  

int
cmd_ccn_test(ui_cmdline_t *cmd,int argc,char *argv[])
{
    int i;
    char *x;
    uint node;

    for (i = 0; i < LAST_NODEID; i++)
	node_valid[i] = node_alive[i] = 0;
    if (argc == 0)
	set_default_nodes();
    else {
	for (i = 0; i < argc; i++) {
	    x = cmd_getarg(cmd, i);
	    node = atoi(x);
	    if (node >= 4 && node <= 14)
		node_valid[node] = 1;
	    }
	}
    return ccn_test();
}
