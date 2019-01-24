/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Polled interrupt dispatch routines	File: bcmcore_irq.c
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

/*  *********************************************************************
    *  This module provides an interface for associating service
    *  routines with BCM47xx system interrupt sources.  The CPU Cause
    *  register is periodically polled and the requested service
    *  routine is invoked when a corresponding interrupt request is
    *  active and enabled.
    *
    *  This is not a full-fledged interrupt handler.
    *
    *  If CFG_INTERRUPTS == 0, it operates synchronously with the
    *  main polling loop and is never invoked directly by the
    *  hardware exception handler.  If CFG_INTERRUPTS == 1, certain
    *  interrupt sources can be handled asynchronously as exceptions.
    *
    ********************************************************************* */

#include "cpu_config.h"
#include "lib_types.h"
#include "lib_printf.h"
#include "lib_malloc.h"

#include "sbmips32.h"
extern void bcmcore_update_sr(uint32_t clear, uint32_t set);
extern void bcmcore_irq_install(void);
extern void bcmcore_irq_arm(void);

#if !defined(_BCM96345_) && !defined(_BCM9635x_) /* XXX ISB, not SB */
#include "sb_utils.h"
#endif
#include "exception.h"
#include "cfe_irq.h"

extern void cfe_ledstr(const char *);   /* debugging */

/* First level dispatching (MIPS IP level). */

/*
 * The BCMCORE provides only a single level of dispatch.  To preserve
 * compatibility with existing clients (XXX temporary?), we accept
 * handlers of type ip_handler_t, which have exclusive ownership of
 * interrupts at the given level, or of type void (*)(void *), which
 * are potentially sharable.
 */

#define IP_LEVELS 8

typedef struct irq_action_s irq_action_t;
struct irq_action_s {
    void (*handler)(void *arg);
    void *arg;
    unsigned long flags;
    int device;              /* to distinguish actions for shared interrupts */
    irq_action_t *next;
};

typedef struct irq_desc_s {
    irq_action_t *actions;
    int depth;
    int status;
} irq_desc_t;

/* status flags */
#define IRQ_DISABLED  0x0001

/* Shared variables that must be protected in non-interrupt code. */

static irq_desc_t irq_desc[IP_LEVELS];
static ip_handler_t ip_handler[IP_LEVELS] = {NULL};

void
cfe_irq_setvector(int index, ip_handler_t handler)
{
    if (index >= 0 && index < IP_LEVELS) {
	uint32_t sr;

	sr = cfe_irq_disable();

	ip_handler[index] = NULL;   /* disable: see demux */
	if (handler == NULL)
	    sr &= ~_MM_MAKEMASK1(S_SR_IMMASK + index);
	else
	    sr |=  _MM_MAKEMASK1(S_SR_IMMASK + index);
	ip_handler[index] = handler;

	cfe_irq_enable(sr);
    }
}


static void
bcmcore_dispatch_irqs(irq_desc_t *desc)
{
    irq_action_t *action;

    for (action = desc->actions; action != NULL; action = action->next) {
	if (action->handler != NULL) {
	    (*action->handler)(action->arg);
	    }
	}
}

/*
 * Dispatch functions called from the exception handler for
 * asynchronous (non-polled) interrupts.
 * info is a pointer to the saved register block.
 *
 * At entry, interrupts will be masked.
 */

/* Called from the exception handler */
static void
bcmcore_irq_demux(int code, mips_reg_t *info)
{
    mips_reg_t pending;
    mips_reg_t mask_ip;
    int i;
    irq_desc_t *desc;

    pending = info[XCP0_CAUSE] & info[XCP0_SR];
    mask_ip = _MM_MAKEMASK1(S_CAUSE_IPMASK + (IP_LEVELS-1));
    desc = &irq_desc[IP_LEVELS-1];

    for (i = IP_LEVELS-1; i >= 0; i--) {
	if (pending & mask_ip) {
	    if (ip_handler[i] != NULL)
		(*(ip_handler[i]))(i);
	    else if (desc->actions != NULL)
		bcmcore_dispatch_irqs(desc);
	    else {
		/* mask off the interrupt or we're caught here forever */
		bcmcore_update_sr(mask_ip, 0);
		}
	    }
	mask_ip >>= 1;
	desc--;
	}
}


/*
 * Dispatch functions called for explicit polling (synchronous
 * interrupts).  The argument pending is a bit mask of pending enabled
 * interrupts with the bits positioned as in C0_CAUSE.
 */

void bcmcore_dispatch_pending(uint32_t pending);
void
bcmcore_dispatch_pending(uint32_t pending)
{
    uint32_t mask_ip;
    int i;
    irq_desc_t *desc;

    mask_ip = _MM_MAKEMASK1(S_CAUSE_IPMASK + (IP_LEVELS-1));
    desc = &irq_desc[IP_LEVELS-1];

    for (i = IP_LEVELS-1; i >= 0; i--) {
	if (pending & mask_ip) {
	    if (ip_handler[i] != NULL)
		(*(ip_handler[i]))(i);
	    else if (desc->actions != NULL)
		bcmcore_dispatch_irqs(desc);
	    else {
		/* mask off the interrupt or we're caught here forever */
		bcmcore_update_sr(mask_ip, 0);
		}
	    }
	mask_ip >>= 1;
	desc--;
	}
}


/*
 * Initialize the MIPS level dispatch vector.
 * This function should be called with interrupts disabled.
 */

static void
bcmcore_irq_vectorinit(void)
{
    int  i;

    for (i = 0; i < IP_LEVELS; i++) {
	ip_handler[i] = NULL;
	}
#if CFG_INTERRUPTS
    _exc_setvector(XTYPE_INTERRUPT, (void *) bcmcore_irq_demux);
    bcmcore_irq_arm();
#else
    (void) bcmcore_irq_demux;
#endif
}

/*
 *  cfe_irq_init is called early in the boot sequence.  It is
 *  responsible for setting up the interrupt mapper and initializing
 *  the handler table that will be used for dispatching pending
 *  interrupts.  If hard interrupts are used, it then enables hardware
 *  interrupts (initially all masked).
 *
 *  This function should be called before interrupts are enabled.
 */

void
cfe_irq_init(void)
{
    int i;
    irq_desc_t *p;

    for (i = 0, p = irq_desc; i < IP_LEVELS; i++, p++) {
        p->actions = NULL;
	p->depth = 0;
	p->status = IRQ_DISABLED;
	}

    bcmcore_irq_install();

    bcmcore_irq_vectorinit();
}


#if 0  /* needed? */
/* cfe_mask_irq() is called to mask an interrupt at the hw level */
void
cfe_mask_irq(int cpu, unsigned int irq)
{
    if (irq < IP_LEVELS) {
	uint32_t sr;

	sr = cfe_irq_disable();
	sr &= ~_MM_MAKEMASK1(S_SR_IMMASK + irq);
	cfe_irq_enable(sr);
	}
}


/* cfe_unmask_irq() is called to unmask an interrupt at the hw level */
void
cfe_unmask_irq(int cpu, unsigned int irq)
{
    if (irq < IP_LEVELS) {
	uint32_t sr;

	sr = cfe_irq_disable();
	sr |= _MM_MAKEMASK1(S_SR_IMMASK + irq);
	cfe_irq_enable(sr);
	}
}
#endif /* 0 */


/*
 * The mapping of backplane interrupts to MIPS interrupts is
 * controlled by the CPU core's SBIPSFLAG and SBINTVEC registers.  The
 * following function further adjusts from MIPS interrupt priority
 * (IPn) to the index in the Status/Cause bit masks.
 */
static inline unsigned int
bcmcore_map_irq(unsigned int irq)
{
#if !defined(_BCM96345_) && !defined(_BCM9635x_) /* XXX cleanup */
    return sb_map_irq(irq) + 2;
#else
    return (irq % 4) + 3;    /* XXX Works for external interrupts only? */
#endif
}


/* If depth is 0, unmask the interrupt. Increment depth. */
void
cfe_enable_irq(unsigned int irq)
{
    unsigned int level = bcmcore_map_irq(irq);

    if (level < IP_LEVELS) {
	uint32_t sr;
	irq_desc_t *desc;

	desc = &irq_desc[level];
	/* The following code must be atomic */
	sr = cfe_irq_disable();
	if (desc->depth == 0) {
	    sr |= _MM_MAKEMASK1(S_SR_IMMASK + level);
	    desc->status &=~ IRQ_DISABLED;
	    }
	desc->depth++;
	cfe_irq_enable(sr);
	}
}


/* Decrement depth. If depth is 0, mask the interrupt. */
void
cfe_disable_irq(unsigned int irq)
{
    unsigned int level = bcmcore_map_irq(irq);

    if (level < IP_LEVELS) {
	uint32_t sr;
	irq_desc_t *desc;

	desc = &irq_desc[level];
	/* The following code must be atomic */
	sr = cfe_irq_disable();
	desc->depth--;
	if (desc->depth == 0) {
	    sr &= ~_MM_MAKEMASK1(S_SR_IMMASK + level);
	    desc->status |= IRQ_DISABLED;
	    }
	cfe_irq_enable(sr);
	}
}


/*
 *  cfe_request_irq() is called by drivers to request addition to the
 *  chain of handlers called for a given interrupt.  
 */
int
cfe_request_irq(unsigned int irq, 
		void (*handler)(void *), void *arg,
		unsigned long irqflags, int device)
{
    int sr;
    unsigned int level;
    irq_action_t *action;
    irq_desc_t *desc;
    irq_action_t *p;

    if (handler == NULL) {
        return -1;
	}

    level = bcmcore_map_irq(irq);

    action = (irq_action_t *) KMALLOC(sizeof(irq_action_t), 0);
    if (action == NULL) {
        return -1;
	}

    action->handler = handler;
    action->arg = arg;
    action->flags = irqflags;
    action->device = device;
    action->next = NULL;

    desc = &irq_desc[level];

    /* The following block of code has to be executed atomically */
    sr = cfe_irq_disable();
    p = desc->actions;
    if (p == NULL) {
        desc->actions = action;
        desc->depth = 0;
        desc->status |= IRQ_DISABLED;

	sr &= ~_MM_MAKEMASK1(S_SR_IMMASK + level);
	}
    else {
        /* Can't share interrupts unless both agree to */
        if ((p->flags & action->flags & CFE_IRQ_FLAGS_SHARED) == 0) {
	    cfe_enable_irq(irq);
	    KFREE(action);
	    xprintf("cfe_request_irq: conflicting unsharable interrupts.\n");
            return -1;
	    }
	while (p->next != NULL) {
	    p = p->next;
	    }
	p->next = action;
	}

    cfe_irq_enable(sr);
    cfe_enable_irq(irq);
    return 0;
}


/*
 *  free_irq() releases a handler set up by request_irq()
 */
void
cfe_free_irq(unsigned int irq, int device)
{
    int sr;
    unsigned int level;
    irq_desc_t *desc;
    irq_action_t *p, *q;

    level = bcmcore_map_irq(irq);

    desc = &irq_desc[level];

    /* The following block of code has to be executed atomically */
    sr = cfe_irq_disable();
    p = desc->actions;
    q = NULL;
    while (p != NULL) {
	if (p->device == device) {
	    if (q == NULL) {
		desc->actions = p->next;
		if (desc->actions == NULL) {
		    desc->status |= IRQ_DISABLED;
		    sr &= ~_MM_MAKEMASK1(S_SR_IMMASK + level);
		    }
		}
	    else
		q->next = p->next;
	    break;
	    }
	else {
	    q = p;
	    p = p->next;
	    }
	}
    cfe_irq_enable(sr);
    if (p != NULL)
	KFREE(p);
}
