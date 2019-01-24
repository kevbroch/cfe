/*  *********************************************************************
    *  PPC Board Support Package
    *  
    *  initialization engine			File: init_engine.h
    *
    *  This routine parses a table of initialization primitives to
    *  set values into MSRs, PCI config space, etc.
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


#define IE_EOT		0
#define IE_PCICFG32	1
#define IE_PCICFG16	2
#define IE_PCICFG8	3
#define IE_SPR		4
#define IE_MSR		5
#define IE_IBAT		6
#define IE_DBAT		7
#define IE_SR		8
#define IE_DELAY	9
#define IE_ENABBATS	10
#define IE_ENABICACHE	11
#define IE_ENABDCACHE	12

#define IET_CMD(c,a,m,v)	.long (c),(a),(m),(v)

#define IET_SPR(s,v)		IET_CMD(IE_SPR,(s),0,(v))
#define IET_SPRX(s,m,v)		IET_CMD(IE_SPR,(s),(m),(v))
#define IET_MSR(v)		IET_CMD(IE_MSR,0,0,(v))
#define IET_MSRX(m,v)		IET_CMD(IE_MSR,0,(m),(v))
#define IET_SR(s,v)		IET_CMD(IE_SR,(s),0,(v))
#define IET_IBAT(s,u,l)		IET_CMD(IE_IBAT,(s),(u),(l))
#define IET_DBAT(s,u,l)		IET_CMD(IE_DBAT,(s),(u),(l))
#define IET_DELAY(x)		IET_CMD(IE_DELAY,0,0,(x))
#define IET_EOT()		IET_CMD(IE_EOT,0,0,0)
#define IET_PCI32(a,d)		IET_CMD(IE_PCICFG32,(a),0,(d))
#define IET_PCI32X(a,m,d)	IET_CMD(IE_PCICFG32,(a),(m),(d))
#define IET_PCI16(a,d)		IET_CMD(IE_PCICFG16,(a),0,(d))
#define IET_PCI16X(a,m,d)	IET_CMD(IE_PCICFG16,(a),(m),(d))
#define IET_PCI8(a,d)		IET_CMD(IE_PCICFG8,(a),0,(d))
#define IET_PCI8X(a,m,d)	IET_CMD(IE_PCICFG8,(a),(m),(d))
#define IET_ENABBATS()		IET_CMD(IE_ENABBATS,0,0,0)
#define IET_ENABICACHE()	IET_CMD(IE_ENABICACHE,0,0,0)
#define IET_ENABDCACHE()	IET_CMD(IE_ENABDCACHE,0,0,0)


