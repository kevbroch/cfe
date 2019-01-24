/*  *********************************************************************
    *  Broadcom Common Firmware Environment (CFE)
    *  
    *  Temperature sensor commands:		File: ui_tempsensor.c
    *  
    *  Temperature sensor commands
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
#include "cfe_smbus.h"
#include "ui_command.h"


/*  *********************************************************************
    *                     Supported chips
    *    chip      vendor id (at 0xFE)    rev ID (at 0xFF)
    *
    *  ADT7461           0x41                0x51
    *  MAX1617           0x4D                0x01
    *  MAX6654           0x4D                0x08
    * 
    ********************************************************************* */

/*  *********************************************************************
    *  Configuration
    ********************************************************************* */

#define _MAX6654_	/* Support Maxim 6654 temperature chip w/parasitic mode */

/*  *********************************************************************
    *  prototypes
    ********************************************************************* */

int ui_init_tempsensorcmds(void);

#if (defined(TEMPSENSOR_SMBUS_DEV) && defined(TEMPSENSOR_SMBUS_CHAN))
static int ui_cmd_showtemp(ui_cmdline_t *cmd,int argc,char *argv[]);
static void temp_timer_proc(void *);
#endif

/*  *********************************************************************
    *  Data
    ********************************************************************* */

#if (defined(TEMPSENSOR_SMBUS_DEV) && defined(TEMPSENSOR_SMBUS_CHAN))
static int64_t temp_timer = 0;
static int temp_prev_local = 0;
static int temp_prev_remote = 0;
#endif

/*  *********************************************************************
    *  ui_init_swarmcmds()
    *  
    *  Add SWARM-specific commands to the command table
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   0
    ********************************************************************* */


int ui_init_tempsensorcmds(void)
{

#if (defined(TEMPSENSOR_SMBUS_DEV) && defined(TEMPSENSOR_SMBUS_CHAN))
    cmd_addcmd("show temp",
	       ui_cmd_showtemp,
	       NULL,
	       "Display CPU temperature",
	       "show temp",
	       "-continuous;Poll for temperature changes|"
	       "-stop;Stop polling for temperature changes");

    cfe_bg_add(temp_timer_proc,NULL);
#endif

    return 0;
}



#if (defined(TEMPSENSOR_SMBUS_DEV) && defined(TEMPSENSOR_SMBUS_CHAN))
/*  *********************************************************************
    *  temp_smbus_read(chan,slaveaddr,devaddr)
    *  
    *  Read a byte from the temperature sensor chip
    *  
    *  Input parameters: 
    *  	   chan - SMBus channel
    *  	   slaveaddr -  SMBus slave address
    *  	   devaddr - byte with in the sensor device to read
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else -1
    ********************************************************************* */

static int temp_smbus_read(cfe_smbus_channel_t *chan,int slaveaddr,int devaddr)
{
    uint8_t buf[1];
    int err;

    /*
     * Read the data byte
     */

    err = SMBUS_XACT(chan,slaveaddr,devaddr,buf,1);
    if (err < 0) return err;

    return buf[0];
}

#ifdef _MAX6654_
/*  *********************************************************************
    *  temp_smbus_write(chan,slaveaddr,devaddr,data)
    *  
    *  write a byte to the temperature sensor chip
    *  
    *  Input parameters: 
    *  	   chan - SMBus channel
    *  	   slaveaddr -  SMBus slave address
    *  	   devaddr - byte with in the sensor device to read
    *  	   
    *  Return value:
    *  	   0 if ok
    *  	   else -1
    ********************************************************************* */

static int temp_smbus_write(cfe_smbus_channel_t *chan,int slaveaddr,int devaddr,int data)
{
    uint8_t buf[2];
    int err;

    /*
     * Write the data byte
     */

    buf[0] = devaddr;
    buf[1] = data;

    err = SMBUS_WRITE(chan,slaveaddr,buf,2);
    return err;
}
#endif


/*  *********************************************************************
    *  temp_showtemp(noisy)
    *  
    *  Display the temperature.  If 'noisy' is true, display it 
    *  regardless of whether it has changed, otherwise only display
    *  when it has changed.
    *  
    *  Input parameters: 
    *  	   noisy - display whether or not changed
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

static int temp_showtemp(int noisy)
{
    int local,remote,status;
    char statstr[50];
    cfe_smbus_channel_t *chan = SMBUS_CHANNEL(TEMPSENSOR_SMBUS_CHAN);

    local  = temp_smbus_read(chan,TEMPSENSOR_SMBUS_DEV,0);
    remote = temp_smbus_read(chan,TEMPSENSOR_SMBUS_DEV,1);
    status = temp_smbus_read(chan,TEMPSENSOR_SMBUS_DEV,2);

    if ((local < 0) || (remote < 0) || (status < 0)) {
	if (noisy) printf("Temperature sensor device did not respond\n");
	return -1;
	}

    if (noisy || (local != temp_prev_local) || (remote != temp_prev_remote)) {
	statstr[0] = 0;
	if (status & 0x80) strcat(statstr,"Busy ");
	if (status & 0x40) strcat(statstr,"HiTempLcl ");
	if (status & 0x20) strcat(statstr,"LoTempLcl ");
	if (status & 0x10) strcat(statstr,"HiTempRem ");
	if (status & 0x08) strcat(statstr,"LoTempRem ");
	if (status & 0x04) strcat(statstr,"Fault ");

	if (noisy || !(status & 0x80)) {	
	    /* don't display if busy, always display if noisy */
	    console_log("Temperature:  CPU: %dC  Board: %dC  Status:%02X [ %s]",
		    remote,local,status,statstr);
	    }
	}

    temp_prev_local = local;
    temp_prev_remote = remote;

    return 0;
}




/*  *********************************************************************
    *  ui_cmd_showtemp(cmd,argc,argv)
    *  
    *  Show temperature 
    *  
    *  Input parameters: 
    *  	   cmd - command structure
    *  	   argc,argv - parameters
    *  	   
    *  Return value:
    *  	   -1 if error occured.  Does not return otherwise
    ********************************************************************* */

static int ui_cmd_showtemp(ui_cmdline_t *cmd,int argc,char *argv[])
{

    do {
	int dev,rev;
	static int didinit = 0;
	cfe_smbus_channel_t *chan = SMBUS_CHANNEL(TEMPSENSOR_SMBUS_CHAN);

	if (!didinit) {
	    didinit = 1;
	    dev = temp_smbus_read(chan,TEMPSENSOR_SMBUS_DEV,0xFE);
	    rev = temp_smbus_read(chan,TEMPSENSOR_SMBUS_DEV,0xFF);
	    printf("Temperature Sensor Device ID %02X rev %02X\n",dev,rev);

#ifdef _MAX6654_
	    if (dev == 0x4D && rev == 0x08) {		/* MAX6654 */
		printf("Switching MAX6654 to parasitic mode\n");
		/* Switch to 1hz conversion rate (1 seconds per conversion) */
		temp_smbus_write(chan,TEMPSENSOR_SMBUS_DEV,0x0A,0x04);
		/* Switch to parasitic mode */
		temp_smbus_write(chan,TEMPSENSOR_SMBUS_DEV,9,0x10);
		}
#endif    
	    }
       } while (0);

    if (temp_showtemp(1) < 0) {
	TIMER_CLEAR(temp_timer);
	return -1;
	}

    if (cmd_sw_isset(cmd,"-continuous")) {
	TIMER_SET(temp_timer,2*CFE_HZ);
	}
    if (cmd_sw_isset(cmd,"-stop")) {
	TIMER_CLEAR(temp_timer);
	}

    return 0;
}

/*  *********************************************************************
    *  temp_timer_proc()
    *  
    *  So we can be fancy and log temperature changes as they happen.
    *  
    *  Input parameters: 
    *  	   nothing
    *  	   
    *  Return value:
    *  	   nothing
    ********************************************************************* */

void temp_timer_proc(void *arg)
{
    if (!TIMER_RUNNING(temp_timer)) return;

    if (TIMER_EXPIRED(temp_timer)) {
	temp_showtemp(0);
	TIMER_SET(temp_timer,2*CFE_HZ);
	}
}
#endif
