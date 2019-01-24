/* Compiled from cpu_spd.c


WIND_BASE=/projects/ntsw-sw/wrs/tornado/t2
export WIND_BASE

WIND_HOST_TYPE=sun4-solaris2
export WIND_HOST_TYPE

WIND_HOST_BIN=${WIND_BASE}/host/${WIND_HOST_TYPE}/bin
export WIND_HOST_BIN

PATH=/bin:/usr/bin:/usr/local/bin:$WIND_HOST_BIN
export PATH

ccppc -ffixed-31 -ffixed-30 -ffixed-29 -ffixed-28 -ffixed-27 -ffixed-26 -ffixed-25 -ffixed-24 -ffixed-23 -fno-builtin -nostdinc -fomit-frame-pointer -fverbose-asm -O3 -DCPU=PPC603 -I${SDK}/include -I${SDK}/systems/vxworks/bmw/mpc824x -I${SDK}/systems/vxworks/bmw/mpc824x/all -I${WIND_BASE}/target/h -DBMW -DVXWORKS -DVX_VERSION=54 -DBROADCOM_BSP -DSOC_CM_MEMORY_BASE=bcm_bde_soc_cm_memory_base -DEXTERN_SOC_CM_MEMORY_BASE=bcm_bde_soc_cm_memory_base -DSOC_CM_MEMORY_OFFSET=0x00010000  -DINCLUDE_L3  -DINCLUDE_I2C  -DINCLUDE_BCMX  -DINCLUDE_MEM_SCAN  -DINCLUDE_EDITLINE  -DINCLUDE_TCL  -DINCLUDE_CUSTOMER  -DINCLUDE_TELNET  -DBCM_RPC_SUPPORT  -DBCM_ESW_SUPPORT  -DBCM_MODENA_SUPPORT -DINCLUDE_LIB_CPUDB -DINCLUDE_LIB_CPUTRANS -DINCLUDE_LIB_DISCOVER -DINCLUDE_LIB_STKTASK  -DVENDOR_CALHOUN  -DVENDOR_GAMMA  -DINCLUDE_PHY_522X  -DINCLUDE_PHY_54XX  -DINCLUDE_PHY_5464  -DINCLUDE_PHY_5421S  -DINCLUDE_PHY_SERDES  -DINCLUDE_PHY_SIMUL  -DINCLUDE_PHY_8703 -DBCM_ALL_CHIPS -DTRUE=1 -DBE_HOST=1 -DBOOTROM=1  -Wall -Werror  -I${SDK}/include -I${SDK}/systems -c ${SDK}/src/appl/diag/cpu_i2c.c -S -o /tmp/cpu_i2c.s

The assembler output of cpu_i2c_spd is used after removing the prelude
and epilog dealing with saving state on the stack, as there *is* no
stack when this code is executed.
        
This code fragment is intended to be executed in-line, not as a subroutine.
The memory description found is returned in r3.

Uses registers r0,r3,r4,r9-11,r14-22
                
 */

#define SDRAM_SPD_ADDR 0xA8

        
cpu_i2c_spd:
        li 16, SDRAM_SPD_ADDR
	li 3,12296
	li 4,0
	bl eumbbar_write
	li 3,12292
	bl eumbbar_read
	ori 4,3,43
	li 3,12292
	bl eumbbar_write
	li 3,12288
	bl eumbbar_read
	ori 4,3,127
	li 3,12288
	bl eumbbar_write
	li 3,12300
	li 4,0
	bl eumbbar_write
	li 3,12296
	bl eumbbar_read
	ori 4,3,128
	li 3,12296
	bl eumbbar_write
	li 3,12296
	bl eumbbar_read
	andi. 0,3,128
	li 18,0
	li 10,-203
	mfcr 0
	rlwinm 0,0,3,1
	neg 0,0
	and. 22,0,10
	li 14,0
	li 17,0
	li 15,0
	bc 4,2,.L264
	li 3,12296
	bl eumbbar_read
	andi. 0,3,128
	li 22,-201
	bc 12,2,.L235
	li 3,12300
	bl eumbbar_read
	rlwinm 3,3,0,26,26
	neg 3,3
	srawi 3,3,31
	li 10,-202
	and 22,3,10
.L235:
	mr. 22,22
	bc 4,2,.L264
	li 19,0
	li 3,12296
	bl eumbbar_read
	ori 4,3,48
	li 3,12296
	bl eumbbar_write
	li 3,12304
	rlwinm 4,16,0,24,30
	li 21,0
	lis 20,0x6ff
	ori 20,20,65535
	bl eumbbar_write
.L245:
	li 3,12300
	bl eumbbar_read
	mr 22,3
	andi. 0,22,2
	bc 12,2,.L244
	li 3,12300
	li 4,0
	bl eumbbar_write
	andi. 0,22,16
	bc 12,2,.L247
	li 22,-207
	b .L241
.L247:
	andi. 0,22,128
	bc 4,2,.L248
	li 22,-208
	b .L241
.L248:
	rlwinm 0,22,0,31,31
	neg 0,0
	li 10,-206
	and 22,0,10
	b .L241
.L244:
	addi 21,21,1
	cmpw 1,21,20
	bc 4,5,.L245
	li 22,-200
.L241:
	nor 0,22,22
	srwi 0,0,31
	xori 9,19,1
	subfic 11,9,0
	adde 9,11,9
	and. 11,9,0
	bc 12,2,.L251
	li 3,12296
	bl eumbbar_read
	rlwinm 4,3,0,28,26
	li 3,12296
	bl eumbbar_write
	li 3,12304
	bl eumbbar_read
.L251:
	mr. 22,22
	bc 4,2,.L264
	li 3,12304
	li 4,0
	li 21,0
	lis 20,0x6ff
	ori 20,20,65535
	bl eumbbar_write
.L258:
	li 3,12300
	bl eumbbar_read
	mr 22,3
	andi. 0,22,2
	bc 12,2,.L257
	li 3,12300
	li 4,0
	bl eumbbar_write
	andi. 0,22,16
	bc 12,2,.L260
	li 0,-307
	b .L254
.L260:
	andi. 0,22,128
	bc 4,2,.L261
	li 0,-308
	b .L254
.L261:
	rlwinm 0,22,0,31,31
	neg 0,0
	li 10,-306
	and 0,0,10
	b .L254
.L257:
	addi 21,21,1
	cmpw 1,21,20
	bc 4,5,.L258
	li 0,-200
.L254:
	mr. 22,0
	bc 4,2,.L264
	li 3,12296
	bl eumbbar_read
	ori 4,3,52
	li 3,12296
	bl eumbbar_write
	li 3,12304
	rlwinm 4,16,0,24,30
	ori 4,4,1
	li 21,0
	lis 20,0x6ff
	ori 20,20,65535
	bl eumbbar_write
.L273:
	li 3,12300
	bl eumbbar_read
	mr 22,3
	andi. 0,22,2
	bc 12,2,.L272
	li 3,12300
	li 4,0
	bl eumbbar_write
	andi. 0,22,16
	bc 12,2,.L275
	li 22,-407
	b .L269
.L275:
	andi. 0,22,128
	bc 4,2,.L276
	li 22,-408
	b .L269
.L276:
	rlwinm 0,22,0,31,31
	neg 0,0
	li 10,-406
	and 22,0,10
	b .L269
.L272:
	addi 21,21,1
	cmpw 1,21,20
	bc 4,5,.L273
	li 22,-200
.L269:
	cmpwi 1,22,0
	bc 12,4,.L279
	li 3,12296
	bl eumbbar_read
	rlwinm 4,3,0,28,26
	li 3,12296
	bl eumbbar_write
	li 3,12304
	bl eumbbar_read
.L279:
	mr. 22,22
	bc 12,0,.L264
	li 3,12296
	bl eumbbar_read
	rlwinm 4,3,0,29,27
	li 3,12296
	li 19,0
	li 16,1
	bl eumbbar_write
	cmpwi 1,19,63
.L283:
	bc 4,6,.L284
	li 3,12296
	bl eumbbar_read
	ori 4,3,8
	li 3,12296
	li 15,1
	bl eumbbar_write
.L284:
	li 21,0
	lis 20,0x6ff
	ori 20,20,65535
	li 0,1
	cmpwi 2,0,0
.L292:
	li 3,12300
	bl eumbbar_read
	mr 22,3
	andi. 0,22,2
	bc 12,2,.L291
	li 3,12300
	li 4,0
	bl eumbbar_write
	andi. 0,22,16
	bc 4,2,.L327
	andi. 0,22,128
	bc 12,2,.L328
	li 0,0
	bc 4,10,.L288
	rlwinm 0,22,0,31,31
	neg 0,0
	li 10,-506
	and 0,0,10
	b .L288
.L291:
	addi 21,21,1
	cmpw 1,21,20
	bc 4,5,.L292
	li 0,-200
.L288:
	cmpwi 1,0,0
	bc 12,4,.L287
	li 3,12304
	bl eumbbar_read
	rlwinm 0,3,0,0xff
.L287:
	mr. 22,0
	bc 12,0,.L264
	cmpwi 1,19,5
	bc 12,6,.L310
	bc 12,5,.L320
	cmpwi 1,19,3
	bc 12,6,.L299
	cmpwi 1,19,4
	bc 12,6,.L303
	b .L298
.L328:
	li 0,-508
	b .L288
.L327:
	li 0,-507
	b .L288
.L320:
	cmpwi 1,19,17
	bc 12,6,.L314
	cmpwi 1,19,63
	bc 12,6,.L317
	b .L298
.L299:
	cmpwi 1,22,13
	bc 4,6,.L300
	ori 18,18,4096
	b .L298
.L300:
	cmpwi 1,22,12
	b .L329
.L303:
	addi 0,22,-8
	cmplwi 1,0,3
	bc 12,5,.L330
	andi. 0,18,4096
	bc 12,2,.L305
	slw 0,16,22
	or 18,18,0
	b .L298
.L305:
	cmpwi 1,22,8
	bc 12,6,.L330
	addi 0,22,-1
	slw 0,16,0
	or 18,18,0
	b .L298
.L310:
	cmpwi 1,22,2
	bc 4,6,.L311
	ori 18,18,8192
	b .L298
.L311:
	cmpwi 1,22,1
.L329:
	bc 12,6,.L298
.L330:
	li 22,-210
	b .L264
.L314:
	cmpwi 1,22,4
	bc 4,6,.L330
	ori 18,18,16384
	b .L298
.L317:
	rlwinm 14,22,0,0xff
.L298:
	addi 19,19,1
	neg 0,15
	srawi 0,0,31
	add 9,17,22
	cmpwi 1,19,63
	and 11,17,0
	andc 0,9,0
	or 17,11,0
	bc 4,5,.L283
	rlwinm 0,17,0,24,31
	xor 0,0,14
	addic 0,0,-1
	subfe 0,0,0
	nor 9,0,0
	li 10,-209
	and 9,9,10
	and 0,18,0
	or 22,0,9
.L264:
	li 3,12296
	bl eumbbar_read
	rlwinm 4,3,0,27,25
	li 3,12296
	bl eumbbar_write
	mr 3,22
cpu_i2c_spd_end:
