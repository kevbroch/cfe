#
# Device Drivers
#

DEVOBJS += dev_newflash.o  dev_null.o dev_promice.o \
	   dev_ide_common.o dev_ns16550.o dev_ds17887clock.o dev_ds1743.o \
	   dev_bcm5780_ht1000.o dev_ide_frodo.o

ALLSMBDEVS = 24lc128 x1241clock m41t81clock x1240eeprom at24c02

SMBDEVS ?= ${ALLSMBDEVS}
DEVOBJS += $(patsubst %,dev_smbus_%.o,${SMBDEVS})
