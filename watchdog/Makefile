

obj-m += wdt-allwinner.o

CUR_DIR = $(PWD)
INCLUDE_COMMON := -I$(PWD)/../include/

ccflags-y += -std=gnu99 -Wno-declaration-after-statement  $(INCLUDE_COMMON)

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules

clean:
	rm -rf  ./*.mod.c  ./*.o  ./*.order  ./*.symvers  ./*.cmd





