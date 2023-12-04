
CUR_DIR = $(PWD)
INCLUDE_COMMON := -I$(PWD)/../include/
ccflags-y := -std=gnu99 -Wno-declaration-after-statement  $(INCLUDE_COMMON)

SPECIAL_KDIR =
DEFAULT_KDIR = /lib/modules/$(shell uname -r)/build/
KDIR = 
USE_CROSS_COMPILE =
COMPILE_ARCH = arm64
COMPILE_CROSS = 

special_exist = $(shell if [ -d $(SPECIAL_KDIR) ]; then echo "exist"; else echo "noexist"; fi)
$(info $(special_exist))

ifneq ($(USE_CROSS_COMPILE), ) 
COMPILE_CROSS = aarch64-none-linux-gnu-
endif
ifneq ($(KERNELRELEASE),)
obj-m += clk-allwinner.o
else

ifneq ($(SPECIAL_KDIR),)
ifeq ("$(special_exist)", "exist")
KDIR = $(SPECIAL_KDIR)
endif
endif

ifeq ($(KDIR), )
KDIR  = $(DEFAULT_KDIR)
endif

.PHONY:  all  clean

all:
	make -C $(KDIR) M=$(PWD) modules  ARCH=$(COMPILE_ARCH) CROSS_COMPILE=$(COMPILE_CROSS)

clean:
	rm -rf  ./*.mod.c  ./*.o  ./*.order  ./*.symvers  ./*.cmd

endif







