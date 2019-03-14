KERNEL_SRC = ../linux-xlnx
BUILD_DIR := $(shell pwd)

MODULES = zgpio.o

obj-m := $(MODULES)

ARCH = arm
MAKEARCH = $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

all:
	$(MAKEARCH) -C $(KERNEL_SRC) SUBDIRS=$(BUILD_DIR) modules

clean:
	rm -f *.o
	rm -f *.ko
	rm -f *.mod.c
