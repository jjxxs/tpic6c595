SRC := $(shell pwd)
KERNEL_SRC := /usr/src/kernel
INCLUDES = -I. -I$(KERNEL_SRC)/include
KBUILD_CFLAGS += -g

obj-m := tpic6c595.o

CFLAGS_tpic6c595.o = -I$(src)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers