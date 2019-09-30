# Needs to define KERNELDIR, TARGET_ARCH, LOCAL_TOOLCHAIN, LOCAL_COMPILER
-include local.cfg

# Check setup
ifeq ($(PWD), $(shell pwd))
  ifeq ($(and $(KERNELDIR),$(TARGET_ARCH),$(LOCAL_TOOLCHAIN),$(LOCAL_COMPILER)),)
  $(error One of the required variables {KERNELDIR, TARGET_ARCH, LOCAL_TOOLCHAIN, LOCAL_COMPILER} is not set!))
  endif
endif

PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(TARGET_ARCH) CROSS_COMPILE=$(LOCAL_TOOLCHAIN) CC=$(LOCAL_COMPILER) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(TARGET_ARCH) CROSS_COMPILE=$(LOCAL_TOOLCHAIN) CC=$(LOCAL_COMPILER) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

.PHONY: modules modules_install clean

obj-m += sja1110.o

sja1110-y := sja1110_init.o
