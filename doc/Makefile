# Try to get Kernel tree path from root directory
-include ../local.cfg
ifndef KERNELDIR
$(error Variable KERNELDIR is not set. Need a kernel tree for Device Tree headers!))
endif

# Paths
DTMERGE           := dtmerge
DTC               := dtc
INCLUDE_PATH      := $(KERNELDIR)/include/

# Filenames
OVERLAY_NAME  := SJA1110-EVM-Overlay
DTB_BASE_NAME := fsl-imx8qxp-mek-rpmsg
ifeq (,$(wildcard $(DTB_BASE_NAME).dtb))
$(error Base device tree ($(DTB_BASE_NAME).dtb) does not exist!))
endif

all: build merge

build:
	cpp -I$(INCLUDE_PATH) -E -P -x assembler-with-cpp $(OVERLAY_NAME).dts -o $(OVERLAY_NAME)_prep.dts
	$(DTC) -O dtb -o $(OVERLAY_NAME).dtbo -b 0 -@ $(OVERLAY_NAME)_prep.dts

merge:
	$(DTMERGE) $(DTB_BASE_NAME).dtb $(DTB_BASE_NAME)_merged.dtb $(OVERLAY_NAME).dtbo
