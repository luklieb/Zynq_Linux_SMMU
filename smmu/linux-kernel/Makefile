SHELL:=/bin/bash

module_header=hsa-smmu.h
files=arm-smmu-regs.h,io-pgtable-arm.c,hsa-smmu.c
lnx_header_dir=../../linux-xlnx-v2018.2-zynqmp-fpga/include/linux
driver_dir=../../linux-xlnx-v2018.2-zynqmp-fpga/drivers/iommu
mmap_dir=../../linux-xlnx-v2018.2-zynqmp-fpga/arch/arm64/kernel
fpga_dir=../../linux-xlnx-v2018.2-zynqmp-fpga/drivers/fpga

all:
	@echo Error: Please specify direction [get,put] for driver src file movement

%.dts:
	$(eval input := $(shell ls $$(echo $@ | cut -f1 -d".").dtb*))
	@echo $(input)
	dtc -O dts -@ -o $@ $(input)

%.dtb:
	dtc -O dtb -@ -b 0 -o $@ system-top.dts

%.dtbo:
	dtc -O dtb -@ -b 0 -o $@ pl.dtsi

%.bit.bin:
	$(eval target := $(shell echo -n $@ | sed 's/....$$//'))
	sed  -i -e "s/\[destination_device = pl].*/[destination_device = pl] $(target)/g" bit_to_bin.bif
	bootgen -image bit_to_bin.bif -arch zynqmp -w -o $@

%.bin:
	#source /opt/Xilinx/Vivado/2018.2/settings.sh
	bootgen -arch zynqmp -image boot.bif -w -o $@

get:
	cp ${driver_dir}/{${files}} .
	cp ${lnx_header_dir}/${module_header} .

put:	
	cp ./{${files}} ${driver_dir}
	cp ./${module_header} ${lnx_header_dir}
	patch -d ${driver_dir} -p5 Makefile < iommu_makefile.patch
	patch -d ${driver_dir} -p1 Kconfig < iommu_kconfig.patch
	patch -d ${mmap_dir} -p1 sys.c < linux_mmap.patch
	patch -d ${fpga_dir} -p1 fpga-mgr.c < fpga-mgr.patch
	patch -d ${fpga_dir} -p1 zynqmp-fpga.c < zynqmp.patch

