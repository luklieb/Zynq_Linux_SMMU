%.dts:
	$(eval input := $(shell ls $$(echo $@ | cut -f1 -d".").dtb*))
	@echo $(input)
	dtc -O dts -@ -o $@ $(input)

%.dtb:
	dtc -O dtb -@ -b 0 -o $@ system-top.dts

%.dtbo:
	dtc -O dtb -@ -b 0 -o $@ pl.dtsi

# Create foo.bit.bin from foo.bit
%.bit.bin:
	# source /opt/Xilinx/Vivado/2018.2/settings.sh
	$(eval target := $(shell echo -n $@ | sed 's/....$$//'))
	sed  -i -e "s/\[destination_device = pl].*/[destination_device = pl] $(target)/g" bit_to_bin.bif
	bootgen -arch zynqmp -image bit_to_bin.bif -w -o $@

# Create foo.bin from file names specified in boot.bif
%.bin:
	# source /opt/Xilinx/Vivado/2018.2/settings.sh
	bootgen -arch zynqmp -image boot.bif -w -o $@
