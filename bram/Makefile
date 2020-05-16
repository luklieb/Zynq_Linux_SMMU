

%.bit.bin:
	$(eval target := $(shell echo $@ | sed 's/....$$//'))
	sed -i -e "s/\[destination_device = pl].*/[destination_device = pl] $(target)/g" bit_to_bin.bif
	bootgen -arch zynqmp -image bit_to_bin.bif -w -o $@

%.bin:
	#source /opt/Xilinx/Vivado/2018.2/settings.sh
	bootgen -arch zynqmp -image boot.bif -w -o $@
