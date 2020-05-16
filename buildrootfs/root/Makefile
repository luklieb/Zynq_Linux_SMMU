
syspath=/sys/class/fpga_manager/fpga0
overlaypath=/config/device-tree/overlays

%.bin:
	echo 0 > $(syspath)/flags
	echo $@ > $(syspath)/firmware

%.dtbo:
	echo 0 > $(syspath)/flags
	mkdir $(overlaypath)/full
	echo -n "$@" > $(overlaypath)/full/path

remove:
	rmdir $(overlaypath)/full
