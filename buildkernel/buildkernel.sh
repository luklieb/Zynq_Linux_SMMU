#!/bin/bash

LINUX_BUILD_DIR=../linux-xlnx-v2018.2-zynqmp-fpga
cd $LINUX_BUILD_DIR
CURR_TAG=$(git describe --tags --abbrev=0)
TARGET_TAG="xlnx-v2018.3-zynqmp-fpga"

if [ "$CURR_TAG" != $TARGET_TAG ]; then
	patch -p1 < ../buildkernel/files/linux-xlnx-builddeb.diff
	git add --update
	git commit -m "[fix] build wrong architecture debian package when ARCH=arm64 and cross compile."

	### Create tag and .version
	git tag -a $TARGET_TAG -m "release xilinx-v2018.3-zynqmp-fpga"
	echo 0 > .version
fi

### Setup for Build 
export ARCH=arm64
export export CROSS_COMPILE=aarch64-linux-gnu-
source /opt/Xilinx/Vivado/2018.3/settings64.sh
make xilinx_zynqmp_defconfig

### Build Linux Kernel and device tree
export DTC_FLAGS=--symbols
echo "How do you want to build it?"
select opt in "regular" "menuconfig" "manual" "cae00"; do
	case $opt in
		regular ) 
			echo "building deb packages now"
			make $1 LOCALVERSION="-$TARGET_TAG" deb-pkg;
			break;;
		menuconfig ) 
			echo "menuconfig opens now"
			make menuconfig;
			make $1 LOCALVERSION="-$TARGET_TAG" deb-pkg;
			break;;
		manual )
			echo "Copy kernel .config file to the correct location. Once done, come back here and press return"; 
			read ok
			echo "Build of deb-pkg starting now"
			make $1 LOCALVERSION="-$TARGET_TAG" deb-pkg;
			break;;
		cae00 )
			echo "Copy kernel .config file to the correct location. Once done, come back here and press return";
			read ok
			echo "Build w/o root permission starting now"
			nice -10 make $1 LOCALVERSION="-$TARGET_TAG";
			break;;
	esac
done
