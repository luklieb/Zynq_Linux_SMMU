#!/bin/bash

RED='\033[0;31m'
NC='\033[0m'
GREEN='\033[0;32m'

qemu_location=/usr/bin/qemu-aarch64-static
cross_compile=/opt/Xilinx/SDK/2018.2/settings64.sh
if [ ! -e $cross_compile ]; then
	echo -e "${RED}No file to source cross compilation found${NC}"
	exit
fi

if [ ! -f ./resize.c ]; then
	echo -e "${RED}Please add source code to automatically resize the serial console in file resize.c${NC}"
	exit
fi

#device_tree=true
#if [ ! -f ./*.dtbo ] || [ ! -f ./*.bin ]; then
#	echo -e "${RED}No device tree blob or binary bitstream found. Do you want to abort?[y/n]${NC}"
#	read answer
#	if [ "$answer" = "y" ]; then
#		exit
#	fi
#	device_tree=false
#fi

firmware=true
if [ ! -d ./lib/firmware ]; then
	echo -e "${RED}No directory lib/firmware/ for device tree blob or binary bitstream found. Do you want to abort?[y/n]${NC}"
	read answer
	if [ "$answer" = "y" ]; then
		exit
	fi
	firmware=false
fi

home=true
if [ ! -d ./root ]; then
	echo -e "${RED}No directory root/ for user data found. Do you want to abort?[y/n]${NC}"
	read answer
	if [ "$answer" = "y" ]; then
		exit
	fi
	home=false
fi
		
wifi=true
if [ ! -d ./usr/share/wpa_ap ]; then
	echo -e "${RED}No wifi configuration found. Do you want to abort?[y/n]${NC}"
	read answer
	if [ "$answer" = "y" ]; then
		exit
	fi
	wifi=false
fi	

echo -e "${GREEN}checking for necessary packages${NC}"
apt-get install qemu-user-static debootstrap binfmt-support

targetdir=debian-rootfs
distro=stable

echo -e "${GREEN}Starting to debootstraß minimal variant of debian root fs${NC}"
mkdir $targetdir
debootstrap --arch=arm64 --foreign --variant=minbase $distro $targetdir http://deb.debian.org/debian/
cp $qemu_location $targetdir$qemu_location
cp second-stage.sh $targetdir
cp pkglist $targetdir

cp linux-{image,header}*.deb $targetdir/root

echo -e "${GREEN}Copying (if specified) wifi related stuff${NC}"
if $wifi; then
	cp  ./etc/init.d/ultra96-ap-setup.sh $targetdir/etc/init.d/
	cp -r ./usr/share/wpa_ap/ $targetdir/usr/share/
fi

echo -e "${GREEN}Starting chroot and executing second-stage.sh in new root${NC}"
chroot $targetdir/ ./second-stage.sh

echo -e "${GREEN}Back from chroot${NC}"
#if $device_tree; then
#	cp *.bin $targetdir/lib/firmware
#	cp *.dtbo $targetdir/lib/firmware
#fi
echo -e "${GREEN}Copying (if specified) root/ and lib/firmware/${NC}"
if $firmware; then
	cp -r ./lib/firmware/. $targetdir/lib/firmware
fi
if $home; then
	cp -r ./root/. $targetdir/root
fi

echo -e "${GREEN}Compiling and copying resize.c${NC}"
source $cross_compile
aarch64-linux-gnu-gcc -o resize2 -O3 resize.c
cp resize2 $targetdir/usr/bin

echo -e "${GREEN}Configuring SSHD${NC}"
cp etc/ssh/sshd_config $targetdir/etc/ssh/
echo -e "${RED}Copy your own public ssh key to directory /home/user/.ssh/authorized_keys.${NC}"
echo -e "${RED}Also check /etc/network/interfaces.d and replace all strings starting${NC}"
echo -e "${RED}with \"enx*\" with the actual name/mac-address of your Ethernet adpater (see ip a).${NC}"

echo -e "${GREEN}Cleaning up...${NC}"
rm $targetdir/second-stage.sh
rm $targetdir/usr/bin/qemu-aarch64-static
rm $targetdir/pkglist
rm $targetdir/root/linux-{image,header}*.deb

echo -e "${GREEN}compressing fs${NC}"
cd $targetdir
tar cfz ../debianfs.tgz *


echo -e "${GREEN}Done!${NC}"

echo -e "${RED}To format your SD card open fdisk /dev/sd*${NC}"
echo -e "${RED}then enter the following: o, n, p, 1, ,+100M, t, b, n, p, 2, , , p, w, q ${NC}"
echo -e "${RED}To create file systems: mkfs.vfat /dev/sd*1 and mkfs.ext3 /dev/sd*2${NC}"
echo -e "${RED}To rename them fatlabel /dev/sd*1 boot and e2label /dev/sd*2 root\n${NC}"

echo -e "${RED}Info: For installation run: tar xfz debianfs.tgz -C /mnt/root${NC}"
exit
