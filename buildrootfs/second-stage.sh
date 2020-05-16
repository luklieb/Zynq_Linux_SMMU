#!/bin/bash

RED='\033[0;31m'
NC='\033[0m'
GREEN='\033[0;32m'

# Ultra96 specific
serialtty=PS0
bootmmc=mmcblk0p1
ntpserver=ntp0.fau.de

echo -e "${GREEN}Welcome to the second stage...${NC}"
distro=stable
export LANG=C

echo -e "${GREEN}debootstrap second-stage. This can take a little...${NC}"
/debootstrap/debootstrap --second-stage


echo -e "${GREEN}Adding new sources${NC}"
cat <<EOT > /etc/apt/sources.list
deb http://deb.debian.org/debian/ stable main contrib non-free
deb-src http://deb.debian.org/debian/ stable main contrib non-free

deb http://deb.debian.org/debian/ stable-updates main contrib non-free
deb-src http://deb.debian.org/debian/ stable-updates main contrib non-free

deb http://deb.debian.org/debian-security stable/updates main
deb-src http://deb.debian.org/debian-security stable/updates main

deb http://ftp.debian.org/debian stretch-backports main
deb-src http://ftp.debian.org/debian stretch-backports main
EOT

echo -e "${GREEN}Installing necessary packages. This can take a while...${NC}"
apt-get update

xargs apt-get install -y < pkglist
dpkg-reconfigure locales
dpkg-reconfigure tzdata

echo -e "${GREEN}Setting new hostname${NC}"
echo debian-fpga > /etc/hostname
passwd

echo -e "${GREEN}Adding serial tty to securetty${NC}"
cat <<EOT >> /etc/securetty
# Seral Port for Ultra96
tty$serialtty
EOT



echo -e "${GREEN}Automatically mounting boot/ and config/${NC}"
cat <<EOT > /etc/fstab
/dev/$bootmmc	/boot	auto		defaults	0	0
none		/config	configfs	defaults	0	0
EOT

echo -e "${GREEN}Setting ntp server${NC}"
sed -i "/#server /a server $ntpserver" /etc/ntp.conf

echo -e "${GREEN}Setting interfaces eth0 and enx0050b622e26 for the USB ethernet adapter${NC}"
echo -e "${GREEN}if you use a different USB ethernet adapter, you should change its interface name accordingly${NC}"
cat <<EOT > /etc/network/interfaces.d/eth0
allow-hotplug eth0
iface eth0 inet dhcp
EOT
cat <<EOT > /etc/network/interfaces.d/enx0050b6228e26
allow-hotplug enx0050b6228e26
iface enx0050b6228e26 inet dhcp
EOT

mkdir /lib/firmware

echo -e "${GREEN}Installing Linux image and headers debian packages${NC}"
mv boot boot.org
mkdir boot
dpkg -i /root/linux-image*.deb
dpkg -i /root/linux-headers*.deb
rm boot/*
rmdir boot
mv boot.org boot

echo -e "${GREEN}Configuring wifi related stuff${NC}"
update-rc.d ultra96-ap-setup.sh defaults
systemctl disable isc-dhcp-server
touch /var/lib/dhcp/dhcpd.leases

echo -e "${GREEN}Setting up set time scripts and services${NC}"
cat <<EOT >> /etc/systemd/system/set-time.service
[Unit]
Description=Set Time with sntp and ntp0.fau.de
After=network.target auditd.service

[Service]
Type=oneshot
ExecStartPre=/bin/sleep 10
ExecStart=/usr/bin/set_time.sh

[Install]
WantedBy=graphical.target
EOT

cat <<EOT >> /usr/bin/set_time.sh
#! /bin/sh
echo -n "current time before: $(date)"
sntp -Ss ntp0.fau.de
echo -n "current time after: $(date)"
exit 0
EOT
systemctl enable set-time.service

chmod +x /usr/bin/set_time.sh


echo -e "${GREEN}Creating regular user account called "user"${NC}"
adduser user
usermod -aG sudo user

echo -e "${GREEN}cleaning...${NC}"
apt-get clean
exit
