#!/bin/bash
# author: Robert Penicka
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install"

sudo apt-get -y update -qq
sudo apt-mark hold openssh-server

# 20.04 problem fix
sudo apt-get -y install grub-efi
sudo update-grub

# the "gce-compute-image-packages" package often freezes the installation at some point
# the installation freezes when it tries to manage some systemd services
# this attempts to install the package and stop the problematic service during the process
((sleep 90 && (sudo systemctl stop google-instance-setup.service && echo "gce service stoped" || echo "gce service not stoped")) & (sudo timeout 120s apt-get -y install gce-compute-image-packages)) || echo "\e[1;31mInstallation of gce-compute-image-packages failed\e[0m"

sudo apt-get -y upgrade --fix-missing

sudo apt-get -y install dpkg git

sudo pip3 install gitman
sudo -H pip3 install gitman

echo "clone uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core.git
cd uav_core
gitman install

echo "install part ended"
