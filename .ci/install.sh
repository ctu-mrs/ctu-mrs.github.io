#!/bin/bash
set -e

echo "Starting install"

sudo apt-get -y install git python3 python3-pip

sudo pip3 install gitman
sudo -H pip3 install gitman

echo "clone mrs_uav_core"
cd
git clone https://github.com/ctu-mrs/mrs_uav_core.git
cd mrs_uav_core
gitman install

echo "install part ended"
