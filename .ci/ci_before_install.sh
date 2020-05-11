#!/bin/bash
# author: Robert Penicka
set -e

echo "Starting install preparation" 
openssl aes-256-cbc -K $encrypted_f2b1af48ae35_key -iv $encrypted_f2b1af48ae35_iv -in ./.ci/deploy_key_github.enc -out ./.ci/deploy_key_github -d
eval "$(ssh-agent -s)"
chmod 600 ./.ci/deploy_key_github
ssh-add ./.ci/deploy_key_github
sudo apt-get update -qq
sudo apt-get install dpkg git python-setuptools python3-setuptools python-pip python3-pip python-pip

sudo pip3 install gitman
sudo -H pip3 install gitman

echo "clone uav_core"
cd
git clone git@github.com:ctu-mrs/uav_core.git
cd uav_core
gitman install

echo "clone simulation"
cd 
git clone git@github.com:ctu-mrs/simulation.git 
cd simulation
gitman install

echo "install part ended"
