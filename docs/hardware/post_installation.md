---
layout: default
title: Post installation guide
parent: Hardware
---

# Linux SWAP size increase

TODO

```bash
~/git/uav_core/miscellaneous/scripts/set_swap.sh
```
OR
```bash
sudo swapoff -a
sudo dd if=/dev/zero of=/swapfile bs=1G count=16
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
grep SwapTotal /proc/meminfo
```

# Disabling automatic sleep/hibernation

TODO

```bash
~/git/uav_core/miscellaneous/scripts/disable_hibernation.sh
```
OR
```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

# Change network interface names to the `nice ones`

TODO

Check [http://github.com/ctu-mrs/uav_core/miscellaneous/network_settings/README.md][http://github.com/ctu-mrs/uav_core/miscellaneous/network_settings/README.md]

# Disabling network manager

TODO

```bash
~/git/uav_core/miscellaneous/scripts/disable_network_manager.sh
```
OR
```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

# udev rules

TODO

## FTDI

## RPLidar

## Arduino

# Time synchronization

TODO

# NUC Power limits in BIOS

TODO

# Setup and validate .bashrc

TODO
