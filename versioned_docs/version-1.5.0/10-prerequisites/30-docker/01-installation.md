---
title: Installation
pagination_label: Installing Docker
description: Learn how to install Docker
---

The following guide is a digest of the official installation guide at [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/).
Please follow the official guide for other operating systems than Ubuntu OS.

# Installation on Ubuntu

## 1. Setup Docker's APT repository:

Add docker's GPG key:
```bash
sudo apt-get -y update
sudo apt-get -y install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

Add Docker's repository to the APT sources:
```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get -y update
```

## 2. Install the Docker engine

```bash
sudo apt-get -y install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
