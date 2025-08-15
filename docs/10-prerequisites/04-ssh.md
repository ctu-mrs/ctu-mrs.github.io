---
title: SSH
pagination_label: Remote access using ssh
description: Remote access using ssh
---

# SSH

SSH (Secure Shell) is a network protocol that enables secure remote access to your drone. When properly configured, you can run commands like this, which will clone the mrs uav system on the drone.

```bash
ssh uav1 'git clone github.com:ctu-mrs/mrs_uav_system.git'
```

This page will guide you in getting ssh working between your computer and your done.

## Configure network

We recommend setting static ip's for your devices, in this guide, we'll assume that on your network:

* Your computer's ip is `192.168.0.11`, with `pc1` as the hostname.

* Your drone's ip is `192.168.0.100`, with `uav1` as the hostname and a `mrs` user on it.

For more info, please check [this page](https://linuxconfig.org/setting-a-static-ip-address-in-ubuntu-24-04-via-the-command-line).

## Configure hostnames

### On your machine

Add your drone's hostname to `/etc/hosts`, your file should look something like this:

```bash
127.0.0.1 localhost
127.0.1.1 pc1

192.168.0.100 uav1
```

### On the drone

Add your machine's hostname in `/etc/hosts`:

```bash
127.0.0.1 localhost
127.0.1.1 uav1

192.168.0.10 pc1
```

We recommend removing any other IPv4 loopbacks on the drone besides these.

### Test connection

Make sure the machines can reach each other using their hostnames:

```bash
ping -c 1 uav1
nc -z uav1 22
```

## Configure SSH

Now that the connection is working, you can proceed to setup SSH.

### Generate SSH key pair

Run this command, which will create a key pair in `~/ssh`:

```bash
ssh-keygen -t rsa -b 4096 -C 'youremail@example.com'
```

`id_rsa` is your private key which should never be shared, `id_rsa.pub` however is public and used to verify if the ssh connection really came from you(r private key).

### Copy public key

Show the content of your public key by running: 

```bash
cat ~/.ssh/id_rsa.pub
```

Copy this key to your GitHub account [here](https://github.com/settings/keys) and to the drone by running:

```bash
ssh-copy-id -i ~/.ssh/id_rsa mrs@uav1
```

You'll be prompted for the password of the `mrs` user and your public key will be added to `~/.ssh/authorized_keys`.

### Edit SSH config

In ∼/.ssh/config you can set aliases for ssh connections which will set the hostname, user and key used when connecting to them. You can set it on both your machines like this:

```bash
host uav1
  hostname uav1
  user mrs
  identityfile ~/.ssh/id_rsa

host github.com
  hostname github.com
  user git
  identityfile ~/.ssh/id_rsa
```

### Test connection

Now you should be able to ssh to your drone without using your password

```bash
ssh uav1 'echo Hello from the drone!'
```

## More info

Unless you also configure an ssh server on your machine, you won't be able to ssh to it from the drone. To copy files from the drone, you should instead run a command like this on your machine:

```bash
scp -rPC uav1:/tmp/file.txt .
rsync -azP uav1:/tmp/file.txt .
```

You may disable ssh password authentication for added security, more info about it [here](https://serverpilot.io/docs/guides/ssh/password-auth/)

Check out our [Cheatsheet](https://github.com/ctu-mrs/mrs_cheatsheet) for some useful tips.
