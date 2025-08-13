---
title: Docker-host 
pagination_label: Remote access to docker 
description: Remote access to docker
---

# Docker-host # 
  The `DOCKER_HOST` environment variable  specifies the location of the Docker
  daemon. In scenarios where the Docker daemon runs on a remote machine
  (robot), `DOCKER_HOST` allows clients to connect to it over TCP.
  
### Settings 
 First the remote machine (robot) must have Docker installed.
 To enable remote access to docker you need to:
 
```bash 
sudo systemctl edit docker.service
```
add the following:
 ```bash 
[Service]
ExecStart=
ExecStart= /usr/bin/dockerd -H fd:// -H tcp://0.0.0.0:2375 --containerd=/run/containerd/containerd.sock
```
and reload the service
```bash
sudo systemctl reload docker.service
```
### Usage 
  Set the `DOCKER_HOST` environment variable to point to the remote Docker
  daemon:

```bash 
export DOCKER_HOST=tcp://<remote-host-ip>:2375
```

Now you can use `docker` commands to manage containers, images and volumes on
the remote machine.

Notice that your local machine and the remote host must be on the same network.
