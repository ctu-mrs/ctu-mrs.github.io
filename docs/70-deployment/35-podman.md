---
title: Podman
pagination_label: Deploying using Podman
description: Deploying using Podman
---

# Deployment using Podman

Podman is a rootless alternative to Docker, it's possible to run the default [example session](https://ctu-mrs.github.io/docs/deployment/docker/) using a compose provider or to use Quadlets

## Compose file

Put the following in your `~/.bashrc` to allow running Docker commands from the `up.sh` script:

```bash
alias docker='podman'
```

Install the [docker-compose-plugin](https://docs.docker.com/compose/install/linux/#install-using-the-repository) or [podman-compose](https://github.com/containers/podman-compose) package

Now you can follow the instructions from the [Docker](https://ctu-mrs.github.io/docs/deployment/docker/) page

## Quadlet

## Useful info

To transfer images between Docker and Podman, you have several options, you can save and load it:

```bash
docker save ctumrs/mrs_uav_system:stable > mrs.tar
podman load -i mrs.tar
```

You can also do that with no temporary file and remotely with SSH

```bash
ssh uav30 'docker save ctumrs/mrs_uav_system:stable | podman load'
```

Or you can setup a [local registry](https://ctu-mrs.github.io/docs/prerequisites/docker/registries#using-a-local-docker-registry) in Docker, add it as an insecure registry

```bash
sudo tee /etc/containers/registries.conf.d/mrs.conf <<EOF
[[registry]]
location = "localhost:5000"
insecure = true
EOF
```

And pull the image into Podman:

```bash
podman pull localhost:5000/ctumrs/mrs_uav_system:latest
```
