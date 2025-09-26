---
title: Podman
pagination_label: Deploying using Podman
description: Deploying using Podman
---

# Deployment using Podman

Podman is a rootless alternative to Docker, it can run the default [example session](https://ctu-mrs.github.io/docs/deployment/docker/) with a compose provider or use Quadlets

Podman is daemonless so you won't be able to follow the instructions at [docker-host](https://ctu-mrs.github.io/docs/prerequisites/docker/docker-host), instead remote connections can be made through [ssh](https://ctu-mrs.github.io/docs/prerequisites/ssh/) by setting

```bash
sock=$(ssh uav30 "podman info --format '{{.Host.RemoteSocket.Path}}'")
export CONTAINER_HOST="ssh://mrs@uav30${sock}"
```

## Compose file

Put the following in your `~/.bashrc` to allow running Docker commands from the `up.sh` script:

```bash
alias docker='podman'
```

Install the [docker-compose-plugin](https://docs.docker.com/compose/install/linux/#install-using-the-repository) or [podman-compose](https://github.com/containers/podman-compose) package. If you want to use docker-compose, you will need to enable the podman.socket user unit and set docker socket environment variable for that user:

```bash
systemctl --user enable --now podman.socket
export DOCKER_HOST=unix://$XDG_RUNTIME_DIR/podman/podman.sock
```

If not, you may want to add `docker.io` to the unqualified-search-registries:

```bash
sudo tee /etc/containers/registries.conf.d/10-unqualified-search-registries.conf <<< 'unqualified-search-registries = ["docker.io"]'
```

Now you can follow the instructions from the [Docker](https://ctu-mrs.github.io/docs/deployment/docker/) page

## Quadlet

[To-do](https://wiki.archlinux.org/title/Podman#Quadlet)

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
