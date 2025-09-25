---
title: Kubernetes
pagination_label: Deploying using Kubernetes
description: Deploying using Kubernetes
---

# Deployment using Kubernetes

Kubernetes, also known as K8s, is an open source system for automating deployment, scaling, and management of containerized applications.

This page will describe how to setup a Kubernetes controller on your machine and a worker node on your drone to run the MRS UAV System, you could potentially use this setup to deploy a swarm of drones, each with their own worker agents.

## Install Kubernetes and Helm

The following command will install [K3s](https://docs.k3s.io/), a lightweight distro of Kubernetes. Alternatively, you can use [K0s](https://docs.k0sproject.io/stable/), [KubeSolo](https://www.kubesolo.io/documentation) or [Talos](https://www.talos.dev/).

```bash
curl -sfL https://get.k3s.io | K3S_KUBECONFIG_MODE="644" INSTALL_K3S_EXEC="server" sh -
```

For some commands to work properly, you should configure [cluster access](https://docs.k3s.io/cluster-access). Put the following in your `~/.bashrc`

```bash
alias kubectl='k3s kubectl'
export KUBECONFIG=/etc/rancher/k3s/k3s.yaml
```

Next, install [Helm](https://helm.sh/docs/), the package manager for K8s, letting you template manifest files and put things like environment variables, which is unfortunately not supported out of the box:

```bash
curl -sfL https://raw.githubusercontent.com/helm/helm/main/scripts/get-helm-3 | bash
```

## Get the mrs_docker repo

Clone the [mrs_docker](https://github.com/ctu-mrs/mrs_docker) repository, then under [deployment/ros2/kubernetes](https://github.com/ctu-mrs/mrs_docker/tree/master/deployment/ros2/kubernetes) you'll find `Chart.yaml` which is used to denote a Helm chart, `up.sh` and `down.sh` wrappers for Kubernetes commands, `shared_data` and the `templates` folder where the cluster configuration is.

## Configure environment variables 

In the `values.yaml` file you can configure deployment variables (`mrsUavSystemVersion` and `homeFolder`), as well as environment variables that will be set in every pod, this is passed by helm to `templates/config.yaml`, the file is also sourced in `up.sh` and `down.sh`.

For more info about setting the environment variables, check the [native installation](https://ctu-mrs.github.io/docs/deployment/native/bashrc_configuration#bashrc-for-a-real-uav).

## Deploy to Kubernetes

Before you start the session, you should edit `templates/deployment.yaml` according to your needs, for example to add or remove sensors your drone uses. The main image used will be [ctumrs/mrs_uav_system](https://hub.docker.com/r/ctumrs/mrs_uav_system).

### On the drone

You can use `./up.sh` to start a session in the background and `./down.sh` to end it.

### Useful info

By default, k3s uses containerd, it can be switched to Docker by adding `--docker` to the install command as detailed [here](https://docs.k3s.io/advanced#using-docker-as-the-container-runtime).

To transfer images between Docker and containerd, you have several options, you can save and import it:

```bash
docker save ctumrs/mrs_uav_system:stable > mrs.tar
sudo ctr images import mrs.tar
```

You can also do that with no temporary file and remotely with SSH

```bash
ssh uav30 'docker save ctumrs/mrs_uav_system:stable | sudo ctr images import -'
```

Or you can setup a [local registry](https://ctu-mrs.github.io/docs/prerequisites/docker/registries#using-a-local-docker-registry) in Docker, set it as a [private registry](https://docs.k3s.io/installation/private-registry) and pull the image into containerd:

```bash
sudo ctr images pull localhost:5000/ctumrs/mrs_uav_system:latest
```

If your ip changes, you may have issues with your nodes. To fix it you can set a static ip and restart k3s:

```bash
sudo tee /etc/rancher/k3s/config.yaml <<< 'node-ip: "192.168.0.105"'
sudo systemctl restart k3s
```

A popular cluster visualiser is [k9s](https://github.com/derailed/k9s).
