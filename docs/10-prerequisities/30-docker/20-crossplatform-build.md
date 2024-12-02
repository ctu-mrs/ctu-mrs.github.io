---
title: Crossplatform build
pagination_label: Building crossplatform Docker images
description: How to build crossplatform Docker images
---

# Crossplatform docker build

A docker image can be build for a desired platform by adding the `--platform` parameter:

```bash
docker build . --file Dockerfile --tag <my_tag> --platform=arm64
```

The crossplatform build is facilitated by QEMU.
Expect a run-time approx. 10x slower than for your native platform.

## Multi-platform docker build

Multi-platform images can be built using the `dockerx` multiplatform builder.
First, instantiate a multiplatform builder and set it to be the default (`--use`):

```bash
docker buildx create --name container-builder --driver docker-container --bootstrap --use
```

Then, build an image using the `buildx` command (which uses the selected builder) and supply the `--platform` parameter with the list of the desired platforms.
We usually `--push` the image immediately after the build into a docker registry.

```bash
docker buildx build . --file Dockerfile --tag <my_tag> --platform=linux/arm64,linux/amd64 --push
```

Do not forget to change the default builder back into the original builder (probably `default`) after you finish with the multiplatform build.

### Listing the available docker builders

```bash
docker buildx list
```

```
NAME/NODE                DRIVER/ENDPOINT                   STATUS    BUILDKIT   PLATFORMS
container-builder        docker-container
 \_ container-builder0    \_ unix:///var/run/docker.sock   running   v0.16.0    linux/amd64, linux/amd64/v2, linux/amd64/v3, linux/amd64/v4, linux/arm64, linux/riscv64, linux/ppc64, linux/ppc64le, linux/s390x, linux/386, linux/mips64le, linux/mips64, linux/arm/v7, linux/arm/v6
 default*                 docker
  \_ default               \_ default                       running   v0.16.0    linux/amd64, linux/amd64/v2, linux/amd64/v3, linux/amd64/v4, linux/arm64, linux/riscv64, linux/ppc64, linux/ppc64le, linux/s390x, linux/386, linux/mips64le, linux/mips64, linux/arm/v7, linux/arm/v6
````

If you want to switch back to the default docker builder, select it then by issuing
```bash
docker buildx use default
```
