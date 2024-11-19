---
title: ROS Workspace caching
pagination_label: Building and caching of ROS workspace in Docker
description: How to build and chache a ROS workspace into a Docker image
---

# ROS Workspace caching with Docker

## Folder contents

<details>
<summary>Dockerfile</summary>
<p>

```dockerfile
ARG WORKSPACE_PATH
ARG BASE_IMAGE=ctumrs/mrs_uav_system:1.5.0_hailo_ai

#############################################################################

# FIRST STAGE: BUILD THE WORKSPACE
FROM $BASE_IMAGE AS stage_build

ARG WORKSPACE_PATH

COPY ./cache/${WORKSPACE_PATH}/ /${WORKSPACE_PATH}/

# create catkin workspace
RUN [ ! -e /${WORKSPACE_PATH}/.catkin_tools ] && cd /${WORKSPACE_PATH} && catkin init && catkin config --profile reldeb --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && catkin profile set reldeb && catkin config --extend /opt/ros/noetic || echo "[Docker]: catkin workspace already exists"

# copy the sources from the local subfolder
COPY src/ /${WORKSPACE_PATH}/src/

RUN cd /${WORKSPACE_PATH} && catkin build -s

#############################################################################

# SECOND STAGE: COPY THE WORKSPACE TO A BLANK APLINE IMAGE
FROM alpine AS stage_cache_workspace

ARG WORKSPACE_PATH

COPY --from=stage_build /${WORKSPACE_PATH} /${WORKSPACE_PATH}

#############################################################################

# THIRD STAGE: copy the workspace to a final blank ROS-equipped base image
FROM alpine AS stage_finalization

ARG WORKSPACE_PATH

COPY ./cache/${WORKSPACE_PATH}/src /${WORKSPACE_PATH}/src
COPY ./cache/${WORKSPACE_PATH}/.catkin_tools /${WORKSPACE_PATH}/.catkin_tools
COPY ./cache/${WORKSPACE_PATH}/devel /${WORKSPACE_PATH}/devel

CMD ["sh"]
```

</p>
</details>
<details>
<summary>build_image.sh</summary>
<p>

```bash
#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

cd ${MY_PATH}

## --------------------------------------------------------------
## |                            setup                           |
## --------------------------------------------------------------

LOCAL_TAG=catkin_workspace:yolo

ARCH=arm64 # robofly
# ARCH=amd64

source ./paths.sh

## --------------------------------------------------------------
## |                            build                           |
## --------------------------------------------------------------

# initialize the cache
[ ! -e ${CACHE_PATH}/${WORKSPACE_PATH} ] && mkdir -p ./${CACHE_PATH}/${WORKSPACE_PATH}

PASS_TO_DOCKER_BUILD="Dockerfile src ${CACHE_PATH}/${WORKSPACE_PATH}"

docker buildx use default

echo ""
echo "$0: building the user's workspace"
echo ""

# this first build compiles the contents of "src" and storest the intermediate
tar -czh $PASS_TO_DOCKER_BUILD 2>/dev/null | docker build - --no-cache --target stage_cache_workspace --output ./${CACHE_PATH} --build-arg WORKSPACE_PATH=${WORKSPACE_PATH} --file Dockerfile --platform=linux/$ARCH

echo ""
echo "$0: packing the workspace into a docker image"
echo ""

# this second build takes the resulting workspace and storest in in a final image
# that can be deployed to a drone
docker build . --no-cache --target stage_finalization --file Dockerfile --build-arg WORKSPACE_PATH=${WORKSPACE_PATH} --tag $LOCAL_TAG --platform=linux/$ARCH

echo ""
echo "$0: workspace was packed into '$LOCAL_TAG'"
echo ""
```

</p>
</details>
<details>
<summary>build_image.sh</summary>
<p>

```bash
export WORKSPACE_PATH=etc/docker/catkin_workspace
export CACHE_PATH=cache

```

</p>
</details>
