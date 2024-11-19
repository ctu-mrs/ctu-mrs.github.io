---
title: Building images
pagination_label: Building Docker images
description: How to build docker images.
---

# Building images

Follow the [official guide](https://docs.docker.com/build/) for more in-depth information.

### Building a Docker Image from a Dockerfile

A Dockerfile is a text file containing a series of instructions to create a Docker image.
Here's how to build an image using a Dockerfile.

1. **Create a Dockerfile**:
   Write a `Dockerfile` with the necessary instructions for your application. Example:
   ```dockerfile
   # Use the MRS UAV System the base image
   FROM ctumrs/mrs_uav_system:latest
   
   # Install additional packages into the image
   RUN sudo apt-get -y install <my_dependency>

   # Specify the default command to run
   CMD ["/ros_entrypoint.sh"]
   ```
2. **Save the Dockerfile:** Save the Dockerfile in the root directory of your project.
3. **Build the Docker Image:** Use the docker build command to build the image. Run this command in the same directory as your Dockerfile:
```bash
docker build -t <image-name>:<tag> .
```
Example:
```bash
docker build -t my-app:latest .
```

* The `-t` flag assigns a name and tag to your image (e.g., my-app:latest).
* The `.` at the end specifies the build context (the current directory).

## Dockerfile for the MRS UAV System

```dockerfile
FROM ctumrs/ros:noetic

RUN apt-get -y update

# workaround interractive prompts during apt installations
RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install keyboard-configuration

# INSTALL the MRS UAV System

RUN apt-get -y install software-properties-common curl bash

RUN curl https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash

RUN apt-get -y install ros-noetic-mrs-uav-system-full

CMD ["/ros_entrypoint.sh"]
```
