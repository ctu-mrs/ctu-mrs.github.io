---
title: Docker Registries
pagination_label: Using Docker Registries
description: Using docker Registries
---

# Docker Registries

Docker registries are storage and distribution systems for Docker images.
They allow you to share images with others or deploy them across different environments.
The default Docker registry is **Docker Hub**, but you can also use private registries, including locally hosted ones.

## Default Docker Registry: Docker Hub

Docker Hub is the default and most widely used public registry. It allows you to:

- Pull publicly available images.
- Push your own images to share or use in deployments.
- Manage private repositories (with a Docker Hub subscription).

## Logging in to a Docker Registry

Before pushing images to a Docker registry, you must authenticate.

- **Log in to Docker Hub**:
  ```bash
  docker login
  ```

## Pushing an Image to a Docker Registry

**Tag your image:** To push an image to a registry, you must tag it with the appropriate registry URL and repository name.

```bash
docker tag <image-id> <registry-url>/<repository-name>:<tag>
```

Example for Docker Hub:
```bash
docker tag my-app:latest username/my-app:latest
```

**Push the image:**

```bash
docker push <registry-url>/<repository-name>:<tag>
```

Example for Docker Hub:
```bash
docker push username/my-app:latest
```

## Using a Local Docker Registry

A local registry can be useful for testing or storing images without relying on external services.

### Starting a local registry

Docker provides an official image for running a registry locally.
Start it by using the following compose file:

```yaml
volumes:

  data:

services:

  registry:

    image: registry:2.7

    ports:
      - 5000:5000

    restart: always
    
    volumes:

      - data:/data 
```
By running the command:
```bash
docker compose up -d
```
The `-d` flag runs the container `--detached`, in the background.
This will start a registry on [http://localhost:5000](http://localhost:5000).

### Tagging your image for the local registry

```bash
docker tag <image-id> localhost:5000/<repository-name>:<tag>
```
Example:
```bash
docker tag my-app:latest localhost:5000/my-app:latest
```

### Pushing the image to the local registry

```bash
docker push localhost:5000/<repository-name>:<tag>
```
Example:
```bash
docker push localhost:5000/my-app:latest
```

### Pulling the image from the local registry

```bash
docker pull localhost:5000/<repository-name>:<tag>
```
Example
```bash
docker pull localhost:5000/my-app:latest
```

### Pulling Images from a Local Registry Using IP or Hostname

To pull an image from a local registry hosted on another computer, use the IP address or hostname of the host machine.

#### Steps to Pull an Image

1. **Ensure Network Connectivity**:
   - Verify that the target computer can reach the host machine over the network.
   - Use `ping` or a similar tool to confirm connectivity:
     ```bash
     ping <host-ip>
     ```

2. **Pull the Image Using the Registry's Address**:
   Use the `docker pull` command with the host's IP address or hostname, followed by the registry port and image details:
   ```bash
   docker pull <host-ip>:<port>/<repository-name>:<tag>
   ```

Example
```bash
docker pull 192.168.1.100:5000/my-app:latest
```
If using a hostname:

```bash
docker pull my-hostname:5000/my-app:latest
```

Handle Insecure Registries (if applicable): If the registry does not have SSL configured, ensure the target computer’s Docker daemon is configured to allow insecure registries (see the following section).

### Pulling Images from a Local Registry without SSL

By default, Docker requires secure (HTTPS) connections to communicate with registries. If your local registry does not have an SSL certificate, you need to configure Docker on the third-party computer to allow insecure connections.

**Steps to Pull an Image from an Insecure Local Registry**

1. **Locate the Docker daemon configuration file**:
   The configuration file is typically located at:
   - **Linux**: `/etc/docker/daemon.json`
   - **Mac/Windows (Docker Desktop)**: Configured via the Docker Desktop UI.

2. **Add the registry as an insecure registry**:
   Modify (or create) the `daemon.json` file and add the following:
   ```json
   {
     "insecure-registries": ["<registry-host>:<port>"]
   }
   ```

### Listing Images in a Local Registry

To view the list of images stored in a local Docker registry, you can use the registry's API.

#### Steps to List Images

1. **Access the Registry API**:
   Use the following `curl` command to query the registry:
   ```bash
   curl http://<registry-host>:<port>/v2/_catalog
   ```

Replace `<registry-host>` and `<port>` with your local registry's address and port (e.g., `localhost:5000`).

Example:

```bash
curl http://localhost:5000/v2/_catalog
```

2. **View Tags for a Specific Repository:** To list all tags for a specific repository, use:
```bash
curl http://<registry-host>:<port>/v2/<repository-name>/tags/list
```
