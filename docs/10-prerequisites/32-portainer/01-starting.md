---
title: Starting Portainer
pagination_label: Starting Portainer
description: Starting Portainer
---

# Starting Portainer

Portainer can be started using a simple compose session.
By default, the containers will respawn when the computer restarts.

## Portainer Agent

The Portainer Agent is responsible for communicating with the local docker daemon.

<details>
<summary>Compose file</summary>

```yaml
services:

  portainer_agent:

    image: portainer/agent:2.21.0

    ports:
      - 9001:9001

    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
      - /var/lib/docker/volumes:/var/lib/docker/volumes
      - /:/host

    restart: always
```

</details>

## Portainer Server

The Portainer Server is what creates the Web UI.
A single server can be connected to more than one **agent**.

<details>
<summary>Compose file</summary>

```yaml
volumes:

  portainer_data:

services:

  portainer:

    image: portainer/portainer-ce:2.21.0

    ports:
      - 8000:8000
      - 9000:9000
      - 9443:9443

    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
      - portainer_data:/data

    restart: always

    command: --http-enabled
```

</details>

## Both the Agent and the Server on a single machine

In case of a single robot, we might run both on a single machine.
In that case, a single session for both will suffice.

<details>
<summary>Compose file</summary>

```yaml
volumes:

  portainer_data:

services:

  portainer_agent:

    image: portainer/agent:2.21.2

    network_mode: host

    ports:
      - 9001:9001

    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
      - /var/lib/docker/volumes:/var/lib/docker/volumes
      - /:/host

    restart: always

  portainer:

    image: portainer/portainer-ce:2.21.2

    network_mode: host

    ports:
      - 8000:8000
      - 9000:9000
      - 9443:9443

    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
      - portainer_data:/data

    restart: always

    command: --http-enabled

```

</details>
