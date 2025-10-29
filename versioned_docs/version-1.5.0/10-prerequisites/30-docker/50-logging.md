---
title: Logging
pagination_label: Running Docker compose sessions
description: How log containers' stdout and stderr
---

# Logging containers' output

Logging of the `stdout` and `stderr` can be facilitated using Docker's logging adapters.
However, using logging adapters is relatively complex and requires additional user's knowledge for extraction of the logs.
For that reason, we are utilizing a custom solution that requires zero configuration and minimal knowledge.

# [DogTail](https://github.com/klaxalk/dogtail)

**DoGTail** (**Docker Go Tail**) is a lightweight, zero-configuration logging sidecar Docker Compose.
It automatically **tails** the **stdout** and **stderr** streams from all containers in a Compose session, storing them in organized, plaintext log files for easy access.

Please follow to the official DogTail repository to learn more about it:
<Button label="🔗 klaxalk/dogtail repository" link="https://github.com/klaxalk/dogtail" block /><br />

## Example usecase

The following `compose` files shows how to load DogTail into a compose session.
The resulting logs will be stored in the `logs` volume and accesible via [Portainer](../32-portainer/index.md).

```yaml
volumes:

  logs:

services:

  myapp:
    image: your-image
    tty: true # this ensures the stdout and stderr are ordered properly in the logs
    # your app configuration here

  dogtail:
    image: klaxalk/dogtail:latest
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
      - logs:/etc/logs:consistent
```
