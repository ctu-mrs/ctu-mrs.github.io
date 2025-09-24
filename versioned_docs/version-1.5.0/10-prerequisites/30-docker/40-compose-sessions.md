---
title: Compose sessions
pagination_label: Running Docker compose sessions
description: How to run Docker compose sessions
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Compose sessions

Compose sessions are a convenient way of launching multiple containers in a pre-determined way.
A compose session is defined by a `yaml` file, that precribes the containers as `service`.
Each `service` have the following parameters:

* source docker image,
* mounted network volumes,
* mounted network interfaces,
* environment variables,
* network port mapping,
* entrypoint,
* base command.

## Starting the compose session

```bash
docker compose up
```

When environment variables are provided from a file, start the session as

```bash
docker compose --env-file ./stack.env up
```

## Stopping the the session

```bash
docker compose down
```

When environment variables are provided from a file, stop the session as

```bash
docker compose --env-file ./stack.env down
```

Additional arguments:
* `--v`, `--volumes`: remove declared volumes
* `--remove-orphans`: remove containers not defined in the compose file

## Piping GUI from containers

The following steps apply only for Ubuntu host OS.

<Tabs>
  <TabItem value="ubuntu" label="Ubuntu" default>
    1. mount `/dev/dri` into the container
      ```bash
      volumes:
        - /dev/dri:/dev/dri
      ```
    2. set the `$DISPLAY` environment variable:
      ```bash
      environment:
          DISPLAY: $DISPLAY
      ```
    3. run `xhost +` before starting the compose session
  </TabItem>
</Tabs>

## Compose session for the MRS System

The compose sessions are available at the [ctu-mrs/mrs_docker](https://github.com/ctu-mrs/mrs_docker).

<Button label="🔗 ctu-mrs/mrs_docker repository" link="https://github.com/ctu-mrs/mrs_docker" block /><br />

## Vanilla compose sessions

The _vanilla_ compose sessions are compsoe out of:

* `session.yaml`: the compose file
* `stack.env`: the environment variables definition
* `config/`: folder containing the custom configs
* `logs/`: folder where stdout logs are going to be stored
* `up.sh`, `down.sh`: scripts for easy starting / stopping of the simulation
