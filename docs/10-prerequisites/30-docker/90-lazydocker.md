---
title: Lazydocker
pagination_label: Lazydocker
description: Lazydocker 
---

# Lazydocker

[Lazydocker](https://github.com/jesseduffield/lazydocker) is a simple **TUI for docker** and **docker-compose**. 
It allows you to monitor, manage and inspect docker **containers**, **images**, **volumes** and **networks**.
Lazydocker has a customizable `config.yml`, where you can add custom commands.

<details>
<summary>Example custom commands</summary>
```yaml
customCommands:
  containers:
  - name: bash
    attach: true
    command: docker exec -it {{ .Container.ID }} /bin/sh
  - name: logs
    attach: true
    command: docker logs -f {{ .Container.ID }} 
  - name: removeall
    attach: true
    command: "script -q -c 'docker rm -f $(docker ps -aq)'"
  - name: kill-rosbag
    attach: true
    command: docker exec -it {{ .Container.ID }} /bin/sh -c "kill -2 \$(pgrep -f record)"
  images:
    - name: "Run Image with Custom Command"
      command: "script -q -c 'docker run --rm -it {{ .Image.Name }}:{{ .Image.Tag }} /bin/sh' /dev/null"
  volumes:
    - name: "bash"
    -  command: "script -q -c 'docker run --rm -it -v {{ .Volume.Name }}:{{ .Volume.Tag }} /bin/sh' /dev/null"
```
</details>

## How it looks like

![Lazydocker](fig/lazy.gif)

## Installation

The GitHub repository shows different ways of installation but the quickest is:

```bash
curl https://raw.githubusercontent.com/jesseduffield/lazydocker/master/scripts/install_update_linux.sh | bash
```
