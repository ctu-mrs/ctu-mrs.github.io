---
title: FAQ
pagination_label: Docker FAQ
description: Docker FAQ
---

# Docker FAQ

## 'Permission denied' for /var/run/docker.sock

### Temporary fix (untill reboot)

```bash
sudo chmod 666 /var/run/docker.sock
```

### Ultimate fix

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
```
reboot might be needed.
