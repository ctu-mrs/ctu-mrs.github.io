---
title: Saving & Loading Images
pagination_label: Saving & Loading Images
description: Saving & Loading Images
---

# Saving & Loading Images

Sometimes, it can be useful to save a docker image into a file, such that you can transport it by, e.g., a flash drive.

## Saving and image

```bash
docker save <image tag> -o <output file>
```

## Loading an image

```bash
docker load < <image file>
```
