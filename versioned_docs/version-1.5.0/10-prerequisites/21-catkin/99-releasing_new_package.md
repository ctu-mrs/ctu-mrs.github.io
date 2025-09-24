---
title: Releasing new package version
pagination_label: Releasing new package version
description: Releasing new package version
---

# Releaseing new version of a package

1. update the **CHANGELOG.rst** file by
  ```bash
  catkin_generate_changelog
  ```
2. edit the changelog and remove unwanted commit messages (like "update"):
  ```bash
  vim CHANGELOG.rst
  ```
3. in the case that you update **CHANGELOG.rst** file, commit changes.
4. generate new release. This will automatically increment the version in *package.xml* and create new git tag:
  ```bash
  catkin_prepare_release
  ```
5. push the changes
