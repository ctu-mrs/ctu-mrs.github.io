---
layout: default
title: Gitman
parent: Software
---

| :warning: **Attention please: This page needs work.**                                                                                             |
| :---                                                                                                                                              |
| The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid. |

# Gitman "submodule" manager

Gitman, a git project manager, is a program for resolving dependency between multiple git repositories.
It substitutes git's built-in submodule mechanism and provides more user-friendly experience and some additional features:

  * Sub-repositories no longer appear in the **DETACHED HEAD** state on their own.
  * Each sub-repository is a full-fledged git repository with the *.git* subdirectory.
  * Sub-repositories can be placed anywhere, not just in the subtree of the main repo.
  * Each sub-repo is checked out to a particular commit (**stable**) and can be checked to the newest commit on a branch (**testing**). This process is automated and naturally fits into our workflow.

## installation

```bash
sudo pip3 install gitman
```

## .gitman.yml config file

Everything is saved in the *.gitman.yml* file, which is placed in the root of the main repository.
Each sub-repository is described by the following section:
```yml
  - repo: https://github.com/ctu-mrs/mrs_uav_controllers.git
    name: mrs_uav_controllers
    rev: master
    type: git
    params:
    sparse_paths:
      -
    links:
      - source: ''
        target: ros_packages/mrs_uav_controllers
    scripts:
      - git submodule update --init --recursive
```
Such section is located twice in the file, first for the **testing** configuration and second for the *locked* **stable** configuration of the sub-repository.
