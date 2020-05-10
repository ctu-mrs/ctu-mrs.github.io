---
layout: default
title: Gitman
parent: Software
---

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
- name: mrs_general
  type: git
  repo: git@mrs.felk.cvut.cz:uav/core/mrs_general.git
  sparse_paths:
  -
  rev: master
  link: ros_packages/mrs_general
  scripts:
  - git submodule update --init --recursive
```
Such section is located twice in the file, first for the **testing** configuration and second for the *locked* **stable** configuration of the sub-repository.

## *stable* vs. *testing* configuration

Gitman remembers two checkout versions of each package. The **stable** one is a particular *locked* commit (specific SHA) of the sub-repository.
All sub-repositories can be set to the **stable** commit by issuing:
```bash
gitman install
```

To test new commits, each sub-repository also tracks a particular branch.
Switching all sub-repositories to the newest commit (**testing**) on its tracking branch is done by:
```bash
gitman update
```
This also updates the *stable* SHA's in the *.gitman.yml*.
If the new versions work well together, you can lock their state just by commiting the *.gitman.yml*.

If you wish to update only particular sub-repos to *stable*, proceed by manually checking out the desired commits/branches in your sub repos.
Then issue
```bash
gitman lock
```
to set the commit SHA as the **stable** and commit the *.gitman.yml* files.

## creating new ROS package version

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

