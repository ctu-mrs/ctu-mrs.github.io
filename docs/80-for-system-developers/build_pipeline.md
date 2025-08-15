---
title: Build pipeline
pagination_label: MRS ROS Build Pipeline
descriptin: MRS ROS Build Pipeline
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# MRS ROS Build Pipeline

## Helper scripts

<Button label="🔗 ctu-mrs/ci_scripts repository" link="https://github.com/ctu-mrs/ci_scripts" block /><br />

This repository contains build scripts and **Github action** workflows that are used for both
* on-push builds of individual repositories
* complete overnight build workflows governed by the [https://github.com/ctu-mrs/buildfarm](https://github.com/ctu-mrs/buildfarm)

## Buildfarm

<Button label="🔗 ctu-mrs/buildfarm repository" link="https://github.com/ctu-mrs/buildfarm" block /><br />

The **buildfarm** repository contains build scripts that facilite a complete build of the MRS system.
The pipeline is divided into thres sub-pipelines: **non-bloom**, **thirdparty ROS**, and **MRS** packages.
The last two ROS-driven pipelines generate a list of Github actions workflows that build the packages in the right order and supply the dependencies using the already build artifacts.

### NonBloom packages

These [packages](https://github.com/ctu-mrs/buildfarm/blob/master/nonbloom.yaml) are not built using the ROS's bloom builder, instead, they rely on the generic mechanism for building **deb** packages.

### Third-party ROS packages

These [packages](https://github.com/ctu-mrs/buildfarm/blob/master/thirdparty.yaml) do not change often and not dependent on anything from the custom MRS packages (the third group).

### MRS packages

These [packages](https://github.com/ctu-mrs/buildfarm/blob/master/mrs.yaml) are the bulk of the pipeline.
They can depend on the third-party packages.

## Pipelines

### Unstable pipeline

The unstable pipeline builds packages into the [unstable PPA](https://github.com/ctu-mrs/ppa-unstable).
The packages are most-commonly built from the **master** branch of the git repository.
The particular branches are encoded in the buildfarm definition files.
Each push to the respective branch generates a **deb** package immediately.
A user won't typically use the unstable PPA, unless he specifically choses to.
The unstable PPA is meant to be used by developers (the packages appear there quickly) and is used to regularly check the state of the main branch in all the repositories.
Tests are executed daily on the unstable PPA.

### Release pipeline

This [pipeline](https://github.com/ctu-mrs/buildfarm/actions/workflows/rostest_and_release_mrs_amd64.yml) builds packages into the [testing PPA](https://github.com/ctu-mrs/ppa-testing).
The packages originate from the **release candidate** branches.
Automated rostests are executed after the packages are compiled.
If the tests pass, the **release candidate** branches are automatically merged to the **release** branches, from which the **stable** pipeline is executed.

## Stable pipeline

This pipeline builds the stable version of the system from the **stable** branches into the [stable PPA](https://github.com/ctu-mrs/ppa-stable).
