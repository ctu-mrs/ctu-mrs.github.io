---
title: Apptainer
pagination_label: Apptainer container system
description: Apptainer container system
---

# What is Apptainer?

Apptainer is an open-source containerization software designed specifically for high-performance computing (HPC), scientific research, and environments requiring strong security.
It enables users to create, manage, and run containers that package applications with their dependencies in a portable, reproducible, and isolated manner.
It was originally known as Singularity and is popular in academic and research institutions due to its focus on HPC environments.

## Relation to Singularity

Apptainer is the continuation of Singularity under a new governance model.
In 2021, the Singularity project moved under the Linux Foundation, where it was rebranded as Apptainer to ensure the long-term sustainability and broader collaboration around the project.
Functionally, Apptainer continues Singularity’s legacy, maintaining compatibility with its containers and focusing on the same core goals.

# Why Use Apptainer Over Docker?

Apptainer and Docker are both containerization tools, but they cater to different needs and environments.
Here are some key points comparing Apptainer to Docker:

1. Designed for HPC and Research
  * Apptainer: Focuses on HPC environments, allowing non-privileged users to run containers on shared clusters. It integrates seamlessly with HPC job schedulers like SLURM and is compatible with shared filesystems.
  * Docker: Requires root privileges to run and is not ideal for HPC due to security concerns in multi-tenant systems.
2. Security Features
  * Apptainer: Operates with a security model where containers are run as the user who invokes them, minimizing privilege escalation risks. It avoids running a daemon that requires elevated privileges.
  * Docker: Relies on a root-privileged daemon (dockerd), which can pose security risks in multi-user environments.
3. Mobility of Compute
  * Apptainer: Containers are single, immutable files (SIF - Singularity Image Format) that can easily be copied, shared, and run on different systems without requiring special setup. This is particularly useful in environments where portability and reproducibility are critical.
  * Docker: Uses layered images stored in a local registry, which requires additional steps to share or move across systems.
4. Simplicity for End Users
  * Apptainer: Users can directly execute containers without needing complex setups or root permissions.
  * Docker: Typically requires more infrastructure and configuration, especially in shared environments.
5. Integration with Existing Tools
  * Apptainer: Better suited for integrating with HPC workflows and environments where Docker might not be permitted.
  * Docker: More suitable for general-purpose containerization, especially for web applications and services.

## When to Use Apptainer

If you are working in HPC, academic research, or other multi-user, secure environments.
When you need portability and reproducibility without requiring elevated privileges.
If you want easy integration with HPC tools like job schedulers or shared filesystems.

Docker remains a better choice for web development, CI/CD pipelines, and general-purpose applications in environments that permit its use.
However, for scientific computing and HPC, Apptainer is the preferred solution.
