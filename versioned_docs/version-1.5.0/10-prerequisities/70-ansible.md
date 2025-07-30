---
title: Ansible 
pagination_label: Ansible 
description: Ansible
---

# Ansible
[Ansible](https://docs.ansible.com/ansible/latest/installation_guide/index.html) is an open-source automation tool. 

**Ansible** is a tool that helps you **automate tasks** on your remote machines (robots) — like installing software, updating settings, or running scripts —
without doing it manually on each machine.

You just need to write instructions in simple text files (called **playbooks**) using
**YAML**, and Ansible follows them. It connects over **SSH**, so you don’t need
to install anything extra on the machines you're managing.

To install Ansible on your machine, refer to the [Installation Guide](https://docs.ansible.com/ansible/latest/installation_guide/index.html).

### Setting on the remote machine(s)

- Ensure the **public SSH key** is added to `~/.ssh/authorized_keys`.  
  After you generate your SSH key (e.g., `~/.ssh/ansible`), you can copy it to the remote machine using: 

```bash
ssh-copy-id -i ~/.ssh/ansible remote_user@192.168.69.1xx
```
### Playbook examples

<details>
<summary>Example Transfer and load Docker image</summary>
```yaml
- name: Transfer and load Docker image 
  hosts: remote
  become: yes
  serial: 0  
  tasks:
    - name: Copy Docker image to remote machine
      ansible.builtin.copy:
        src: "~/docker/shared_data:worlds.tar.gz"
        dest: "/tmp/tmp_img.tar.gz"
        mode: '0644'

    - name: Load Docker image on remote machine
      ansible.builtin.shell: docker load -i /tmp/tmp_img.tar.gz
      register: docker_load_output

    - name: Show Docker load output
      ansible.builtin.debug:
        msg: "{{ docker_load_output.stdout }}"
```
</details>


<details>
<summary>Example Fetch rosbags</summary>
```yaml
- name: Fetch .bag files from remote to local
  hosts: remote
  become: yes
  serial: 0
  tasks:

    - name: Ensure temp directory exists
      ansible.builtin.file:
        path: /tmp/rosbags
        state: directory
        mode: '0755'

    - name: Copy .bag files to temp directory
      ansible.builtin.shell: |
        cp /var/lib/docker/volumes/stack_bag_files/_data/*.bag /tmp/rosbags/
      args:
        executable: /bin/bash

    - name: Get list of .bag files
      ansible.builtin.find:
        paths: /tmp/rosbags
        patterns: "*.bag"
      register: bag_files

    - name: Fetch .bag files one by one
      ansible.builtin.fetch:
        src: "{{ item.path }}"
        dest: "~/rosbags/{{ inventory_hostname }}/"
        flat: yes
      loop: "{{ bag_files.files }}"
```
</details>


### Usage

Once the .yml file is created you can run the following command:

```bash
ansible-playbook -i inventory.ini load_docker_image.yml --ask-become-pass
```

where `inventory.ini` is the file that contains the list of remote machines:

```bash
[remote]
192.168.69.160 ansible_user=uav 
192.168.69.161 ansible_user=uav 
192.168.69.162 ansible_user=uav 
192.168.69.163 ansible_user=uav 
192.168.69.164 ansible_user=uav 
192.168.69.165 ansible_user=uav 
192.168.69.166 ansible_user=uav 
192.168.69.167 ansible_user=uav 
192.168.69.168 ansible_user=uav 
192.168.69.169 ansible_user=uav 
```
and in a file named `ansible.cfg` you can specify the private key to use e.g.:

```bash
[defaults]
private_key_file = ~/.ssh/ansible
```


