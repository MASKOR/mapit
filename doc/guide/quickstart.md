---
layout: page
title: Quickstart
permalink: /quickstart/
order: 200
---
## Usage	(for non-developers)

To use mapit to access and edit data there is no need to develop a line of code.

### Basics

All commands of mapit are accessible through the command

```bash
mapit <command> [<args>] [--repository-directory| --url] [--compute-local]
```

All commands work on repositories. A repository can be either a local directory (usually called ".mapit") or a network address.

- *\-\-repository-directory* uses a local repository,
- *\-\-url* specifies a network repository.

This is one of the few places where the user has to know about the place of a repository. For the most time it is transparent - meaning it makes no difference - where a repository resides. The software handles network access (if neccesary) without futher contribution of the user. If none of the two paramters is given, the local repository at "./.mapit" will be used/created. Examples:

```bash
mapit <cmd> <args> --repository-directory ~/repositories/mymaps
mapit <cmd> <args> --url tcp://nucular.local:55555
```

The *\-\-compute-locale*-Flags specifies, that all computation has to take place on the local machine. If the repository is a remote one, all data is downloaded and uploaded. If the falg is not specified the default behaviour is that the computer with the repository (e.g. the computer that runs maptitd) does the calculation. Different repositories may have differten operators installed. For example there might be a Microsoft Windows machine, witch has no version of PCL on it. A person using this computer can trigger the execution of a PCL algorithm on a remote machine.

Currently there is a small list of *commands*:

```bash
checkout_create
execute_operator
mapitd
checkout2filesystem
```

(TODO: the names are about to change in order to create a streamlined interface)

### Read Data into the software

Getting data into the system is done by using a special operator, which accesses the filesystem it is running on. For Pointlocuds the operator is calls *load_pointcloud*

```bash
mapit execute_operator testcheckout load_pointcloud '{"filename":"./data/bunny.pcd", "target":"testmap/testlayer/bunny"}'
```

### Write Data to filesystem

All data is hidden from the user by default to prevent accidential edits of the data. To track all changes it is important that all changes are done via operators (and in the best case can be represented as metadata). In later versions it might be possible to create read-only symlinks to checked-out files.

There is a tool which extracts/copies a complete checkout to a directory:

```bash
mapit checkout2filesystem <checkout> <destination>
```

For example:

```bash
mapit checkout2filesystem testcheckout ./export
```

### Edit data

Data can only be changed by operators. Operator always work on a specific version of the data, which is represented as a *checkout*. There are no operator that works across multiple versions of data.
Executing an operator is possible with:

```bash
mapit execute_operator <checkout> <operator> <params>
```

The list of operators is always growing, currently these operators are available/in-development

### Access remote Repositories

See **Usage Basics** to see how \-\-compute-local flag works.
Downloading und uploading data is done by configuring the repository to the remote by using the *--url* flag.
For downloading use

```bash
mapit export2filesystem <checkout> <local_destination_directory> --url tcp://<ip|hostname>:<port>
```

To upload pointclouds to a remote repository use:

```bash
mapit execute_operator <checkout> load_pointcloud '{"filename":"<filename>", "target":"<name_of_new_entity>"}'
```

### Create an remote accesible Repository

```bash
mapitd [--repository-directory| --url] [--compute-local] <port>
```

```Mapitd``` is implemented as a tool that works on a given repository and makes it available over network.
