---
layout: page
title: Architecture
permalink: /architecture/
order: 500
---

## Architecture
Mapit is designed in a modular way.


## Artifacts
Mapit is delivered as a shared library and a set of executables. Public headers are included for development.
This makes it ready to use from the commandline and extensible.

We also provide a docker container which takes care of the installation of the dependencies witch would be hard to configure otherwise.

### Binaries:

- bin/mapit: executable that servers for an entrypoint for other mapit-tools
- bin/upns_tools: tools and aliases. This contains basically the first argument of the *mapit* command.


### Libaries

- lib/libupns_mapmanager.so: shared library containing symbols for all extensions. This is the core of mapit.
- lib/libstandard_repository_factory.so: library used by tools to create C++-Classes to access and manipulate *Repositories* and *Checkouts*.
- lib/upns_layertypes/*: types of sensordata the system can work with. For example Pointclouds (PCL).
- lib/upns_operators/*: each shared library represents an algorithm that can be executed on the data.

### Headers

**common headers**

- include/upns/upns_layertypes/*: Headers for concrete layertypes. Also the corresponding shared library of a concrete layertype must be linked. Note that most layertypes come with dependencies which must be satisfied (e.g. PCL). Only needed layertypes should be included into a project. Concrete layertypes are needed for **operators** and **tools**, but very unlikely for other layertypes.



**for implementing new operators**
**for implementing new layertypes**



**for implementing new tools**

- Repository Factories:
    - include/upns/versioning/repositoryfactorystandard.h: Header for commandline tools. This is only needed for new *tools*.
    - include/upns/versioning/repositoryfactory.h: Header to create a local directory repository. This is only needed for new *tools*.
    - include/versioning/repositorynetworkingfactory.h: Header to create a remote repository that always uses network. This is only needed for new *tools*.


- include/upns/upns_operators/*: Headers for new algorithms. This is only needed for new *operators*.

**Docker Container**

- server/mapitd
- helper (TODO)

### Checkout

For basic usage it is only required to understand the concept of *checkouts*. Details about versioning can be ignored, when working only with one checkout. When using git as an analogy, this is like working with an directory tree, completely forgetting about verioning.
A checkout is nothing more than a directory tree. In mapit, folders are called 'tree' (at the moment) and 'entities', which represent files.
A checkout is one specific version of all the data you are currently working on.
In contrast to git and other versioning systems, a checkout has name in mapit. There can be multiple checkouts and checkouts might also exist on remote machines.

A checkout can only be accessed by mapit tools or by writing a new tool (see *using mapit as a library*). Usually tools only **read** the structure.
**Writing** is meant to be done by executing operators.

### Repository

If it is required to access the history of the data, a checkout is not enough. A repository gives you access to existing checkouts. Moreover you can create a checkout from version which existed in the past.
These version
