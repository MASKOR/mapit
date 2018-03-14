---
layout: page
title: Architecture
permalink: /architecture/
order: 500
is_in_menu: true
---

Mapit is designed in a modular way. Complexity is hidden behind a handful of classes. These classes implement the facade design pattern and offer easy interfaces to multiple parts of mapit.
For new tools, that want to use mapit to access checkouts there are the *Checkout* and *CheckoutCommon* class.
For new operators which should use a checkout to manipulate it's data there are the *CheckoutRaw* and *CheckoutCommon* class.
Everything that needs access to history of the data, *Repository* can be used to access versions of entities, commits, ...
For new layertypes the class *Entitydata* must be inherited.
A more advanced usecase is extending mapit for new kinds of serialization (e.g. Webservice or writing directly to a database). This is not yet an easy task and is described shortly at the end of this chapter.


## Artifacts
Mapit is delivered as a shared library and a set of executables. Public headers are included for development.
This makes it ready to use from the commandline and extensible through C++.

We also provide a docker container which takes care of the installation of the dependencies witch would be hard to configure otherwise.

### Binaries:

- bin/mapit: executable that servers for an entrypoint for other mapit-tools
- libexec/mapit_tools: tools and aliases. This contains basically the first argument of the *mapit* command.


### Libaries

- lib/libmapit_core.so: shared library containing symbols for all extensions. This is the core of mapit.
- lib/libmapitstandard_repository_factory.so: library used by tools to create C++-Classes to access and manipulate *Repositories* and *Checkouts*.
- lib/mapit_layertype_\*: types of sensordata the system can work with. For example Pointclouds (PCL).
- lib/mapit_operator_\*: each shared library represents an algorithm that can be executed on the data.

### Headers

**common headers**

- include/mapit/versioning/repository.h: Used to navigate through history. This must be used to obtain a Checkout.
- include/mapit/versioning/checkoutcommon.h: Used to read data from a checkout. Checkouts (at this point) are immutable (may already be commited). The class is extended by CheckoutCommon (see tools) or CheckoutRaw (see operators).

**for implementing new operators**

- include/mapit/versioning/checkoutraw.h: Extends CheckoutCommon with the ability to write and change a checkout. Changes can be tracked implicitly by this class (which entities were changed?).
- include/mapit/serialization/abstractentitydataprovider.h: Access bits and bytes of entities. Usually this is not needed and a concrete implementation of a layertype will be used. This is only needed for typeless / data agnostic calculations (like *delete*, *copy*, *rename*, calculate data size, lossless compression, ...). In almost all usecases it is wrong to use this class directly.
- include/mapit/operators/module.h: Used to create a shared library for mapit. This defines entrypoint of the operator and data like *version*, *author*, ...

**for implementing new layertypes**

- include/mapit/mapit_layertypes/\*: Headers for concrete layertypes (e.g. PCLPointcloudLayertype, LASLayertype, OpenVDBLayertype). The shared library of a concrete layertype must be linked. Note that moreover, most layertypes come with dependencies to third party libraries which must also be satisfied (e.g. PCL). Only needed layertypes should be included into a project. Concrete layertypes are needed for **operators** (which need to handle and calculate the concrete layertypes) and special **tools**, but very unlikely for other layertypes.


**for implementing new tools**

- include/mapit/versioning/checkout.h: Extends CheckoutCommon with the ability to execute operations on the checkout.

- Repository Factories:
    - include/mapit/versioning/repositoryfactorystandard.h: Header for commandline tools. This is only needed for new *tools*.
    - include/mapit/versioning/repositoryfactory.h: Header to create a local directory repository. This is only needed for new *tools*.
    - include/versioning/repositorynetworkingfactory.h: Header to create a remote repository that always uses network. This is only needed for new *tools*.


- include/mapit/mapit_operators/*: Headers for new algorithms. This is only needed for new *operators*.

**Docker Container**

- mapitd: server
- mapit: entypoint for all mapit subcommands/tools

### Checkout

For basic usage it is only required to understand the concept of *checkouts*. Details about versioning can be ignored, when working only with one checkout. When using git as an analogy, this is like working with an directory tree.
A checkout is nothing more than a directory tree. In mapit, folders are called 'tree' (at the moment) and 'entities', which represent files.
A checkout is one specific version of all the data you are currently working on.
In contrast to git and other versioning systems, checkouts have names in mapit. There can be multiple checkouts at a time. Checkouts might also exist on remote machines.

A checkout can only be accessed by mapit tools or by writing a new tool (see *using mapit as a library*). Usually tools only **read** the structure.
**Writing** is meant to be done by executing operators. You can not use a file explorer on an checkout.

### Repository

If it is required to access the history of the data, a checkout is not enough. A repository gives you access to existing checkouts. Moreover you can create a checkout from version which existed in the past.

### Operators

Operators are the only way to write data in mapit. This is done to make changes traceable and executable on remote machines. The parametriation of the execution of an operator is stored as metadata. Using metadata an operator can be executed on demand by the system. This allowed the system to internally delete the physical representation of a version from the harddrive.

There is the concept of *untraceable Operators* which allow to get data into the system or execute algorithms which can not be traced (e.g. because they are done with third party GUI-editors). Their usage means to miss out on many benefits of mapit. These operators are always executed where they are invoked (they are actually part of a **tool**) and they will always tkae up phyiscal space for memory (\* duplicates only take up space once).
