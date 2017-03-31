---
layout: page
title: Installation
permalink: /install/
order: 100
---

## Setup

We will create a virtual machine to install and execute mapit in a known working environment.
This has the advantage, that you do not need to compile mapit yourself. We can not yet proved ABI compatible versions of the programm for most operating systems with all features. Using docker, you can use all features as surface reconstruction and pointcloud manipulation without installing/compiling a multitude of dependencies.
By using a system alias, it will look like mapit is installed on you host system directly.

We assume you obtained the upns **Dockerfile** and **upns-<version\>-Linux_and_dependencies.tar.gz** before.

First, build the docker container, by invoking the following command **in the directory containig both files**:

    docker build . -t fhac/mapit:0.1

This will take a few minutes depending on internet connection, as it will download an operating system to execute mapit in.

You may have to start
```bash
dockerd
```
with admin-privileges before!

All docker images can be listed using

    docker image ls

At this time there should be at least one image named *fhac/mapit* (in version 0.1).

Commands can be executed using

    docker run fhac/mapit:0.1 mapit <mapitcommand> <args>

For speeding things up we can start the the container once and subsequently execute commands.
Starting the container and store the container id:

    export MAPITINSTANCE=`docker run -t -d fhac/mapit:0.1 bash`

For convenience an alias can be defined to execute mapit commands:

    alias mapit='docker exec $MAPITINSTANCE mapit'  

now mapit can be used as if it was installed on the host system directly. Note that the alias works only in the terminal, where it was defined.
