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

We assume you obtained the upns **Dockerfile** and **upns-\<version\>-Linux\_and\_dependencies.tar.gz** before.

First, build the docker container, by invoking the following command **in the directory containig both files**:

    docker build . -t fhac/mapit:0.1

This will take a few minutes depending on internet connection, as it will download an operating system into the dockercontiner. We use this as a known environment to execute mapit in, without altering the host system.

You may have to start ```dockerd``` with admin-privileges before!

All docker images can be listed using

    docker image ls

At this time there should be at least one image named *fhac/mapit* (in version 0.1).

To test if everything is set up correctly, ```./testrun.sh``` can be executed.

## Running Commands with docker

Commands can be executed using

```sh
docker run -v ./.mapit:/.mapit -v ./data:/data -v ./export:/export fhac/mapit:0.1 mapit <mapitcommand> <args>
```

This command is too long to use it regularly and there is a simple solution to that. The option ```-v <host_dir>:<container_dir>``` is used to make a directory from the host visible to the container.

- **.mapit** contains the repository itself.
- **data** is used to exchange data between container and host
- **export** is meant to be used to export the repository to a filesystem (using ```mapit export2filesystem /export``` )

The simple solution is to use ```docker-compose```. The parameters for the container can be put into a docker-compose.yml. There is such a Yaml file available, which will start a server, listening on port 5555 and start a client to execute commands in.

For speeding things up we can start the the container once and subsequently execute commands.
Starting the container and store the container id:

    export MAPITINSTANCE=`docker run -t -d -v $PWD/.mapit:/.mapit -v $PWD/data:/data -v $PWD/export:/export fhac/mapit:0.1 bash`

For convenience an alias can be defined to execute mapit commands:

    alias mapit='docker exec $MAPITINSTANCE mapit'  

now mapit can be used as if it was installed on the host system directly. Note that the alias works only in the terminal, where it was defined.
