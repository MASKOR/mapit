---
# You don't need to edit this file, it's empty on purpose.
# Edit theme's home layout instead if you wanna make some changes
# See: https://jekyllrb.com/docs/themes/#overriding-theme-defaults
layout: home
theme: minima
---
# Mapit

## Overview

Mapit is designed to offer easy communication between multiple participants for accessing and editing sensordata. This is archieved by following design principles of distributed versioning systems and algorithm pipelines (workflows).

For example  the following shell script creates a new repository locally (*.mapit* directory).
It creates a new *checkout*. Note, that *checkouts*, in contrast to other versioning systems, are not visible in the filesystem (yet).
Data is read into the checkout and is edited with *execute_operator* command. In the end data is made visible to the filesystem again (*export2filesystem*).

	mapit create_checkout testcheckout master
	mapit execute_operator testcheckout load_pointcloud '{"filename":"./data/bunny.pcd", "target":"testmap/testlayer/bunny"}'
	mapit execute_operator testcheckout voxelgridfilter '{"leafsize":0.2, "target":"testmap/testlayer/bunnyVoxelgrid"}'
	mapit execute_operator testcheckout normalestimation '{"radius":0.2, "target":"testmap/testlayer/bunnyNormalEst"}'
	mapit checkout2filesystem testcheckout ./export

## Features
Mapit is meant to be used with huge files while still maintaining the history of data. This requires the use of new paradigms.

**Distributed access and computation**

![workflow](assets/repositories.svg "Workflow" )

Data managed with mapit can be distributed across multiple *repositories*. When executing algorithms on data, the download/upload of huge amounts of data can be overcome and calculation can be done in-place. At the same time there is no need to repeatedly store all data in every repository.


**Mapit tracks Metadata**

<img src="assets/workflow.svg" width="60%" style="float: right">
[//]: # (![workflow](assets/workflow.svg "Workflow" ) )

Sensordata is not (always) copied and stored in history once the data is changed. By keeping a description of the executed algorithms (Metadata) it ensures that snapshots from the past can be recovered at any time. This comes with the downside, that data must never be changed by the user directly. Thus, data is not visible as editable files in the filesystem. Data can be changed by existing or self implemented operators in the system.


Every executed operation is described by one chunk of metadata. A group of operations, executed in sequence is called a *workflow*. With mapit its possible to group several operations together into one workflow which then can be easly applied onto new sensoric data. (This is not yet supported/implemented).


**Extensible in multiple ways**

For developers mapit can be extended in 3 ways.

- It is easy to add **new operators**, by using an CMake-Template we provide. You implement a function that receives a simple C++-Class wich enables reading/writing to data. The functions receives a string which can contain more user defined parametrization in any string format (e.g. XML, YAML, JSON).
- If the provided layertypes (PCD, OVDB, LAS, ...) are not enough, **new layertypes** can be implemented by writing a C++-class that serializes your custom data to/from a std::stream. Also serialization directly to/from files or memoryadresses is possible to boost performance in certain scenarios (e.g. network access), but these are optional.
- **Tools** (e.g. for visualization, merging, browsing, ...) can be implemented by using mapit as a library. When used as a library the full power of mapit is exposed to you, featuring e.g. versioning access. This is only meant for advanced use cases.

We identified that the most common extension will be to add more operators. Consequently we made the interface for new operators as easy to use as possible.
