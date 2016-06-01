#!/bin/sh
./checkout_create repo.yaml testcheckout master
./execute_operator repo.yaml testcheckout load_pointcloud '{"filename":"./data/bunny.pcd", "target":"testmap/testlayer/bunny"}'
./execute_operator repo.yaml testcheckout voxelgridfilter '{"leafsize":0.2, "target":"testmap/testlayer/bunny"}'
./checkout2filesystem repo.yaml testcheckout ./export./checkout2filesystem repo.yaml testcheckout ./export
