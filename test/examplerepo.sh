#!/bin/sh
./checkout_create repo.yaml testcheckout master
./execute_operator repo.yaml testcheckout load_pointcloud '{"filename":"./data/bunny.pcd", "target":"testmap/testlayer/bunny"}'
./execute_operator repo.yaml testcheckout copy '{"source":"testmap/testlayer/bunny", "target":"testmap/testlayer/bunnyVoxelgrid"}'
./execute_operator repo.yaml testcheckout voxelgridfilter '{"leafsize":0.2, "target":"testmap/testlayer/bunnyVoxelgrid"}'
./execute_operator repo.yaml testcheckout copy '{"source":"testmap/testlayer/bunny", "target":"testmap/testlayer/bunnyNormalEst"}'
./execute_operator repo.yaml testcheckout normalestimation '{"leafsize":0.2, "target":"testmap/testlayer/bunnyNormalEst"}'
./checkout2filesystem repo.yaml testcheckout ./export
