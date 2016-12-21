#!/bin/sh
./checkout_create --yaml repo.yaml testcheckout master
./execute_operator --yaml repo.yaml testcheckout load_pointcloud '{"filename":"./data/bunny.pcd", "target":"testmap/testlayer/bunny"}'
./execute_operator --yaml repo.yaml testcheckout copy '{"source":"testmap/testlayer/bunny", "target":"testmap/testlayer/bunnyVoxelgrid"}'
./execute_operator --yaml repo.yaml testcheckout voxelgridfilter '{"leafsize":0.2, "target":"testmap/testlayer/bunnyVoxelgrid"}'
./execute_operator --yaml repo.yaml testcheckout copy '{"source":"testmap/testlayer/bunny", "target":"testmap/testlayer/bunnyNormalEst"}'
./execute_operator --yaml repo.yaml testcheckout normalestimation '{"radius":0.2, "target":"testmap/testlayer/bunnyNormalEst"}'
./checkout2filesystem --yaml repo.yaml testcheckout ./export
./visualization_standalone --yaml repo.yaml
