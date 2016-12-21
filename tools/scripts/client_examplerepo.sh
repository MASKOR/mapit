#!/bin/bash
PARAMS="--url tcp://localhost:5555 --compute-local"
./checkout_create testcheckout master $PARAMS
./execute_operator testcheckout load_pointcloud '{"filename":"./data/bunny.pcd", "target":"testmap/testlayer/bunny"}' $PARAMS
./execute_operator testcheckout copy '{"source":"testmap/testlayer/bunny", "target":"testmap/testlayer/bunnyVoxelgrid"}' $PARAMS
./execute_operator testcheckout voxelgridfilter '{"leafsize":0.2, "target":"testmap/testlayer/bunnyVoxelgrid"}' $PARAMS
./execute_operator testcheckout copy '{"source":"testmap/testlayer/bunny", "target":"testmap/testlayer/bunnyNormalEst"}' $PARAMS
./execute_operator testcheckout normalestimation '{"radius":0.2, "target":"testmap/testlayer/bunnyNormalEst"}' $PARAMS
./checkout2filesystem testcheckout ./export $PARAMS
./visualization_standalone $PARAMS
