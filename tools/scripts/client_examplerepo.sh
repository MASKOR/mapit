#!/bin/bash
PARAMS="--url tcp://localhost:5555 --compute-local"
./mapit-checkout testcheckout master $PARAMS
./mapit-execute testcheckout load_pointcloud '{"filename":"./data/bunny.pcd", "target":"testmap/testlayer/bunny"}' $PARAMS
./mapit-execute testcheckout copy '{"source":"testmap/testlayer/bunny", "target":"testmap/testlayer/bunnyVoxelgrid"}' $PARAMS
./mapit-execute testcheckout voxelgridfilter '{"leafsize":0.2, "target":"testmap/testlayer/bunnyVoxelgrid"}' $PARAMS
./mapit-execute testcheckout copy '{"source":"testmap/testlayer/bunny", "target":"testmap/testlayer/bunnyNormalEst"}' $PARAMS
./mapit-execute testcheckout normalestimation '{"radius":0.2, "target":"testmap/testlayer/bunnyNormalEst"}' $PARAMS
./mapit-checkout2filesystem testcheckout ./export $PARAMS
./mapit-gui $PARAMS
