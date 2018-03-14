#!/bin/bash
PARAMS="--repository-directory common.repo"
./mapit-checkout testcheckout master $PARAMS
./mapit-execute testcheckout load_pointcloud '{"filename":"/all_binary_compressed.pcd", "target":"tunnel/laser/all"}' $PARAMS
./mapit-execute testcheckout centroid_to_origin '{"target":"tunnel/laser/all"}' $PARAMS
./mapit-execute testcheckout surfrecon_openvdb '{"input":"tunnel/laser/all", "output":"tunnel/laser/all_levelset", "radius":0.01, "voxelsize":0.004}' $PARAMS
./mapit-execute testcheckout levelset_to_mesh '{"input":"tunnel/laser/all_levelset", "output":"tunnel/laser/all_mesh", "detail":0.01}' $PARAMS
#./mapit-gui $PARAMS

