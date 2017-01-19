#!/bin/bash
PARAMS="--repository-directory common.repo"
./checkout_create testcheckout master $PARAMS
./execute_operator testcheckout load_pointcloud '{"filename":"/home/dbulla/Files/Pointclouds/MILAN/pcd/all_binary_compressed.pcd", "target":"tunnel/laser/all"}' $PARAMS
./execute_operator testcheckout centroid_to_origin '{"target":"tunnel/laser/all"}' $PARAMS
./execute_operator testcheckout surfrecon_openvdb '{"input":"tunnel/laser/all", "output":"tunnel/laser/all_levelset", "radius":0.01, "voxelsize":0.004}' $PARAMS
./execute_operator testcheckout levelset_to_mesh '{"input":"tunnel/laser/all_levelset", "output":"tunnel/laser/all_mesh", "detail":0.01}' $PARAMS
#./visualization_standalone $PARAMS

