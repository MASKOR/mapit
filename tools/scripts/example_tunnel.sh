#!/bin/bash
PARAMS="--repository-directory common.repo"
./checkout_create testcheckout master $PARAMS
./execute_operator testcheckout load_pointcloud '{"filename":"/home/dbulla/Files/Pointclouds/MILAN/pcd/all_binary_compressed.pcd", "target":"tunnel/laser/all"}' $PARAMS
./execute_operator testcheckout centroid_to_origin '{"target":"tunnel/laser/all"}' $PARAMS
./execute_operator testcheckout voxelgridfilter '{"leafsize":0.2, "target":"tunnel/laser/all"}' $PARAMS
./visualization_standalone $PARAMS

