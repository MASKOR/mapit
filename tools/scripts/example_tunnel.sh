#!/bin/bash
PARAMS="--repository-directory common.repo"
./mapit-checkout testcheckout master $PARAMS
./mapit-execute testcheckout load_pointcloud '{"filename":"/home/dbulla/Files/Pointclouds/MILAN/pcd/all_binary_compressed.pcd", "target":"tunnel/laser/all"}' $PARAMS
./mapit-execute testcheckout centroid_to_origin '{"target":"tunnel/laser/all"}' $PARAMS
./mapit-execute testcheckout voxelgridfilter '{"leafsize":0.2, "target":"tunnel/laser/all"}' $PARAMS
./mapit-gui $PARAMS

