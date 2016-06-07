#!/bin/sh
./checkout_create repo.yaml testcheckout2 master
./execute_operator repo.yaml testcheckout2 load_pointcloud "{\"filename\":\"$HOME/pointclouds/1465223257087387.pcd\", \"target\":\"corridor/laser/eins\"}"
./execute_operator repo.yaml testcheckout2 normalestimation '{"radius":0.2, "target":"corridor/laser/eins"}'
./checkout2filesystem repo.yaml testcheckout2 ./export
