#!/bin/sh
./mapit-checkout --yaml repo.yaml testcheckout master
./mapit-execute --yaml repo.yaml testcheckout load_pointcloud "{\"filename\":\"$HOME/develop/upns_software/tools/Qt3DPointcloudRenderer/example/data/bunny.pcd\", \"target\":\"corridor/laser/eins\"}"
#./mapit-execute repo.yaml testcheckout copy '{"source":"corridor/laser/eins", "target":"corridor/laser/zwei"}'
./mapit-execute --yaml repo.yaml testcheckout normalestimation '{"radius":0.09, "target":"corridor/laser/eins"}'
./checkout2filesystem --yaml repo.yaml testcheckout ./export
