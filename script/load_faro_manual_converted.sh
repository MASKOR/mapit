#!/bin/bash

# example call
# ./load_faro_manual_converted.sh <path-to-mapit> <ws> <folder containg *pcd and *meta files>

PCD_FOLDER=$3
pushd $PCD_FOLDER
PCDS=$(echo *pcd)
popd

for PCD in $PCDS; do
  META=$PCD_FOLDER$( echo $PCD | cut -d "." -f 1 )".meta"
  SEC=$( cat $META | grep scan_end_sec | cut -d " " -f 2 )
  NSEC=$( cat $META | grep scan_end_nsec | cut -d " " -f 2 )
#  echo $PCD
#  echo $META
#  echo $SEC
#  echo $NSEC

pushd ../build/tools/execute/
./execute --repository-directory $1 $2 load_pointcloud \
  "{  \"filename\" : \"$PCD_FOLDER$PCD\"
     , \"target\" : \"/faro/raw/faro_focus3dx_sensor_fixed$SEC.${NSEC}_broken_ros\"
     , \"sec\" : $SEC
     , \"nsec\" : $NSEC
     , \"frame_id\" : \"faro_focus3dx_sensor_fixed\"
}"
popd

done
