#!/bin/bash

# example call
# ./load_bags.sh <path-to-mapit> <ws> <regular expression or/and list of all bag files>

I=0
TF_PRE="/tf"
FARO_PRE="/faro/raw"

for param in $@; do
  if [ 0 -eq $I ]; then
    MAPIT_REPO=$param
  fi
  if [ 1 -eq $I ]; then
    WS=$param
  fi

  if [ 2 -eq $I ]; then
    BAGS="    \"${param}\""
  fi
  if [ 2 -lt $I ]; then
    BAGS=${BAGS}"
    , \"${param}\""
  fi

  I=$(($I + 1))
done

pushd ../build/tools/execute/
./execute --repository-directory $MAPIT_REPO $WS load_bags "
{ \"bags\" : [
  $BAGS
  ]
  , \"translation_pairs\" : [
      { \"topic\" : \"/tf\", \"prefix\" : \"${TF_PRE}\", \"type\" : \"tf\" }
    , { \"topic\" : \"/tf_static\", \"prefix\" : \"${TF_PRE}\", \"type\" : \"tf\" }
    , { \"topic\" : \"/mascor/faro/fls_pointcloud\", \"prefix\" : \"${FARO_PRE}\", \"type\" : \"pointcloud\" }
  ]
}"
popd
