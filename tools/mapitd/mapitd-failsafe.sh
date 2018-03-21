#!/bin/bash

./mapitd $@
while [ $? != 0 ]; do
    ./mapitd $@
done
