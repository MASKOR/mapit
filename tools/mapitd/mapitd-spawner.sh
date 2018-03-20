#!/bin/bash
echo "Usage: first  argument: number of processes"
echo "       second argument: first port"
echo "       other arguments are handed to mapitd"
echo "Spawning $1 processes"
for ((i=0; i<$1; i++)); do 
    the_port=$(($2 + $i))
    echo "starting mapitd on port $the_port"
    ( ./loop_single_server.sh --port $the_port "${@:3}" & )
done
