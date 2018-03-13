#!/bin/bash
rm -rf ./mapit_guide
jekyll serve &

i="0"
wget_status="1"

while [[ "$i" -lt 10  && "$wget_status" -gt 0 ]]
do
wget --convert-links -r http://127.0.0.1:4000
wget_status=$?
sleep 1
i=$[$i+1]
done

kill %1
mv 127.0.0.1:4000 ./mapit_guide
zip -r mapit_guide mapit_guide
rm -rf ./mapit_guide
