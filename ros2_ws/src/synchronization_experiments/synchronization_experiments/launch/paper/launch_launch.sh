#!/bin/bash
index=0
for i in {1..12}
do
    gnome-terminal --window -- bash -c "./launch_multiple.sh $index; exec bash;"
    index=$((index+9))
    sleep 1
done