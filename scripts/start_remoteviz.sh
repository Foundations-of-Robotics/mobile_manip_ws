#!/bin/bash
if [ -z "$1" ]
  then
    echo "Vous avez oublié d'indiquer le nom de la launch file!"
    exit 1
fi
b=$((1 + $RANDOM % 100))
/opt/TurboVNC/bin/vncserver :$b
xhost +local:
export DISPLAY=:$b.0
#echo $DISPLAY
vglrun roslaunch mobile_manip $1 #dingo_gen3_world.launch
