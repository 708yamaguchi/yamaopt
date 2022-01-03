#!/bin/sh

# Check if xacro command is installed
which xacro > /dev/null
if [ $? -ne 0 ]; then
    echo "xacro is not installed."
    echo "Please install xacro by pip install xacro"
fi

# Create urdf with sensor frames by xacro command
for robot in pr2 fetch; do
    IN_XACRO="config/"$robot"_sensors.xacro"
    ORIG_URDF=$HOME"/.skrobot/"$robot"_description/"$robot".urdf"
    OUT_URDF=$HOME"/.skrobot/"$robot"_description/"$robot"_sensors_test.urdf"

    if [ ! -e $IN_XACRO ]; then
        echo "Cannot find $IN_XACRO"
        echo "Run this script under tests directory"
    else
        xacro $IN_XACRO orig_urdf:=$ORIG_URDF > $OUT_URDF
        echo "Create $OUT_URDF"
    fi
done
