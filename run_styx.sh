#!/bin/bash

cd ros
catkin_make
source devel/setup.sh
roslauch lauch/styx.launch
cd ..
