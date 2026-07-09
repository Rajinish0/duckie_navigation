#!/bin/bash

source /environment.sh

dt-launchfile-init
# rosrun my_package ros_communication.py
# rosrun localization localization.py
# rosrun navigation navigation_node.py
dt-exec roslaunch navigation navigation.launch
dt-launchfile-join
