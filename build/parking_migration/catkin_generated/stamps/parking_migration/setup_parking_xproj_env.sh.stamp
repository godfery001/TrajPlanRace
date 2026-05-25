#!/usr/bin/env bash

# Keep TrajPlanRace as primary overlay, then append pathplan_ws packages.
source /opt/ros/noetic/setup.bash
source /home/siegf/Dftpav-main/devel/setup.bash
source /home/siegf/TrajPlanRace/devel/setup.bash

export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/home/siegf/pathplan_ws/devel"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/siegf/pathplan_ws/src"
export PYTHONPATH="${PYTHONPATH:-}:/home/siegf/pathplan_ws/devel/lib/python3/dist-packages"

echo "Environment ready:"
echo "  cav_control      -> $(rospack find cav_control)"
echo "  cav_msgs         -> $(rospack find cav_msgs)"
echo "  xproj_simulation -> $(rospack find xproj_simulation)"
echo "  xproj_msgs       -> $(rospack find xproj_msgs)"