#!/usr/bin/env bash
set -euo pipefail

# Publish a free parking goal pose for initial planner bring-up.
# Run this after the parking_migration nodes and CSSR are already started.

echo "Publishing goal pose to /your_project/goal_pose"
rostopic pub -1 /your_project/goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 442662.12674916047, y: 4428721.749872416, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"

echo "Goal pose published."