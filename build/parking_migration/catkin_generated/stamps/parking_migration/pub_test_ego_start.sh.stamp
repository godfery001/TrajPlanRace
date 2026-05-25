#!/usr/bin/env bash
set -euo pipefail

# Publish a free ego start pose for initial planner bring-up.
# Run this after the parking_migration nodes and CSSR are already started.

echo "Publishing ego odom to /your_project/ego_odom"
rostopic pub -1 /your_project/ego_odom nav_msgs/Odometry "{
  header: {frame_id: 'map'},
  child_frame_id: 'base_link',
  pose: {
    pose: {
      position: {x: 442660.12674916047, y: 4428719.749872416, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  twist: {
    twist: {
      linear: {x: 0.0, y: 0.0, z: 0.0},
      angular: {x: 0.0, y: 0.0, z: 0.0}
    }
  }
}"

echo "Ego start published."