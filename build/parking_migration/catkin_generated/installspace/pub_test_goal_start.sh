#!/usr/bin/env bash
set -euo pipefail

# Compatibility wrapper that publishes both the free goal and free start pose.
# Keep this for one-shot bring-up, but prefer the two dedicated scripts below.

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

"${script_dir}/pub_test_ego_start.sh"
"${script_dir}/pub_test_goal.sh"


echo "Done."