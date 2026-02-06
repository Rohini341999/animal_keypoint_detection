#!/usr/bin/env bash
set -euo pipefail

# Remove ROS2 colcon build artifacts for a clean rebuild.
rm -rf build/ install/ log/

echo "Cleaned: build/ install/ log/"
