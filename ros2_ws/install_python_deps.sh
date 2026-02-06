#!/usr/bin/env bash
set -euo pipefail

# Create a local virtual environment with access to system site-packages (requested workflow).
uv venv virtual --python 3.12 --system-site-packages

# Prevent colcon from scanning python packages inside the virtual environment.
touch virtual/COLCON_IGNORE

# Activate the environment so ros2 Python nodes can import installed packages.
source virtual/bin/activate

# Pin numpy to avoid ROS2/OpenCV ABI mismatches and install Ultralytics for pose inference.
uv pip install 'numpy==1.26.0' ultralytics

echo "Python dependencies installed in $(pwd)/virtual"
