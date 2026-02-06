#!/usr/bin/env bash
set -euo pipefail

# Capture one frame from the active X11 display (use after focusing Gazebo or RViz window).
out_file="${1:-$(pwd)/screenshots/frame_$(date +%Y%m%d_%H%M%S).png}"
mkdir -p "$(dirname "$out_file")"

# DISPLAY must be available (for example :0 or :1).
if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set. Run this from your desktop session." >&2
  exit 1
fi

# ffmpeg grabs a single frame from the current display.
ffmpeg -hide_banner -loglevel error -y \
  -f x11grab -video_size 1920x1080 -i "${DISPLAY}+0,0" \
  -frames:v 1 "$out_file"

echo "Saved screenshot: $out_file"
