#!/usr/bin/env bash
set -euo pipefail

# Record desktop session to create assignment demo evidence.
out_file="${1:-$(pwd)/videos/horse_demo_$(date +%Y%m%d_%H%M%S).mp4}"
mkdir -p "$(dirname "$out_file")"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set. Run this from your desktop session." >&2
  exit 1
fi

# Press Ctrl+C to stop recording.
ffmpeg -hide_banner -loglevel warning \
  -f x11grab -video_size 1920x1080 -framerate 30 -i "${DISPLAY}+0,0" \
  -c:v libx264 -preset veryfast -crf 20 "$out_file"
