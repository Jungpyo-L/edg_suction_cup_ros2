#!/bin/bash
# Snapshot of the lab camera (RealSense D435 colour stream, /dev/video4) every N seconds, for watching a robot run
# remotely. Writes ~/EDG_Experiment/cam/latest.jpg (always the newest) and a time-stamped copy every 10th frame.
#   sg video -c "bash ~/ros2_ws/src/suction_cup/ais/tools/cam_watch.sh 30"      (the account must be in group video)
N=${1:-30}; OUT=~/EDG_Experiment/cam; mkdir -p "$OUT"; k=0
while true; do
  # 8 frames so that auto-exposure settles; keep the last
  if timeout 20 ffmpeg -nostdin -loglevel error -y -f v4l2 -input_format yuyv422 -video_size 1280x720 -i /dev/video4 \
       -frames:v 8 -update 1 "$OUT/.tmp.jpg" </dev/null 2>/dev/null; then
    mv "$OUT/.tmp.jpg" "$OUT/latest.jpg"
    [ $((k % 10)) -eq 0 ] && cp "$OUT/latest.jpg" "$OUT/cam_$(date +%Y%m%d_%H%M%S).jpg"
    k=$((k + 1))
  fi
  sleep "$N"
done
