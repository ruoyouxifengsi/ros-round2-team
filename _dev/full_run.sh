#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$workspace/devel/setup.bash"
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-/root/.Xauthority}
output="/root/trials/round2-$(date +%Y%m%d-%H%M%S-%N)"

exec python3 -u "$workspace/src/round2_baseline/tools/run_trial.py" "$output" \
  --low-com --cones --seed 42 --speed-profile sprint \
  --efficient-approach --timeout-clock sim --seconds 350 "$@"
