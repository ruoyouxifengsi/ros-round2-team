#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=${ROUND2_WORKSPACE:-$HOME/round2_team_ws}
if [ -f "$project/devel/setup.bash" ]; then
  workspace=${ROUND2_WORKSPACE:-$project}
fi
source "$workspace/devel/setup.bash"
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-/root/.Xauthority}
output="${ROUND2_TRIALS:-$HOME/trials}/round2-$(date +%Y%m%d-%H%M%S-%N)"

exec python3 -u "$workspace/src/round2_baseline/tools/run_trial.py" "$output" \
  --low-com --cones --seed 42 --speed-profile sprint \
  --efficient-approach --timeout-clock sim --seconds 350 "$@"
