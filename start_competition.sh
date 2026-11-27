#!/usr/bin/env bash
set -eo pipefail

container="${ROUND2_CONTAINER:-roc}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${ROUND2_WORKSPACE:-$script_dir}"
if [[ -z "${ROUND2_WORKSPACE:-}" && ! -f "$workspace/devel/setup.bash" ]]; then
    workspace="$HOME/round2_team_ws"
fi
export DISPLAY="${DISPLAY:-:0}"

if [[ "$(docker inspect --format '{{.State.Running}}' "$container")" != true ]]; then
    docker start "$container" >/dev/null
fi

# Open the X11 connection for the container's root user on this host only.
x11_access="$(xhost)"
if ! grep -Fqi 'SI:localuser:root' <<< "$x11_access"; then
    xhost +SI:localuser:root >/dev/null
    trap 'xhost -SI:localuser:root >/dev/null 2>&1 || true' EXIT
fi

docker exec -it --user root -e DISPLAY="$DISPLAY" -e XAUTHORITY=/dev/null \
    -w "$workspace" "$container" bash -c '
    set -e
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash

    if pgrep -f "^(/usr/bin/python3 )?/opt/ros/noetic/bin/roslaunch round2_baseline baseline\.launch([[:space:]]|$)" >/dev/null; then
        echo "比赛程序已在运行，请使用现有 Gazebo 和计时器窗口。" >&2
        exit 1
    fi

    exec roslaunch round2_baseline baseline.launch \
        gui:=true timer:=true low_com:=true \
        hybrid_full:=true hybrid_optimize:=true \
        speed_profile:=sprint timeout_clock:=wall efficient_approach:=true \
        vision:=true qr_preposition:=true play_audio:=true
'
