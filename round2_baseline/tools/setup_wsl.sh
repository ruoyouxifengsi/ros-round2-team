#!/usr/bin/env bash
set -eo pipefail
package_dir="$(cd "$(dirname "$0")/.." && pwd)"
project_dir="$(dirname "$package_dir")"
workspace_dir="${ROUND2_WORKSPACE:-$HOME/round2_team_ws}"
source /opt/ros/noetic/setup.bash
set -u
for package in gazebo_ros move_base teb_local_planner; do rospack find "$package" >/dev/null; done
mkdir -p "$workspace_dir/src"
mkdir -p "$HOME/.gazebo/models"
for model in construction_cone white_line; do
  if [ ! -e "$HOME/.gazebo/models/$model" ]; then
    cp -a "$project_dir/gazebo_models/$model" "$HOME/.gazebo/models/"
  fi
done
for package in car3 ucar_accumtimer; do
  if [ ! -e "$workspace_dir/src/$package" ]; then
    cp -a "$project_dir/第二次分站赛下发/src/$package" "$workspace_dir/src/"
  fi
done
for package in round2_baseline round2_vision; do
  source_dir="$project_dir/$package"
  target="$workspace_dir/src/$package"
  if [ ! -e "$target" ]; then
    ln -s "$source_dir" "$target"
  elif [ "$(readlink -f "$target")" != "$source_dir" ]; then
    echo 'Workspace points to another checkout; use a new ROUND2_WORKSPACE.' >&2
    exit 1
  fi
done
chmod +x "$workspace_dir/src/car3/scripts/"*.py "$workspace_dir/src/ucar_accumtimer/scripts/"*.py
cd "$workspace_dir"
catkin_make
printf 'Workspace ready: %s\n' "$workspace_dir"
