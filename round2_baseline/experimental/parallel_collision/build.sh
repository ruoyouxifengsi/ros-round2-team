#!/usr/bin/env bash
set -euo pipefail
base="$HOME/round2_engine_experiment"
src="$base/source"
here="$(cd "$(dirname "$0")" && pwd)"
test "$(pkg-config --modversion gazebo)" = 11.15.1
test "$(git -C "$src" rev-parse HEAD)" = b22c6e15e52299865b31093b8feebc9ca19e26e8
mkdir -p "$base/build"
python3 "$here/prepare_source.py" "$src" "$base/build/adapter.cc" "$here/parallel_collision.cc"
c++ -O3 -fPIC -shared -fopenmp -Wl,-Bsymbolic-functions "$base/build/adapter.cc" \
  -I"$src" -I"$src/gazebo/physics/ode" $(pkg-config --cflags gazebo) \
  -L/usr/lib/x86_64-linux-gnu -lgazebo_physics -lgazebo_ode -ldl \
  -o "$base/build/libround2_parallel_collision.so.new"
mv "$base/build/libround2_parallel_collision.so.new" "$base/build/libround2_parallel_collision.so"
echo "$base/build/libround2_parallel_collision.so"
