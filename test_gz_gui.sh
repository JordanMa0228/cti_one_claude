#!/usr/bin/env bash
# Quick test: open Gazebo GUI with the living room world.
# Run this DIRECTLY in your Ubuntu-24.04 terminal (not via ros2 launch).
# A Gazebo window should appear on your Windows desktop within ~5 seconds.

source /opt/ros/kilted/setup.bash
source ~/CTI_One_interview/install/setup.bash

export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export GZ_IP=127.0.0.1
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

WORLD=$(ros2 pkg prefix w102_path_planning)/share/w102_path_planning/worlds/living_room.sdf

echo "Opening Gazebo with world: $WORLD"
echo "A Gazebo window will appear — the simulation starts PAUSED."
echo "Press the ▶ Play button (bottom-left toolbar) to start."
gz sim "$WORLD"
