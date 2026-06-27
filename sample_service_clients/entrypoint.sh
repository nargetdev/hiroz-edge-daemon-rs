#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/lyrical/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"
