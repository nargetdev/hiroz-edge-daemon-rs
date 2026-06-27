#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/lyrical/setup.bash
source /ros2_ws/install/setup.bash

set -u
exec "$@"
