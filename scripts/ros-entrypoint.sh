#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/rover_interfaces/install/setup.bash
exec "$@"
