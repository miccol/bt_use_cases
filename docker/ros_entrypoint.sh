#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /usr/local/ws/install/setup.bash

exec "$@"