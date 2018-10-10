#!/bin/bash

source /opt/ros/kinetic/setup.bash

# make sure there is no xserver running (only neded for docker-compose)
#killall Xvfb || true
rm -f /tmp/.X99-lock || true

# Start Xvfb
Xvfb :99 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
xvfb=$!

export DISPLAY=:99

echo PYTHONPATH = $PYTHONPATH
echo python = `which python`

$@

kill $xvfb
