#!/usr/bin/env bash
set -e

_kill_procs() {
  kill -TERM $gym
  wait $gym
#  kill -TERM $xvfb
}

## Setup a trap to catch SIGTERM and relay it to child processes
trap _kill_procs SIGTERM
trap _kill_procs INT


#export PYTHONPATH=/project
export PYTHONPATH=$PYTHONPATH:/project
env

python gym_simulation_launcher.py $@ &
gym=$!

wait $gym

