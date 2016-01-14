#!/bin/bash

sudo cpulimit -e move_base_3d -l 50 -z &
sudo cpulimit -e exploration_map_node -l 50 -z &

