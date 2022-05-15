#!/bin/bash
source /home/ubuntu/dev_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\
/home/ubuntu/dev_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models