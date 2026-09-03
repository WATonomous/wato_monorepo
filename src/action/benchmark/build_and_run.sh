#!/bin/bash
set -e
. /opt/ros/jazzy/setup.bash
. /ws/install/setup.bash

OSQPE=/ws/install/osqp_eigen_vendor/opt/osqp_eigen_vendor
TRAJ=/ws/install/wato_trajectory_msgs
MPCSRC=/ws/src/action/mpc_controller

mkdir -p /tmp/bench && cd /tmp/bench

g++ -O2 -std=c++17 -o bench \
  /tmp/bench/controller_benchmark.cpp \
  $MPCSRC/src/mpc_core.cpp \
  $MPCSRC/src/bicycle_model.cpp \
  -I$MPCSRC/include \
  -I$OSQPE/include \
  -I$TRAJ/include/wato_trajectory_msgs \
  -I$TRAJ/include \
  -I/usr/include/eigen3 \
  -I/opt/ros/jazzy/include \
  -I/opt/ros/jazzy/include/osqp \
  $(find /opt/ros/jazzy/include -maxdepth 1 -type d -printf '-I%p ') \
  -L$OSQPE/lib -lOsqpEigen \
  -L/opt/ros/jazzy/lib -losqp \
  -L$TRAJ/lib -lwato_trajectory_msgs__rosidl_generator_c -lwato_trajectory_msgs__rosidl_typesupport_cpp \
  -Wl,-rpath,$OSQPE/lib -Wl,-rpath,/opt/ros/jazzy/lib -Wl,-rpath,$TRAJ/lib \
  2>&1 | head -40

echo "=== COMPILE OK ==="
ls -la /tmp/bench/bench
