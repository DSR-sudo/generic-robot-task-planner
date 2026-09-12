#!/usr/bin/env bash
# ROS 2 setup scripts reference optional variables that are not nounset-safe.
set -Eeo pipefail

cd "/home/pi/generic-robot-task-planner"
source /opt/ros/jazzy/setup.bash
source /home/pi/LSLIDARN10P/install/setup.bash
if [[ -f install-current/setup.bash ]]; then
  source install-current/setup.bash
else
  source install/setup.bash
fi

exec ros2 launch mavros_xyz_position_offboard flight_stack.launch.py \
  fcu_url:=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:2000000 \
  enable_control:=true \
  udp_bind_ip:=0.0.0.0 \
  udp_bind_port:=5005 \
  udp_remote_ip:=10.42.0.1 \
  udp_remote_port:=5005 \
  udp_whitelist_ip:=10.42.0.1 \
  udp_whitelist_port:=5005
