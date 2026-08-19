source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

alias ll='ls -alF'
alias l='ls -alF'

export TURTLEBOT3_MODEL=waffle
export RCUTILS_COLORIZED_OUTPUT=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0

# Planificador agricola de cobertura (modo Campo). Va antes del workspace de
# SALUS para que este pueda superponerlo.
if [ -f /opt/salus_coverage_ws/install/setup.bash ]; then
  source /opt/salus_coverage_ws/install/setup.bash
fi

if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi
