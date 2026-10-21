#!/usr/bin/env sh
# Activa ROS de Pixi, el underlay compartido (si existe) y el workspace del TP1.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1
export GZ_PARTITION=default
export TURTLEBOT3_MODEL=burger
. "$CONDA_PREFIX/setup.sh"

# Si existe el workspace padre con turtlebot3_gazebo compilado, usarlo como underlay
if [ -f "$PIXI_PROJECT_ROOT/../ros_ws/install/setup.sh" ]; then
  . "$PIXI_PROJECT_ROOT/../ros_ws/install/setup.sh"
  export GZ_SIM_RESOURCE_PATH="$PIXI_PROJECT_ROOT/../ros_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH}"
fi

# Overlay del workspace propio del TP1
if [ -f "$PIXI_PROJECT_ROOT/ros_ws/install/setup.sh" ]; then
  . "$PIXI_PROJECT_ROOT/ros_ws/install/setup.sh"
fi
