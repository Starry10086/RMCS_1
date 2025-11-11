#!/bin/bash

export ROS_LOCALHOST_ONLY=1
export RMCS_PATH="/workspaces/RMCS"

source /opt/ros/humble/setup.zsh

# 添加 OGRE 库路径（RViz2 需要）
if [ -d "/opt/ros/humble/opt/rviz_ogre_vendor/lib" ]; then
    export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:$LD_LIBRARY_PATH
fi

if [ -f "/rmcs_install/local_setup.zsh" ]; then
    source /rmcs_install/local_setup.zsh
elif [ -f "${RMCS_PATH}/rmcs_ws/install/local_setup.zsh" ]; then
    source ${RMCS_PATH}/rmcs_ws/install/local_setup.zsh
fi

eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

export RMCS_ROBOT_TYPE=""
