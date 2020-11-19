source devel/setup.bash
export GAZEBO_MODEL_PATH=`pwd`/models:`pwd`/models_field:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=`pwd`/devel/lib:/opt/ros/melodic/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_RESOURCE_PATH=`pwd`:${GAZEBO_RESOURCE_PATH}
