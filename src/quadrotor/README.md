# quadrotor

This sample shows a quadrotor robot model.  
In this sample, you can find a quadrotor robot model.  
There is a world file as "worlds/quadrotor.world" to show a quadrotor robot.  
And there is a world file as "worlds/quadrotor4.world" to show 4 quadrotor robots.  

## ROBOT  
quadrotor\_ros was imported from ![tu-darmstad](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor) and converted into sdf.

## IMPORTING PACKAGES by rosinstalling  
* hector_components_description  
* hector_compressed_map_transport  
* hector_gazebo  
* hector_gazebo_plugins  
* hector_gazebo_thermal_camera  
* hector_gazebo_worlds  
* hector_geotiff  
* hector_geotiff_plugins  
* hector_imu_attitude_to_tf  
* hector_imu_tools  
* hector_localization  
* hector_mapping  
* hector_map_server  
* hector_map_tools  
* hector_marker_drawing  
* hector_models  
* hector_models.rosinstall  
* hector_nav_msgs  
* hector_pose_estimation  
* hector_pose_estimation_core  
* hector_quadrotor  
* hector_quadrotor_actions  
* hector_quadrotor_controller_gazebo  
* hector_quadrotor_controllers  
* hector_quadrotor_demo  
* hector_quadrotor_description  
* hector_quadrotor_gazebo  
* hector_quadrotor_gazebo_plugins  
* hector_quadrotor_interface  
* hector_quadrotor_model  
* hector_quadrotor_pose_estimation  
* hector_quadrotor_teleop  
* hector_sensors_description  
* hector_sensors_gazebo  
* hector_slam  
* hector_slam_launch  
* hector_trajectory_server  
* hector_uav_msgs  
* hector_xacro_tools  
* rtt_hector_pose_estimation  

## How to fly with a quadrotor  
You need 3 terminals for spawning a robot and controlling the robot.  

    Terminal 1(To spawn a robot):  

    $ cd Samples_Gazebo_ROS  
    $ source setup.bash  
    $ roslaunch quadrotor quadrotor.launch  
    
    Terminal 2(To control the robot):  

    $ cd Samples_Gazebo_ROS  
    $ source setup.bash  
    $ roslaunch quadrotor buffalo_gamepad.launch robot:=robot  
     (AND PUSH No.4 BUTTON TO START!!)  
     (You can also use logitech_gamepad.launch or sony_dualshock3.launch or xbox_controller.launch, and you should read them to find which button is for start.)  
    
    Terminal 3(To watch the camera view of the robot):  

    $ cd Samples_Gazebo_ROS  
    $ source setup.bash  
    $ rostopic list | grep robot | grep image
    $ rosrun image_view2 image_view2 image:=/robot/camera_ros/image  
    
## Model.  
This package uses following model.  

|Model Name|Plugin(Program) Filename(s)|
|---|---|
|[quadrotor](https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/master/models/quadrotor)|There is no original plugin.|

|Program file|Description|
|---|---|
|There is no original executable program.|-|


UPDATED : 25th Jun. 2020
