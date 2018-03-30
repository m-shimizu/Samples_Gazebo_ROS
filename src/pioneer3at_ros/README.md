# pioneer3at_ros  

This sample shows a traditional gazebo sample 4 wheeled robot model.  
The robot can talk and hear in ROS topic.  
A rgb camera image, a thermal camera image and a hokuyo output are published.  
There is a world file as "worlds/pioneer2dx_ros.world" to show a pioneer2dx_ros robot model.  

## How to use the robot.  
You need two terminals.  

### At Terminal 1 :  
You can run this sample by using a following command.  

    $ roslaunch pioneer2dx_ros spawn_pioneer2dx_ros.launch   

### At Terminal 2 :  
You can move the robot with an ordinary teleop software by following command:  

    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot/cmd_vel  

### At Terminal 3 :  
You can see the robot with the rviz by following command:  

    $ rviz -d pioneer2dx_ros.rviz  

## Model and plugin.  
This package uses following model.  

|Model Name|Plugin(Program) Filename(s)|
|---|---|
|[pioneer2dx_ros](https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/master/models/pioneer2dx_ros)|-|

Date : 16 Mar. 2018
