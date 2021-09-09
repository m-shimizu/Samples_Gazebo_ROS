# pioneer3at_ros  

This sample shows a traditional gazebo sample 4 wheeled robot model.  
The robot can talk and hear in ROS topic.  
A rgb camera image, a thermal camera image and a hokuyo output are published.  
There is a world file as "worlds/pioneer3at_ros.world" to show a pioneer3at_ros robot model.  

## How to use the robot.  
You need two terminals.  

### At Terminal 1 :  
You can run this sample by using a following command.  

    $ roslaunch pioneer3at_ros spawn_pioneer3at_ros.launch   

### At Terminal 2 :  
You can move the robot with an ordinary teleop software by following command:  

    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot/cmd_vel  

### At Terminal 3 :  
You can see the robot with the rviz by following command:  

    $ rviz -d pioneer3at_ros.rviz  

## Model and plugin.  
This package uses following model.  

|Model Name|Plugin(Program) Filename(s)|
|---|---|
|[pioneer3at_ros](https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/master/models/pioneer3at_ros)|-|

Date : 9 Sep. 2021
