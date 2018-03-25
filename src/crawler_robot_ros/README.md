# crawler_robot_ros  

This sample shows a crawler robot model with a joystick teleoperation software.  
The robot and the teleope software can talk in ROS topic.  
In this sample, you can find a crawler robot model having 4 flipper arms and a plugin "libcrawler_flipper_ros.so" and a source file as "src/crawler_flipper_ros.cc".  
There is a world file as "worlds/crawler_robot.world" to show a crawler_robot robot model.  

## How to use the robot.  
You need two terminals.  

### At Terminal 1 :  
You can run this sample by using a following command.  

    $ roslaunch crawler_robot_ros spawn_crawler_robot_ros.launch use_joystick:=false  

If use_joystick is true, the kind of the default game controller is iBUFFALO-bsgp1601.  
You can find a description of other kinds of game controllers in the following.  

### At Terminal 2 :  
You can move the robot with an ordinary teleop software by following command:  

    $ roslaunch crawler_robot_ros joy_and_teleop_crawler.launch robot_name:=robot dev:=/dev/input/js0 gc_bsgp1601:=true  

You can use gc\_ds4 or gc\_dux60 instead of gc\_bsgp1601.  
gc\_ds4 means a game controller as the Dual Shock 4 of SONY.  
gc\_dux60 means a game controller as the JC-DUX60 of ELECOM.  
gc\_bsgp1601 means a game controller as the BSGP1601 of iBUFFALO.  

## Model and plugin.  
This package uses following model.  

|Model Name|Plugin(Program) Filename(s)|
|---|---|
|[crawler_robot_ros](https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/master/models/crawler_robot_ros)|crawler_flipper_ros.cc<br>teleop_crawler.cc|

|Program file|Description|
|---|---|
|crawler_flipper_ros.cc|The plugin program.<br>This program can hear ROS topic.|
|teleop_crawler.cc|An executable tele-operation program for crawler_robot_ros in ROS.<br>It can control robot moving and flipper moving with a joy stick.<br>This program can tell only ROS topic.|

Date : 16 Mar. 2018
