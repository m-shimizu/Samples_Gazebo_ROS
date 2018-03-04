# centaur

This sample shows a centaur robot model with a sample pose controling software.  
In this sample, you can find a crawler robot model having 4 flipper arms and a plugin "libcrawler_flipper.so" and a source file as "src/crawler_flipper.cc".  
There is a world file as "worlds/centaur.world" to show a centaur robot.  
And there is a world file as "worlds/centaur4.world" to show 4 centaur robots.  

## ROBOT
A centaur robot model was imported from ![tu-darmstad](https://github.com/tu-darmstadt-ros-pkg/centaur_robot_tutorial), and converted into sdf.
    
## How to use a centaur robot
You need 2 terminals for spawning a robot and controlling a centaur robot.  

    Terminal 1(To spawn a robot):  

    $ cd ~/Samples_Gazebo_ROS  
    $ source setup.bash  
    $ roslaunch centaur centaur.launch  
    (A centaur robot will be spawned and initialized in pose automatically)  
    
    Terminal 2(To move the robot):  

    $ rostopic list  
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot/cmd_vel_raw  
    
    To control other joints of the robot, use a rqt trajectory controller that was opened when the gazebo started.  
    You can know how to use the rqt trajectory controller heuristically.  
    Try it!  

## Model.  
This package uses following model.  

|Model Name|Plugin(Program) Filename(s)|
|---|---|
|[centaur](https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/master/models/centaur)|There is no original plugin.|

|Program file|Description|
|---|---|
|centaur_init_pose.cpp|An executable initilizing robot pose program.<br>This program shows how to control angle of joints with a joint trajectory control.|


UPDATED : 28th Feb 2018
