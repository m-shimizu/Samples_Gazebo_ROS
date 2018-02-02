# crawler_robot  

This sample shows a crawler robot model with a teleoperation software.  
The robot and the teleope software can talk in Gazebo topic.  
In this sample, you can find a crawler robot model having 4 flipper arms and a plugin "libcrawler_flipper.so" and a source file as "src/crawler_flipper.cc".  
There is a world file as "worlds/crawler_robot.world" to show a crawler_robot robot model.  

## How to use the robot.  
You need two terminals.  

### At Terminal 1 :  
You can run this sample by using a following command.  

    $ roslaunch crawler_robot spawn_crawler_robot.launch  

Or you can run this sample by using a following command instead of using roslaunch.  

    $ gazebo ~/Samples_Gazebo_ROS/src/crawler_robot/worlds/crawler_robot.world  

### At Terminal 2 :  
You can control the robot with a teleop software by following command:  

    $ rosrun crawler_robot teleop_crawler_robot crawler_robot 2  
    
At the end of command, where "crawler_robot" is the robot name on Gazebo, where "2" is the direction of z axis of twist message.  
You can find more details in crawler_flipper.cc and teleop_crawler_robot.cc.  
    
Date : 30 Nov. 2017
