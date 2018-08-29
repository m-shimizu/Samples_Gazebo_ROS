# twoLinkArm  

This sample shows a two link arm robot model.  
In this sample, you can find a motionable an arm robot model with a plugin "twoLinkArmPlugin.so" and a source file as "src/twoLinkArmPlugin.cc" and "pack_push.cc".  
There is a world file as "worlds/twoLinkArm.world" to show a two arm robot, a pack and a field.  

## Description of limitations of joints of the twoLinkArm robot.  
There is a real one of the twoLinkArm. The original one has angle limitations in it's joints. 

|Joint|Limitation of angle|
|---|---|
|Shoulder|45 degrees for every diretion from center of movable angle|
|Elbow|90 degrees for clockwise from straight pose|

## How to run.  
You can run this sample by using a following command.  

    $ roslaunch twolinkarm spawn_twoLinkArm.launch  

Or you can run this sample by using a following command instead of using roslaunch.  

    $ gazebo ~/Samples_Gazebo_ROS/src/twolinkarm/worlds/twoLinkArm.world  
    
You can see an usage for operating the fourlegs robot on the terminal you started this repository.  
You can move the pack by typing keys(diamond cursor formation by i,j,k,l and ,), then the twoLinkArm follow the moving pack.  
If the pack was near by the end of the twoLinkArm, the twoLinkArm hits the pack.  

## Model and plugin.  
This package uses following model.  

|Model Name|Plugin(Program) Filename(s)|
|---|---|
|[twoLinkArm](https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/master/models/twoLinkArm)|twoLinkArmPlugin.cc<br>push_pack.cc|

|Program file|Description|
|---|---|
|twoLinkArmPlugin.cc|The plugin program for the twoLinkArm robot.|
|push_pack.cc|A program for moving the pack.|

Date : 2 Feb. 2018
