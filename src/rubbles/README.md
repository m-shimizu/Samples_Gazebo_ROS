# rubbles  

This sample shows how to generate models from a world plugin.  
In this sample, you can know 4 ways to spawn a model into a world in src/spawn_rubbles.cc.  
There is a world file as "worlds/rubbles.world" to call a world plugin as "libspawn_fubbles.so".  

## How to run.  
You can run this sample by using a following command.  

    $ roslaunch rubbles spawn_rubbles.launch  

Or you can run this sample by using a following command instead of using roslaunch.  

    $ gazebo ~/Samples_Gazebo_ROS/src/rubbles/worlds/rubbles.world  

## Plugin.  
This package uses following model.  

|Model Name|Plugin(Program) Filename(s)|
|---|---|
|None|spawn_rubbles.cc|

|Program file|Description|
|---|---|
|spawn_rubbles.cc|The plugin program.|

Date : 2 Feb. 2018
