# fourlegs_wheeled  

This sample shows a quadruped robot model.  
In this sample, you can find a motionable quadruped model with a plugin "libQuadrupedMotion.so" and a source file as "src/QuadrupedMotion.cc" and "Motion.cc".  
There is a world file as "worlds/fourlegs_wheeled.world" to show a quadruped robot model.  

## How to run.  
You can run this sample by using a following command.  

    $ roslaunch fourlegs_wheeled spawn_fourlegs_wheeled.launch  

Or you can run this sample by using a following command instead of using roslaunch.  

    $ gazebo ~/Samples_Gazebo_ROS/src/fourlegs_wheeled/worlds/fourlegs_wheeled.world  
    
You can see an usage for operating the fourlegs_wheeled robot on the terminal you started this repository.  

## Model and plugin.  
This package uses following model.  

|Model Name|Plugin(Program) Filename(s)|
|---|---|
|[fourlegs_wheeled](https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/master/models/fourlegs_wheeled)|QuadrupedMotion.cc<br>Motion.cc|

|Program file|Description|
|---|---|
|QuadrupedMotion.cc|The plugin program.|
|SkidSteerPluginFourlegsWheeled.cc|A program for skidsteering with key command inputs.|

Date : 9 Sep. 2021
