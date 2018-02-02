# spring  

This sample shows a functionable spring model.  
In this sample, you can find a spring model with a plugin "libSpring.so" and a source file as "src/Spring.cc".  
You can find how to get the location of a link consist of a liner joint.  
There is a world file as "worlds/spring.world" to show a spring model.  

## How to run.  
You can run this sample by using a following command.  

    $ roslaunch spring spawn_spring.launch  

Or you can run this sample by using a following command instead of using roslaunch.

    $ gazebo ~/Samples_Gazebo_ROS/src/spring/worlds/spring.world  
    
You shoud push CTRL-R after starting gazebo, you can see a spring falling down.

Date : 18 Oct. 2017
