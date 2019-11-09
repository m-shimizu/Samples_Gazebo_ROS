# Samples_Gazebo_ROS
This repository has several packages to show sample codes for Gazebo and/or ROS, sample models and worlds for Gazebo.  
Please find more information in [wiki page](https://github.com/m-shimizu/Samples_Gazebo_ROS/wiki).  

## About this branch  
This branch has been maintained with ROS-Melodic and Gazebo 9.  

## REQUIREMENT OF THIS REPOSITORY

### OS
Ubuntu 18.04.  

### Install ROS Melodic and Gazebo9 from PPA
#### *[Ubuntu install of ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
#### *[Install Gazebo using Ubuntu packages](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0)  
Do followings:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654  
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'  
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -   
    sudo apt-get update  
    sudo apt-get install -y cmake g++ protobuf-compiler pavucontrol libgazebo9 libgazebo9-dev ros-melodic-desktop ros-melodic-ros-controllers ros-melodic-image-view2 ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-gmapping ros-melodic-joy ros-melodic-joystick-drivers ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-amcl ros-melodic-navigation ros-melodic-audio-common ros-melodic-costmap-2d ros-melodic-image-transport ros-melodic-image-transport-plugins ros-melodic-urdf-tutorial ros-melodic-tf2-geometry-msgs ros-melodic-map-server ros-melodic-move-base ros-melodic-robot-state-publisher ros-melodic-diagnostic-updater ros-melodic-rgbd-launch ros-melodic-moveit ros-melodic-rosbridge-server ros-melodic-gazebo-ros* ros-melodic-hector-* python-rosinstall python-catkin-tools libignition-msgs-dev libignition-transport4-dev  
    sudo rosdep init  
    rosdep update  
    gazebo (and wait for finish of downloading fundamental models)  

## Preparing for using this repository.  
Just do the next commands.  

    $ cd ~  
    $ git clone https://github.com/m-shimizu/Samples_Gazebo_ROS/  
    $ cd ~/Samples_Gazebo_ROS  
    $ catkin_make  
    $ source setup.bash  

## How to run each package.  
In the table of [wiki page](https://github.com/m-shimizu/Samples_Gazebo_ROS/wiki), you can find a link for each package.  
And please visit each package directory to find how to run the package and more details.  
Please click a package name which you want.  

## Each package direcotry.  
You can find each package under "~/Sample_Gazebo_ROS/src" directory.  
And you can find source files, world files, launch files and other resources under sub-directories of a package.  

## Location of models.
But all models are under "~/Sample_Gazebo_ROS/models" directory.  
This models folder exists for Gazebo, so it is good that all models are collected in a directory.  
A resonable solution to do this, make a symbolic link of the model folder in the repository into the model folder.  

Edit date: 9th Nov. 2019
