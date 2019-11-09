# Samples_Gazebo_ROS
This repository has several packages to show sample codes for Gazebo and/or ROS, sample models and worlds for Gazebo.  
Please find more information in [wiki page](https://github.com/m-shimizu/Samples_Gazebo_ROS/wiki).  

## About this branch  
This branch had been maintained with ROS-Kinetic and Gazebo 8.  

## REQUIREMENT OF THIS REPOSITORY  

### Install ROS Kinetic and Gazebo8 from PPA  
#### *[Ubuntu install of ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
#### *[Install Gazebo using Ubuntu packages](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=8.0)  
Do followings:  

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116  
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'  
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -  
    sudo apt-get update  
    sudo apt-get install -y cmake g++ protobuf-compiler pavucontrol libgazebo8 libgazebo8-dev ros-kinetic-desktop ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-image-view2 ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-message-to-tf ros-kinetic-tf2-geometry-msgs ros-kinetic-audio-common ros-kinetic-costmap-2d ros-kinetic-image-transport ros-kinetic-image-transport-plugins ros-kinetic-urdf-tutorial ros-kinetic-gazebo8-ros*  
    sudo rosdep init  
    rosdep update  
    sudo apt-get install -y python−rosinstall  
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
Models exist for Gazebo, it is good that all models are collected in a directory.  

Edit date: 2 Feb. 2018
