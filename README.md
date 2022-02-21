# Samples_Gazebo_ROS
This repository has several packages to show sample codes for Gazebo and/or ROS, sample models and worlds for Gazebo.  
Please find more information in [wiki page](https://github.com/m-shimizu/Samples_Gazebo_ROS/wiki).  

## About this branch  
This branch has been maintained with ROS-noetic and Gazebo 11.  

## REQUIREMENT OF THIS REPOSITORY

### OS
Ubuntu 20.04LTS.  

### Install ROS noetic and Gazebo11 from PPA
#### *[Ubuntu install of ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
#### *[Install Gazebo 11 using Ubuntu packages](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=11.0)  
Do followings:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654  
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'  
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -   
    sudo apt-get update  
    sudo apt-get install -y cmake g++ protobuf-compiler pavucontrol libgazebo11 libgazebo11-dev ros-noetic-desktop ros-noetic-ros-controllers ros-noetic-image-view2 ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-joint-trajectory-controller ros-noetic-gmapping ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-amcl ros-noetic-navigation ros-noetic-audio-common ros-noetic-costmap-2d ros-noetic-image-transport ros-noetic-image-transport-plugins ros-noetic-urdf-tutorial ros-noetic-tf2-geometry-msgs ros-noetic-map-server ros-noetic-move-base ros-noetic-robot-state-publisher ros-noetic-diagnostic-updater ros-noetic-rgbd-launch ros-noetic-moveit ros-noetic-rosbridge-server ros-noetic-gazebo-ros* ros-noetic-hector-* ros-noetic-ros-ign* python3-rosinstall python3-rosinstall-generator python3-catkin-tools python3-bloom python3-vcstool libignition-msgs-dev libignition-transport4-dev  
    sudo rosdep init  
    rosdep update  
    gazebo (and wait for finish of downloading fundamental models)  
    
And some Hector Quadrotor programs need QT4.  
    
#### *[Packages in “Qt4 for Ubuntu 20.04”](https://launchpad.net/~rock-core/+archive/ubuntu/qt4/+packages)  
#### *[How to Install Qt4 Libraries in Ubuntu 20.04 LTS](https://ubuntuhandbook.org/index.php/2020/07/install-qt4-ubuntu-20-04/)  
#### *[Pyqt4 in Ubuntu 20.04](https://stackoverflow.com/questions/61818849/pyqt4-in-ubuntu-20-04)  
    sudo add-apt-repository ppa:rock-core/qt4  
    sudo apt update  
    sudo apt install libqt4-declarative libqt4* libqtcore4 libqtgui4 libqtwebkit4 qt4*  

## Preparing for using this repository.  
Just do the next commands.  

    $ cd ~  
    $ git clone https://github.com/m-shimizu/Samples_Gazebo_ROS/  
    $ cd ~/Samples_Gazebo_ROS  
    $ rosinstall src /opt/ros/noetic
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

## SEE ALSO STRONGLY.  
* [osrf/gazebo/Migration](https://bitbucket.org/osrf/gazebo/src/default/Migration.md)  
* [Migration to Gazebo 9 / ROS noetic](https://github.com/wuwushrek/sim_cf/issues/2)  

Edit date: 28th Feb. 2022
