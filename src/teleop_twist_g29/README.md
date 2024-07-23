__ teleop\_twist\_g29 __  

DESCRIPTION:
  This package is for using a Logitech G29 Driving Force Racing Wheel and Floor Pedals instead of a game-pad.  
  This package calls not only ros-g29-force-feedback package with a tuned configuration for driving robot, but also teleop\_twist\_joy.  

REQUIREMENT:  
  This package needs the ros-g29-force-feedback package.  

  $ cd ~/SAME\_ws/src ; git clone -b ros1 https://github.com/kuriatsu/ros-g29-force-feedback.git 

SETUP CONFIG FILE:  
  You can add the rule for automatic changing the device file permission with following command.  

  $ sudo sh -c 'echo SUBSYSTEMS==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTRS{idVendor}==\"046d\", ATTRS{idProduct}==\"c24f\", MODE=\"0666\" >> /etc/udev/rules.d/99-uvc.rules'

  Investigate g29's device files. g29 has 2 files in /dev/input.  
  At first, unplug your g29 USB connector from your PC, and get the list of the files in the /dev/input.  
  At second, plug the g29 USB connector to the PC, and you can find added 2 files in the /dev/input.  
  One file named as jsX(X is number), another file named as eventY(Y is number).  

  Those 2 files name should be set into teleop\_twist\_g29/config/g29.yaml, line 1 nad 2.  
  Open and edit the g29.yaml before using this package.  


HOW TO USE:  
  Call the launch file.
  While the shift nob be set at the left-top or right-down position, the cmd\_vel will be active.  
  The left-top position means "go foreward", the right-down position means "go backword".  
  Please check the topic "/robot/cmd\_vel".  

  $ roslaunch teleop\_twist\_g29 teleop\_twist\_g29.launch robot\_name:=myrobot  

WHAT YOU CAN GET:  
  /myrobot/cmd\_vel  

EDITED: 23 July 2024  
