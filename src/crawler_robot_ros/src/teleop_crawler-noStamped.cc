//=============================================================================
// This program is a modification of 
//                  $(find hector_quadrotor_teleop)/src/quadrotor_teleop.cpp
// Editor  : Masaru Shimizu
// E-mail  : shimizu@sist.chukyo-u.ac.jp
// Updated : 25 Mar.2018
//=============================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=============================================================================

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>

namespace teleop_crawler
{

class Teleop
{
private:

  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber;
  ros::Publisher  velocity_publisher, flipper_publisher;
  std::string     topicname_cmd_vel, 
                  topicname_cmd_flipper,
                  topicname_joy,
                  framename_base_link,
                  framename_world;

  struct Axis
  {
    Axis(const std::string& _name)
      : axis(0), factor(0.0), offset(0.0), name(_name)
    {}
    int axis;
    double factor;
    double offset;
    std::string name;
  };

  struct Velocity
  {
    Velocity()
      : speed("Speed"), turn("Turn")
    {}
    Axis speed, turn;
  } velocity;

  struct Flipper
  {
    Flipper(const std::string& _name) 
     : w(5.0), currentAngle(0.0), defaultAngle(M_PI/4.0), 
       upButton(0), downButton(0), name(_name) 
    {}
    int    upButton, downButton;
    double w;
    double currentAngle;
    double defaultAngle;
    std::string name;
  };

  struct Flippers 
  {
    Flippers()
      : fr("fr"), fl("fl"), rr("rr"), rl("rl")
    {}
    Flipper fr, fl, rr, rl;
  } flipper;

public:
  Teleop()
  {
    // Make a nodehandle for reading parameters from the local namespace.
    ros::NodeHandle _nh("~");
    // TODO Read Topicnames and framenames
    _nh.param<std::string>("topicNameJoy",        topicname_joy, "joy");
    _nh.param<std::string>("frameNameWorld",      framename_world,"world");
    _nh.param<std::string>("frameNameBaselink",   framename_base_link, 
                                                            "base_link");
    _nh.param<std::string>("topicNameCmdvel",     topicname_cmd_vel, 
                                                              "cmd_vel");
    _nh.param<std::string>("topicNameCmdflipper", topicname_cmd_flipper,
                                                          "cmd_flipper");
    // Read parameters for structure velocity
    _nh.param<int>(   "speedAxis",      velocity.speed.axis, 1);
    _nh.param<int>(   "turnAxis",       velocity.turn.axis, 0);
    _nh.param<double>("maxSpeed",       velocity.speed.factor, .5);
    _nh.param<double>("maxTurnW",       velocity.turn.factor, 2.0*M_PI/4.0);
    // Read parameters for structure flipper.fr
    _nh.param<int>(   "frUpButton",     flipper.fr.upButton, 9);
    _nh.param<int>(   "frDownButton",   flipper.fr.downButton, 7);
    _nh.param<double>("frW",            flipper.fr.w, 2.0*M_PI/20.0);
    _nh.param<double>("frDefaultAngle", flipper.fr.defaultAngle, M_PI/4.0);
    flipper.fr.currentAngle = flipper.fr.defaultAngle;
    // Read parameters for structure flipper.fr
    _nh.param<int>(   "flUpButton",     flipper.fl.upButton, 8);
    _nh.param<int>(   "flDownButton",   flipper.fl.downButton, 6);
    _nh.param<double>("flW",            flipper.fl.w, 2.0*M_PI/20.0);
    _nh.param<double>("flDefaultAngle", flipper.fl.defaultAngle, M_PI/4.0);
    flipper.fl.currentAngle = flipper.fl.defaultAngle;
    // Read parameters for structure flipper.fr
    _nh.param<int>(   "rrUpButton",     flipper.rr.upButton, 13);
    _nh.param<int>(   "rrDownButton",   flipper.rr.downButton, 15);
    _nh.param<double>("rrW",            flipper.rr.w, 2.0*M_PI/20.0);
    _nh.param<double>("rrDefaultAngle", flipper.rr.defaultAngle, M_PI/4.0);
    flipper.rr.currentAngle = flipper.rr.defaultAngle;
    // Read parameters for structure flipper.fr
    _nh.param<int>(   "rlUpButton",     flipper.rl.upButton, 12);
    _nh.param<int>(   "rlDownButton",   flipper.rl.downButton, 14);
    _nh.param<double>("rlW",            flipper.rl.w, 2.0*M_PI/20.0);
    _nh.param<double>("rlDefaultAngle", flipper.rl.defaultAngle, M_PI/4.0);
    flipper.rl.currentAngle = flipper.rl.defaultAngle;

    joy_subscriber = node_handle_.subscribe<sensor_msgs::Joy>(topicname_joy, 1,
                       boost::bind(&Teleop::joyCallback, this, _1));
//MEMO:velocity_publisher = node_handle_.advertise<geometry_msgs::TwistStamped>(
    velocity_publisher = node_handle_.advertise<geometry_msgs::Twist>(
                           topicname_cmd_vel, 10);
    flipper_publisher  = node_handle_.advertise<geometry_msgs::Twist>(
                           topicname_cmd_flipper, 10);
    // Publishment for the initial posing.
    publish_flipper();
  }

  ~Teleop()
  {
    stop();
  }

  double updateCurrentFlipperAngle(Flipper& flpr, 
                                   const sensor_msgs::JoyConstPtr& joy)
  {
    if(getUpButton(joy, flpr) && getDownButton(joy, flpr))
      flpr.currentAngle = flpr.defaultAngle;
    else if(getUpButton(joy, flpr))
      flpr.currentAngle += flpr.w;
    else if(getDownButton(joy, flpr))
      flpr.currentAngle -= flpr.w;
    return flpr.currentAngle;
  }

  void publish_flipper(void)
  {
/*MEMO
    geometry_msgs::TwistStamped flp_ts;
    flp_ts.header.frame_id = framename_base_link;
    flp_ts.header.stamp = ros::Time::now();
    flp_ts.twist.linear.x  = flipper.fr.currentAngle;
    flp_ts.twist.linear.y  = flipper.fl.currentAngle;
    flp_ts.twist.linear.z  = 0;
    flp_ts.twist.angular.x = flipper.rr.currentAngle;
    flp_ts.twist.angular.y = flipper.rl.currentAngle;
    flp_ts.twist.angular.z = 0;
*/
    geometry_msgs::Twist flp_ts;
    flp_ts.linear.x  = flipper.fr.currentAngle;
    flp_ts.linear.y  = flipper.fl.currentAngle;
    flp_ts.linear.z  = 0;
    flp_ts.angular.x = flipper.rr.currentAngle;
    flp_ts.angular.y = flipper.rl.currentAngle;
    flp_ts.angular.z = 0;
    flipper_publisher.publish(flp_ts);
  }

  void joyCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    // Publish about velocity
/*MEMO
    geometry_msgs::TwistStamped vel_ts;
    vel_ts.header.frame_id = framename_base_link;
    vel_ts.header.stamp = ros::Time::now();
    vel_ts.twist.linear.x  = getAxis(joy, velocity.speed);
    vel_ts.twist.linear.y  = vel_ts.twist.linear.z  = 0;
    vel_ts.twist.angular.x = vel_ts.twist.angular.y = 0;
    vel_ts.twist.angular.z = getAxis(joy, velocity.turn)
                                            *((vel_ts.twist.linear.x<0)?-1:1);
    velocity_publisher.publish(vel_ts);
*/
    geometry_msgs::Twist vel_ts;
    vel_ts.linear.x  = getAxis(joy, velocity.speed);
    vel_ts.linear.y  = vel_ts.linear.z  = 0;
    vel_ts.angular.x = vel_ts.angular.y = 0;
    vel_ts.angular.z = getAxis(joy, velocity.turn)*((vel_ts.linear.x<0)?-1:1);
    velocity_publisher.publish(vel_ts);
    // Publish about flippers
    updateCurrentFlipperAngle(flipper.fr, joy);
    updateCurrentFlipperAngle(flipper.fl, joy);
    updateCurrentFlipperAngle(flipper.rr, joy);
    updateCurrentFlipperAngle(flipper.rl, joy);
    publish_flipper();
  }

  double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis)
  {
    if(axis.axis < 0 || axis.axis >= joy->axes.size())
    {
      ROS_ERROR_STREAM("Axis " << axis.name << " is out of range, joy has " << joy->axes.size() << " axes");
      return 0;
    }
    double output = joy->axes[axis.axis] * axis.factor + axis.offset;
    // TODO keep or remove deadzone? may not be needed
    // if(std::abs(output) < axis.max_ * 0.2)
    // {
    //   output = 0.0;
    // }
    return output;
  }

  bool getUpButton(const sensor_msgs::JoyConstPtr &joy, const Flipper &flpr)
  {
    if(flpr.upButton < 0 || flpr.upButton >= joy->buttons.size())
    {
      ROS_ERROR_STREAM("upButton of " << flpr.name << " is out of range, joy has " << joy->buttons.size() << " buttons");
      return false;
    }
    return joy->buttons[flpr.upButton] > 0;
  }

  bool getDownButton(const sensor_msgs::JoyConstPtr &joy, const Flipper &flpr)
  {
    if(flpr.downButton < 0 || flpr.downButton >= joy->buttons.size())
    {
      ROS_ERROR_STREAM("downButton of " << flpr.name << " is out of range, joy has " << joy->buttons.size() << " buttons");
      return false;
    }
    return joy->buttons[flpr.downButton] > 0;
  }

  void stop()
  {
    if(velocity_publisher.getNumSubscribers() > 0)
    {
      velocity_publisher.publish(geometry_msgs::TwistStamped());
    }
    if(flipper_publisher.getNumSubscribers() > 0)
    {
      flipper_publisher.publish(geometry_msgs::TwistStamped());
    }
  }
};

} // namespace teleop_crawler

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_crawler");

  teleop_crawler::Teleop teleop;
  ros::spin();

  return 0;
}
