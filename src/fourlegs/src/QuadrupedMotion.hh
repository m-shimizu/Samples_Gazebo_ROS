/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_QUADRUPED_PLUGIN_HH_
#define _GAZEBO_QUADRUPED_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include "gzIntervalTimer.hh"

#define MAX_MOTORS 100

enum gzIT_number
{
  gzIT_Motion = 0,
  gzIT_Motor,
  gzIT_MaxTimers
};

namespace gazebo
{
  class GAZEBO_VISIBLE QuadrupedMotion : public ModelPlugin
  {
    public: QuadrupedMotion();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();
    private: void Move_A_Joint(int _motor);

    private: void OnVelMsg(ConstPosePtr &_msg);
    private: void OnKeyPress(ConstAnyPtr &_msg);

    private: transport::NodePtr node;
    private: transport::SubscriberPtr velSub, keySub;

    private: physics::ModelPtr model;
    private: physics::JointPtr Joint[MAX_MOTORS];
    private: event::ConnectionPtr updateConnection;
    private: double torque[MAX_MOTORS];
    private: ATI_PACK* mdblp;
    private: common::Time prevUpdateTime;

    private: physics::LinkPtr link, leftWheelLink, rightWheelLink;
    
    // For controling OnUpdate Speed
    private: gzIntervalTimer gzIT[gzIT_MaxTimers];
  };
}
#endif
