//THIS PROGRAM WAS A MODIFY OF ....
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
#ifndef _2LINKARMPLUGINHH
#define _2LINKARMPLUGINHH

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE _2link_arm : public ModelPlugin
  {
    public: _2link_arm();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();
    private: void PID_Control(void);

    private: void OnVelMsg(ConstVector3dPtr &_msg);
    private: void check_key_command(void);

    private: transport::NodePtr node;
    private: transport::SubscriberPtr velSub;

    private: float Px, Py;
    private: physics::ModelPtr model;
    private: physics::JointPtr JointS, JointE;
    private: event::ConnectionPtr updateConnection;
    private: common::Time prevUpdateTime;

    private: common::PID PidS, PidE; // PID parameters in velocity control
  };
}
#endif
