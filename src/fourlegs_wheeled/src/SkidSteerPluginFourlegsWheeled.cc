/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <string>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "SkidSteerPluginFourlegsWheeled.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(SkidSteerDrivePlugin)

/////////////////////////////////////////////////
SkidSteerDrivePlugin::SkidSteerDrivePlugin()
{
  this->wheelRadius = 0.0;
  this->wheelSeparation = 0.0;
}

/////////////////////////////////////////////////
int SkidSteerDrivePlugin::RegisterJoint(int _index, const std::string &_name
                                                       , sdf::ElementPtr _sdf)
{
  if(!_sdf->HasElement(_name.c_str()))
  {
    gzerr << "Quadruped plugin missing <" << _name 
          << "> element" << std::endl;
    return 1;
  }

  // Bounds checking on index
  if (_index < 0 || _index >= NUMBER_OF_WHEELS)
  {
    gzerr << "Joint index " << _index <<  " out of bounds [0, "
          << NUMBER_OF_WHEELS << "] in model " << this->model->GetName()
          << "." << std::endl;
    return 1;
  }

  // Find the specified joint and add it to out list
  this->joints[_index] = this->model->GetJoint(
                     _sdf->GetElement(_name.c_str())->Get<std::string>());
  if (!this->joints[_index])
  {
    gzerr << "Unable to find the " << _name
          <<  " joint in model " << this->model->GetName() << "." << std::endl;
    return 1;
  }

  // Success!
  return 0;
}

/////////////////////////////////////////////////
void SkidSteerDrivePlugin::Disp_Usage()
{
  static int dec = -1;
  if(dec < 0)
  {
    dec = 10;
  }
  else
  {
    dec--;
    return;
  }
  printf("\n");
  printf("Key commands for wheels:\n");
  printf(" i:    Go forward\n");
  printf(" j:    Turn left\n");
  printf(" l:    Turn right\n");
  printf(" k:    Stop\n");
  printf(" ,:    Go backward\n");
  printf(" 6:    Speed slowest\n");
  printf(" 7:    Speed slow\n");
  printf(" 8:    Speed middle(default)\n");
  printf(" 9:    Speed fast\n");
  printf(" 0:    Speed fastest\n");
  printf("\n");
}

/////////////////////////////////////////////////
void SkidSteerDrivePlugin::Load(physics::ModelPtr _model,
                                sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());

  int err = 0;

  err += RegisterJoint(RIGHT_FRONT, "right_front", _sdf);
  err += RegisterJoint(RIGHT_REAR,  "right_rear",  _sdf);
  err += RegisterJoint(LEFT_FRONT,  "left_front",  _sdf);
  err += RegisterJoint(LEFT_REAR,   "left_rear",   _sdf);

  if (err > 0)
    return;

  this->joints[RIGHT_FRONT]->SetVelocity(0, 0);
  this->joints[RIGHT_REAR ]->SetVelocity(0, 0);
  this->joints[LEFT_FRONT ]->SetVelocity(0, 0);
  this->joints[LEFT_REAR  ]->SetVelocity(0, 0);

  // This assumes that front and rear wheel spacing is identical
  this->wheelSeparation = this->joints[RIGHT_FRONT]->Anchor(0).Distance(
                          this->joints[LEFT_FRONT]->Anchor(0));

  // This assumes that the largest dimension of the wheel is the diameter
  // and that all wheels have the same diameter
  physics::EntityPtr wheelLink = boost::dynamic_pointer_cast<physics::Entity>(
                                        this->joints[RIGHT_FRONT]->GetChild() );
  if (wheelLink)
  {
#if(GAZEBO_MAJOR_VERSION >= 11)
    ignition::math::AxisAlignedBox bb = wheelLink->BoundingBox();
#else
    ignition::math::Box bb = wheelLink->BoundingBox();
#endif
    this->wheelRadius = bb.Size().Max() * 0.5;
  }

  // Validity checks...
  if (this->wheelSeparation <= 0)
  {
    gzerr << "Unable to find the wheel separation distance." << std::endl
          << "  This could mean that the right_front link and the left_front "
          << "link are overlapping." << std::endl;
    return;
  }
  if (this->wheelRadius <= 0)
  {
    gzerr << "Unable to find the wheel radius." << std::endl
          << "  This could mean that the sdf is missing a wheel link on "
          << "the right_front joint." << std::endl;
    return;
  }

  Disp_Usage();

/*
  this->velSub = this->node->Subscribe(
    std::string("~/") + this->model->GetName() + std::string("/vel_cmd"),
    &SkidSteerDrivePlugin::OnVelMsg, this);
*/
  this->keySub = this->node->Subscribe(std::string("~/keyboard/keypress"), 
      &SkidSteerDrivePlugin::OnKeyPress, this);
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&SkidSteerDrivePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void SkidSteerDrivePlugin::OnVelMsg(ConstPosePtr &_msg)
{
  // gzmsg << "cmd_vel: " << msg->position().x() << ", "
  //       << msgs::Convert(msg->orientation()).GetAsEuler().z << std::endl;

  double vel_lin = _msg->position().x() / this->wheelRadius;
  double vel_rot = -1 * msgs::ConvertIgn(_msg->orientation()).Euler().Z()
                   * (this->wheelSeparation / this->wheelRadius);

  this->joints[RIGHT_FRONT]->SetVelocity(0, vel_lin - vel_rot);
  this->joints[RIGHT_REAR ]->SetVelocity(0, vel_lin - vel_rot);
  this->joints[LEFT_FRONT ]->SetVelocity(0, vel_lin + vel_rot);
  this->joints[LEFT_REAR  ]->SetVelocity(0, vel_lin + vel_rot);
}

static double wheel_x = 0;
static double wheel_z = 0;
static double wheel_speed = 0.5;

/////////////////////////////////////////////////
void SkidSteerDrivePlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  const auto key = static_cast<const unsigned int>(_msg->int_value());
  gzmsg << "KEY(" << key << ") pressed\n";
  switch(key)
  {
    case 'k': wheel_x = 0 , wheel_z = 0;
          break;
    case 'i': wheel_x = 1 , wheel_z = 0;
          break;
    case ',': wheel_x = -1 , wheel_z = 0;
          break;
    case 'l': wheel_x = 0 , wheel_z = -1;
          break;
    case 'j': wheel_x = 0 , wheel_z = 1;
          break;
    case 'o': wheel_x = 1 , wheel_z = -1;
          break;
    case 'u': wheel_x = 1 , wheel_z = 1;
          break;
    case '.': wheel_x = -1 , wheel_z = 1;
          break;
    case 'm': wheel_x = -1 , wheel_z = -1;
          break;
    case '6': wheel_speed = 0.1;
          break;
    case '7': wheel_speed = 0.5;
          break;
    case '8': wheel_speed = 1;
          break;
    case '9': wheel_speed = 1.5;
          break;
    case '0': wheel_speed = 2;
          break;
    default : printf("%c is not command\n", key);
          Disp_Usage();
          break;
  }   
}

/////////////////////////////////////////////////
void SkidSteerDrivePlugin::DoDrive()
{
  double x, z;
  x = wheel_x * wheel_speed;
  z = wheel_z * wheel_speed * 0.8;
  double vel_lin = x / this->wheelRadius;
  double vel_rot = -z * (this->wheelSeparation / this->wheelRadius);

  this->joints[RIGHT_FRONT]->SetVelocity(0, vel_lin - vel_rot);
  this->joints[RIGHT_REAR ]->SetVelocity(0, vel_lin - vel_rot);
  this->joints[LEFT_FRONT ]->SetVelocity(0, vel_lin + vel_rot);
  this->joints[LEFT_REAR  ]->SetVelocity(0, vel_lin + vel_rot);
}

/////////////////////////////////////////////////
void SkidSteerDrivePlugin::OnUpdate()
{
  /* double d1, d2;
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  */
  static int dec = 10;
  if(dec < 0)
  {
    dec = 10;
    DoDrive();
  }
  else
    dec--;
}
