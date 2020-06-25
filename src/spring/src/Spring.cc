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

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "Spring.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(Spring)

/////////////////////////////////////////////////
Spring::Spring()
{
}

/////////////////////////////////////////////////
void Spring::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());

  if (!_sdf->HasElement("joint_name"))
    gzerr << "Spring plugin missing <Joint_Name> element\n";

  if (!_sdf->HasElement("spring_constant"))
    gzerr << "Spring plugin missing <spring_constant> element\n";

  this->joint           = _model->GetJoint(
      _sdf->GetElement("joint_name")->Get<std::string>());
  this->spring_constant = 
      _sdf->GetElement("spring_constant")->Get<double>();

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Spring::OnUpdate, this));
}

/////////////////////////////////////////////////
void Spring::Init()
{
  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->joint->GetChild());

  ignition::math::Box bb = parent->BoundingBox();
  // This assumes that the largest dimension of the wheel is the diameter
  this->head_distance = bb.Size().Max();
}

/////////////////////////////////////////////////
void Spring::OnUpdate()
{
  /* double d1, d2;
  double dr, da;

  this->prevUpdateTime = currTime;

  // Distance travelled by front wheels
  d1 = stepTime.Double() * this->wheelRadius * this->leftJoint->GetVelocity(0);
  d2 = stepTime.Double() * this->wheelRadius * this->rightJoint->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / this->wheelSeparation;
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  */
//  this->joint->SetForce(0, -this->spring_constant * this->joint->Position(0).Radian());
//  if( this->joint->Position(0).Radian() > 0.045 )
  this->joint->SetForce(0, -this->spring_constant * this->joint->Position(0));
  if( this->joint->Position(0) > 0.045 )
  {
     this->joint->SetPosition(0, 0.045);
  }
}
