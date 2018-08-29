//THIS PROGRAM WAS A MODIFY OF ....
/* This is my first plugin file. */
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
#include "twoLinkArmPlugin.hh"

// Useful defines
#define _MAX(X,Y)	((X > Y)?X:Y)
#define _MIN(X,Y)	((X < Y)?X:Y)

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(_2link_arm)

/////////////////////////////////////////////////
_2link_arm::_2link_arm()
{
  Px = 0.4, Py = 0;
}

/////////////////////////////////////////////////
void _2link_arm::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
#if(GAZEBO_MAJOR_VERSION <= 7)
  this->node->Init(this->model->GetWorld()->GetName());
#endif
#if(GAZEBO_MAJOR_VERSION >= 8)
  this->node->Init(this->model->GetWorld()->Name());
#endif

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &_2link_arm::OnVelMsg, this);

  if (!_sdf->HasElement("shoulder"))
    gzerr << "DiffDrive plugin missing <shoulder> element\n";

  if (!_sdf->HasElement("elbow"))
    gzerr << "DiffDrive plugin missing <elbow> element\n";

  this->JointS = _model->GetJoint(
      _sdf->GetElement("shoulder")->Get<std::string>());
  this->JointE = _model->GetJoint(
      _sdf->GetElement("elbow")->Get<std::string>());

  if (!this->JointS)
    gzerr << "Unable to find shoulder joint["
          << _sdf->GetElement("shoulder")->Get<std::string>() << "]\n";
  if (!this->JointE)
    gzerr << "Unable to find elbow joint["
          << _sdf->GetElement("elbow")->Get<std::string>() << "]\n";

  this->keySub = this->node->Subscribe(std::string("~/keyboard/keypress"), 
      &_2link_arm::OnKeyPress, this);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&_2link_arm::OnUpdate, this));
}

/////////////////////////////////////////////////
void _2link_arm::Init()
{
}

// Before Gazebo7 including Gazebo7, doslike_kbhit and doslike_getch worked correctly.
// But after Gazebo8, they no longer worked.
// Especially, the funcion tcsetattr won't work.
/////////////////////////////////////////////////
// To know pushing any key
int	doslike_kbhit(void)
{
	struct termios	oldt, newt;
	int	ch;
	int	oldf;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);
	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}
	return 0;
}

// Before Gazebo7 including Gazebo7, doslike_kbhit and doslike_getch worked correctly.
// But after Gazebo8, they no longer worked.
// Especially, the funcion tcsetattr won't work.
/////////////////////////////////////////////////
// To gwt a charactor code of a pushed key
int	doslike_getch(void)
{
	static struct termios	oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	int c = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return c;
}

///////////////////////////////////////////////////
// The inverse kinematics calculation
void ik(float* th1, float* th2, float px, float py)
{
  float L1 = 0.2 , L2 = 0.2 , D;
  D = sqrt(px*px + py*py);
  if(D < L1 + L2)
  {
    *th2 = -(3.14 - acos( (L1*L1 + L2*L2 - D*D) / (2*L1*L2) ));
    *th1 = atan2(py, px) + acos( (L1*L1 + D*D - L2*L2) / (2*L1*D) );
  }
  else
  {
    *th2 = 0;
    *th1 = atan2(py, px);
  }
}

/////////////////////////////////////////////////
// To control joint behaviors by keyboard input directory
void	_2link_arm::OnKeyPress(ConstAnyPtr &_msg)
{
  const auto key = static_cast<const unsigned int>(_msg->int_value());
  gzmsg << "KEY(" << key << ") pressed\n";
  switch(key)
	{
		case 'f': Px += 0.05;
			  break;
		case 's': Px -= 0.05;
			  break;
		case 'e': Py += 0.05;
			  break;
		case 'c': Py -= 0.05;
			  break;
		case 'd': Px = 0.4, Py = 0;
			  break;
	}
}

/////////////////////////////////////////////////
// OLD VERSION!! To control joint behaviors by keyboard input directory
void	_2link_arm::check_key_command(void)
{
	if(doslike_kbhit())
	{
		int cmd = doslike_getch();
		switch(cmd)
		{
			case 'f': Px += 0.05;
				  break;
			case 's': Px -= 0.05;
				  break;
			case 'e': Py += 0.05;
				  break;
			case 'c': Py -= 0.05;
				  break;
			case 'd': Px = 0.4, Py = 0;
				  break;
		}
	}
}

#define LEVEL 1 // 0 : Through at all.
                // 1 : Simple state machine.
/////////////////////////////////////////////////
void Strategy(float* Xt, float* Yt, double Xp, double Yp)
{
#if(LEVEL==0)
  *Xt = Xp;
  *Yt = Yp;
#endif 
#if(LEVEL==1)
  int state = 0; // 0 means "Stand by".
  if(Xp > 0.4)
  {
    state = 1; // 1 means "Aiming".
  }
  else if(Xp > 0.3 && Xp < 0.4)
  {
    state = 2; // 2 means "Hitting".
  }
  switch(state)
  {
    case 0 : *Xt = 0.2*sqrt(2);
             *Yt = 0;
             break;
    case 1 : *Xt = 0.2*sqrt(2);
             *Yt = Yp;
             break;
    case 2 : *Xt = Xp;
             *Yt = Yp;
             break;
  }
#endif
}

/////////////////////////////////////////////////
void _2link_arm::OnVelMsg(ConstVector3dPtr &_msg)
{
//  printf("== %3.1f, %3.1f\n", _msg->x(), _msg->y());
  Strategy(&Px, &Py, _msg->x(), _msg->y() );
}

/////////////////////////////////////////////////
void _2link_arm::PID_Control(void)
{
  float Monitor_Angle_Shoulder, Monitor_Angle_Elbow;
  float Target_Angle_Shoulder , Target_Angle_Elbow;
  float OrderS, OrderE;
  ik(&Target_Angle_Shoulder, &Target_Angle_Elbow, Px, Py);
  Monitor_Angle_Shoulder = this->JointS->GetAngle(0).Radian();
  Monitor_Angle_Elbow    = this->JointE->GetAngle(0).Radian();
/*
	printf("Monitor Angle Soulder : %f\n", this->JointS->GetAngle(0).Degree());
	printf("Monitor Angle Elbow   : %f\n", this->JointE->GetAngle(0).Degree());
	printf("Target Angle Shoulder : %f\n", Target_Angle_Shoulder);
	printf("Target Angle Elbow    : %f\n", Target_Angle_Elbow);
*/
  // Proportional Control
  OrderS = -10 * (Monitor_Angle_Shoulder - Target_Angle_Shoulder);
  OrderE = -10 * (Monitor_Angle_Elbow - Target_Angle_Elbow);

  this->PidS = common::PID(0.1, 0, 0);
  this->PidE = common::PID(0.1, 0, 0);

  this->model->GetJointController()->SetVelocityPID(
        this->JointS->GetScopedName(), this->PidS);
  this->model->GetJointController()->SetVelocityPID(
        this->JointE->GetScopedName(), this->PidE);

  this->JointS->SetVelocity(0, OrderS);
  this->JointE->SetVelocity(0, OrderE);
}

/////////////////////////////////////////////////
void _2link_arm::OnUpdate()
{
//  check_key_command(); <= Needless forever!!
  PID_Control();
}
