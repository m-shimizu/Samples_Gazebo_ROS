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
}

/////////////////////////////////////////////////
void _2link_arm::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

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

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&_2link_arm::OnUpdate, this));
}

/////////////////////////////////////////////////
void _2link_arm::Init()
{
}

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

/////////////////////////////////////////////////
// To control joint behaviors by keyboard input directory
void	_2link_arm::check_key_command(void)
{
	if(doslike_kbhit())
	{
		int cmd = doslike_getch();
		switch(cmd)
		{
			case 'q': this->JointS->SetForce(0, 1);
				  break;
			case 'a': this->JointS->SetForce(0, -1);
				  break;
			case 'e': this->JointE->SetForce(0, 1);
				  break;
			case 'd': this->JointE->SetForce(0, -1);
				  break;
		}
	}
	printf("Soulder : %f\n", this->JointS->GetAngle(0).Degree());
	printf("Elbow : %f\n", this->JointE->GetAngle(0).Degree());
}

/////////////////////////////////////////////////
void _2link_arm::OnVelMsg(ConstPosePtr &_msg)
{
}

/////////////////////////////////////////////////
void _2link_arm::OnUpdate()
{
  check_key_command();
}
