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
#include "Motion.hh"
#include "QuadrupedMotion.hh"

// Before Gazebo7 including Gazebo7, doslike_kbhit and doslike_getch worked correctly.
// But after Gazebo8, they no longer worked.
// Especially, the funcion tcsetattr won't work.
int  doslike_kbhit(void)
{
  struct termios  oldt, newt;
  int  ch;
  int  oldf;
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
int  doslike_getch(void)
{
  static struct termios  oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(QuadrupedMotion)

const char*joint_name[]={"J_FL_B20","J_FR_B20","J_RR_B20","J_RL_B20"
                           ,"J_FL_021", "J_FR_021", "J_RR_021", "J_RL_021"
                           ,"J_FL_122", "J_FR_122", "J_RR_122", "J_RL_122"};

#define G_JOINTS (int)(sizeof(joint_name)/sizeof(char*))

float current_joint_target_angle[G_JOINTS];

void set_angle(int _motor, float _angle) // _angle's unit is degree
{
  if(-1 < _motor && G_JOINTS > _motor)
  {
    current_joint_target_angle[_motor] = _angle / 180.0 * 3.14;
  }
}
//////////////////////////////////////////////////////////////////////

// Write Initializing codes 
/************************************/
/*      Sample Motion Data Base     */
/************************************/

int FL_N[] = {0,4,8}; // Front-Left Leg's Motor Number
int RR_N[] = {2,6,10}; // Rear-Left Leg's Motor Number
int FR_N[] = {1,5,9}; // Front-Right Leg's Motor Number
int RL_N[] = {3,7,11}; // Rear-Left Leg's Motor Number
ATI STDFL;
ATI STDRR;
ATI STDFR;
ATI STDRL;
ATI_CELL  STAND_M[] = {{0, 0} // Motion Data by Each Motor
            ,{  0, 0}
            ,{0, 0} };
/*
ATI_CELL  STAND_M[] = {{0, 50}, {-30, 50} // Motion Data by Each Motor
            ,{  0, 50}, {0, 50}
            ,{0, 50}, {30, 50} };
            */
ATI*    STAND_ATI[]= {&STDFR,&STDFL,&STDRR,&STDRL};
float   STAND_MS[] = {1, 1, 1, 1};
int     STAND_SS[] = {0, 0, 0, 0};
START_ID  STAND_SI[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
ATI_PACK  STAND_L;

void  Set_Stand_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array
  Init_ATI(&STDFR, 3, 1, FR_N, STAND_M);
  Init_ATI(&STDFL, 3, 1, FL_N, STAND_M);
  Init_ATI(&STDRR, 3, 1, RR_N, STAND_M);
  Init_ATI(&STDRL, 3, 1, RL_N, STAND_M);
//ATI_PACK, ATIs, {&ATI1, &ATI2..}, {Speed1, Speed2..}, {Start Step1, Start Step2..}, {Wait1, Wait2..}
  Clear_ATI_PACK(&STAND_L);
  Init_ATI_PACK(&STAND_L, 4, STAND_ATI, STAND_MS, STAND_SS, STAND_SI);
  Set_Speed_of_ATI_PACK(&STAND_L, 1.0);
}

ATI WLKFL;
ATI WLKRR;
ATI WLKFR;
ATI WLKRL;
ATI_CELL  MDLGOF_M[] =  {{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25}// Motion Data by Each Motor
            ,{ 30,  25},{ 30, 50},{  0, 50},{-30,  25},{-30, 25},{  0, 25}
            ,{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25} };
ATI_CELL  MDLGOB_M[] =  {{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25}
            ,{-30,  25},{-30, 50},{  0, 50},{ 30,  25},{ 30, 25},{  0, 25}
            ,{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25} };
int WLKFL_I[] = {0,1,2,3,4,5};// Motion Index
// ATIs, {&ATI1, &ATI2..}, {Speed1, Speed2..}, MasterSpeed, {Start Step1, Start Step2..}, {Wait1, Wait2..}
ATI*    WALK_ATI[]= {&WLKFL,&WLKRR,&WLKFR,&WLKRL};
float   WALK_MS[] = {1, 1, 1, 1};
int     WALK_SS[] = {5, 5, 2, 2};
START_ID  WALK_SI[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
ATI_PACK  WALK_L;

void  Set_Forward_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array
  Init_ATI(&WLKFL, 3, 6, FL_N, MDLGOF_M);
  Set_Index_to_ATI(&WLKFL, WLKFL_I);
  Init_ATI(&WLKRR, 3, 6, RR_N, MDLGOB_M);
  Init_ATI(&WLKFR, 3, 6, FR_N, MDLGOF_M);
  Init_ATI(&WLKRL, 3, 6, RL_N, MDLGOB_M);
}

void  Set_Backward_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array
  Init_ATI(&WLKFL, 3, 6, FL_N, MDLGOB_M);
  Set_Index_to_ATI(&WLKFL, WLKFL_I);
  Init_ATI(&WLKRR, 3, 6, RR_N, MDLGOF_M);
  Init_ATI(&WLKFR, 3, 6, FR_N, MDLGOB_M);
  Init_ATI(&WLKRL, 3, 6, RL_N, MDLGOF_M);
}

ATI_CELL  MDLTRNF_M[] = {{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25}// Motion Data by Each Motor
            ,{ 30,  25},{ 30, 50},{  0, 50},{-30,  25},{-30, 25},{  0, 25}
            ,{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25} };
ATI_CELL  MDLTRNB_M[] = {{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25}
            ,{-30,  25},{-30, 50},{  0, 50},{ 30,  25},{ 30, 25},{  0, 25}
            ,{ 40,  25},{-30, 50},{-30, 50},{-30,  25},{ 40, 25},{ 40, 25} };

void  Set_TurnRight_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array
  Init_ATI(&WLKFL, 3, 6, FL_N, MDLTRNF_M);
  Set_Index_to_ATI(&WLKFL, WLKFL_I);
  Init_ATI(&WLKRR, 3, 6, RR_N, MDLTRNF_M);
  Init_ATI(&WLKFR, 3, 6, FR_N, MDLTRNB_M);
  Init_ATI(&WLKRL, 3, 6, RL_N, MDLTRNB_M);
}

void  Set_TurnLeft_Motion(void)
{
//ATI, Motors, Motions, Motor number array, Motion Data(Motors X Motion) array, Index array
  Init_ATI(&WLKFL, 3, 6, FL_N, MDLTRNB_M);
  Set_Index_to_ATI(&WLKFL, WLKFL_I);
  Init_ATI(&WLKRR, 3, 6, RR_N, MDLTRNB_M);
  Init_ATI(&WLKFR, 3, 6, FR_N, MDLTRNF_M);
  Init_ATI(&WLKRL, 3, 6, RL_N, MDLTRNF_M);
}

void  Init_ML(void)
{
  Set_Stand_Motion();
  Set_Forward_Motion();
//ATI_PACK, ATIs, {&ATI1, &ATI2..}, {Speed1, Speed2..}, {Start Step1, Start Step2..}, {Wait1, Wait2..}
  Clear_ATI_PACK(&WALK_L);
  Init_ATI_PACK(&WALK_L, 4, WALK_ATI, WALK_MS, WALK_SS, WALK_SI);
  Set_Speed_of_ATI_PACK(&WALK_L, 1);
}

////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////
QuadrupedMotion::QuadrupedMotion()
{
}

/////////////////////////////////////////////////
void get_joint_info_from_sdf(physics::JointPtr& _joint
                           , const char* joint_name_in_sdf
                           , physics::ModelPtr _model
                           , sdf::ElementPtr _sdf)
{
  if(_sdf->HasElement(joint_name_in_sdf))
  {
    _joint = _model->GetJoint(
        _sdf->GetElement(joint_name_in_sdf)->Get<std::string>());
    if(!_joint)
      gzerr << "Unable to find " << joint_name_in_sdf << "["
            << _sdf->GetElement(joint_name_in_sdf)->Get<std::string>()
            << "]" << std::endl;
  }
  else
  {
    gzerr << "Quadruped plugin missing <" << joint_name_in_sdf 
          << "> element" << std::endl;
  }
}

/////////////////////////////////////////////////
void QuadrupedMotion::Load(physics::ModelPtr _model,
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
      this->model->GetName()+"/vel_cmd", &QuadrupedMotion::OnVelMsg, this);
  for(int J=0; J < G_JOINTS; J++)
    get_joint_info_from_sdf(this->Joint[J], joint_name[J], _model, _sdf);
  Init();
  this->keySub = this->node->Subscribe(std::string("~/keyboard/keypress"), 
      &QuadrupedMotion::OnKeyPress, this);
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&QuadrupedMotion::OnUpdate, this));
}

void Disp_Usage(void)
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
  printf("Key commands:\n");
  printf(" e:    Go forward\n");
  printf(" s:    Turn left at the place\n");
  printf(" f:    Turn right at the place\n");
  printf(" d:    Stop and get stop pose\n");
  printf(" c:    Go backward\n");
  printf(" 1:    Speed slowest\n");
  printf(" 2:    Speed slow\n");
  printf(" 3:    Speed middle(default)\n");
  printf(" 4:    Speed fast\n");
  printf(" 5:    Speed fastest\n");
  printf(" space:Freeze\n");
  printf("\n");
}

void QuadrupedMotion::Init()
{
/*
  this->wheelSeparation = this->leftJoint->GetAnchor(0).Distance(
      this->rightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->leftJoint->GetChild());

  math::Box bb = parent->GetBoundingBox();
  // This assumes that the largest dimension of the wheel is the diameter
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;
*/
  mdblp = &STAND_L;
  Init_Motion(16);
  Init_ML();
  Set_FitSteps(25);
  for(int J=0; J < G_JOINTS; J++)
    current_joint_target_angle[J] = 0;
  Disp_Usage();
}

/////////////////////////////////////////////////
void QuadrupedMotion::OnVelMsg(ConstPosePtr &_msg)
{
  /*
  double vr, va;
  vr = _msg->position().x();
#if(GAZEBO_MAJOR_VERSION == 5)
  va =  msgs::Convert(_msg->orientation()).GetAsEuler().z;
#endif
#if(GAZEBO_MAJOR_VERSION >= 7)
  va =  msgs::ConvertIgn(_msg->orientation()).Euler().Z();
#endif
  this->wheelSpeed[LEFT] = vr + va * this->wheelSeparation / 2.0;
  this->wheelSpeed[RIGHT] = vr - va * this->wheelSeparation / 2.0;
  */
}

#include "Motion.hh"
void set_walk_motion(ATI_PACK ** mdblp, int kind_of_motion, float motion_speed)
{
  if(kind_of_motion >= 1 && kind_of_motion <= 4)
  {
    Set_Speed_of_ATI_PACK(*mdblp, motion_speed);
    switch(kind_of_motion)
    {
      case 1: Set_Forward_Motion();   break;
      case 2: Set_Backward_Motion();  break;
      case 3: Set_TurnRight_Motion(); break;
      case 4: Set_TurnLeft_Motion();  break;
    }
    *mdblp = &WALK_L;
    ResetAllMotions(*mdblp);
    StartAllMotions(*mdblp);
  }
}

// This is the old version, currently no use.
void  check_key_command(ATI_PACK ** mdblp)
{
  static int   motion_flag  = 0;
  static float motion_speed = 1;
  if(doslike_kbhit())
  {
    int cmd = doslike_getch();
    switch(cmd)
    {
      case ' ': StopAllMotions(*mdblp);
            break;
      case 'd': motion_flag = 0;
            *mdblp = &STAND_L;
            StartAllMotions(*mdblp);
            break;
      case 'e': motion_flag = 1;
            set_walk_motion(mdblp, motion_flag, motion_speed);
            break;
      case 'c': motion_flag = 2;
            set_walk_motion(mdblp, motion_flag, motion_speed);
            break;
      case 'f': motion_flag = 3;
            set_walk_motion(mdblp, motion_flag, motion_speed);
            break;
      case 's': motion_flag = 4;
            set_walk_motion(mdblp, motion_flag, motion_speed);
            break;
      case '1': motion_speed = 0.3;
            set_walk_motion(mdblp, motion_flag, motion_speed);
            break;
      case '2': motion_speed = 0.8;
            set_walk_motion(mdblp, motion_flag, motion_speed);
            break;
      case '3': motion_speed = 1;
            set_walk_motion(mdblp, motion_flag, motion_speed);
            break;
      case '4': motion_speed = 1.5;
            set_walk_motion(mdblp, motion_flag, motion_speed);
            break;
      case '5': motion_speed = 2;
            set_walk_motion(mdblp, motion_flag, motion_speed);
            break;
      default : printf("%c is not command\n", cmd);
            Disp_Usage();
            break;
    }   
  }
}

/////////////////////////////////////////////////
void QuadrupedMotion::OnKeyPress(ConstAnyPtr &_msg)
{
  static int   motion_flag  = 0;
  static float motion_speed = 1;
  const auto key = static_cast<const unsigned int>(_msg->int_value());
  gzmsg << "KEY(" << key << ") pressed\n";
  switch(key)
  {
    case ' ': StopAllMotions(mdblp);
          break;
    case 'd': motion_flag = 0;
          mdblp = &STAND_L;
          StartAllMotions(mdblp);
          break;
    case 'e': motion_flag = 1;
          set_walk_motion(&mdblp, motion_flag, motion_speed);
          break;
    case 'c': motion_flag = 2;
          set_walk_motion(&mdblp, motion_flag, motion_speed);
          break;
    case 'f': motion_flag = 3;
          set_walk_motion(&mdblp, motion_flag, motion_speed);
          break;
    case 's': motion_flag = 4;
          set_walk_motion(&mdblp, motion_flag, motion_speed);
          break;
    case '1': motion_speed = 0.3;
          set_walk_motion(&mdblp, motion_flag, motion_speed);
          break;
    case '2': motion_speed = 0.8;
          set_walk_motion(&mdblp, motion_flag, motion_speed);
          break;
    case '3': motion_speed = 1;
          set_walk_motion(&mdblp, motion_flag, motion_speed);
          break;
    case '4': motion_speed = 1.5;
          set_walk_motion(&mdblp, motion_flag, motion_speed);
          break;
    case '5': motion_speed = 2;
          set_walk_motion(&mdblp, motion_flag, motion_speed);
          break;
    default : printf("%c is not command\n", key);
          Disp_Usage();
          break;
  }   
}

/////////////////////////////////////////////////
void QuadrupedMotion::Move_A_Joint(int _motor)
{
//  float P = current_joint_target_angle[_motor] - this->Joint[_motor]->Angle(0).Radian();
  float P = current_joint_target_angle[_motor] - this->Joint[_motor]->Position(0);
  P *= 10;
  // See also [JointController](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1JointController.html)
  //  Set torque fitting power and direction calculated by each angle.
  //  Seting calculated P as torque is very effective to stop shaking legs!!
  this->Joint[_motor]->SetForce(0, P);
  // Set PID parameters
  this->model->GetJointController()->SetPositionPID(
    this->Joint[_motor]->GetScopedName(), common::PID(0.4, 1, 0.005));
  // Set distination angle
  this->model->GetJointController()->SetPositionTarget(
    this->Joint[_motor]->GetScopedName(), current_joint_target_angle[_motor]); 
}

/////////////////////////////////////////////////
void QuadrupedMotion::OnUpdate()
{
  /* double d1, d2;
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  */
  static int dec = 10;
  if(dec < 0)
  {
    dec = 10;
//  check_key_command(&mdblp);
    MotionPlayer(mdblp);
  }
  else
    dec--;
  for(int _motor = 0; _motor < G_JOINTS; _motor++)
    Move_A_Joint(_motor);
  this->model->GetJointController()->Update();
}
