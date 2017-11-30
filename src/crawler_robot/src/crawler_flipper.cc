#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <stdio.h>
#include <gazebo/math/gzmath.hh>
#include "flipper_control_msgs.hh"

#include <termios.h>
#include <iostream>

#define Gp 3.0  //0.05
#define Gi 0.00 //0.00005

#define D_GET_JOINT(J,N) if(!(J = this->model->GetJoint(N))) gzerr << "Unable to find " << N << std::endl

namespace gazebo
{
class MobileBasePlugin : public ModelPlugin
{
  transport::NodePtr node;
  physics::ModelPtr  model;
  common::Time       simTime;

  transport::SubscriberPtr velSub;
  transport::SubscriberPtr flpSub;
  transport::SubscriberPtr statsSub;
  event::ConnectionPtr updateConnection;

  physics::JointPtr hinge1;
  physics::JointPtr hinge2;
  physics::JointPtr hinge3;
  physics::JointPtr hinge4;
  physics::JointPtr hinge5;
  physics::JointPtr hinge6;
  physics::JointPtr hinge7;
  physics::JointPtr hinge8;
  physics::JointPtr hinge9;
  physics::JointPtr hinge10;
  physics::JointPtr hinge11;
  physics::JointPtr hinge12;
  physics::JointPtr hinge13;
  physics::JointPtr hinge14;
  physics::JointPtr hinge15;
  physics::JointPtr hinge16;
  physics::JointPtr hinge17;
  physics::JointPtr hinge18;
  physics::JointPtr hinge19;
  physics::JointPtr hinge20;
  physics::JointPtr hinge21;
  physics::JointPtr hinge22;
  physics::JointPtr hinge23;
  physics::JointPtr hinge24;
  physics::JointPtr hinge25;
  physics::JointPtr hinge26;
  physics::JointPtr hinge27;
  physics::JointPtr hinge28;

  //sensors::RaySensorPtr laser;
  physics::LinkPtr  sensor;

  /// Wheel speed and gain
  double THETA[30];
  double gain;
 
  /// Distance between wheels on the same axis (Determined from SDF)
  double wheelSeparation;

  /// Radius of the wheels (Determined from SDF)
  double wheelRadius;

  /// Flipper target angle 
  double FLP_FR, FLP_FL, FLP_RR, FLP_RL;

  /// Flipper angle controller
  double dev_FLP_FR;
  double dev_FLP_FL;
  double dev_FLP_RR;
  double dev_FLP_RL;
  double dev_FLP_FR_sum;
  double dev_FLP_FL_sum;
  double dev_FLP_RR_sum;
  double dev_FLP_RL_sum;

  public:
  MobileBasePlugin(void)
  {
    FLP_FR = FLP_FL = FLP_RR = FLP_RL = M_PI / 4;
    dev_FLP_FR = dev_FLP_FL = dev_FLP_RR = dev_FLP_RL = 0;
    dev_FLP_FR_sum = dev_FLP_FL_sum = dev_FLP_RR_sum = dev_FLP_RL_sum = 0;
    wheelRadius     = 0.2;
    wheelSeparation = 1;
    for(unsigned int i = 0; i < sizeof(THETA)/sizeof(THETA[0]); i++)
      THETA[i] = 0;
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // physics::WorldPtr world = physics::get_world("default");
    this->model = _model;
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());
    if(this->LoadParams(_sdf))
    {
      this->velSub = this->node->Subscribe(
      std::string("~/") + this->model->GetName() + std::string("/vel_cmd"),
      &MobileBasePlugin::OnVelMsg, this);
      this->flpSub = this->node->Subscribe(
      std::string("~/") + this->model->GetName() + std::string("/flp_cmd"),
      &MobileBasePlugin::OnFlpMsg, this);
      this->updateConnection
        = event::Events::ConnectWorldUpdateBegin(
                  boost::bind(&MobileBasePlugin::OnUpdate, this));
    }
  }
 
  bool LoadParams(sdf::ElementPtr _sdf)
  {
    if(!_sdf->HasElement("gain"))
    {
        gzerr << "param [gain] not found\n";
        return false;
    }
    else
        this->gain = _sdf->Get<double>("gain");
    D_GET_JOINT(hinge1,"right_front");
    D_GET_JOINT(hinge2,"right_center1");
    D_GET_JOINT(hinge3,"right_center2");
    D_GET_JOINT(hinge4,"right_rear");

    D_GET_JOINT(hinge5,"left_front");
    D_GET_JOINT(hinge6,"left_center1");
    D_GET_JOINT(hinge7,"left_center2");
    D_GET_JOINT(hinge8,"left_rear");

    D_GET_JOINT(hinge9,"right_front_arm");
    D_GET_JOINT(hinge10,"right_rear_arm");
    D_GET_JOINT(hinge11,"left_front_arm");
    D_GET_JOINT(hinge12,"left_rear_arm");

    D_GET_JOINT(hinge13,"right_front_arm_wheel_1");
    D_GET_JOINT(hinge14,"right_front_arm_wheel_2");
    D_GET_JOINT(hinge15,"right_front_arm_wheel_3");

    D_GET_JOINT(hinge16,"left_front_arm_wheel_1");
    D_GET_JOINT(hinge17,"left_front_arm_wheel_2");
    D_GET_JOINT(hinge18,"left_front_arm_wheel_3");

    D_GET_JOINT(hinge19,"right_rear_arm_wheel_1");
    D_GET_JOINT(hinge20,"right_rear_arm_wheel_2");
    D_GET_JOINT(hinge21,"right_rear_arm_wheel_3");

    D_GET_JOINT(hinge22,"left_rear_arm_wheel_1");
    D_GET_JOINT(hinge23,"left_rear_arm_wheel_2");
    D_GET_JOINT(hinge24,"left_rear_arm_wheel_3");

    D_GET_JOINT(hinge25,"right_sub2");
    D_GET_JOINT(hinge26,"right_sub3");
    D_GET_JOINT(hinge27,"left_sub2");
    D_GET_JOINT(hinge28,"left_sub3");
    return true;
  }

  /////////////////////////////////////////////////
  void OnVelMsg(ConstPosePtr &_msg)
  {
    // gzmsg << "cmd_vel: " << msg->position().x() << ", "
    //       <<msgs::Convert(msg->orientation()).GetAsEuler().z<<std::endl;
    double vel_lin = _msg->position().x() / this->wheelRadius;
#if(GAZEBO_MAJOR_VERSION == 5)
    double vel_rot = -1 * msgs::Convert(_msg->orientation()).GetAsEuler().z
                     * (this->wheelSeparation / this->wheelRadius);
#endif
#if(GAZEBO_MAJOR_VERSION == 7)
    double vel_rot = -1 * msgs::ConvertIgn(_msg->orientation()).Euler().Z()
                     * (this->wheelSeparation / this->wheelRadius);
#endif
    set_velocity(vel_lin - vel_rot, vel_lin + vel_rot);
  }

  void set_velocity(double  vr, double  vl)
  {
    THETA[1] = vr;
    THETA[2] = vr;
    THETA[3] = vr;
    THETA[4] = vr;
    THETA[13] = vr;
    THETA[14] = vr;
    THETA[15] = vr;
    THETA[19] = vr;
    THETA[20] = vr;
    THETA[21] = vr;
    THETA[25] = vr;
    THETA[26] = vr;

    THETA[5] = vl;
    THETA[6] = vl;
    THETA[7] = vl;
    THETA[8] = vl;
    THETA[16] = vl;
    THETA[17] = vl;
    THETA[18] = vl;
    THETA[22] = vl;
    THETA[23] = vl;
    THETA[24] = vl;
    THETA[27] = vl;
    THETA[28] = vl;
  }

  /////////////////////////////////////////////////
  void OnFlpMsg(ConstFlipperControlPtr &_msg)
  {
    FLP_FR = _msg->fr();
    FLP_FL = _msg->fl();
    FLP_RR = _msg->rr();
    FLP_RL = _msg->rl();
  }

  /////////////////////////////////////////////////
  void MoveWheel(void)
  {
    hinge1->SetVelocity(0, THETA[1]);
    hinge2->SetVelocity(0, THETA[2]);
    hinge3->SetVelocity(0, THETA[3]);
    hinge4->SetVelocity(0, THETA[4]);
    hinge5->SetVelocity(0, THETA[5]);
    hinge6->SetVelocity(0, THETA[6]);
    hinge7->SetVelocity(0, THETA[7]);
    hinge8->SetVelocity(0, THETA[8]);
    hinge13->SetVelocity(0, THETA[13]);
    hinge14->SetVelocity(0, THETA[14]);
    hinge15->SetVelocity(0, THETA[15]);
    hinge16->SetVelocity(0, THETA[16]);
    hinge17->SetVelocity(0, THETA[17]);
    hinge18->SetVelocity(0, THETA[18]);
    hinge19->SetVelocity(0, THETA[19]);
    hinge20->SetVelocity(0, THETA[20]);
    hinge21->SetVelocity(0, THETA[21]);
    hinge22->SetVelocity(0, THETA[22]);
    hinge23->SetVelocity(0, THETA[23]);
    hinge24->SetVelocity(0, THETA[24]);
    hinge25->SetVelocity(0, THETA[25]);
    hinge26->SetVelocity(0, THETA[26]);
    hinge27->SetVelocity(0, THETA[27]);
    hinge28->SetVelocity(0, THETA[28]);
  }

  void MoveFlipper(void)
  {
    // Get current arm angle
    double flp_fr = hinge9->GetAngle(0).Radian();
    double flp_fl = hinge11->GetAngle(0).Radian();
    double flp_rr = hinge10->GetAngle(0).Radian();   
    double flp_rl = hinge12->GetAngle(0).Radian();
//printf("right_front=%6.3f right_rear=%6.3f left_front=%6.3f left_rear=%6.3f \r ", flp_fr, flp_fl, flp_rr, flp_rl);
    // Following codes are manual flipper PI controls by velocity.
    // Calc anguler velocity of Front Right flipper
    dev_FLP_FR = flp_fr - (-FLP_FR);
    THETA[9] = dev_FLP_FR*(-1)*Gp + dev_FLP_FR_sum*(-1)*Gi;
    dev_FLP_FR_sum += dev_FLP_FR;
    // Calc anguler velocity of Front Left flipper
    dev_FLP_FL = flp_fl - (-FLP_FL);    
    THETA[11] = dev_FLP_FL*(-1)*Gp + dev_FLP_FL_sum*(-1)*Gi; 
    dev_FLP_FL_sum += dev_FLP_FL;
    // Calc anguler velocity of Rear Right flipper
    dev_FLP_RR = flp_rr - FLP_RR;
    THETA[10] = dev_FLP_RR*(-1)*Gp + dev_FLP_RR_sum*(-1)*Gi;
    dev_FLP_RR_sum += dev_FLP_RR;
    // Calc anguler velocity of Rear Left flipper
    dev_FLP_RL = flp_rl - FLP_RL;    
    THETA[12] = dev_FLP_RL*(-1)*Gp + dev_FLP_RL_sum*(-1)*Gi; 
    dev_FLP_RL_sum += dev_FLP_RL;
    // Set flipper anguler velocity
    hinge9->SetVelocity(0, THETA[9]);
    hinge10->SetVelocity(0, THETA[10]);
    hinge11->SetVelocity(0, THETA[11]);
    hinge12->SetVelocity(0, THETA[12]);
  }
 
  public:
  void OnUpdate()
  {
    MoveWheel();
    MoveFlipper();
  }
};

GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}
