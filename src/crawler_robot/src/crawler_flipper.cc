#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <stdio.h>
#include <gazebo/math/gzmath.hh>
#include "flipper_control_msgs.hh"

#include <termios.h>
#include <iostream>

#define D_SDFGET_NAME          this->model->GetName().c_str()
#define D_SDFGET_STRINGX(N)    ((!_sdf->HasElement(N))?\
                               NULL:_sdf->GetElement(N)->Get<std::string>())
#define D_SDFGET_JOINT(J,N)    gzmsg<<N<<":"<< D_SDFGET_STRINGX(N)<<std::endl; \
                               if(!(J = this->model->GetJoint(D_SDFGET_STRINGX(N))))\
                               {gzerr<<D_SDFGET_NAME<<":No JOINT <"<<N<<">"\
                                <<std::endl; return false;}

namespace gazebo
{
class MobileBasePlugin : public ModelPlugin
{
  transport::NodePtr node;
  physics::ModelPtr  model;
  common::Time       simTime;
  // Gazebo Topic
  transport::SubscriberPtr velSub;
  transport::SubscriberPtr flpSub;
  // Loop Event
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
  physics::JointPtr hinge29;
  physics::JointPtr hinge30;

  /// Wheel speed and gain
  double Target_VEL_R, Target_VEL_L;
  double gain;
 
  /// Distance between wheels on the same axis (Determined from SDF)
  double wheelSeparation;

  /// Radius of the wheels (Determined from SDF)
  double wheelRadius;

  /// Flipper target angle 
  double Target_FLP_FR, Target_FLP_FL, Target_FLP_RR, Target_FLP_RL;

  public:
  MobileBasePlugin(void)
  {
    Target_FLP_FR = Target_FLP_FL = Target_FLP_RR = Target_FLP_RL = M_PI / 4;
    wheelRadius     = 0.2;
    wheelSeparation = 1;
    Target_VEL_R = Target_VEL_L = 0;
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
    D_SDFGET_JOINT(hinge1,"right_front");
    D_SDFGET_JOINT(hinge2,"right_center1");
    D_SDFGET_JOINT(hinge3,"right_center2");
    D_SDFGET_JOINT(hinge29,"right_center3");
    D_SDFGET_JOINT(hinge4,"right_rear");

    D_SDFGET_JOINT(hinge5,"left_front");
    D_SDFGET_JOINT(hinge6,"left_center1");
    D_SDFGET_JOINT(hinge7,"left_center2");
    D_SDFGET_JOINT(hinge30,"left_center3");
    D_SDFGET_JOINT(hinge8,"left_rear");

    D_SDFGET_JOINT(hinge9,"right_front_arm");
    D_SDFGET_JOINT(hinge10,"right_rear_arm");
    D_SDFGET_JOINT(hinge11,"left_front_arm");
    D_SDFGET_JOINT(hinge12,"left_rear_arm");

    D_SDFGET_JOINT(hinge13,"right_front_arm_wheel_1");
    D_SDFGET_JOINT(hinge14,"right_front_arm_wheel_2");
    D_SDFGET_JOINT(hinge15,"right_front_arm_wheel_3");

    D_SDFGET_JOINT(hinge16,"left_front_arm_wheel_1");
    D_SDFGET_JOINT(hinge17,"left_front_arm_wheel_2");
    D_SDFGET_JOINT(hinge18,"left_front_arm_wheel_3");

    D_SDFGET_JOINT(hinge19,"right_rear_arm_wheel_1");
    D_SDFGET_JOINT(hinge20,"right_rear_arm_wheel_2");
    D_SDFGET_JOINT(hinge21,"right_rear_arm_wheel_3");

    D_SDFGET_JOINT(hinge22,"left_rear_arm_wheel_1");
    D_SDFGET_JOINT(hinge23,"left_rear_arm_wheel_2");
    D_SDFGET_JOINT(hinge24,"left_rear_arm_wheel_3");

    D_SDFGET_JOINT(hinge25,"right_sub2");
    D_SDFGET_JOINT(hinge26,"right_sub3");
    D_SDFGET_JOINT(hinge27,"left_sub2");
    D_SDFGET_JOINT(hinge28,"left_sub3");
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
#if(GAZEBO_MAJOR_VERSION >= 7)
    double vel_rot = -1 * msgs::ConvertIgn(_msg->orientation()).Euler().Z()
                     * (this->wheelSeparation / this->wheelRadius);
#endif
    set_velocity(vel_lin - vel_rot, vel_lin + vel_rot);
  }

  void set_velocity(double  vr, double  vl)
  {
    Target_VEL_R = vr;
    Target_VEL_L = vl;
  }

  /////////////////////////////////////////////////
  void OnFlpMsg(ConstFlipperControlPtr &_msg)
  {
    Target_FLP_FR = _msg->fr();
    Target_FLP_FL = _msg->fl();
    Target_FLP_RR = _msg->rr();
    Target_FLP_RL = _msg->rl();
  }

  /////////////////////////////////////////////////
  void Move_A_Joint_In_Velocity(physics::JointPtr _joint, double _target_vel)
  {
    float P = _target_vel - _joint->GetVelocity(0);
    P *= 1;
    // See also [JointController](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1JointController.html)
    //  Set torque fitting power and direction calculated by each angle.
    //  Seting calculated P as torque is very effective to stop shaking legs!!
//    _joint->SetForce(0, 10);
    // Set PID parameters
    model->GetJointController()->SetVelocityPID(_joint->GetScopedName(),
                                                        common::PID(1, 0, 0));
    // Set distination angle
    model->GetJointController()->SetVelocityTarget(_joint->GetScopedName(), 
                                                                 _target_vel); 
  }

  /////////////////////////////////////////////////
  void MoveWheel(void)
  {
    // Right Side
    Move_A_Joint_In_Velocity(hinge1, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge2, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge3, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge4, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge13, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge14, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge15, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge19, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge20, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge21, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge25, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge26, Target_VEL_R);
    Move_A_Joint_In_Velocity(hinge29, Target_VEL_R);
    // Left side
    Move_A_Joint_In_Velocity(hinge5, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge6, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge7, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge8, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge16, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge17, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge18, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge22, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge23, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge24, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge27, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge28, Target_VEL_L);
    Move_A_Joint_In_Velocity(hinge30, Target_VEL_L);
  }

  /////////////////////////////////////////////////
  void Move_A_Joint_In_Angle(physics::JointPtr _joint, double _target_angle)
  {
    float P = _target_angle - _joint->GetAngle(0).Radian();
    P *= 10;
    // See also [JointController](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1JointController.html)
    //  Set torque fitting power and direction calculated by each angle.
    //  Seting calculated P as torque is very effective to stop shaking legs!!
    _joint->SetForce(0, P);
    // Set PID parameters
    model->GetJointController()->SetPositionPID(_joint->GetScopedName(), 
                                                  common::PID(0.4, 1, 0.005));
    // Set distination angle
    model->GetJointController()->SetPositionTarget(_joint->GetScopedName(),
                                                               _target_angle); 
  }

  void MoveFlipper(void)
  {
    Move_A_Joint_In_Angle(hinge9, -Target_FLP_FR);
    Move_A_Joint_In_Angle(hinge10, Target_FLP_RR);
    Move_A_Joint_In_Angle(hinge11, -Target_FLP_FL);
    Move_A_Joint_In_Angle(hinge12, Target_FLP_RL);
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
