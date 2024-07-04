#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <stdio.h>
#if(GAZEBO_MAJOR_VERSION <= 8)
#include <gazebo/math/gzmath.hh>
#endif
#include "flipper_control_msgs.hh"

#include <termios.h>
#include <iostream>

#include "gzJoystick.hh"                         // <<<<<<<<<<<<<<<<< ADD THIS
#include "gzIntervalTimer.hh"


#define D_SDFGET_NAME          this->model->GetName().c_str()
#define D_SDFGET_STRINGX(N)    ((!_sdf->HasElement(N))?\
                               NULL:_sdf->GetElement(N)->Get<std::string>())
#define D_SDFGET_JOINT(J,N)    gzmsg<<N<<":"<< D_SDFGET_STRINGX(N)<<std::endl; \
                               if(!(J = this->model->GetJoint(D_SDFGET_STRINGX(N))))\
                               {gzerr<<D_SDFGET_NAME<<":No JOINT <"<<N<<">"\
                                <<std::endl; return false;}

enum gzIT_number
{
  gzIT_Joy = 0,
  gzIT_MaxTimers
};

namespace gazebo
{
class MobileBasePlugin : public ModelPlugin
{
  gzIntervalTimer          gzIT[gzIT_MaxTimers];
  gzJoystick               gzJS; 
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
#if(GAZEBO_MAJOR_VERSION <= 7)
    this->node->Init(this->model->GetWorld()->GetName());
#endif
#if(GAZEBO_MAJOR_VERSION >= 8)
    this->node->Init(this->model->GetWorld()->Name());
#endif
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
    
    int err = 0;  
    err += gzJS.Init("Joy_Dev", _sdf);        // <<<<<<<<<<<<<<<<< ADD THIS
    
    gzIT[gzIT_Joy].Init(this->model);
    gzIT[gzIT_Joy].setintervalFreq(100);  // Hz

    // Read parameters for structure flipper.fr
    flipper.fr.upButton = 8;     // Printed as 9
    flipper.fr.downButton = 9;   // Printed as 10
    flipper.fr.w = 2.0 * M_PI / 20.0;
    flipper.fr.defaultAngle = M_PI / 4.0;
    flipper.fr.currentAngle = flipper.fr.defaultAngle;
    flipper.fr.last_upB = flipper.fr.last_downB = 0;
    // Read parameters for structure flipper.fr
    flipper.fl.upButton = 6;     // Printed as 7
    flipper.fl.downButton = 7;   // Printed as 8
    flipper.fl.w = 2.0 * M_PI / 20.0;
    flipper.fl.defaultAngle = M_PI / 4.0;
    flipper.fl.currentAngle = flipper.fl.defaultAngle;
    flipper.fl.last_upB = flipper.fl.last_downB = 0;
    // Read parameters for structure flipper.fr
    flipper.rr.upButton = 5;     // Printed as 6
    flipper.rr.downButton = 2;   // Printed as 3
    flipper.rr.w = 2.0 * M_PI / 20.0;
    flipper.rr.defaultAngle = M_PI / 4.0;
    flipper.rr.currentAngle = flipper.rr.defaultAngle;
    flipper.rr.last_upB = flipper.rr.last_downB = 0;
    // Read parameters for structure flipper.fr
    flipper.rl.upButton = 4;     // Printed as 5
    flipper.rl.downButton = 1;   // Printed as 2
    flipper.rl.w = 2.0 * M_PI / 20.0;
    flipper.rl.defaultAngle = M_PI / 4.0;
    flipper.rl.currentAngle = flipper.rl.defaultAngle;
    flipper.rl.last_upB = flipper.rl.last_downB = 0;
    
    Usage();
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
#if(GAZEBO_MAJOR_VERSION <= 8)
    float P = _target_angle - _joint->GetAngle(0).Radian();
#else
    float P = _target_angle - _joint->Position(0);
#endif
    P *= 10;
    // See also [JointController](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1JointController.html)
    //  Set torque fitting power and direction calculated by each angle.
    //  Seting calculated P as torque is very effective to stop shaking legs!!
    _joint->SetForce(0, P);
    // Set PID parameters
    model->GetJointController()->SetPositionPID(_joint->GetScopedName(), 
                                                  common::PID(10, 1, 0.005));
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

  void set_joy_vel(float jx, float jy)
  {
    // gzmsg << "cmd_vel: " << msg->position().x() << ", "
    //       <<msgs::Convert(msg->orientation()).GetAsEuler().z<<std::endl;
    double vel_lin = jx / this->wheelRadius;
#if(GAZEBO_MAJOR_VERSION == 5)
    double vel_rot = -1 * msgs::Convert(_msg->orientation()).GetAsEuler().z
                     * (this->wheelSeparation / this->wheelRadius);
#endif
#if(GAZEBO_MAJOR_VERSION >= 7)
    double vel_rot = -1 * jy
                     * (this->wheelSeparation / this->wheelRadius);
#endif
    set_velocity(vel_lin - vel_rot, vel_lin + vel_rot);
  }
 
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
    int    upButton, downButton, last_upB, last_downB;
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
  void Usage(void)
  {
    printf("==================== Joystick Usage ====================\n");
    printf("\n");
    printf(" Robot Moving   : The left analog stick\n");
    printf("--------------------------------------------------------\n");
    printf(" Flipper Moving : (followings are number of button)\n");
    printf("    Front Right : up=%d , down=%d\n"
                             , flipper.fr.upButton+1, flipper.fr.downButton+1);
    printf("    Front Left  : up=%d , down=%d\n"
                             , flipper.fl.upButton+1, flipper.fl.downButton+1);
    printf("    Rear  Right : up=%d , down=%d\n"
                             , flipper.rr.upButton+1, flipper.rr.downButton+1);
    printf("    Rear  Left  : up=%d , down=%d\n"
                             , flipper.rl.upButton+1, flipper.rl.downButton+1);
    printf("\n");
    printf("========================================================\n");    
  }
 
  #define Kjrx ( 0.02/32767.0) // m/sec で 1/32767.0 だと 1 m/sec
  #define Kjry ( 0.02/32767.0) 
  #define Kjlx (-2.0/32767.0) // rad/sec で 1/32767.0 だと 1 rad/sec
  #define Kjly (-2.0/32767.0)
  #define rxDeadzone (100)
  #define ryDeadzone (100)
  #define lxDeadzone (100)
  #define lyDeadzone (100)
  #define clippingDeadzone(X,D) (float)(((X)<(D)&&(X)>-(D))?0:(X))
  #define lxClipped clippingDeadzone(gzJS.axis[0].data(), lxDeadzone)
  #define lyClipped clippingDeadzone(gzJS.axis[1].data(), lyDeadzone)
  #define rxClipped clippingDeadzone(gzJS.axis[2].data(), rxDeadzone)
  #define ryClipped clippingDeadzone(gzJS.axis[3].data(), ryDeadzone)

  #define FlpAngStp 0.1
 // jrx,y が右スティック jlx,y が左スティック Button は 1~16 が 0~15 に対応

  void check_joystick(void)
  {
    /* [BUFFALO BSGP1601]
       Left Analog Stick       Right Analog Stick         Hat Swtiches
             [1]                     [3]                      [5]
            -32767                 -32767                    -32767
   [0]-32767  +  +32767    [2]-32767  +  +32767    [4]-32767   +   +32767
            +32767                 +32767                    +32767
    */
    // Get the joystick current status.
    gzJS.check_joystick();              // <<<<<<<<<<<<<<<<< ADD THIS and follows
    // Display the joystick current status for debugging.
//    if(gzJS.updated())
//      gzJS.disp_joystick();
    float  joyLX, joyLY;
//    float  joyRX, joyRY;
    joyLX = lxClipped * Kjlx; // X axis of the Analog Right Stick
    joyLY = lyClipped * Kjly; // Y axis of the Analog Right Stick
//    joyRX = rxClipped * Kjrx; // X axis of the Analog Right Stick
//    joyRY = ryClipped * Kjry; // Y axis of the Analog Right Stick
    // At pushing or releasing a button.........
    
    //ここでモーションを設定する．
    
  //right_front_fripper_motion
    if(gzJS.button[flipper.fr.upButton].changed() || gzJS.button[flipper.fr.downButton].changed())
    {
      if(gzJS.button[flipper.fr.upButton].changed() && gzJS.button[flipper.fr.downButton].changed())
      {
//         printf("Make the RIGHT FRONT flipper arm home position.\n"); 
         Target_FLP_FR = flipper.fr.defaultAngle;
      }
      if(gzJS.button[flipper.fr.upButton].changed())
      {
        if(gzJS.button[flipper.fr.upButton].pushed())
        {
//          printf("Button 9 was pushed.\n"); 
          Target_FLP_FR = Target_FLP_FR + FlpAngStp;
        }
        else if(gzJS.button[flipper.fr.upButton].released())
        {
//          printf("Button 9 was released.\n");
        }
        gzJS.button[flipper.fr.upButton].reset_change_status();
      }
      if(gzJS.button[flipper.fr.downButton].changed())
      {
        if(gzJS.button[flipper.fr.downButton].pushed())
        {
//          printf("Button 10 was pushed.\n");
          Target_FLP_FR = Target_FLP_FR - FlpAngStp;
        }
        else if(gzJS.button[flipper.fr.downButton].released())
        {
//          printf("Button 10 was released.\n"); 
        }
        gzJS.button[flipper.fr.downButton].reset_change_status();
      }
    }
    
  //left_front_fripper_motion
    if(gzJS.button[flipper.fl.upButton].changed() || gzJS.button[flipper.fl.downButton].changed())
    {
      if(gzJS.button[flipper.fl.upButton].changed() && gzJS.button[flipper.fl.downButton].changed())
      {
//        printf("Make the LEFT FRONT flipper arm home position.\n"); 
        Target_FLP_FL = flipper.fl.defaultAngle;
      }
      if(gzJS.button[flipper.fl.upButton].changed())
      {
        if(gzJS.button[flipper.fl.upButton].pushed())
        {
//          printf("Button 7 was pushed.\n"); 
          Target_FLP_FL = Target_FLP_FL + FlpAngStp;
        }
        else if(gzJS.button[flipper.fl.upButton].released())
        {
//          printf("Button 7 was released.\n");
        }
        gzJS.button[flipper.fl.upButton].reset_change_status();
      }
      if(gzJS.button[flipper.fl.downButton].changed())
      {
        if(gzJS.button[flipper.fl.downButton].pushed())
        {
//          printf("Button 8 was pushed.\n");
          Target_FLP_FL = Target_FLP_FL - FlpAngStp;
        }
        else if(gzJS.button[flipper.fl.downButton].released())
        {
//          printf("Button 8 was released.\n"); 
        }
        gzJS.button[flipper.fl.downButton].reset_change_status();
      }
    }
    
  //right_rear_fripper_motion
    if(gzJS.button[flipper.rr.upButton].changed() || gzJS.button[flipper.rr.downButton].changed())
    {
      if(gzJS.button[flipper.rr.upButton].changed() && gzJS.button[flipper.rr.downButton].changed())
      {
//         printf("Make the RIGHT REAR flipper arm home position.\n"); 
         Target_FLP_RR = flipper.rr.defaultAngle;
      }
      if(gzJS.button[flipper.rr.upButton].changed())
      {
        if(gzJS.button[flipper.rr.upButton].pushed())
        {
//          printf("Button 6 was pushed.\n"); 
          Target_FLP_RR = Target_FLP_RR + FlpAngStp;
        }
        else if(gzJS.button[flipper.rr.upButton].released())
        {
//          printf("Button 6 was released.\n");
        }
        gzJS.button[flipper.rr.upButton].reset_change_status();
      }
      if(gzJS.button[flipper.rr.downButton].changed())
      {
        if(gzJS.button[flipper.rr.downButton].pushed())
        {
//          printf("Button 3 was pushed.\n");
          Target_FLP_RR = Target_FLP_RR - FlpAngStp;
        }
        else if(gzJS.button[flipper.rr.downButton].released())
        {
//          printf("Button 3 was released.\n"); 
        }
        gzJS.button[flipper.rr.downButton].reset_change_status();
      }
    }
    
  //left_rear_fripper_motion
    if(gzJS.button[flipper.rl.upButton].changed() || gzJS.button[flipper.rl.downButton].changed())
    {
      if(gzJS.button[flipper.rl.upButton].changed() && gzJS.button[flipper.rl.downButton].changed())
      {
//         printf("Make the LEFT REAR flipper arm home position.\n"); 
         Target_FLP_RL = flipper.rl.defaultAngle;
      }
      if(gzJS.button[flipper.rl.upButton].changed())
      {
        if(gzJS.button[flipper.rl.upButton].pushed())
        {
//          printf("Button 5 was pushed.\n"); 
          Target_FLP_RL = Target_FLP_RL + FlpAngStp;
        }
        else if(gzJS.button[flipper.rl.upButton].released())
        {
//          printf("Button 5 was released.\n");
        }
        gzJS.button[flipper.rl.upButton].reset_change_status();
      }
      if(gzJS.button[flipper.rl.downButton].changed())
      {
        if(gzJS.button[flipper.rl.downButton].pushed())
        {
//          printf("Button 2 was pushed.\n");
          Target_FLP_RL = Target_FLP_RL - FlpAngStp;
        }
        else if(gzJS.button[flipper.rl.downButton].released())
        {
//          printf("Button 2 was released.\n"); 
        }
        gzJS.button[flipper.rl.downButton].reset_change_status();
      }
    }
    
  //robot moving control
    set_joy_vel(joyLY, joyLX);
  }

  public:
  void OnUpdate()
  {
    MoveWheel();
    MoveFlipper();
    if(gzIT[gzIT_Joy].overIntervalPeriod())
    {
      check_joystick();
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}
