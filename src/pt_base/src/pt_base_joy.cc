// DETAILS OF TREATING JOY TOPIC in https://wiki.ros.org/joy/Tutorials/WritingTeleopNode

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#if(GAZEBO_MAJOR_VERSION <= 8)
#include <gazebo/math/gzmath.hh>
#endif
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <stdio.h>
//#include "flipper_control_msgs.hh"

#include <termios.h>
#include <iostream>

#include <gzReadXML.hh>
//#include <gzJoystick.hh>
#include <gzIntervalTimer.hh>

enum gzIT_number
{
  gzIT_TIMER1 = 0,
  gzIT_MaxTimers
};

/*
#define D_SDFGET_NAME          this->model->GetName().c_str()
//#define D_SDFGET_NAME          this->model->URI().Str().c_str()
//#define D_SDFGET_NAME          this->sdf->GetName().c_str()
//#define D_SDFGET(N,T)          this->sdf->GetElement(N)->Get<## T ##>()
//#define D_SDFGET_TYPE(X,N,D,T) if(!this->sdf->HasElement(N))\
//                               {ROS_WARN("%s:No <%s>, used default value",\
//                                D_SDFGET_NAME,N);X=D;} else {X=SDFGET(N,T);}
#define D_SDFGET_ELMT(N)       ((!this->sdf->HasElement(N))?NULL:\
                               this->sdf->GetElement(N)->Get<std::string>()\
                               .c_str())
#define D_SDFGET_JOINT(J,N)    ROS_INFO("%s:%s=%s",\
                                            D_SDFGET_NAME,N,D_SDFGET_ELMT(N));\
                               if(!(J=this->model->GetJoint(D_SDFGET_ELMT(N))))\
                               {ROS_ERROR("%s:No JOINT <%s>",D_SDFGET_NAME,N);\
                                return false;}
#define D_SDFGET_STRING(X,N,D) if(!this->sdf->HasElement(N))\
                               {ROS_WARN("%s:No <%s>, used default value",\
                                D_SDFGET_NAME,N);X=D;} else\
                               {X=this->sdf->GetElement(N)->Get<std::string>();}
#define D_SDFGET_DOUBLE(X,N,D) if(!this->sdf->HasElement(N))\
                               {ROS_WARN("%s:No <%s>, used default value",\
                                D_SDFGET_NAME,N);X=D;} else\
                               {X=this->sdf->GetElement(N)->Get<double>();}
#define D_SDFGET_BOOL(X,N,D) if(!this->sdf->HasElement(N))\
                               {ROS_WARN("%s:No <%s>, used default value",\
                                D_SDFGET_NAME,N);X=D;} else\
                               {X=this->sdf->GetElement(N)->Get<bool>();}
//#define D_SDFGET_STRING(S,N,D) D_SDFGET_TYPE(S,N,D,std::string)
//#define D_SDFGET_DOUBLE(S,N,D) D_SDFGET_TYPE(S,N,D,double)
//#define D_SDFGET_BOOL(S,N,D)   D_SDFGET_TYPE(S,N,D,bool)
*/

namespace gazebo
{
  class MobileBasePlugin : public ModelPlugin
  {
  private:
    transport::NodePtr node;
    physics::ModelPtr  model;
    sdf::ElementPtr    sdf;
    gzIntervalTimer    gzIT[gzIT_MaxTimers];
    gzReadXML          gzXML;

    // Loop Event
    event::ConnectionPtr updateConnection;

    physics::JointPtr  Hinge_PAN;
    physics::JointPtr  Hinge_TILT;
    
    // pt_base joystick axis and scale 
    int                Axes_PAN, Axes_TILT;
    int                Button_RESET_PAN, Button_RESET_TILT;
    double             Scale_PAN, Scale_TILT;

    // pt_base target angle 
    double             Target_PAN, Target_TILT;

    // Update Rate
    double             update_rate_;
              
    // ROS Callback Queue
    ros::CallbackQueue queue_;
    GazeboRosPtr       gazebo_ros_;
    boost::mutex       lock_;
    boost::thread      callback_queue_thread_;

    // ROS STUFF
    ros::Subscriber joy_subscriber_;
    ros::Publisher  pt_base_publisher_;
    std::string robotnamespace_;
    std::string topicname_joy_;
    std::string topicname_pt_base_;

    // Not in use
    double torque;
    double gain;
    
  public:
    MobileBasePlugin(void)
    {
      Axes_PAN    = 0;
      Axes_TILT   = 1;
      Scale_PAN   = 3.14;
      Scale_TILT  = 1.57;
      Target_PAN  = 0;
      Target_TILT = 0;
      torque      = 5.0;
      gain        = 1.0;
    }

    ~MobileBasePlugin()
    {
//      alive_ = false;
      queue_.clear();
      queue_.disable();
      gazebo_ros_->node()->shutdown();
      callback_queue_thread_.join();
    }

    void QueueThread()
    {
      static const double timeout = 0.01;
      while(/*alive_ &&*/gazebo_ros_->node()->ok())
      {
        queue_.callAvailable (ros::WallDuration(timeout));
      }
    }

    /////////////////////////////////////////////////
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) 
    {
//  printf("Calling joyCallback\n");
      boost::mutex::scoped_lock scoped_lock(lock_);
      Target_PAN  += joy->axes[Axes_PAN ] * Scale_PAN;
      Target_TILT += joy->axes[Axes_TILT] * Scale_TILT;
      if(joy->buttons[Button_RESET_PAN])
        Target_PAN = 0;
      if(joy->buttons[Button_RESET_TILT])
        Target_TILT = 0; 
      if(pt_base_publisher_.getNumSubscribers() > 0)
      {
        geometry_msgs::Twist t;
        t.linear.x  = 0;
        t.linear.y  = 0;
        t.linear.z  = 0;
        t.angular.x = 0;
        t.angular.y = Target_TILT;
        t.angular.z = Target_PAN;
        pt_base_publisher_.publish(t);
      }
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // physics::WorldPtr world = physics::get_world("default");
      this->model = _model;
      this->sdf   = _sdf;
      // Make sure the ROS node for Gazebo has already been initialized
      gazebo_ros_ = GazeboRosPtr(new GazeboRos(model, _sdf, "robot"));
      gazebo_ros_->isInitialized();
      this->node = transport::NodePtr(new transport::Node());
  #if(GAZEBO_MAJOR_VERSION <= 8)
      this->node->Init(this->model->GetWorld()->GetName());
  #else
      this->node->Init(this->model->GetWorld()->Name());
  #endif
      gzXML.Init(this->model, this->sdf);

      // SDF: getting parameters
      int err = 0;
      err += gzXML.RegisterJoint(Hinge_PAN, "hinge_pan");
//  (err > 0)?printf("Hinge_PAN\n"):0;
      err += gzXML.RegisterJoint(Hinge_TILT,"hinge_tilt");
      err += gzXML.GetString(robotnamespace_, "robotNamespace");
      err += gzXML.GetDouble(gain, "gain");
      err += gzXML.GetDouble(torque, "torque");
      err += gzXML.GetString(topicname_joy_, "joyTopic");
//  printf("topicname_joy:%s\n", topicname_joy_.c_str());
      err += gzXML.GetString(topicname_pt_base_, "pt_baseTopic");
//  printf("topicname_joy:%s\n", topicname_joy_.c_str());
      err += gzXML.GetDouble(update_rate_, "updateRate"); 
//  printf("Update Rate:%f\n", update_rate_);
      err += gzXML.GetInt(Axes_PAN , "axes_pan"); 
//  printf("Axes_PAN:   %d\n", Axes_PAN);
      err += gzXML.GetInt(Axes_TILT, "axes_tilt"); 
//  printf("Axes_TILT:  %d\n", Axes_TILT);
      err += gzXML.GetDouble(Scale_PAN , "scale_pan"); 
//  printf("Scale_PAN:  %f\n", Scale_PAN);
      err += gzXML.GetDouble(Scale_TILT, "scale_tilt"); 
//  printf("Scale_TILT: %f\n", Scale_TILT);
      err += gzXML.GetInt(Button_RESET_PAN, "button_reset_pan"); 
//  printf("Button_RESET_PAN: %d\n", Button_RESET_PAN);
      err += gzXML.GetInt(Button_RESET_TILT, "button_reset_tilt"); 
//  printf("Button_RESET_TILT: %d\n", Button_RESET_TILT);

      if (err > 0)
      {
//  printf("Load Error\n");
        gzerr << "There are some errors in loading XML values\n";
        return;
      }
      // ROS: getting parameters with overwriting gazebo parameters
/*
      gazebo_ros_->getParameter<std::string> (robotnamespace_, 
                                            "robotNamespace", robotnamespace_);
      gazebo_ros_->getParameter<std::string> (topicname_joy_, 
                                            "topicname_joy", topicname_joy_);
      gazebo_ros_->getParameter<double>(update_rate_,"updateRate",update_rate_); 
*/
      // Settings for interval timers in the OnUpdate function.
      gzIT[gzIT_TIMER1].Init(this->model);
      gzIT[gzIT_TIMER1].setintervalFreq(update_rate_);  // Hz 

      // ROS: Registering Subscribers and Publishers
      ros::SubscribeOptions sojoy =
//      ros::SubscribeOptions::create<geometry_msgs::Twist>
        ros::SubscribeOptions::create<sensor_msgs::Joy>
            (topicname_joy_,
            1, boost::bind(&MobileBasePlugin::joyCallback, this, _1),
            ros::VoidPtr(), &queue_);
      joy_subscriber_ = gazebo_ros_->node()->subscribe(sojoy);
      pt_base_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::Twist>
                           (topicname_pt_base_, 1);
      // Start custom queue for hearing ROS topics
      this->callback_queue_thread_ =
        boost::thread(boost::bind(&MobileBasePlugin::QueueThread, this));
      // Automatic loop event
      this->updateConnection
        = event::Events::ConnectWorldUpdateBegin(
                  boost::bind(&MobileBasePlugin::OnUpdate, this));
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
    void Move_A_Joint(physics::JointPtr _joint, double _target_angle)
    {
  #if(GAZEBO_MAJOR_VERSION <= 8)
      float P = _target_angle - _joint->GetAngle(0).Radian();
  #else
      float P = _target_angle - _joint->Position(0);
  #endif
      P *= 1;
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
  //ROS_INFO("TA:%f", _target_angle);
    }

    void move_pt_base(void)
    {
      Move_A_Joint(Hinge_PAN,  Target_PAN);
      Move_A_Joint(Hinge_TILT, Target_TILT);
//  printf("Target_PAN : %f\n", Target_PAN);
//  printf("Target_TILT: %f\n", Target_TILT);
    }
   
    public:
    void OnUpdate(void)
    {
//  printf("OnUpdate\n");
      if(gzIT[gzIT_TIMER1].overIntervalPeriod())
      {
        move_pt_base();
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}
