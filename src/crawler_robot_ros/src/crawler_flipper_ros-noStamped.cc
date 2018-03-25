// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <stdio.h>
//#include "flipper_control_msgs.hh"

#include <termios.h>
#include <iostream>

#define D_SDFGET_NAME          this->model->GetName().c_str()
//#define D_SDFGET_NAME          this->model->URI().Str().c_str()
//#define D_SDFGET_NAME          this->sdf->GetName().c_str()
//#define D_SDFGET(N,T)          this->sdf->GetElement(N)->Get<## T ##>()
//#define D_SDFGET_TYPE(X,N,D,T) if(!this->sdf->HasElement(N))\
//                               {ROS_WARN("%s:No <%s>, used default value",\
//                                D_SDFGET_NAME,N);X=D;} else {X=SDFGET(N,T);}
#define D_SDFGET_JOINT(J,N)    if(!(J = this->model->GetJoint(N)))\
                               {ROS_ERROR("%s:No <%s>",D_SDFGET_NAME,N);\
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

namespace gazebo
{
class MobileBasePlugin : public ModelPlugin
{
  transport::NodePtr node;
  physics::ModelPtr  model;
  sdf::ElementPtr    sdf;
  common::Time       simTime;
/*
  // Gazebo Topic
  transport::SubscriberPtr velSub;
  transport::SubscriberPtr flpSub;
*/
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
 
  /// Flipper target angle 
  double Target_FLP_FR, Target_FLP_FL, Target_FLP_RR, Target_FLP_RL;

  // Update Rate
  double       update_rate_;
  double       update_period_;
  common::Time last_update_time_;
            
  // ROS Callback Queue
  ros::CallbackQueue queue_;
  GazeboRosPtr       gazebo_ros_;
  boost::mutex       lock_;
  boost::thread      callback_queue_thread_;

  // ROS STUFF
  ros::Subscriber cmdvel_subscriber_;
  ros::Subscriber cmdflipper_subscriber_;
  ros::Publisher  odom_publisher_;

  std::string robotnamespace_;
  std::string topicname_cmdvel_;
  std::string topicname_cmdflp_;
  std::string topicname_odom_;
  std::string framename_odom_;
  std::string framename_robotBaseFrame_;
  std::string tf_prefix_;
  bool        broadcast_tf_;
  tf::TransformBroadcaster* transform_broadcaster_;
  nav_msgs::Odometry odom_;

  // Odom STUFF
  double wheel_separation_;
  double wheel_diameter_;
  double covariance_x_;
  double covariance_y_;
  double covariance_yaw_;

  // Not in use
  double torque;
  double gain;
  
  public:
  MobileBasePlugin(void)
  {
    Target_FLP_FR = Target_FLP_FL = Target_FLP_RR = Target_FLP_RL = M_PI / 4;
    wheel_separation_ = 1;
    wheel_diameter_ = 0.4;
    torque = 5.0;
    covariance_x_ = 0.0001;
    covariance_y_ = 0.0001;
    covariance_yaw_ = 0.01;
    Target_VEL_R = Target_VEL_L = 0;
  }

  ~MobileBasePlugin()
  {
//    alive_ = false;
    delete transform_broadcaster_;
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
  void velCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock_);
    double vlin = cmd_msg->linear.x / (wheel_diameter_/2);
    double vrot = cmd_msg->angular.z * wheel_separation_ / (wheel_diameter_/2);
    set_velocity(vlin + vrot, vlin - vrot);    
  }

  /////////////////////////////////////////////////
  void flipperCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock_);
    Target_FLP_FR = cmd_msg->linear.x;
    Target_FLP_FL = cmd_msg->linear.y;
    Target_FLP_RR = cmd_msg->angular.x;
    Target_FLP_RL = cmd_msg->angular.y;
  }

//##########################################################################
//##########################################################################
//##########################################################################
//##########################################################################
//##########################################################################
  void publishOdometry(double step_time)
  {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, framename_odom_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, framename_robotBaseFrame_);

    // TODO create some non-perfect odometry!
    // getting data for base_footprint to odom transform
    math::Pose pose = model->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    if (this->broadcast_tf_) 
    {
      transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time,
            odom_frame, base_footprint_frame));
    }
    // publish odom topic
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;
    odom_.pose.covariance[0] = this->covariance_x_;
    odom_.pose.covariance[7] = this->covariance_y_;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = this->covariance_yaw_;

    // get velocity in /odom frame
    math::Vector3 linear;
    linear = this->model->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->model->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;
    odom_.twist.covariance[0] = this->covariance_x_;
    odom_.twist.covariance[7] = this->covariance_y_;
    odom_.twist.covariance[14] = 1000000000000.0;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;
    odom_.twist.covariance[35] = this->covariance_yaw_;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odom_publisher_.publish(odom_);
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
    this->node->Init(this->model->GetWorld()->GetName());
    if(this->LoadParams(_sdf))
    {
/*
      this->velSub = this->node->Subscribe(
      std::string("~/") + this->model->GetName() + std::string("/vel_cmd"),
      &MobileBasePlugin::OnVelMsg, this);
      this->flpSub = this->node->Subscribe(
      std::string("~/") + this->model->GetName() + std::string("/flp_cmd"),
      &MobileBasePlugin::OnFlpMsg, this);
*/
      // ROS: setting parameters
      gazebo_ros_->getParameter<std::string> (robotnamespace_, 
                                            "robotNamespace", robotnamespace_);
      gazebo_ros_->getParameter<std::string> (topicname_cmdvel_, 
                                                 "cmd_vel", topicname_cmdvel_);
      gazebo_ros_->getParameter<std::string> (topicname_cmdflp_, 
                                             "cmd_flipper", topicname_cmdflp_);
      gazebo_ros_->getParameter<std::string> (topicname_odom_, 
                                                      "odom", topicname_odom_);
      gazebo_ros_->getParameter<std::string> (framename_odom_, 
                                             "odometryFrame", framename_odom_);
      gazebo_ros_->getParameter<std::string> (framename_robotBaseFrame_, 
                                  "robotBaseFrame", framename_robotBaseFrame_);
      gazebo_ros_->getParameter<bool> (broadcast_tf_, 
                                                 "broadcastTF", broadcast_tf_);
      gazebo_ros_->getParameter<double>(update_rate_,"updateRate",update_rate_); 
      update_period_ = (update_rate_ > 0.0)?(1.0/update_rate_):0.0;
      last_update_time_ = model->GetWorld()->GetSimTime();
      // ROS: get tf parameters
      tf_prefix_ = tf::getPrefixParam(*gazebo_ros_->node());
      transform_broadcaster_ = new tf::TransformBroadcaster();
      // ROS: Registering Subscribers and Publishers
      ros::SubscribeOptions sovel =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(topicname_cmdvel_,
            1, boost::bind(&MobileBasePlugin::velCallback, this, _1),
            ros::VoidPtr(), &queue_);
      cmdvel_subscriber_ = gazebo_ros_->node()->subscribe(sovel);
      ros::SubscribeOptions soflp =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(topicname_cmdflp_,
            1, boost::bind(&MobileBasePlugin::flipperCallback, this, _1),
            ros::VoidPtr(), &queue_);
      cmdflipper_subscriber_ = gazebo_ros_->node()->subscribe(soflp);
      odom_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>
                                                           (framename_odom_,1);
      // Start custom queue for hearing ROS topics
      this->callback_queue_thread_ =
        boost::thread(boost::bind(&MobileBasePlugin::QueueThread, this));
      // Automatic loop event
      this->updateConnection
        = event::Events::ConnectWorldUpdateBegin(
                  boost::bind(&MobileBasePlugin::OnUpdate, this));
    }
  }
 
  bool LoadParams(sdf::ElementPtr _sdf)
  {
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

    D_SDFGET_DOUBLE(gain, "gain", 1.0);
    D_SDFGET_STRING(robotnamespace_, "robotNamespace", "robot");
    D_SDFGET_BOOL(broadcast_tf_,"broadcastTF", false);
    D_SDFGET_DOUBLE(wheel_separation_, "wheel_separation", 1);
    D_SDFGET_DOUBLE(wheel_diameter_, "wheel_diameter", 0.4);
    D_SDFGET_DOUBLE(torque, "torque", 5.0);
    D_SDFGET_STRING(topicname_cmdvel_, "cmdvelTopic", "cmd_vel");
    D_SDFGET_STRING(topicname_cmdflp_, "cmdflipperTopic", "cmd_flipper");
    D_SDFGET_STRING(topicname_odom_, "odometryTopic", "odom");
    D_SDFGET_STRING(framename_odom_, "odometryFrame", "odom");
    D_SDFGET_STRING(framename_robotBaseFrame_, "robotBaseFrame", 
                                                             "base_footprint");
    D_SDFGET_DOUBLE(update_rate_, "updateRate", 100.0); 
    D_SDFGET_DOUBLE(covariance_x_, "covariance_x", 0.0001);
    D_SDFGET_DOUBLE(covariance_y_, "covariance_y", 0.0001);
    D_SDFGET_DOUBLE(covariance_yaw_, "covariance_yaw", 0.01);
    return true;
  }

/*
  /////////////////////////////////////////////////
  void OnVelMsg(ConstPosePtr &_msg)
  {
    // gzmsg << "cmd_vel: " << msg->position().x() << ", "
    //       <<msgs::Convert(msg->orientation()).GetAsEuler().z<<std::endl;
    double vel_lin = _msg->position().x() / this->(wheel_diameter_/2);
#if(GAZEBO_MAJOR_VERSION == 5)
    double vel_rot = -1 * msgs::Convert(_msg->orientation()).GetAsEuler().z
                     * (this->wheel_separation_ / this->(wheel_diameter_/2));
#endif
#if(GAZEBO_MAJOR_VERSION == 7)
    double vel_rot = -1 * msgs::ConvertIgn(_msg->orientation()).Euler().Z()
                     * (this->wheel_separation_ / this->(wheel_diameter_/2));
#endif
    set_velocity(vel_lin - vel_rot, vel_lin + vel_rot);
  }
*/

  void set_velocity(double  vr, double  vl)
  {
    Target_VEL_R = vr;
    Target_VEL_L = vl;
  }

/*
  /////////////////////////////////////////////////
  void OnFlpMsg(ConstFlipperControlPtr &_msg)
  {
    Target_FLP_FR = _msg->fr();
    Target_FLP_FL = _msg->fl();
    Target_FLP_RR = _msg->rr();
    Target_FLP_RL = _msg->rl();
  }
*/

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
  void Move_A_Joint(physics::JointPtr _joint, double _target_angle)
  {
    float P = _target_angle - _joint->GetAngle(0).Radian();
    P *= 100;
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
//ROS_INFO("TA:%f", _target_angle);
  }

  void MoveFlipper(void)
  {
    Move_A_Joint(hinge9, -Target_FLP_FR);
    Move_A_Joint(hinge10, Target_FLP_RR);
    Move_A_Joint(hinge11, -Target_FLP_FL);
    Move_A_Joint(hinge12, Target_FLP_RL);
  }
 
  public:
  void OnUpdate(void)
  {
    common::Time current_time = model->GetWorld()->GetSimTime();
    double seconds_since_last_update=(current_time-last_update_time_).Double();
    if(seconds_since_last_update > update_period_)
    {
      publishOdometry(seconds_since_last_update);
      MoveWheel();
      MoveFlipper();
      last_update_time_ = current_time;
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}
