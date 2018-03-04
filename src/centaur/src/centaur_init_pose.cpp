#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#define D_INIT_DURATION_TIME 5.0

#define D_MAKE_PUB_TRAJ(X) ros::Publisher X##_traj_controller;
#define D_MAKE_JS_TRAJ(X) trajectory_msgs::JointTrajectory X##_js;

namespace centaur_pub_pose {
//  ros::Subscriber twistInput;
  ros::Publisher cameraCommandOutput;
  ros::Publisher laser1CommandOutput;
  D_MAKE_PUB_TRAJ(flipper)
  D_MAKE_PUB_TRAJ(head)
  D_MAKE_PUB_TRAJ(left_arm)
  D_MAKE_PUB_TRAJ(left_hand)
  D_MAKE_PUB_TRAJ(right_arm)
  D_MAKE_PUB_TRAJ(right_hand)
  D_MAKE_PUB_TRAJ(torso)

//  geometry_msgs::Twist twistCommand;
  geometry_msgs::QuaternionStamped cameraCommand;
  geometry_msgs::QuaternionStamped laser1Command;
  D_MAKE_JS_TRAJ(flipper)
  D_MAKE_JS_TRAJ(head)
  D_MAKE_JS_TRAJ(left_arm)
  D_MAKE_JS_TRAJ(left_hand)
  D_MAKE_JS_TRAJ(right_arm)
  D_MAKE_JS_TRAJ(right_hand)
  D_MAKE_JS_TRAJ(torso)
};

using namespace centaur_pub_pose;

#define D_MAKE_ADV_TRAJ(X,Y) X##_traj_controller = n.advertise<trajectory_msgs::JointTrajectory>(Y, 10, false);
//#define D_MAKE_HEADER_TRAJ(X,Y) X##_js.header.stamp=ros::Time::now();X##_js.header.frame_id=Y;
#define D_MAKE_HEADER_TRAJ(X) X##_js.header.stamp=ros::Time::now();
#define D_RESIZE_TRAJ(X,Y) X##_js.joint_names.resize(Y);X##_js.points.resize(1);X##_js.points[0].positions.resize(0);X##_js.points[0].time_from_start = ros::Duration(1);
#define D_PUB_TRAJ(X) X##_traj_controller.publish(X##_js);

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle n;

//  twistInput = n.subscribe("cmd_vel", 10, twistCallback);
  cameraCommandOutput = n.advertise<geometry_msgs::QuaternionStamped>("camera/command", 10, false);
  laser1CommandOutput = n.advertise<geometry_msgs::QuaternionStamped>("laser1/command", 10, false);
  D_MAKE_ADV_TRAJ(flipper, "flipper_traj_controller/command")
  D_MAKE_ADV_TRAJ(head, "head_traj_controller/command")
  D_MAKE_ADV_TRAJ(left_arm, "left_arm_traj_controller/command")
  D_MAKE_ADV_TRAJ(left_hand, "left_hand_traj_controller/command")
  D_MAKE_ADV_TRAJ(right_arm, "right_arm_traj_controller/command")
  D_MAKE_ADV_TRAJ(right_hand, "right_hand_traj_controller/command")
  D_MAKE_ADV_TRAJ(torso, "torso_traj_controller/command")
  D_MAKE_HEADER_TRAJ(flipper)
  D_MAKE_HEADER_TRAJ(head)
  D_MAKE_HEADER_TRAJ(left_arm)
  D_MAKE_HEADER_TRAJ(left_hand)
  D_MAKE_HEADER_TRAJ(right_arm)
  D_MAKE_HEADER_TRAJ(right_hand)
  D_MAKE_HEADER_TRAJ(torso)
  D_RESIZE_TRAJ(flipper,2)
  flipper_js.joint_names[0] = "front_flipper_joint";
  flipper_js.joint_names[1] = "rear_flipper_joint";
/*  flipper_js.points[0].positions.resize(2);
  flipper_js.points[0].positions[0]=-0.75;
  flipper_js.points[0].positions[1]=1.38; */
  flipper_js.points[0].positions.push_back(-0.75);
  flipper_js.points[0].positions.push_back(1.38);
  D_RESIZE_TRAJ(head,2)
  head_js.joint_names[0] = "head_pan";
  head_js.joint_names[1] = "head_tilt";
  head_js.points[0].positions.push_back(0);
  head_js.points[0].positions.push_back(0);
  D_RESIZE_TRAJ(left_arm,7)
  left_arm_js.joint_names[0] = "l_elbow";
  left_arm_js.joint_names[1] = "l_shoulder_pitch";
  left_arm_js.joint_names[2] = "l_shoulder_roll";
  left_arm_js.joint_names[3] = "l_shoulder_yaw";
  left_arm_js.joint_names[4] = "l_wrist_roll";
  left_arm_js.joint_names[5] = "l_wrist_yaw1";
  left_arm_js.joint_names[6] = "l_wrist_yaw2";
  left_arm_js.points[0].positions.push_back(-2.0);
  left_arm_js.points[0].positions.push_back(0);
  left_arm_js.points[0].positions.push_back(0);
  left_arm_js.points[0].positions.push_back(0.0211927);
  left_arm_js.points[0].positions.push_back(1.15);
  left_arm_js.points[0].positions.push_back(-2.9);
  left_arm_js.points[0].positions.push_back(0.816801445);
  D_RESIZE_TRAJ(right_arm,7)
  right_arm_js.joint_names[0] = "r_elbow";
  right_arm_js.joint_names[1] = "r_shoulder_pitch";
  right_arm_js.joint_names[2] = "r_shoulder_roll";
  right_arm_js.joint_names[3] = "r_shoulder_yaw";
  right_arm_js.joint_names[4] = "r_wrist_roll";
  right_arm_js.joint_names[5] = "r_wrist_yaw1";
  right_arm_js.joint_names[6] = "r_wrist_yaw2";
  right_arm_js.points[0].positions.push_back(2.3);
  right_arm_js.points[0].positions.push_back(0);
  right_arm_js.points[0].positions.push_back(0);
  right_arm_js.points[0].positions.push_back(-0.02119781);
  right_arm_js.points[0].positions.push_back(0.79);
  right_arm_js.points[0].positions.push_back(-0.16);
  right_arm_js.points[0].positions.push_back(-0.816799906);
  D_RESIZE_TRAJ(left_hand,2)
  left_hand_js.joint_names[0] = "l_f0_j0";
  left_hand_js.joint_names[1] = "l_f1_j0";
  left_hand_js.points[0].positions.push_back(1);
  left_hand_js.points[0].positions.push_back(1);
  D_RESIZE_TRAJ(right_hand,2)
  right_hand_js.joint_names[0] = "r_f0_j0";
  right_hand_js.joint_names[1] = "r_f1_j0";
  right_hand_js.points[0].positions.push_back(1);
  right_hand_js.points[0].positions.push_back(1);
  D_RESIZE_TRAJ(torso,2)
  torso_js.joint_names[0] = "waist_pan";
  torso_js.joint_names[1] = "waist_tilt";
  torso_js.points[0].positions.push_back(0);
  torso_js.points[0].positions.push_back(0);

//  ros::param::param("~joint1_angle", centaur_pub_pose::joint1_angle, 0.0);
//  ros::param::param<std::string>("~joint1_name", centaur_pub_pose::joint1_name, "joint1");

  cameraCommand.header.frame_id = "pelvis";
  cameraCommand.quaternion.x = 0;
  cameraCommand.quaternion.y = 0;
  cameraCommand.quaternion.z = 0;
  cameraCommand.quaternion.w = 1;

  laser1Command.header.frame_id = "pelvis";
  laser1Command.quaternion.x = 0;
  laser1Command.quaternion.y = 0;
  laser1Command.quaternion.z = 0;
  laser1Command.quaternion.w = 1;

  ros::Rate rate(20.0);
/*  
    D_PUB_TRAJ(flipper)
    D_PUB_TRAJ(head)
    D_PUB_TRAJ(left_arm)
    D_PUB_TRAJ(right_arm)
    D_PUB_TRAJ(left_hand)
    D_PUB_TRAJ(right_hand)
    D_PUB_TRAJ(torso)
    ros::spinOnce();
    rate.sleep();  
*/  
  ros::Time start;
  start = ros::Time::now();
  while(ros::ok()) {
    //twistCommandOutput.publish(twistCommand);
//    cameraCommand.header.stamp = ros::Time::now() + ros::Duration(1);
//    laser1Command.header.stamp = ros::Time::now() + ros::Duration(1);
    cameraCommand.header.stamp = ros::Time::now();
    laser1Command.header.stamp = ros::Time::now();
    cameraCommandOutput.publish(cameraCommand);
    laser1CommandOutput.publish(laser1Command);

    D_PUB_TRAJ(flipper)
    D_PUB_TRAJ(head)
    D_PUB_TRAJ(left_arm)
    D_PUB_TRAJ(right_arm)
    D_PUB_TRAJ(left_hand)
    D_PUB_TRAJ(right_hand)
    D_PUB_TRAJ(torso)
//    ROS_INFO("Centaur init pose rmain time=%f\n", 5-(ros::Time::now() - start).toSec());
    if(0 > D_INIT_DURATION_TIME - (ros::Time::now() - start).toSec())
      break;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}


