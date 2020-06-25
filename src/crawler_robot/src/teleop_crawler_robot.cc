/*
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
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

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#if(GAZEBO_MAJOR_VERSION <= 8)
#include <gazebo/math/gzmath.hh>
#endif
#include "flipper_control_msgs.hh"

#include <termios.h>
#include <iostream>

#ifndef numof
#define numof(X) (sizeof(X)/sizeof(typeof(X[0])))
#endif

#ifndef _MAX
#define _MAX(X,Y) (((X)>(Y))?(X):(Y))
#endif

#ifndef _MIN
#define _MIN(X,Y) (((X)<(Y))?(X):(Y))
#endif

enum DRIVE_TYPE
{
  RT_DiffarentialDrive = 1,
  RT_SkidsteerDrive    = 2
};

//char  DefaultRobotName[][20] = {"Default", "pioneer2dx", "pioneer3at"};

void  publish_vel_cmd(gazebo::transport::PublisherPtr pub, 
                  float speed = 0, float turn = 0, int RobotType = 0)
{
  switch(RobotType)
  {
    case 1: //turn = turn;
      // Pioneer 2DX(Skidsteer); Turn > 0 then robot turn clock wise
      break;
    case 2: turn = -turn;
      // Pioneer 3AT(Diffarential); Turn < 0 then robot turn clock wise
      break;
    default: speed = turn = 0;
      break;
  }
#if(GAZEBO_MAJOR_VERSION == 5)
  gazebo::math::Pose pose(speed, 0, 0, 0, 0, turn);
#endif
#if(GAZEBO_MAJOR_VERSION >= 7)
  ignition::math::Pose3d pose(speed, 0, 0, 0, 0, turn);
#endif
  gazebo::msgs::Pose msg;
  gazebo::msgs::Set(&msg, pose);
  pub->Publish(msg);
}

void  publish_flp_cmd(gazebo::transport::PublisherPtr pub,
             double fr = 0, double fl = 0, double rr = 0, double rl = 0)
{
  flipper_control_msgs::msgs::FlipperControl flpmsg;
  fr = _MIN(fr, M_PI * 2);
  fr = _MAX(fr, M_PI * -2);
  fl = _MIN(fl, M_PI * 2);
  fl = _MAX(fl, M_PI * -2);
  rr = _MIN(rr, M_PI * 2);
  rr = _MAX(rr, M_PI * -2);
  rl = _MIN(rl, M_PI * 2);
  rl = _MAX(rl, M_PI * -2);
  flpmsg.set_fr(fr);
  flpmsg.set_fl(fl);
  flpmsg.set_rr(rr);
  flpmsg.set_rl(rl);
  pub->Publish(flpmsg);
}

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

void  disp_usage_control_and_status(float speed, float turn)
{
  static int  loop_counter = 0;
  if(0 == (loop_counter++ % 10))
  {
    printf("\n");
    printf("\n");
    printf("[q]:Flipper FL [w]:speed +    [e]:Flipper FR  [r]:Flipper up\n");
    printf("[a]:left turn  [s]:stop       [d]:right turn  [f]:Flipper home\n");
    printf("[z]:Flipper RL [x]:speed -    [c]:Flipper RR  [v]:Flipper down\n");
    printf("To move flippers : At 1st push [qezc] and push [rfv].\n");
		printf("\n");
    printf("With a game controller, you can use the right analog stick to move the robot\n");
		printf("Current programmed game controllers are:\n  BUFFALO BSGP1601\n");
  }
  printf("  Speed : %8.3f , Turn : %8.3f\r", speed, turn);
}

#include <linux/joystick.h>
#define BUTTON_DATA_MAX 20
#define STICK_DATA_MAX 4

#define Kjx	( 2.0/30000.0)
#define Kjy (-2.0/30000.0)

void  check_joystick(gazebo::transport::PublisherPtr pub, int RobotType
                     , float& speed, float& turn)
{
  // See http://wlog.flatlib.jp/item/1682
  // and https://www.kernel.org/doc/Documentation/input/joystick-api.txt
//  unsigned char ButtonData[BUTTON_DATA_MAX];
//  signed int    StickData[STICK_DATA_MAX];
  struct js_event jse;
  int fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
  if(0 > fd)
  {
//    fprintf(stderr, "Joystick open error\n");
    return;
  }
  while(read(fd, &jse, sizeof(jse)) > 0)
  {
//  printf("%02X : %5d, %5d\n", jse.type, jse.number, jse.value);
    switch(jse.type & 0x7f)
    {
      case JS_EVENT_BUTTON:
        if(jse.number < BUTTON_DATA_MAX)
        {
//          printf("BUTTON[%d]:%-5d\n", jse.number, jse.value);
  //        ButtonData[jse.number] = jse.value;
        }
        break;
      case JS_EVENT_AXIS:
        if(jse.number < STICK_DATA_MAX)
        {
          switch(jse.number)
          {
            case 3: speed = jse.value * Kjy; break;
            case 2: turn  = jse.value * Kjx; break;
          }
  /* [BUFFALO BSGP1601]
     Left Analog Stick       Right Analog Stick         Hat Swtiches
           [1]                     [3]                      [5]
          -32767                 -32767                    -32767
 [0]-32767  +  +32767    [2]-32767  +  +32767    [4]-32767   +   +32767
          +32767                 +32767                    +32767
  */
//          printf("STICK[%d]:%-5d\n", jse.number, jse.value);
  //      StickData[jse.number] = jse.value;
        }
        break;
    }
  }
  close(fd);
  if(speed < 0)
    turn *= -1.0; // For natural backward behavior 
  publish_vel_cmd(pub, speed, turn, RobotType);
  disp_usage_control_and_status(speed, turn);
}

void  check_key_command(gazebo::transport::PublisherPtr velpub,
                        gazebo::transport::PublisherPtr flppub,
                        int RobotType , float& speed, float& turn,
                        int& flp_ch, double flp[])
{
  if(doslike_kbhit())
  {
    int cmd = doslike_getch();
    switch(cmd)
    {
      case 's': turn  = 0;
      case 'S': speed = 0; break;
      case 'W': turn   = 0;
      case 'w': speed += 0.2; break;
      case 'X': turn   = 0;
      case 'x': speed -= 0.2; break;
      case 'A': speed = 0;
      case 'a': turn -= 0.2; turn = _MAX(turn, -3.14); break;
      case 'D': speed = 0;
      case 'd': turn += 0.2; turn = _MIN(turn, 3.14); break;
      case 'e': flp_ch = 0; break; // FR
      case 'q': flp_ch = 1; break; // FL
      case 'c': flp_ch = 2; break; // RR
      case 'z': flp_ch = 3; break; // RL
      case 'F': flp[0] = flp[1] = flp[2] = flp[3] = M_PI / 4; break;
      case 'f': flp[flp_ch] = M_PI / 4; break;
      case 'r': flp[flp_ch] += (M_PI / 180 * 5); break;
      case 'R': flp[flp_ch] += (M_PI / 180 * 10); break;
      case 'v': flp[flp_ch] -= (M_PI / 180 * 5); break;
      case 'V': flp[flp_ch] -= (M_PI / 180 * 10); break;
    }
    publish_vel_cmd(velpub, speed, turn, RobotType);
    publish_flp_cmd(flppub, flp[0], flp[1], flp[2], flp[3]);
    disp_usage_control_and_status(speed, turn);
  }
}

void  disp_usage_option(char* arg)
{
  printf("%s robotname robottype{1:diff, 2:skid} [worldname]\n", arg);
  printf("Example1: %s pioneer2dx 1\n", arg);
  printf("Example2: %s pioneer3at 2\n", arg);
  printf("\n   [worldname] is an optional argument and it is defined in *.world file.\n");
  printf("Example3: %s pioneer3at 2 test_world\n", arg);
}

#define INIT_FLIPPER_ANGLE (M_PI/4)
int main(int argc, char* argv[])
{
  char  worldname[50];
	float speed = 0, turn = 0;
  int   flipper_number = 0;
  double flipper_angles[4] ={INIT_FLIPPER_ANGLE, INIT_FLIPPER_ANGLE, 
                             INIT_FLIPPER_ANGLE, INIT_FLIPPER_ANGLE};
  if(3 != argc && 4 != argc)
  {
    disp_usage_option(argv[0]);
    return -1;
  }
  else
  {
    if(3 == argc)
      strcpy(worldname, "default");
    else if(4 == argc)
      strcpy(worldname, argv[3]);
    printf("Robot name = %s\nRobot Type = %d\nWorld name = %s\n", 
                                      argv[1], atoi(argv[2]), worldname);
  }
  int  RobotType = atoi(argv[2]);
  char VelTopicName[100];
  char FlpTopicName[100];
  gazebo::transport::init();
  gazebo::transport::run();
  gazebo::transport::NodePtr  node(new gazebo::transport::Node());
  node->Init(worldname);
  // Create velpub
  sprintf(VelTopicName, "~/%s/vel_cmd", argv[1]);
  gazebo::transport::PublisherPtr velpub 
                     = node->Advertise<gazebo::msgs::Pose>(VelTopicName);
  // Create flppub
  sprintf(FlpTopicName, "~/%s/flp_cmd", argv[1]);
  gazebo::transport::PublisherPtr flppub 
          = node->Advertise<flipper_control_msgs::msgs::FlipperControl>
                                                          (FlpTopicName);
  // Wait for response of the target robot
  velpub->WaitForConnection();
  disp_usage_control_and_status(0, 0);
  for(;1;)
  {
    gazebo::common::Time::MSleep(100);
    check_key_command(velpub, flppub, RobotType, speed, turn, 
                           flipper_number, flipper_angles);
    check_joystick(velpub, RobotType, speed, turn);
  }
  return 0;
}
