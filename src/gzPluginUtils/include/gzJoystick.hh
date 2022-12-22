// The original file is in https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/ROS-Noetic_Gazebo11/src/gzPluginUtils/include

#ifndef _GAZEBO_JOYSTICK_HH_
#define _GAZEBO_JOYSTICK_HH_

#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#if(GAZEBO_MAJOR_VERSION <= 8)
#include <gazebo/math/gzmath.hh>
#endif
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/TransportTypes.hh>
// Joystick
#include <linux/joystick.h>
// My Convenience functions for Gazebo Programming
#include "gzReadXML.hh"

using namespace std;

/* Analog Axis Binding and Values.
 [BUFFALO BSGP1601 & SANWA 400-JYP62US]
   Left Analog Stick       Right Analog Stick         Hat Swtiches
         [1]                     [3]                      [5]
        -32767                 -32767                    -32767
[0]-32767  +  +32767    [2]-32767  +  +32767    [4]-32767   +   +32767
        +32767                 +32767                    +32767
*/

class joyButton
{
public:
  int data(void)
  {
    return (int)value;
  }
  int changed(void)
  {
    return (value != last_value)?1:0;
  }
  int pushed(void)
  {
    return (value != last_value)?value:0;
  }
  int released(void)
  {
    return (value != last_value)?!value:0;
  }
  void reset_change_status(void)
  {
    set_last_value(value);
  }
  void set(int _data)
  {
    set_last_value(value);
    value = (unsigned char)_data;
  }
private:
  unsigned char value, last_value;
  void set_last_value(int _data)
  {
    last_value = (unsigned char)_data;
  }
};

class joyAxis
{
public:
  int data(void)
  {
    return value;
  }
  int changed(void)
  {
    return (value != last_value)?1:0;
  }
  void reset_change_status(void)
  {
    set_last_value(value);
  }
  void set(int _data)
  {
    set_last_value(value);
    value = _data;
  }
private:
  int  value, last_value;
  void set_last_value(int _data)
  {
    last_value = _data;
  }
};

namespace gazebo
{
  class gzJoystick
  {
  public:
    vector<joyButton> button;
    vector<joyAxis>   axis;
    char              name_joy[100];
    gzJoystick(void) {;}
    ~gzJoystick() 
    {
      if(-1 != fd_joy)
        close(fd_joy);
    }
    int Init(const std::string _xmlTag_devJoy, sdf::ElementPtr _sdf)
    {
      gzXML.Init(_sdf);
      err        = 0;
      maxButtons = 0;
      maxAxises  = 0;
      fd_joy     = -1;
      err        = gzXML.GetString(dev_joy, _xmlTag_devJoy);
      if(err > 0)
      {
        gzerr << "Not found xml tag: " << _xmlTag_devJoy << "\n";
        return err;
      }
      if(0 > (fd_joy = open(dev_joy.c_str(), O_RDONLY | O_NONBLOCK)))
      {
        gzerr << "Could not open: " << dev_joy << "\n";
        return ++err;
      }
      ioctl(fd_joy, JSIOCGBUTTONS,  &maxButtons);
      ioctl(fd_joy, JSIOCGAXES,     &maxAxises);
      ioctl(fd_joy, JSIOCGNAME(sizeof(name_joy)), name_joy);
      button.resize(maxButtons);
      axis.resize(maxAxises);
      gzmsg << "Joystick: " << name_joy << "\n" 
        << "       buttons: " << maxButtons << "\n" 
        << "        axises: " << maxAxises << "\n";
      return err;
    }
    void check_joystick(void)
    {
      // See http://wlog.flatlib.jp/item/1682
      // and https://www.kernel.org/doc/Documentation/input/joystick-api.txt
      if(err) 
        return;
      while(read(fd_joy, &jse, sizeof(jse)) > 0)
      {
        switch(jse.type & ~JS_EVENT_INIT)
        {
          case JS_EVENT_BUTTON:
            if(jse.number < maxButtons)
              button[(int)jse.number].set(jse.value);
            break;
          case JS_EVENT_AXIS:
            if(jse.number < maxAxises)
              axis[(int)jse.number].set(jse.value);
            break;
        }
      }
    }
    int updated(void)
    {
      // RETURN={1: There were some updates. 0: No updates}
      int ret = 0;
      for(size_t i(0); i < button.size(); i++)
        if(button[i].changed())
        {
          ret = 1;
          for(; i < button.size(); i++)
            button[i].reset_change_status();
          break;
        }
      for(size_t i(0); i < axis.size(); i++)
        if(axis[i].changed())
        {
          ret = 1;
          for(; i < axis.size(); i++)
            axis[i].reset_change_status();
          break;
        }
      return ret;
    }
    void disp_joystick(void)
    {
      std::string buf_bttn, buf_axis;
      for(size_t i(0); i < button.size(); i++)
        buf_bttn.append(std::to_string((int)button[i].data()))
                                                       , buf_bttn.append("  ");        
      for(size_t i(0); i < axis.size(); i++)
        buf_axis.append(std::to_string((int)axis[i].data()))
                                                       , buf_axis.append("  ");
      gzmsg << "Button: " << buf_bttn << "\n        Axis: " << buf_axis << "\n";
    }
  private: 
    std::string       dev_joy;
    int               fd_joy;
    int               maxButtons;
    int               maxAxises;
    int               err;
    struct js_event   jse;
    gzReadXML         gzXML;
  };
}
#endif

#ifdef _____SAMPLE_TO_USE______

#include "gzJoystick.hh"                         // <<<<<<<<<<<<<<<<< ADD THIS
#include "gzIntervalTimer.hh"

enum gzIT_number
{
  gzIT_Joy = 0,
  gzIT_MaxTimers
};

namespace gazebo
{
  class GAZEBO_VISIBLE XLegRobot : public ModelPlugin
  {
  public: 
    XLegRobot(void);
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Init();
    void         OnUpdate();

  private:
    physics::ModelPtr        model;
    sdf::ElementPtr          sdf;
    transport::NodePtr       node;
    event::ConnectionPtr     updateConnection;
    gzIntervalTimer          gzIT[gzIT_MaxTimers];
    gzJoystick               gzJS;               // <<<<<<<<<<<<<<<<< ADD THIS
    void                     check_joystick(void);
  };
}

void XLegRobot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->sdf   = _sdf;

  this->node = transport::NodePtr(new transport::Node());
#if(GAZEBO_MAJOR_VERSION <= 7)
  this->node->Init(this->model->GetWorld()->GetName());
#endif
#if(GAZEBO_MAJOR_VERSION >= 8)
  this->node->Init(this->model->GetWorld()->Name());
#endif
      
  int err = 0;  
  err += gzJS.Init("Joy_Dev", this->sdf);        // <<<<<<<<<<<<<<<<< ADD THIS
  
  if (err > 0)
  {
    gzerr << "There are some errors in loading XML values\n";
    return;
  }

/*
    <plugin name="XLegRobotPlugin" filename="libXLegRobotPlugin.so">
      <Joy_Dev>/dev/input/js0</Joy_Dev>      <!-- <<<<<<<<<<<<<<<< ADD THIS -->
    </plugin>
*/

  // Setup interval timers using in OnUpdate function.
  gzIT[gzIT_Joy].Init(this->model);
  gzIT[gzIT_Joy].setintervalFreq(100);  // Hz

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&XLegRobot::OnUpdate, this));
}

#define Kjrx ( 0.0005/32767.0)
#define Kjry (-0.0005/32767.0)
#define Kjlx (-10.0/32767.0)
#define Kjly (-10.0/32767.0)
#define rxDeadzone (100)
#define ryDeadzone (100)
#define lxDeadzone (100)
#define lyDeadzone (100)
#define clippingDeadzone(X,D) (float)(((X)<(D)&&(X)>-(D))?0:(X))
#define lxClipped clippingDeadzone(gzJS.axis[0].data(), lxDeadzone)
#define lyClipped clippingDeadzone(gzJS.axis[1].data(), lyDeadzone)
#define rxClipped clippingDeadzone(gzJS.axis[2].data(), rxDeadzone)
#define ryClipped clippingDeadzone(gzJS.axis[3].data(), ryDeadzone)

void XLegRobot::check_joystick(void)
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
//  if(gzJS.updated())
//    gzJS.disp_joystick();
  float  joyLX, joyLY, joyRX, joyRY;
  joyLX = lxClipped * Kjlx; // X axis of the Analog Right Stick
  joyLY = lyClipped * Kjly; // Y axis of the Analog Right Stick
  joyRX = rxClipped * Kjrx; // X axis of the Analog Right Stick
  joyRY = ryClipped * Kjry; // Y axis of the Analog Right Stick
  // At pushing or releasing a button.........
  if(gzJS.button[0].changed() || gzJS.button[1].changed())
  {
    if(gzJS.button[0].changed())
    {
      if(gzJS.button[0].pushed())
        foo1(); // foo1 can be called just once at the pushing moment 
      else if(gzJS.button[0].released())
        foo2(); // foo1 can be called just once at the releasing moment
      gzJS.button[0].reset_change_status(); // flags have to be reset after all
    }
    if(gzJS.button[1].changed())
    {
      if(gzJS.button[1].pushed())
        foo3();
      else if(gzJS.button[1].released())
        foo4();
      gzJS.button[1].reset_change_status();
    }
  }
  // Using Joysticks' value
  else
    // With pushing a button and/or buttons.........
    if( gzJS.button[6].data() || gzJS.button[7].data())
    {
      if(gzJS.button[6].data())
        foo5(joyX, joyY); // foo5 can be called if the button leaving push
      if(gzJS.button[7].data())
        foo6(joyX, joyY); // foo6 can be called if the button leaving push
    }
    else
    {
      foo7(joyX, joyY); // foo7 can be called if any buttons are NOT pushing
    }
}

/////////////////////////////////////////////////
void XLegRobot::OnUpdate()
{
  if(gzIT[gzIT_Joy].overIntervalPeriod())
  {
//    gzmsg << "Call check_joystick\n";
    check_joystick();
  }
}
#endif
