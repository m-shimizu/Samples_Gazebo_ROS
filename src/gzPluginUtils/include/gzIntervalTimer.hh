// The original file is in https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/ROS-Noetic_Gazebo11/src/gzPluginUtils/include

#ifndef _GAZEBO_INTERVAL_TIMER_HH_
#define _GAZEBO_INTERVAL_TIMER_HH_

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

namespace gazebo
{
  class gzIntervalTimer
  {
    public: gzIntervalTimer(void) {;}
    public: void Init(physics::ModelPtr _model, double _intervalFreq = 30)
    {
      model = _model;
#if(GAZEBO_MAJOR_VERSION <= 8)
      lastUpdateTime = model->GetWorld()->GetSimTime();
#else
      lastUpdateTime = model->GetWorld()->SimTime();
#endif
      setintervalFreq(_intervalFreq);
    }
    public: void setintervalFreq(double _intervalFreq)
    {
      intervalFreq   = _intervalFreq;
      intervalPeriod = (intervalFreq > 0.0)?(1.0/intervalFreq):0.0;
    }
    public: void setIntervalPeriod(double _intervalPeriod)
    {
      intervalPeriod = _intervalPeriod; // Seconds
    }
    public: int overIntervalPeriod(void) 
    {
#if(GAZEBO_MAJOR_VERSION <= 8)
      currentTime = model->GetWorld()->GetSimTime();
#else
      currentTime = model->GetWorld()->SimTime();
#endif
      double secondsSinceLastUpdate = (currentTime-lastUpdateTime).Double();
      if(secondsSinceLastUpdate > intervalPeriod)
      {
        setLastUpdateTime();
//        gzmsg << ">>> " << this->model->GetName() << ": " << currentTime << "\n";
        return 1;
      }
      else
        return 0;
    }
    public: void setLastUpdateTime(void) 
    {
      lastUpdateTime = currentTime;
    }
    private: physics::ModelPtr model;
    private: double       intervalFreq;
    private: double       intervalPeriod;
    private: common::Time currentTime;
    private: common::Time lastUpdateTime;
  };
}
#endif

#ifdef _____SAMPLE_TO_USE______

#include "gzIntervalTimer.hh"                    // <<<<<<<<<<<<<<<<< ADD THIS

enum gzIT_number                                 // <<<<<<<<<<<<<<<<< ADD THIS
{
  gzIT_TIMER1 = 0,
  gzIT_TIMER2,
  gzIT_MaxTimers
};

namespace gazebo
{
  class GAZEBO_VISIBLE XLegRobot : public ModelPlugin
  {
  public: 
    QuadrupedMotion();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Init();
    void         OnUpdate();
  private:
    physics::ModelPtr    model;
    sdf::ElementPtr      sdf;
    transport::NodePtr   node;
    event::ConnectionPtr updateConnection;
    gzIntervalTimer      gzIT[gzIT_MaxTimers];         // <<<<<<<<<<< ADD THIS
  };
}

void XLegRobot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->node = transport::NodePtr(new transport::Node());
#if(GAZEBO_MAJOR_VERSION <= 7)
  this->node->Init(this->model->GetWorld()->GetName());
#endif
#if(GAZEBO_MAJOR_VERSION >= 8)
  this->node->Init(this->model->GetWorld()->Name());
#endif

  // Settings for interval timers in the OnUpdate function.
  gzIT[gzIT_TIMER1].Init(this->model);           // <<<<<<<<<<<<<<<<< ADD THIS
  gzIT[gzIT_TIMER1].setintervalFreq(100);  // Hz // <<<<<<<<<<<<<<<<< ADD THIS
  gzIT[gzIT_TIMER2].Init(this->model);           // <<<<<<<<<<<<<<<<< ADD THIS
  gzIT[gzIT_TIMER2].setintervalFreq(500);  // Hz // <<<<<<<<<<<<<<<<< ADD THIS

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&QuadrupedMotion::OnUpdate, this));
}

void XLegRobot::OnUpdate()
{
  if(gzIT[gzIT_TIMER1].overIntervalPeriod())     // <<<<<<<<<<<<<<<<< ADD THIS
  {
    foo1();  // foo1 can be called in 100Hz
  }
  if(gzIT[gzIT_TIMER1].overIntervalPeriod())     // <<<<<<<<<<<<<<<<< ADD THIS
  {
    foo2();  // foo2 can be called in 500Hz
  }
}
#endif
