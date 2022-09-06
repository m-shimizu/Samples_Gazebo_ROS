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
  
enum gzIT_number
{
  gzIT_Motion = 0,
  gzIT_Motor,
  gzIT_MaxTimers
};

namespace gazebo
{
  class GAZEBO_VISIBLE QuadrupedMotion : public ModelPlugin
  {
    public: QuadrupedMotion();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();
    private: void OnUpdate();
    // For controling OnUpdate Speed
    private: gzIntervalTimer gzIT[gzIT_MaxTimers];
  };
}

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

  // Settings for interval timers in the OnUpdate function.
  gzIT[gzIT_Motion].Init(this->model);
  gzIT[gzIT_Motion].setintervalFreq(100);  // Hz
  gzIT[gzIT_Motor].Init(this->model);
  gzIT[gzIT_Motor].setintervalFreq(500);  // Hz

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&QuadrupedMotion::OnUpdate, this));
}

void QuadrupedMotion::OnUpdate()
{
  if(gzIT[gzIT_Motion].overIntervalPeriod())
  {
//    check_key_command(&mdblp);
    MotionPlayer(mdblp);
  }
  if(gzIT[gzIT_Motor].overIntervalPeriod())
  {
    for(int _motor = 0; _motor < G_JOINTS; _motor++)
      Move_A_Joint(_motor);
//    this->model->GetJointController()->Update();
  }
}
#endif
