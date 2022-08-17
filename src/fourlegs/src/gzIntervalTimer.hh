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
    public: void Init(physics::ModelPtr _model)
    {
      model = _model;
#if(GAZEBO_MAJOR_VERSION <= 8)
      lastUpdateTime = model->GetWorld()->GetSimTime();
#else
      lastUpdateTime = model->GetWorld()->SimTime();
#endif
    }
    public: void setIntervalRate(double _intervalRate)
    {
      intervalRate   = _intervalRate;
      intervalPeriod = (intervalRate > 0.0)?(1.0/intervalRate):0.0;
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
    private: double       intervalRate;
    private: double       intervalPeriod;
    private: common::Time currentTime;
    private: common::Time lastUpdateTime;
  };
}
#endif
