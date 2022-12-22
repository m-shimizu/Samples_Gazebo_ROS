// The original file is in https://github.com/m-shimizu/Samples_Gazebo_ROS/tree/ROS-Noetic_Gazebo11/src/gzPluginUtils/include

#ifndef _GAZEBO_READXML_HH_
#define _GAZEBO_READXML_HH_

#include <string>
#include <vector>
// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/MessageTypes.hh>
#if(GAZEBO_MAJOR_VERSION <= 8)
#include <gazebo/math/gzmath.hh>
#endif
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/TransportTypes.hh>
// Ignition
#include <ignition/math.hh>

using namespace std;

namespace gazebo
{
  class gzReadXML
  {
  public:
    gzReadXML(void)
    {
      flag_model = flag_sdf = 0;
    }
    void Init(physics::ModelPtr _model)
    {
      this->model = _model;
      flag_model  = 1;
    }
    void Init(sdf::ElementPtr _sdf)
    {
      this->sdf   = _sdf;
      flag_sdf    = 1;
    }
    void Init(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      Init(_model);
      Init(_sdf);
    }
    void Init(sdf::ElementPtr _sdf, physics::ModelPtr _model)
    {
      Init(_model);
      Init(_sdf);
    }
    int need_sdf(void)
    {
      if(0 == flag_sdf)
      {
        gzerr << "Need sdf infomation. Add gzXML.Init(sdf) in the Load function.\n";
        return 1;
      }
      return 0;
    }
    int need_model(void)
    {
      if(0 == flag_model)
      {
        gzerr << "Need model infomation. Add gzXML.Init(model) in the Load function.\n";
        return 1;
      }
      return 0;
    }
    int GetInt(int &_num, const std::string &_xmlTag)
    {
      // Bounds checking on index
      // Check the element is existing
      if(need_sdf())
        return 1;
      if(!this->sdf->HasElement(_xmlTag))
      {
        gzerr << "No " << _xmlTag << " element.\n";
        return 1;
      }
      // Find the specified joint and add it to out list
      _num = this->sdf->Get<int>(_xmlTag);
//      _num = this->sdf->GetElement(_xmlTag)->Get<int>();
      // Success!
      return 0;
    }
    int GetFloat(float &_num, const std::string &_xmlTag)
    {
      // Bounds checking on index
      // Check the element is existing
      if(need_sdf())
        return 1;
      if(!this->sdf->HasElement(_xmlTag))
      {
        gzerr << "No " << _xmlTag << " element.\n";
        return 1;
      }
      // Read number from XML
      _num = this->sdf->Get<float>(_xmlTag);
//      _num = this->sdf->GetElement(_xmlTag)->Get<float>();
      // Success!
      return 0;
    }
    int GetDouble(double &_num, const std::string &_xmlTag)
    {
      // Bounds checking on index
      // Check the element is existing
      if(need_sdf())
        return 1;
      if(!this->sdf->HasElement(_xmlTag))
      {
        gzerr << "No " << _xmlTag << " element.\n";
        return 1;
      }
      // Read number from XML
      _num = this->sdf->Get<double>(_xmlTag);
//      _num = this->sdf->GetElement(_xmlTag)->Get<double>();
      // Success!
      return 0;
    }
    int Get3Floats(float &_f1, float &_f2, float &_f3
                                                  , const std::string &_xmlTag)
    {
      int         err = 0;
      std::string buf;
      // Bounds checking on index
      // Check the element is existing
      if(need_sdf())
        return 1;
      if((err = GetString(buf, _xmlTag)))
        return err;
      if(3 != sscanf(buf.c_str(), "%f %f %f", &_f1, &_f2, &_f3))
        return ++err;
      // Success!
      return 0;
    }
    int Get3Floats(ignition::math::Vector3<float> &_v3
                                                  , const std::string &_xmlTag)
    {
      int err;
      float x, y, z;
      if((err = Get3Floats(x, y, z, _xmlTag)))
        return err;
      _v3.X(x);
      _v3.Y(y);
      _v3.Z(z);
      return 0;
    }
    int GetSizeF(ignition::math::Vector3<float> &_pose
                                                  , const std::string &_xmlTag)
    { return Get3Floats(_pose, _xmlTag); }
    int GetAxisF(ignition::math::Vector3<float> &_axis
                                                  , const std::string &_xmlTag)
    { return Get3Floats(_axis, _xmlTag); }
    int Get6Floats(float &_f1, float &_f2, float &_f3
                 , float &_f4, float &_f5, float &_f6
                 , const std::string &_xmlTag)
    {
      int         err = 0;
      std::string buf;
      // Bounds checking on index
      // Check the element is existing
      if(need_sdf())
        return 1;
      if((err = GetString(buf, _xmlTag)))
        return err;
      if(6 != sscanf(buf.c_str(), "%f %f %f %f %f %f"
                                         , &_f1, &_f2, &_f3, &_f4, &_f5, &_f6))
        return ++err;
      // Success!
      return 0;
    }
    int Get6Floats(ignition::math::Vector3<float> &_v3_1
                 , ignition::math::Vector3<float> &_v3_2
                 , const std::string &_xmlTag)
    {
      int   err;
      float x1, y1, z1, x2, y2, z2;
      if((err = Get6Floats(x1, y1, z1, x2, y2, z2, _xmlTag)))
        return err;
      _v3_1.X(x1);
      _v3_1.Y(y1);
      _v3_1.Z(z1);
      _v3_2.X(x2);
      _v3_2.Y(y2);
      _v3_2.Z(z2);
      return 0;
    }
    int GetPoseF(ignition::math::Vector3<float> &_linear
                 , ignition::math::Vector3<float> &_angular
                 , const std::string &_xmlTag)
    { return Get6Floats(_linear, _angular, _xmlTag); }
    int GetString(std::string &_str, const std::string &_xmlTag)
    {
      std::string buf;
      // Bounds checking on index
      // Check the element is existing
      if(need_sdf())
        return 1;
      if(!this->sdf->HasElement(_xmlTag))
      {
        gzerr << "No " << _xmlTag << " element.\n";
        return 1;
      }
      // Find the specified joint and add it to out list
      buf = this->sdf->Get<std::string>(_xmlTag);
//      buf = this->sdf->GetElement(_xmlTag)->Get<std::string>();
      if(buf.empty())
      {
        gzerr << "Unable to load the " << _xmlTag << ".\n";
        return 1;
      }
      _str = buf;
      // Success!
      return 0;
    }
    int RegisterJoint(physics::JointPtr &_joint, const std::string &_xmlTag)
    {
      int               err = 0;
      std::string       nameJoint;
      physics::JointPtr buf;
      if(need_sdf() || need_model())
        return 1;
      if((err = this->GetString(nameJoint, _xmlTag)))
        return err;
      // Find the specified joint and add it to out list
/*
      buf = this->model->GetJoint(
                           this->sdf->GetElement(_xmlTag)->Get<std::string>());
*/
      if(!(buf = this->model->GetJoint(nameJoint)))
      {
        gzerr << "Unable to find the joint:" << nameJoint
              <<  " in model:" << this->model->GetName() << ".\n";
        return 1;
      }
      _joint = buf;
      // Success!
      return 0;
    }
  private:
    physics::ModelPtr model;
    sdf::ElementPtr   sdf;
    int               flag_model, flag_sdf;
  };
}
#endif

#ifdef _____SAMPLE_TO_USE______

#include "gzReadXML.hh"                          // <<<<<<<<<<<<<<<<< ADD THIS
 
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
    physics::ModelPtr              model;
    sdf::ElementPtr                sdf;
    transport::NodePtr             node;
    physics::JointPtr              joints[2];
    std::string                    name;
    int                            the_number_of_legs;
    float                          _lengthF;
    double                         _lengthD;
    ignition::math::Vector3<float> _sizeF, _linearF, _angularF;
    gzReadXML                      gzXML;         // <<<<<<<<<<<<<<<<< ADD THIS
  };
}
void XLegRobot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->node  = transport::NodePtr(new transport::Node());
#if(GAZEBO_MAJOR_VERSION <= 7)
  this->node->Init(this->model->GetWorld()->GetName());
#endif
#if(GAZEBO_MAJOR_VERSION >= 8)
  this->node->Init(this->model->GetWorld()->Name());
#endif

  gzXML.Init(this->model, this->sdf);            // <<<<<<<<<<<<<<<<< ADD THIS

  int err = 0;
  //                     <<<<<<<<<<<<<<<<< ADD FOLLOWING LINES INCLUDING gzXML
  err += gzXML.RegisterJoint(this->joints[0], "Right_Front_Wheel");
  err += gzXML.RegisterJoint(this->joints[1], "Right_Front_Elbow");
  err += gzXML.GetString(this->name, "Robot_Name");
  err += gzXML.GetInt(this->the_number_of_legs, "The_Number_Of_Legs");
  err += gzXML.GetFloat(this->_lengthF, "Length_float");
  err += gzXML.GetDouble(this->_lengthD, "Length_double");
  err += gzXML.GetSizeF(this->_sizeF, "SIZE_XYZ");
  err += gzXML.GetPoseF(this->_linearF, thies->_angularF, "POSE_XYZRPY");

  if (err > 0)
  {
    gzerr << "There are some errors in loading XML values\n";
    return;
  }
}
#endif
