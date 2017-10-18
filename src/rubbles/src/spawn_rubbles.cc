#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#if(GAZEBO_MAJOR_VERSION == 7)
#include "ignition/math/Pose3.hh"
#include "ignition/math/Vector3.hh"
#include "ignition/math/Quaternion.hh"
#endif

#define MAX_MODELS (4*6*6)

namespace gazebo
{
class Factory : public WorldPlugin
{
  sdf::SDF modelSDF[MAX_MODELS];
  public: void Spawn_Box(physics::WorldPtr _parent, char* modelname, int modelnumber
                                              , float mass, float sx, float sy, float sz, float x, float y, float z
                                              , float roll=0, float pitch=0, float yaw=0)
  {
    char modelfullname[100];
    sprintf(modelfullname, "%s%d", modelname, modelnumber);
    char stringcmd[1024]; 
    sprintf(stringcmd,  
       "<sdf version ='1.5'>\
          <model name ='%s'>\
            <pose>%f %f %f %f %f %f</pose>\
            <link name ='link'>\
              <inertial>\
                <mass>%f</mass>\
              </inertial>\
              <collision name ='collision'>\
                <geometry>\
                  <box><size>%f %f %f</size></box>\
                </geometry>\
                <surface>\
                  <friction>\
                    <ode>\
                      <mu>0.7</mu>\
                      <mu2>0.7</mu2>\
                      <slip1>0</slip1>\
                      <slip2>0</slip2>\
                    </ode>\
                  </friction>\
                </surface>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <box><size>%f %f %f</size></box>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>", modelfullname, x, y, z, roll, pitch, yaw, mass, sx, sy, sz, sx, sy, sz);
//    sdf::SDF modelSDF;
    modelSDF[modelnumber].SetFromString(stringcmd);
    sdf::ElementPtr model = modelSDF[modelnumber].root->GetElement("model");
    model->GetAttribute("name")->SetFromString(modelfullname);
    _parent->InsertModelSDF(modelSDF[modelnumber]);
  }

  public: void Option4(physics::WorldPtr _parent)
  {
    int modelcount=0;
    float dx, dy, dz, cx = 0, cy = 0;
    for(dz = 0; dz < 4; dz += 1)
      for(dx = 3; dx > -3; dx -= 1)
        for(dy = 3; dy > -3; dy -= 1)
        {
          Spawn_Box(_parent, "box", modelcount, 1.0, .03, .3, .7, cx+dx, cy+dy, 1.5+dz*1.2
                                                                       , modelcount/2, modelcount/3, modelcount/5);
          modelcount++;
        }
  }

/*
  public: void Option4(physics::WorldPtr _parent)
  {
    char modelname[100];
    int modelcount=0;
    float dx, dy, dz, cx = -10, cy = 0;
    transport::NodePtr node(new transport::Node());
    node->Init(_parent->GetName());
    transport::PublisherPtr factoryPub =
      node->Advertise<msgs::Factory>("~/factory");
    msgs::Factory msg;
    msg.set_sdf_filename("model://box");
    for(dz = 0; dz < 3; dz += 1)
      for(dx = 1; dx > 0; dx -= 1)
        for(dy = 1; dy > 0; dy -= 1)
        {
          sprintf(modelname,"box%d", modelcount++);
          msg.set_edit_name(modelname);
          msg.set_clone_model_name(modelname);
          msgs::Set(msg.mutable_pose(),
            math::Pose(math::Vector3(cx + dx, cy + dy, 0.5 + dz)
                     , math::Quaternion(0, 0, 0)));
          factoryPub->Publish(msg);
        }
  }
*/

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
#if(__OPTION1__)
    // Option 1: Insert model from file via function call.
    // The filename must be in the GAZEBO_MODEL_PATH environment variable.
    _parent->InsertModelFile("model://box");
#endif

#if(__OPTION2__)
    // Option 2: Insert model from string via function call.
    // Insert a sphere model from string
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(
       "<sdf version ='1.5'>\
          <model name ='sphere1'>\
            <pose>1 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 .5 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
    // Demonstrate using a custom model name.
    sdf::ElementPtr model = sphereSDF.root->GetElement("model");
    model->GetAttribute("name")->SetFromString("unique_sphere");
    _parent->InsertModelSDF(sphereSDF);
#endif

#if(__OPTION3__)
    // Option 3: Insert model from file via message passing.
    {
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->GetName());

      // Create a publisher on the ~/factory topic
      transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

      // Create the message
      msgs::Factory msg;

      // Model file to load
    //  msg.set_sdf_filename("model://cylinder");
    // msg.set_sdf_filename("model://pr2");

      // Pose to initialize the model to
#if(GAZEBO_MAJOR_VERSION == 5)
      msgs::Set(msg.mutable_pose(),
      math::Pose(math::Vector3(1, -2, 0), math::Quaternion(0, 0, 0)));
#endif
#if(GAZEBO_MAJOR_VERSION == 7)
      msgs::Set(msg.mutable_pose(),ignition::math::Pose3d(ignition::math::Vector3d(1,-2,0),ignition::math::Quaterniond(0,0,0)));
#endif

      // Send the message
      factoryPub->Publish(msg);
    }
#endif
    Option4(_parent);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
