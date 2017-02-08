#include <gazebo/gazebo.hh>

// #include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  class EnergyMonitorPlugin : public ModelPlugin
  {
    public: EnergyMonitorPlugin() : ModelPlugin()
            {
				gzdbg << "Constructed energy_monitor." << "\n";
            }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
            {
				gzdbg << "Loaded energy_monitor." << "\n";
            }
  };
  GZ_REGISTER_MODEL_PLUGIN(EnergyMonitorPlugin)
}

