#include <gazebo/gazebo.hh>

namespace gazebo
{
  class EnergyMonitorPlugin : public WorldPlugin
  {
    public: EnergyMonitorPlugin() : WorldPlugin()
            {
				gzdbg << "Constructed energy_monitor." << "\n";
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
				gzdbg << "Loaded energy_monitor." << "\n";
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(EnergyMonitorPlugin)
}

