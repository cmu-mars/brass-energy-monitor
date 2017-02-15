#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <cmath>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

namespace gazebo {
  class EnergyMonitorPlugin : public ModelPlugin {
    public: 
		EnergyMonitorPlugin() : ModelPlugin() {
				gzdbg << "Constructed energy_monitor." << "\n";
            }

		~EnergyMonitorPlugin() {
			this->rosNode->shutdown();
		}
	
	private:
		// A node use for ROS transport
		std::unique_ptr<ros::NodeHandle> rosNode;

		// A ROS subscriber
		ros::Subscriber rosSub;

		// A ROS callbackqueue that helps process messages
		ros::CallbackQueue rosQueue;

		// A thread the keeps the running rosQueue
		std::thread rosQueueThread;

		// Charge level
		double battery_capacity = 10000.0 /* mwhr */;
		bool charging;
		double discharge_rate = -100.0 /* mwhr / sec */;
		double charge_rate = 50.0 /* mwhr / sec */;
		double cur_charge /* mwhr */;

		// Time management
		double last_time /* s */;

		// gazebo stuff
		physics::WorldPtr world;
		event::ConnectionPtr updateConnection;

    public: 
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
			this->world = _model->GetWorld();

			last_time = this->world->GetSimTime().Double(); 
			gzdbg << "Initial time: " << last_time << "\n";

			cur_charge = battery_capacity;
			charging = false;

			// Register callback for every simulation tick
			this->updateConnection = event::Events::ConnectWorldUpdateBegin( 
					boost::bind(&EnergyMonitorPlugin::UpdateChild, this)); 

			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "energy_monitor_client",
				  ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("energy_monitor_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
			  ros::SubscribeOptions::create<std_msgs::Bool>(
				  "/energy_monitor/set_charging",
				  1,
				  boost::bind(&EnergyMonitorPlugin::OnSetChargingMsg, this, _1),
				  ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  std::thread(std::bind(&EnergyMonitorPlugin::QueueThread, this));

			gzdbg << "Loaded energy_monitor." << "\n";
		}

		void UpdateChild() {
			// Update time
			double curr_time = this->world->GetSimTime().Double(); // measured in seconds
			double dt = curr_time - last_time; 
			last_time = curr_time;
		    
			if (charging) {
				cur_charge += charge_rate * dt;
			} else {
				cur_charge += discharge_rate * dt;
			}

			if (cur_charge < 0.0) {
				cur_charge = 0.0;
			}
			
			if (fmod(cur_charge, 50.0) < 1.0) { 
				gzdbg << "current charge: " << cur_charge << "\n";
			}
		}

		// Handle an incoming message from ROS
		public: void OnSetChargingMsg(const std_msgs::BoolConstPtr &_msg)
		{
			charging = _msg->data;
			gzdbg << "received message" << charging << "\n";
		}

		/// ROS helper function that processes messages
		private: void QueueThread()
		{
		  static const double timeout = 0.01;
		  while (this->rosNode->ok())
		  {
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		  }
		}
  };
  GZ_REGISTER_MODEL_PLUGIN(EnergyMonitorPlugin)
}

