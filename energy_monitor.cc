#include <gazebo/gazebo.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo {
  class EnergyMonitorPlugin : public ModelPlugin {
    public: 
		EnergyMonitorPlugin() : ModelPlugin() {
				gzdbg << "Constructed energy_monitor." << "\n";
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

    public: 
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
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
			  ros::SubscribeOptions::create<std_msgs::Float32>(
				  "/" + this->model->GetName() + "/energy_monitor",
				  1,
				  boost::bind(&EnergyMonitorPlugin::OnRosMsg, this, _1),
				  ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  std::thread(std::bind(&EnergyMonitorPlugin::QueueThread, this));

			gzdbg << "Loaded energy_monitor." << "\n";
		}

		// Handle an incoming message from ROS
		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
		{
			gzdbg << "received message" << _msg->data;
		}

		// ROS helper function that processes messages
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

