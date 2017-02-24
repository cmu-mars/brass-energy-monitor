# brass-energy-monitor
Gazebo energy monitor plugin for BRASS project

# ROS interface
## Publishes
```
/energy_monitor/energy_level : Float64

  The current energy level, in mwh.

/energy_monitor/voltage : Float64

  The current simulated voltage, determined from the energy level according to the empirical data Ivan gathered. The relevant data is in v_data.cc.
```

## Command Subscriptions
```
/energy_monitor/set_charging : Bool

  True if the robot is in charging mode, False otherwise (default: False)

/energy_monitor/set_voltage : Int32

  Set the voltage to a value between [104, 166].

/nuc/utilization : Float64

  Set the NUC utilization to a value between 0.0 and 100.0.
```

We also subscribe to `/sensor/kinect/onoff : String` to monitor the Kinect's state, and `/gazebo/get_model_state : Twist` to monitor the simulated odometry.

