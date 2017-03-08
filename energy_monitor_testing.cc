//#include <gazebo/gazebo.hh>
//#include <gazebo/physics/physics.hh>
//#include <gazebo/common/common.hh>

#include <boost/thread/mutex.hpp>

#include <cmath>
#include <iostream>
#include <thread>
using namespace std;

#include "v_data.cc"

#undef ENERGY_MONITOR_DEBUG
#define ENERGY_LEVEL_DBG_INTERVAL 5.0


const double battery_capacity /* mwh */ = 32560.0;

// used to send voltage sensor values to the robot (a main use case)
int voltage_of_charge(double charge) {
	double pct = charge / battery_capacity;
	double idx_dbl = pct * (NUM_V_DATA - 1); // -1 is important here (mapping from 0 to num-1), otherwise the index goes out of range for pct = 0 
	int idx_int = round(idx_dbl);
	cout << "index is " << idx_int << " fetching " << NUM_V_DATA - 1 - idx_int << "\n";
	// The highest voltage is idx 0 in v_data, so need 
	// reverse this when calculating
	int idx_int_r = NUM_V_DATA -1 - idx_int; 
	if (idx_int_r < 0 || idx_int_r > NUM_V_DATA-1)
		cout << "Error: voltage index " << idx_int_r << " out of bounds [0, " << NUM_V_DATA - 1 << "]\n"; 

	return gazebo::v_data[idx_int_r];
}

// used to interpret the messed-up MIT-LL input (a crutch)
// from voltage (what it shouldn't be)
// to charge (what it should be)
double charge_of_voltage(int voltage) {
	double pct = gazebo::percent_of_v[voltage - MIN_VOLTAGE];
	return pct * battery_capacity;
}

double charge_of_voltage_opt(int voltage) {
	double pct = gazebo::percent_of_v_opt[voltage - MIN_VOLTAGE];
	return pct * battery_capacity;
}

double charge_of_voltage_pess(int voltage) {
	double pct = gazebo::percent_of_v_pess[voltage - MIN_VOLTAGE];
	return pct * battery_capacity;
}


int main(int argc, char* argv[]){ 
	//cout << "voltage of " << argv[1] << " is " << voltage_of_charge(atof(argv[1])) << "\n";
	cout << "charge of " << argv[1] << " is " << charge_of_voltage(atoi(argv[1])) << "\n";
	//cout << "charge of " << argv[1] << " is " << charge_of_voltage_opt(atoi(argv[1])) << "\n";
	//cout << "charge of " << argv[1] << " is " << charge_of_voltage_pess(atoi(argv[1])) << "\n";
	return 0;
}

