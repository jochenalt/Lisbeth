#include <cstdlib>
#include <iostream>
#include "Utils.hpp"
#include "Params.hpp"

#include "KeyboardListener.hpp"
#include "Controller.hpp"


using namespace std;
int main(int argc, char** argv) {
	cout << "Lisbeth walking controller";

	// setup keyboard listener
	KeyboardListener& kb = KeyboardListener::getInstance();
	kb.setup();

	// read configuration parameters from config.yaml, WALK_PARAMETERS_YAML is defined in CMakeLists.txt
	Params params;
	params.initialize(WALK_PARAMETERS_YAML);

	// initialise walking controller
	Controller& ctrl = Controller::getInstance();
	ctrl.initialize(params);

	// main loop
	bool stop_main_loop = false;
	int gaitCode = 0;

	// speed and orientation coming from remote
	double vX = 0;
	double vY = 0;
	double heightZ= 0;
	double rotX = 0;
	double rotY = 0;
	double angSpeedZ = 0;

	uint64_t frequency = 1000;
	uint64_t last_call_us = get_micros();

	while (!stop_main_loop) {

		// wait until next loop
		uint64_t now_us = get_micros();
		delay_us((int)((last_call_us + 1000000/frequency) - now_us));
		last_call_us = now_us;

		// check input from the keyboard
		// and set speed and gait
		if (kb.isKeyAvailable()) {
			int key = kb.getKey();
			if (kb.isSpecialKey(key)) {
				switch (key) {
					case KeyboardListener::ARROW_UP:
						vX += 0.1;
						break;
					case KeyboardListener::ARROW_DOWN:
						vX -= 0.1;
						break;
					default:
						cout << "unknown special key " << key << endl;
				}
				ctrl.command_speed(vX, vY, heightZ, rotX, rotY, angSpeedZ);
			} else {
				char ch = (char)(key & 0xFF);
				if ((ch >= '1') && (ch <= '7')) {
					gaitCode = ch - '0';
					ctrl.command_gait(gaitCode);
				} else {
					cout << "unexpected input " << ch << endl;
				}
			}
		}

		Vector3 imuLinearAcceleration;
		Vector3 imuGyroscopse;
		Vector3 imuAttitudeEuler;
		Vector4 imuAttitudeQuat;

		Vector12 jointsPositions;
		Vector12 jointsVelocities;

		// run controller loop
		ctrl.compute(imuLinearAcceleration, imuGyroscopse,
						 imuAttitudeEuler, imuAttitudeQuat,
						 jointsPositions, jointsVelocities);

	   params.inc_k();
	}

	KeyboardListener::getInstance().teardown();
	return EXIT_SUCCESS;
}
