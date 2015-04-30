
#include <iostream>
#include <time.h>
#include "WMRA_module.h"
#include "WMRA.h"

using namespace std;
using namespace WMRA;
using namespace tthread;

tthread::thread* t;
wmra _wmra;


void running(void * aArg);

WMRA_module::WMRA_module()
{
}

bool WMRA_module::initialize()
{

	t = new thread(running,0);
	return _wmra.isInitialized();
}

bool WMRA_module::sendInputValues(std::vector<double> in)
{
	return _wmra.sendInputValues(in);
}



void running(void * aArg) {

	_wmra.initialize();

	clock_t last_time, current_time;
	last_time = clock();
	current_time = clock();
	double dt;
	int count = 0;

	vector<double> X_dot;
	for (int i = 0; i < 7; i++)
	{
		X_dot.push_back(0.0);
		_wmra.Qarm.push_back(0.0);
	}

	_wmra.sendInputValues(); // zero input

	while (true)
	{
		/****Calculate dt****/
		clock_t current_time = clock();
		dt = (current_time - last_time) / CLOCKS_PER_SEC;
		last_time = current_time;
		/********************/

		/****Data Output (cout)****/
		std::cout.flush();
		std::cout << "\rRunning at " << dt << " seconds per loop" << std::endl;
		std::cout << "Omni Input = [" << _wmra.inputDevice[0] << ", " << _wmra.inputDevice[1] << ", " << _wmra.inputDevice[2] << "]" << std::endl;

		//cout << "\rRunning... dt= " << count;
		/********************/

		_wmra.Jacobian_Ground2Endeffector();
		_wmra.weighted_pseudoinverse();
		_wmra.control_joint(_wmra.inputDevice[4], _wmra.inputDevice[5]);
		_wmra.sendInputValues(); // zero input values after they are used

		/**********Updating Devices**********/
		_wmra.ARM.updateArmPosition();
		_wmra.WHEELCHAIR.WMRA_Theta_dot2Xphi_dot();
		_wmra.phi = _wmra.phi + _wmra.WHEELCHAIR.DXphi_dot[1];
		/********************/
	}
}