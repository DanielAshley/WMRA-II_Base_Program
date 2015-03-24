#ifndef DMC_4143_CONTROLLER_H
#define DMC_4143_CONTROLLER_H

#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <exception>
#include <vector>
#include <ctype.h>
#include "matrix.h" 
#include "tinythread.h"

class client_tcpsocket;

using namespace std;
using namespace math;

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

class DMC4143 {
public:
	DMC4143();
	bool isInitialized();
	bool initialize();
	void stop();
	bool setPosition(long position,int motor);
	bool setVelocity(long velocity,int motor);
	bool setAcceleration(long acceleration,int motor);
	bool setTorque(long torque,int motor);
	long getPosition(int motor);
	long getVelocity(int motor);
	long getTorque(int motor);
	std::string command(std::string Command); // user command structure, used by MotorController
	vector<int> chairParameter;
	vector<double> enc2Radian;
private:
	double encToAng(int motor, long encCount);
	tthread::thread* t;
	static void running(void * aArg);
	bool initialized;
	int commandGalil(char* Command, char* Response, int ResponseSize); // Galil Controller command structure, used by command()
	static client_tcpsocket sock; // The socket class used to communicate with galil controller
	bool setupSocket();
	bool startup();
	bool setDefaults(); // set defaults
	Matrix J_Dtheta2Dphi;
};

#endif;