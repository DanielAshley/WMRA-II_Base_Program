#ifndef ARM_H
#define ARM_H

#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>
#include <vector>
#include <fstream>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <time.h>
#include <math.h>
#include "matrix.h" 
#include "tinythread.h"
#include "MotorController.h"
#include "SockStream.h"


class Arm{
public:
	Arm();
	bool initialize();
	bool setDefaults();
	bool autonomous(WMRA::Pose dest, WMRA::CordFrame crodFr, bool blocking = true);
	bool teleoperation(WMRA::Pose deltaPose);
	bool teleoperation(WMRA::Pose deltaPose, double deltaTime);
	bool teleoperation(WMRA::Pose dest, WMRA::CordFrame cordFr);
	bool openGripper(bool blocking = true);
	bool closeGripper(bool blocking = true);
    bool isGripperOpen();
	void closeDebug();
	bool toReady(bool blocking = true);
	bool ready2Park(bool blocking = true);
	bool park2Ready(bool blocking = true);
	bool motionComplete();
	bool moveJoint(int jointNum, double angle, int ref);
	bool stop();
	WMRA::Pose getPose();
	WMRA::JointValueSet getJointAngles();
	void sendValues();
	bool isInitialized();
	bool setInitialJointAngles(WMRA::JointValueSet& joints);
	static void sendData(void* aArg);
	WMRA::JointValueSet getLastKnownJointPosition();
	vector<double> getPosition();
	tthread::thread* t;
	std::string command(std::string Command);
	vector<double> updateWheelchairPosition();
	vector<double> Q_arm_position; //radians

private:
	static void running(void * aArg);
	bool autonomousMove(Matrix start, Matrix dest, bool blocking = true);
	bool teleoperationMove(Matrix start, Matrix dest);
	int joint_limit_avoidance(int joint_name, int encoder_count, int speed);
	int joint_max_speed(int joint_name);
	int joint_speed_limit(int joint_name, int speed);
	double dt;	// the default time between milestones
	double dt_mod;	// the default time between milestones
	double maxAngularVelocity;
	int control_velocity;
	
	WMRA::JointValueSet readyPosition; //joint angles for ready position
	WMRA::JointValueSet lastKnownJointPos;

	std::ofstream xyz_way; // Waypoint XYZ values
	std::ofstream xyz_sent; // command XYZ values
	std::ofstream xyz_cont; // command XYZ values
	std::ofstream jointVel; // joint velocity values
	bool initialized;
    bool gripperOpen;
	MotorController controller; 
	Matrix gripperInitRotDiff;
	vector<double> initial_Arm_position; //radians
	vector<double> current_arm_position; //radians
	vector<double> Arm_link; //radians
};
#endif;