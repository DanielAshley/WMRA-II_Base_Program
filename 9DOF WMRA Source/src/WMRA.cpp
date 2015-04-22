
#include "ConfigReader.h"
#include "ARM_module.h"
#include "WHEELCHAIR_module.h"
#include "WMRA.h"
#include <time.h>

//#pragma comment(lib, "ARM_module.lib")
//#pragma comment(lib, "WHEELCHAIR_module.lib")

using namespace std;
using namespace WMRA;
using namespace tthread;

WMRA::ARM_module ARM;
WMRA::WHEELCHAIR_module WHEELCHAIR;

wmra::wmra()
{
	cout << "Initializing WMRA" << endl;
}

bool wmra::initialize()
{
	/*
	if(!wmraDefaults())
		return 0;
	if(!ARM.initialize())
		return 0;
	if(!WHEELCHAIR.initialize())
		return 0;
		*/

//	link_parameters = ARM.getLinkParameters();
//	Length_parameters = ARM.getLengthParameters();
	t = new thread(running,this);
	
	return 1;
}

bool wmra::WMRA_Jacobian_Ground2Endeffector()
{ 
	Matrix temp(6,9);
	double L1 = Length_parameters[0];
	double L2 = Length_parameters[1];
	double L3 = Length_parameters[2];
	// L4 = Length_parameters[3];
	double L5 = Length_parameters[4];
	double d1=link_parameters[0];
	double d2=link_parameters[1]; 
	double d3=link_parameters[2]; 
	double d4=link_parameters[3]; 
	double d5=link_parameters[4]; 
	double d6=link_parameters[5]; 
	double d7=link_parameters[6];
	double theta1=Qarm[0];
	double theta2=Qarm[1];
	double theta3=Qarm[2];
	double theta4=Qarm[3];
	double theta5=Qarm[4];
	double theta6=Qarm[5];
	// double theta7=Qarm[6];

	//JG7 respect to armbase. so it is armbase control, using for teleoperation.
  if(controlType)
  {
	  temp(0,0) = d4*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - d6*(sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) + d7*(cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)))) + d2*cos(theta1) + d5*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + d3*sin(theta1)*sin(theta2);
 /*Matrix temp =[d4*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - d6*(sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) + d7*(cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)))) + d2*cos(theta1) + d5*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + d3*sin(theta1)*sin(theta2), - d6*(sin(theta5)*(cos(theta1)*cos(theta2)*sin(theta4) + cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta1)*cos(theta5)*sin(theta2)*sin(theta3)) - d7*(cos(theta6)*(cos(theta1)*cos(theta2)*cos(theta4) - cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4)) - sin(theta6)*(cos(theta5)*(cos(theta1)*cos(theta2)*sin(theta4) + cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)) - cos(theta1)*sin(theta2)*sin(theta3)*sin(theta5))) - d5*(cos(theta1)*cos(theta2)*cos(theta4) - cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4)) - d3*cos(theta1)*cos(theta2) - d4*cos(theta1)*sin(theta2)*sin(theta3), d5*sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) - d6*(cos(theta5)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta4)*sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d7*(sin(theta6)*(sin(theta5)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - cos(theta6)*sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d4*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)), d7*(cos(theta6)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*sin(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2))) + d5*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + d6*sin(theta5)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)), - d6*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d7*sin(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))), -d7*(sin(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) - cos(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)))),                                                                                                                                                                                                                                                                                                                                                             0, (L5*cos(conj(phi))*(cos(phi) + (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2 + (L5*sin(conj(phi))*(sin(phi) - (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2, (L5*cos(conj(phi))*(cos(phi) - (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2 + (L5*sin(conj(phi))*(sin(phi) + (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                   d7*(sin(theta6)*(cos(theta5)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)) + cos(theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta4)*sin(theta2) + cos(theta2)*cos(theta3)*sin(theta4))) - d5*(cos(theta4)*sin(theta2) + cos(theta2)*cos(theta3)*sin(theta4)) - d3*sin(theta2) - d6*(sin(theta5)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)) - cos(theta2)*cos(theta5)*sin(theta3)) + d4*cos(theta2)*sin(theta3),                                                                                                                                                                                                                                                                                                                                                sin(theta2)*(d4*cos(theta3) + d6*cos(theta3)*cos(theta5) + d5*sin(theta3)*sin(theta4) - d6*cos(theta4)*sin(theta3)*sin(theta5) + d7*cos(theta6)*sin(theta3)*sin(theta4) + d7*cos(theta3)*sin(theta5)*sin(theta6) + d7*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta6)),                                                                                                                                                                                                                 d6*sin(theta5)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)) - d7*(cos(theta6)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4))) - d5*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)),                                                                                                                                                                                           d6*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5)) + d7*sin(theta6)*(sin(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta2)*sin(theta3)),                                                                                                                                                 -d7*(cos(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5)) + sin(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4))),                                                                                                                                                                                                                                                                                                                                                             0, (L5*cos(conj(phi))*(sin(phi) - (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2 - (L5*sin(conj(phi))*(cos(phi) + (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2, (L5*cos(conj(phi))*(sin(phi) + (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2 - (L5*sin(conj(phi))*(cos(phi) - (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2;
  d6*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d7*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)))) - d4*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) - d2*sin(theta1) - d5*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + d3*cos(theta1)*sin(theta2),   d5*(cos(theta2)*cos(theta4)*sin(theta1) - cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4)) + d6*(sin(theta5)*(cos(theta2)*sin(theta1)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)) + cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3)) - d7*(sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta1)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)) - sin(theta1)*sin(theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta2)*cos(theta4)*sin(theta1) - cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4))) + d3*cos(theta2)*sin(theta1) + d4*sin(theta1)*sin(theta2)*sin(theta3), d5*sin(theta4)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - d6*(cos(theta5)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - d7*(sin(theta6)*(sin(theta5)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - cos(theta6)*sin(theta4)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - d4*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)), d7*(cos(theta6)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*sin(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2))) + d5*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + d6*sin(theta5)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)), - d6*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - d7*sin(theta6)*(sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))), -d7*(sin(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) - cos(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)))),                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                             0,                                                                                                                                             0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      sin(theta1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -cos(theta1)*sin(theta2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3),                                                                                                                                                                                                                                                                                                                                     sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2),                                                                                                                                                       cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) - sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)), cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))),                                                                                                                                             0,                                                                                                                                             0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           cos(theta2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          sin(theta2)*sin(theta3),                                                                                                                                                                                                                                                                                                                                                                                         cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4),                                                                                                                                                                                                                                                   sin(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta2)*sin(theta3),                                                                                                                                                 cos(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)) - sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5)),                                                                                                                                             0,                                                                                                                                             0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      cos(theta1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               sin(theta1)*sin(theta2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3),                                                                                                                                                                                                                                                                                                                                     sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2),                                                                                                                                                       cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)), cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))),                                                                                                                                        -L5/L1,                                                                                                                                         L5/L1
  ];
  */
  }
  Jac = temp;

	//Matrix destLoc_T(4,4);
	return 0;
}

void wmra::running(void * aArg) {
	wmra* w = (wmra*)aArg;
	clock_t last_time, current_time;
	last_time = clock();
	current_time = clock();
	double dt;
	int count = 0;

	vector<double> X_dot;
	for(int i = 0; i < 7; i++)
	{
		X_dot.push_back(0.0);
		w->Qarm.push_back(0.0);
	}

	while(count < 1000)
	{
		count++;

		/****Calculate dt****/
		clock_t current_time = clock();
		dt = (current_time - last_time)/CLOCKS_PER_SEC;
		last_time = current_time;
		/********************/
		
		/****Data Output (cout)****/
		std::cout.flush();
		//cout << "\rRunning... dt= " << dt;
		cout << "\rRunning... dt= " << count;
		/********************/

		w->WMRA_Jacobian_Ground2Endeffector();
		
		/*********Omni Input***********/
			// Or other input device
		/********************/

	}
}

bool wmra::wmraDefaults()
{
	ConfigReader reader;
	reader.parseFile("settings_wheelchair_controller.conf");
	reader.setSection("RUN MODE");
	if(reader.keyPresent("DEBUGMODE")){			
		debugMode = reader.getInt("DEBUGMODE");
	}
	else{
		cout << "'DEBUGMODE' default not found" << endl;		
		return 0;
	}
	return 1;
}

