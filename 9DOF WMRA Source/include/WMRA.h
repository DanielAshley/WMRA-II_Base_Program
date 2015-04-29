#ifndef WMRA_H
#define WMRA_H

#include <iostream>
#include "tinythread.h"
#include "matrix.h" 
#include <vector>

using namespace math;

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

namespace WMRA{
	class wmra
	{
	public:
		wmra(void);
		//~wmra(void);
		bool initialize();
		tthread::thread* t;
		double phi;
		bool sendInputValues(); // sets input to zeros
		bool sendInputValues(vector<double> in);
	private:
		bool Jacobian_Ground2Endeffector();
		bool weighted_pseudoinverse();
		bool control_joint(double pitch, double roll);
		bool JointSpeed_limitation();
		static void running(void * aArg);
		bool wmraDefaults();
		bool debugMode;
		bool controlType;
		vector<double> Qarm;
		vector<double> link_parameters;
		vector<double> Length_parameters;
		Matrix Jac;
		Matrix q_dot;
		vector<double> inputDevice;
	};
};


#endif;