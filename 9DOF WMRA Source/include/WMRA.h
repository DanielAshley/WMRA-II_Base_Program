#ifndef WMRA_H
#define WMRA_H

#include <iostream>
#include "tinythread.h"
#include "matrix.h" 

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
	private:
		bool WMRA_Jacobian_Ground2Endeffector();
		static void running(void * aArg);
		bool wmraDefaults();
		bool debugMode;
		bool controlType;
		vector<double> Qarm;
		vector<double> link_parameters;
		vector<double> Length_parameters;
		Matrix Jac;
	};
};


#endif;