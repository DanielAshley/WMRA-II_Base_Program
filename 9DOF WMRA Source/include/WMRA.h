#ifndef WMRA_H
#define WMRA_H

#include <iostream>
#include "tinythread.h"

namespace WMRA{
	class wmra
	{
	public:
		wmra(void);
		//~wmra(void);
		bool initialize();
		tthread::thread* t;
	private:
		static void wmra::running(void * aArg);
		bool wmraDefaults();
		bool debugMode;
	};
};


#endif;