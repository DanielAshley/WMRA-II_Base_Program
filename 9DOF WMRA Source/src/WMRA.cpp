
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
}

bool wmra::initialize()
{
	//if(!ARM.initialize())
	//	return 0;
	//if(!WHEELCHAIR.initialize())
	//	return 0;
		
	t = new thread(running,this);
	
	return 1;
}

void wmra::running(void * aArg) {
	wmra* w = (wmra*)aArg;
	clock_t last_time, current_time;
	float dt;
	int count = 0;

	while(count < 1000)
	{
		count++;

		/****Calculate dt****/
		clock_t current_time = clock();
		dt = (current_time - last_time)/CLOCKS_PER_SEC;
		last_time = current_time;
		/********************/

		std::cout.flush();
		//cout << "\rRunning... dt= " << dt;
		cout << "\rRunning... dt= " << count;
		
	}
}

