
#include <iostream>
#include "WMRA_module.h"
#include "WMRA.h"

using namespace std;
using namespace WMRA;

wmra _wmra;

WMRA_module::WMRA_module()
{
}

bool WMRA_module::initialize()
{
	return _wmra.initialize();
}

bool WMRA_module::sendInputValues(std::vector<double> in)
{
	return _wmra.sendInputValues(in);
}
