#include "WHEELCHAIR_module.h"
#include "galilController4143.h"
#include "WmraTypes.h"

using namespace WMRA;
galilController4143 controller;

WHEELCHAIR_module::WHEELCHAIR_module(void)
{
}

bool WHEELCHAIR_module::initialize()
{
	return controller.initialize();
}

bool WHEELCHAIR_module::isInitialized()
{
	return controller.isInitialized();
}

std::vector<double> WHEELCHAIR_module::getPosition()
{
	vector<double> pos;
	pos.push_back(controller.getPosition(1)*controller.enc2Radian[0]);
	pos.push_back(controller.getPosition(2)*controller.enc2Radian[1]);
	return pos;
}
