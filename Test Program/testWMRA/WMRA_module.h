#pragma once

#include <string>
#include <vector>

namespace WMRA{
	class Pose;
	class JointValueSet;
	enum CordFrame;
};

namespace WMRA{
	class WMRA_module
	{
	public:
		WMRA_module(void);
		~WMRA_module(void);
		bool initialize();
	};
};