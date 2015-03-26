#pragma once

#include <vector>

namespace WMRA{
	class Pose;
	class Omni_data;
	class JointValueSet;
	enum CordFrame;
};


namespace WMRA{
	class WHEELCHAIR_module
	{
	public:
		WHEELCHAIR_module(void);
		bool initialize();
		bool isInitialized();
		std::vector<double> getPosition();
//	private:
	};
};

