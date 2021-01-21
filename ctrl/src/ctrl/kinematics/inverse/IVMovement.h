#pragma once
#include <Configuration.h>
#include <Trajectory.h>
#include<SixDPos.h>
#include "inverse_kinematics.h"
class IVMovement
{
public:
	IVMovement();
	~IVMovement();
	Trajectory* getMovement(vector<SixDPos *>* _positions, Configuration * start_cfg);

private:
	Configuration* IVMovement::GetClosestConfiguration(vector<Configuration*>* _configs, Configuration* _prevConfig);
	InvKinematics* invK;
	Trajectory * trajectory;
};

