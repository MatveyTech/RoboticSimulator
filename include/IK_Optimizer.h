#pragma once

#include "../include/IK_EnergyFunction.h"
#include "../include/IK_Plan.h"


/**
	This is a locomotion engine that works for arbitrary robot types
*/
class IK_Optimizer{
public:
	IK_Optimizer(IK_Plan* IKPlan, IK_EnergyFunction* energyFunction);
	virtual ~IK_Optimizer(void);

	double optimizePlan(int maxIterNum = 3);

public:
	IK_EnergyFunction* energyFunction;
	IK_Plan* IKPlan;
	bool checkDerivatives = false;
};



