#pragma once

#include "../include/MathLib.h"
#include "../include/P3D.h"
#include "../include/V3D.h"
#include "../include/MathLib.h"
#include "../include/Trajectory.h"
#include "../include/Robot.h"
#include "../include/GeneralizedCoordinatesRobotRepresentation.h"
#include <vector>
#include "../include/IK_RobotStateRegularizer.h"
#include "../include/IK_Optimizer.h"
#include "../include/IK_Plan.h"



/**
	This plan is for one specific moment in time - given targets for end effectors, COM and full-body state (with various weights to mimic regularizers or hard constraints), we need to compute the robot's joint angles
*/
class IK_Solver{
public:
	IK_Solver(Robot* robot);
	IK_Solver(Robot* robot, bool freezeRootConfiguration);

	virtual ~IK_Solver(void);
	GeneralizedCoordinatesRobotRepresentation* GetgcRobotRepresentation();

public:
	//stores all objectives
	IK_Plan *ikPlan;
	//this is the energy function that operates on objectives stored in ik plan
	IK_EnergyFunction *ikEnergyFunction;
	//and the optimizer that minimizes the energy function
	IK_Optimizer *ikOptimizer;

	void solve(int nSteps = 10, bool resetTargetState = false, bool resetInitialState = true);
};

