#include "../include/IK_Solver.h"
#include "../include/BodyFrame.h"
#include "../include/GenericLimb.h"

IK_Solver::IK_Solver(Robot* robot){
	ikPlan = new IK_Plan(robot);
	ikEnergyFunction = new IK_EnergyFunction(ikPlan);
	ikOptimizer = new IK_Optimizer(ikPlan, ikEnergyFunction);
	ikPlan->setCurrentIKStateFromRobot();
}

IK_Solver::IK_Solver(Robot* robot, bool freezeRootConfiguration) {
	ikPlan = new IK_Plan(robot);
	ikPlan->optimizeRootConfiguration = !freezeRootConfiguration;
	ikEnergyFunction = new IK_EnergyFunction(ikPlan);
	ikOptimizer = new IK_Optimizer(ikPlan, ikEnergyFunction);
	ikPlan->setCurrentIKStateFromRobot();
}


IK_Solver::~IK_Solver(void){
	delete ikPlan;
	delete ikEnergyFunction;
	delete ikOptimizer;
}

GeneralizedCoordinatesRobotRepresentation * IK_Solver::GetgcRobotRepresentation()
{
	return ikPlan->gcRobotRepresentation;
}

void IK_Solver::solve(int nSteps, bool resetTargetState, bool resetInitialState) {
	if (resetInitialState)
		ikPlan->setCurrentIKStateFromRobot();
	if (resetTargetState)
		ikPlan->setTargetIKStateFromRobot();
	ikOptimizer->optimizePlan(nSteps);
	ikPlan->setCurrentIKStateToRobot();
}
