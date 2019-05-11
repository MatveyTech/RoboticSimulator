#pragma once

#include "ObjectiveFunction.h"
#include "Robot.h"
#include "SingleArmKinematicsSolver.h"

using namespace Eigen;

class CloseToPointObjective : public ObjectiveFunction
{
	int m_numOfJoints;
	P3D& m_point;
	SingleArmKinematicsSolver kSolver;
public:
	CloseToPointObjective(int numOfJoints, int weight, P3D& point, Robot* robot);
	~CloseToPointObjective();

	static bool UseBaseAddGradient;

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);
};

