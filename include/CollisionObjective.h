#pragma once

#include "ObjectiveFunction.h"
#include "Robot.h"
#include "SingleArmKinematicsSolver.h"

using namespace Eigen;

class CollisionObjective : public ObjectiveFunction
{
	int m_numOfJoints;
	P3D m_point;
	Robot* m_robot;
	SingleArmKinematicsSolver kSolver;
public:
	CollisionObjective(int numOfJoints, double weight, P3D point, Robot* robot);
	~CollisionObjective();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);
};

