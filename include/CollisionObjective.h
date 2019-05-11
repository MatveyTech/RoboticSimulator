#pragma once

#include "ObjectiveFunction.h"
#include "Robot.h"
#include "SingleArmKinematicsSolver.h"

using namespace Eigen;

class CollisionObjective : public ObjectiveFunction
{
	int m_numOfJoints;
	P3D& m_point;
	double& m_radius;
	double m_safetyMarginSq = 1.1*1.1;
	SingleArmKinematicsSolver kSolver;
	double norm_const = 41649.31279 * 10000;
public:
	CollisionObjective(int numOfJoints, int weight, P3D& point, double& radius, Robot* robot);
	~CollisionObjective();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	static bool UseBaseAddGradient;
};


