#pragma once

#include "ObjectiveFunction.h"
#include "Robot.h"
#include "SingleArmKinematicsSolver.h"

class CollisionSphere
{
public:
	P3D Location;
	double Radius;
	int IndexInViewer;

	CollisionSphere(P3D loc, double rad, int ind);
	bool CollidesRobot(P3D eePosition);
};

using namespace Eigen;

class CollisionObjective : public ObjectiveFunction
{
	int m_numOfJoints;
	P3D m_point;
	double m_radiusSq;
	SingleArmKinematicsSolver kSolver;
	double norm_const = 41649.31279 * 10000;
public:
	CollisionObjective(int numOfJoints, double weight, P3D point, double radius, Robot* robot);
	~CollisionObjective();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);
};

