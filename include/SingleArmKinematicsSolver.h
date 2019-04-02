#pragma once

#include "../include/Robot.h"
#include "../include/Matrix.h"
#include "GeneralizedCoordinatesRobotRepresentation.h"

//This class works only for the right arm for now
class SingleArmKinematicsSolver
{
	RigidBody* m_rb;
	Robot* m_robot;
	GeneralizedCoordinatesRobotRepresentation m_gcrp;
	dVector ConvertSingleArmtoAllQ(const dVector & q);
	int m_numOfJoints;

	bool IsOnlyOneArmQ(const dVector & q);
public:

	SingleArmKinematicsSolver(Robot* r);

	P3D CalcForwardKinematics(const dVector& q);
	void compute_dpdq(const dVector& q,  MatrixNxM &dpdq);
	static P3D GetGripLocalCoordinates() { return P3D(0.09, -0.08, -0.1); }
};