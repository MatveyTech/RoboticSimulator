#include "..\include\SingleArmKinematicsSolver.h"
#include <iostream>

bool SingleArmKinematicsSolver::IsOnlyOneArmQ(const dVector & q)
{
	return q.size() == m_numOfJoints;
}

SingleArmKinematicsSolver::SingleArmKinematicsSolver(Robot * r):
	m_gcrp(GeneralizedCoordinatesRobotRepresentation(r)),
	m_rb(r->getRigidBody(13)),//13 for the right hand gripper
	m_robot(r),
	m_numOfJoints(7)
{

}


dVector SingleArmKinematicsSolver::ConvertSingleArmtoAllQ(const dVector & q)
{
	dVector current_q = m_robot->GetQ();
	//quick and dirty
	current_q(6) = RAD(q(0));
	current_q(8) = RAD(q(1));
	current_q(10) = RAD(q(2));
	current_q(12) = RAD(q(3));
	current_q(14) = RAD(q(4));
	current_q(16) = RAD(q(5));
	current_q(18) = RAD(q(6));
	return current_q;
}


P3D SingleArmKinematicsSolver::CalcForwardKinematics(const dVector & q)
{	
	const dVector& qq = IsOnlyOneArmQ(q) ? ConvertSingleArmtoAllQ(q) : q;
	m_gcrp.setQ(qq);
	P3D res = m_gcrp.getWorldCoordinatesFor(GetGripLocalCoordinates(), m_rb);
	return res;
}


void SingleArmKinematicsSolver::compute_dpdq(const dVector & q, MatrixNxM & dpdq)
{
	const dVector& qq = IsOnlyOneArmQ(q) ? ConvertSingleArmtoAllQ(q) : q;
	m_gcrp.setQ(ConvertSingleArmtoAllQ(q));
	m_gcrp.compute_dpdq(GetGripLocalCoordinates(), m_rb, dpdq);
}
