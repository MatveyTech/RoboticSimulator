#include "..\include\CollisionObjective.h"
#include <iostream>



CollisionObjective::CollisionObjective(int numOfJoints, double weight, P3D point, Robot* robot):
	kSolver(robot),
	m_numOfJoints(numOfJoints),
	m_point(point),
	m_robot(robot)
{
	this->weight = weight;
}

CollisionObjective::~CollisionObjective()
{
}

double CollisionObjective::computeValue(const dVector & p)
{
	double result = 0;
	int numOfPoints = p.rows() / m_numOfJoints;
	for (size_t i = 0; i < numOfPoints; i++)
	{
		VectorXd q = p.segment(m_numOfJoints*i, m_numOfJoints);
		P3D cart_pos = kSolver.CalcForwardKinematics(q);
		result += (cart_pos - m_point).squaredNorm();
	}
	return result;
}


void CollisionObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{

}

void CollisionObjective::addGradientTo(dVector & grad, const dVector & p)
{
	int numOfPoints = p.rows() / m_numOfJoints;
	//for (int i = 0; i < numOfPoints; ++i)
	//{
	//	MatrixNxM J;
	//	GeneralizedCoordinatesRobotRepresentation gc(m_robot);
	//	VectorXd q(7);
	//	
	//	//RAD!!!!!!!!!!!!!!!!!!
	//	VectorXd q = p.segment(m_numOfJoints*i, m_numOfJoints);
	//	
	//	gc.setQ(q);
	//	gc.compute_dpdq(m_robot->GetGripLocalCoordinates(), m_rightEE, J);

	//	MatrixNxM Ja(3, 7);
	//	Ja.col(0) = J.col(6);
	//	Ja.col(1) = J.col(8);
	//	Ja.col(2) = J.col(10);
	//	Ja.col(3) = J.col(12);
	//	Ja.col(4) = J.col(14);
	//	Ja.col(5) = J.col(16);
	//	Ja.col(6) = J.col(18);

	//	P3D cart_pos = gc.getWorldCoordinatesFor(m_robot->GetGripLocalCoordinates(), m_rightEE);

	//	MatrixNxM res(1, 7);
	//	res = (cart_pos - m_point).transpose()*Ja;
	//	grad(i * 7 + 0) += res(0);
	//	grad(i * 7 + 1) += res(1);
	//	grad(i * 7 + 2) += res(2);
	//	grad(i * 7 + 3) += res(3);
	//	grad(i * 7 + 4) += res(4);
	//	grad(i * 7 + 5) += res(5);
	//}
}
