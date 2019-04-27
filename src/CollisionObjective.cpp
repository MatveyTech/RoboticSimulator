#include "..\include\CollisionObjective.h"


CollisionObjective::CollisionObjective(int numOfJoints, double weight, P3D point, double radius, Robot* robot) :
	kSolver(robot),
	m_numOfJoints(numOfJoints),
	m_point(point),
	m_radiusSq(radius*radius*1.1*1.1) // add 10% safety margin
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
		double dSq = (cart_pos - m_point).squaredNorm();
		double diff = dSq - m_radiusSq;
		result += dSq > m_radiusSq ? 0 : diff*diff;
	}
	return result*norm_const * weight;
}


void CollisionObjective::addHessianEntriesTo(DynamicArray<MTriplet>&	 hessianEntries, const dVector & p)
{

}

void CollisionObjective::addGradientTo(dVector & grad, const dVector & p)
{
	int numOfPoints = p.rows() / m_numOfJoints;
	for (int i = 0; i < numOfPoints; ++i)
	{
		MatrixNxM J;
		VectorXd q = p.segment(m_numOfJoints*i, m_numOfJoints);
		kSolver.compute_dpdq(q, J);

		MatrixNxM Ja(3, 7);
		Ja.col(0) = J.col(6);
		Ja.col(1) = J.col(8);
		Ja.col(2) = J.col(10);
		Ja.col(3) = J.col(12);
		Ja.col(4) = J.col(14);
		Ja.col(5) = J.col(16);
		Ja.col(6) = J.col(18);

		P3D cart_pos = kSolver.CalcForwardKinematics(q);

		MatrixNxM res(1, 7);
		double dSq = (cart_pos - m_point).squaredNorm();
		double diff = dSq - m_radiusSq;
		double f_val = dSq > m_radiusSq ? 0 : 2*diff;
		res = 2*Ja.transpose()*(cart_pos - m_point)*f_val;
		grad.segment(m_numOfJoints*i, m_numOfJoints) += res * norm_const * weight;
	}
}
