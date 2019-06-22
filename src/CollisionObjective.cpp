#include "..\include\CollisionObjective.h"
#include <igl/readOFF.h>
#include <stdio.h>

using namespace std;


CollisionObjective::CollisionObjective(int numOfJoints, int numOfPoints, int weight, P3D& point, double& radius, Robot* robot) :
	kSolver(robot),
	m_numOfJoints(numOfJoints),
	m_numOfPoints(numOfPoints),
	m_point(point),
	m_radius(radius)	
{
	this->weight = weight;
}

CollisionObjective::~CollisionObjective()
{
}




double CollisionObjective::computeValue(const dVector & curr)
{
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
	double radiusSq = m_radius*m_radius*m_safetyMarginSq;
	double result = 0;
	for (size_t i = 0; i < m_numOfPoints; i++)
	{
		VectorXd q = p.segment(m_numOfJoints*i, m_numOfJoints);		
		P3D cart_pos = kSolver.CalcForwardKinematics(q);
		double dSq = (cart_pos - m_point).squaredNorm();
		double diff = dSq - radiusSq;
		result += dSq > radiusSq ? 0 : diff*diff;
	}
	return result*norm_const * weight*.5;
}


void CollisionObjective::addHessianEntriesTo(DynamicArray<MTriplet>&	 hessianEntries, const dVector & curr)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addHessianEntriesTo(hessianEntries, curr);
		return;
	}
}

bool CollisionObjective::UseBaseAddGradient = false;

void CollisionObjective::addGradientTo(dVector & grad, const dVector & curr)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addGradientTo(grad, curr);
		return;
	}

	dVector p = curr.head(m_numOfJoints*m_numOfPoints);

	double radiusSq = m_radius*m_radius*m_safetyMarginSq;

	for (int i = 0; i < m_numOfPoints; ++i)
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
		double diff = dSq - radiusSq;
		double f_val = dSq > radiusSq ? 0 : 2*diff;
		res = /*2**/Ja.transpose()*(cart_pos - m_point)*f_val;
		grad.segment(m_numOfJoints*i, m_numOfJoints) += res * norm_const * weight;
	}
}
