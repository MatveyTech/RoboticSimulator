#include "..\include\CloseToPointObjective.h"
#include <iostream>

bool CloseToPointObjective::UseBaseAddGradient = false;

CloseToPointObjective::CloseToPointObjective(int numOfJoints, int numOfPoints, int weight, P3D& point, Robot* robot):
	kSolver(robot),
	m_numOfJoints(numOfJoints),
	m_point(point),
	m_numOfPoints(numOfPoints)
{
	this->weight = weight;
}

CloseToPointObjective::~CloseToPointObjective()
{
}

void CloseToPointObjective::UpdatePoint(P3D point)
{
	m_point = point;
}

double CloseToPointObjective::computeValue(const dVector & curr)
{
	double result = 0;
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
	for (size_t i = 0; i < m_numOfPoints; i++)
	{
		VectorXd q = p.segment(m_numOfJoints*i, m_numOfJoints);
		P3D cart_pos = kSolver.CalcForwardKinematics(q);
		result += (cart_pos - m_point).squaredNorm();
	}
	return result*weight*.5;
}


void CloseToPointObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & curr)
{
	throw ("Not implemented");
}

void CloseToPointObjective::addGradientTo(dVector & grad, const dVector & curr)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addGradientTo(grad, curr);
		return;
	}
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
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
		res = (cart_pos - m_point).transpose()*Ja;
		//grad.segment(m_numOfJoints*i, m_numOfJoints) += res*weight;
		for (size_t j = 0; j < m_numOfJoints; j++)
		{
			grad(i * m_numOfJoints + j) += res(j)*weight;
		}
	}
}