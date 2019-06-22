#include "..\include\SmoothnessObjective.h"
#include "..\include\StartFromFirstObjective.h"
#include "..\include\FinishAtLastObjective.h"
#include "..\include\CloseToPointObjective.h"
#include <iostream>

bool SmoothnessObjective::UseBaseAddGradient = false;

SmoothnessObjective::SmoothnessObjective(int numOfJoints, int numOfPoints, int weight):
	m_numOfJoints(numOfJoints),
	m_numOfPoints(numOfPoints)
{
	this->weight = weight;

	int c = numOfJoints * numOfPoints;
	int r = c - numOfJoints;
	
	MatrixXd a(r, c);
	a.setConstant(0);
	for (int i = 0; i < r; ++i)
	{
		a(i, i) = -1;
		a(i, i + numOfJoints) = 1;
	}
	A = a;
	ATA = A.transpose()*A;
 }


SmoothnessObjective::~SmoothnessObjective()
{
}

double SmoothnessObjective::computeValue(const dVector & curr)
{
	/*if (p.rows() % m_numOfJoints != 0)
		throw("BAD");*/
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
	return (A*p).squaredNorm() * weight;
}

void SmoothnessObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & curr)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addHessianEntriesTo(hessianEntries, curr);
		return;
	}

	for (int i = 0; i < ATA.rows(); ++i)
	{
		for (int j = 0; j < ATA.cols(); ++j)
		{
			addMTripletToList_ignoreUpperElements(hessianEntries, i, j, ATA(i, j) * 2 * weight);
		}
	}
}

void SmoothnessObjective::addGradientTo(dVector & grad, const dVector & curr)
{
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addGradientTo(grad, p);
		return;
	}
	if (weight == 0)
		return;

	VectorXd tres = 2 * A.transpose() * A * p * weight;
	grad.head(m_numOfJoints*m_numOfPoints) += tres;
	return;
}