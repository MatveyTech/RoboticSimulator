#include "..\include\SmoothnessObjective.h"
#include "..\include\StartFromFirstObjective.h"
#include "..\include\FinishAtLastObjective.h"
#include "..\include\CloseToPointObjective.h"

bool SmoothnessObjective::UseBaseAddGradient = false;

SmoothnessObjective::SmoothnessObjective(int numOfJoints, int numOfPoints, int weight):
	m_numOfJoints(numOfJoints)
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
}


SmoothnessObjective::~SmoothnessObjective()
{
}

double SmoothnessObjective::computeValue(const dVector & p)
{
	if (p.rows() % m_numOfJoints != 0)
		throw("BAD");
	return (A*p).squaredNorm() * weight;
}

void SmoothnessObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addHessianEntriesTo(hessianEntries, p);
		return;
	}
}

void SmoothnessObjective::addGradientTo(dVector & grad, const dVector & p)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addGradientTo(grad, p);
		return;
	}
	if (weight == 0)
		return;

	VectorXd tres = 2 * A.transpose() * A * p * weight;
	grad += tres;
	return;
}