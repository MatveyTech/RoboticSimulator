#include "..\include\EqualDistributionObjective.h"
#include "..\include\StartFromFirstObjective.h"
#include "..\include\FinishAtLastObjective.h"
#include "..\include\CloseToPointObjective.h"

bool EqualDistributionObjective::UseBaseAddGradient = false;

EqualDistributionObjective::EqualDistributionObjective(int numOfJoints, int numOfPoints, int weight):
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


EqualDistributionObjective::~EqualDistributionObjective()
{
}

double EqualDistributionObjective::computeValue(const dVector & p)
{
	if (p.rows() % m_numOfJoints != 0)
		throw("BAD");
	return (A*p).squaredNorm() * weight;
}

void EqualDistributionObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addHessianEntriesTo(hessianEntries, p);
		return;
	}
}

void EqualDistributionObjective::addGradientTo(dVector & grad, const dVector & p)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addGradientTo(grad, p);
		return;
	}
	for (size_t i = 0; i < p.rows(); i++)
	{
		double currVal = 0;
		if (i < m_numOfJoints)
		{
			currVal = 2 * p(i) - 2 * p(i + m_numOfJoints);
		}
		else if (i > p.rows() - 1 - m_numOfJoints)
		{
			currVal = 2 * p(i) - 2 * p(i - m_numOfJoints);
		}
		else
		{
			currVal = 4 * p(i) - 2 * p(i - m_numOfJoints) - 2 * p(i + m_numOfJoints);
		}
		grad(i) += currVal * weight;
	}
}