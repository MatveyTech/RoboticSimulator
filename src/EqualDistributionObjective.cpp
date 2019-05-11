#include "..\include\EqualDistributionObjective.h"
#include "..\include\StartFromFirstObjective.h"
#include "..\include\FinishAtLastObjective.h"
#include "..\include\CloseToPointObjective.h"

bool EqualDistributionObjective::UseBaseAddGradient = false;

EqualDistributionObjective::EqualDistributionObjective(int NumOfJoints, int weight):
	m_numOfJoints(NumOfJoints)
{
	this->weight = weight;
}


EqualDistributionObjective::~EqualDistributionObjective()
{
}

double EqualDistributionObjective::computeValue(const dVector & p)
{
	if (p.rows() % m_numOfJoints != 0)
		throw("BAD");
	double res=0;
	int numOfPoints = p.rows() / m_numOfJoints;
	for (size_t i = 1; i < numOfPoints; i++)
	{
		for (size_t j = 0; j < m_numOfJoints; j++)
		{
			double p1 = p(i*m_numOfJoints + j);
			double p2 = p((i - 1)*m_numOfJoints + j);
			double diff =  p1-p2;
			res += diff*diff;
		}
	}
	return res*weight;
}

void EqualDistributionObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{
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