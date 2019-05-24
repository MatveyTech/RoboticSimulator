#include "..\include\FinishAtLastObjective.h"

bool FinishAtLastObjective::UseBaseAddGradient = false;

FinishAtLastObjective::FinishAtLastObjective(const VectorXd & endPos, int weight):
	m_endPos(endPos),
	m_numOfJoints(m_endPos.rows())
{	
	this->weight = weight;
}

FinishAtLastObjective::~FinishAtLastObjective()
{
}

double FinishAtLastObjective::computeValue(const dVector & p)
{
	VectorXd lastQ = p.bottomRows(m_numOfJoints);
	double res = 0;
	for (size_t i = 0; i < m_numOfJoints; i++)
	{
		double diff = lastQ(i) - m_endPos(i);
		res += diff*diff;
	}
	return res * weight;
}

void FinishAtLastObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addHessianEntriesTo(hessianEntries, p);
		return;
	}

	int matSize = p.size();
	for (int i = matSize-m_numOfJoints; i < matSize; ++i)
	{
		ADD_HES_ELEMENT(hessianEntries, i, i, 2, weight);
	}
}

void FinishAtLastObjective::addGradientTo(dVector & grad, const dVector & p)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addGradientTo(grad, p);
		return;
	}
	//the derivation is 2ql1 - 2qf1
	int startIndex = p.rows() - m_numOfJoints;
	int stopIndex = p.rows();
	for (size_t i = startIndex; i < stopIndex; i++)
	{
		int ind = i - startIndex;
		grad(i) += (2 * p(i) - 2 * m_endPos(ind))*weight;
	}
}
