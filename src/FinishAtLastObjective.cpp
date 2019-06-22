#include "..\include\FinishAtLastObjective.h"

bool FinishAtLastObjective::UseBaseAddGradient = false;

FinishAtLastObjective::FinishAtLastObjective(const VectorXd & endPos,int numOfPoints, int weight):
	m_endPos(endPos),
	m_numOfJoints(m_endPos.rows()),
	m_numOfPoints(numOfPoints)
{	
	this->weight = weight;
}

FinishAtLastObjective::~FinishAtLastObjective()
{
}

double FinishAtLastObjective::computeValue(const dVector & curr)
{
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
	VectorXd lastQ = p.bottomRows(m_numOfJoints);
	double res = 0;
	for (size_t i = 0; i < m_numOfJoints; i++)
	{
		double diff = lastQ(i) - m_endPos(i);
		res += diff*diff;
	}
	return res * weight;
}

void FinishAtLastObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & curr)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addHessianEntriesTo(hessianEntries, curr);
		return;
	}
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
	int matSize = p.size();
	for (int i = matSize-m_numOfJoints; i < matSize; ++i)
	{
		ADD_HES_ELEMENT(hessianEntries, i, i, 2, weight);
	}
}

void FinishAtLastObjective::addGradientTo(dVector & grad, const dVector & curr)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addGradientTo(grad, curr);
		return;
	}
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
	//the derivation is 2ql1 - 2qf1
	int startIndex = p.rows() - m_numOfJoints;
	int stopIndex = p.rows();
	for (size_t i = startIndex; i < stopIndex; i++)
	{
		int ind = i - startIndex;
		grad(i) += (2 * p(i) - 2 * m_endPos(ind))*weight;
	}
}
