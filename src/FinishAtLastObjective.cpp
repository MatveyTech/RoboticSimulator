#include "..\include\FinishAtLastObjective.h"



FinishAtLastObjective::FinishAtLastObjective(const VectorXd & endPos, const std::string & objectiveDescription)
{
	m_endPos = endPos;
	m_numOfJoints = m_endPos.rows();
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
	return res;
}

void FinishAtLastObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{

}

void FinishAtLastObjective::addGradientTo(dVector & grad, const dVector & p)
{
	//the derivation is 2ql1 - 2qf1
	int startIndex = p.rows() - m_numOfJoints -1;
	int stopIndex = p.rows() - 1;
	for (size_t i = startIndex; i < stopIndex; i++)
	{
		grad(i) = 2 * p(i) - 2 * m_endPos(i);
	}
}
