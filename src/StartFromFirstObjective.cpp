#include "..\include\StartFromFirstObjective.h"
#include <iostream>
using namespace std;

void printVector1(VectorXd pp)
{
	for (size_t i = 0; i < pp.size(); i++)
	{
		cout << pp(i) << " ";
		if (i % 7 == 6)
			cout << endl;
	}
}


StartFromFirstObjective::StartFromFirstObjective(const VectorXd & startPos, double weight)
{
	m_startPos = startPos;
	m_numOfJoints = m_startPos.rows();
	this->weight = weight;
}

StartFromFirstObjective::~StartFromFirstObjective()
{
}

double StartFromFirstObjective::computeValue(const dVector & p)
{
	VectorXd firstQ = p.topRows(m_numOfJoints);
	double res = 0;
	for (size_t i = 0; i < m_numOfJoints; i++)
	{
		double diff = firstQ(i) - m_startPos(i);
		res += diff*diff;
	}
	return res * weight;
}

void StartFromFirstObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{

}

void StartFromFirstObjective::addGradientTo(dVector & grad, const dVector & p)
{
	//the derivation is 2ql1 - 2qf1
	/*int startIndex = p.rows() - m_numOfJoints -1;
	int stopIndex = p.rows() - 1;*/
	for (size_t i = 0; i < m_numOfJoints; i++)
	{
		grad(i) += 2 * p(i) - 2 * m_startPos(i);
	}

	//printVector1(grad);
}
