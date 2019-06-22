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

bool StartFromFirstObjective::UseBaseAddGradient = false;

StartFromFirstObjective::StartFromFirstObjective(const VectorXd & startPos, int numOfPoints, int weight):
	m_startPos(startPos),
	m_numOfJoints(m_startPos.rows()),
	m_numOfPoints(numOfPoints)
{
	this->weight = weight;
}

StartFromFirstObjective::~StartFromFirstObjective()
{
}

double StartFromFirstObjective::computeValue(const dVector & curr)
{
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
	VectorXd firstQ = p.topRows(m_numOfJoints);
	double res = 0;
	for (size_t i = 0; i < m_numOfJoints; i++)
	{
		double diff = firstQ(i) - m_startPos(i);
		res += diff*diff; 
	}
	return res * weight;
}

void StartFromFirstObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & curr)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addHessianEntriesTo(hessianEntries, curr);
		return;
	}
	for (int i = 0; i < m_numOfJoints; ++i)
	{
		ADD_HES_ELEMENT(hessianEntries, i, i, 2, weight);
	}
	

}

void StartFromFirstObjective::addGradientTo(dVector & grad, const dVector & curr)
{
	if (UseBaseAddGradient)
	{
		ObjectiveFunction::addGradientTo(grad, curr);
		return;
	}
	dVector p = curr.head(m_numOfJoints*m_numOfPoints);
	//the derivation is 2ql1 - 2qf1
	/*int startIndex = p.rows() - m_numOfJoints -1;
	int stopIndex = p.rows() - 1;*/
	for (size_t i = 0; i < m_numOfJoints; i++)
	{
		grad(i) += (2 * p(i) - 2 * m_startPos(i))*weight;
	}

	//printVector1(grad);
}
