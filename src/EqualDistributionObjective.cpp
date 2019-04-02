#include "..\include\EqualDistributionObjective.h"
#include "..\include\StartFromFirstObjective.h"
#include "..\include\FinishAtLastObjective.h"
#include "..\include\CollisionObjective.h"

EqualDistributionObjective::EqualDistributionObjective(int NumOfJoints)
{
	m_numOfJoints = NumOfJoints;
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
	return res;
}

void EqualDistributionObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{
}

void EqualDistributionObjective::addGradientTo(dVector & grad, const dVector & p)
{
	for (size_t i = 0; i < p.rows(); i++)
	{
		if (i < m_numOfJoints)
		{
			grad(i) += 2 * p(i) - 2 * p(i + m_numOfJoints);
		}
		else if (i > p.rows() - 1 - m_numOfJoints)
		{
			grad(i) += 2 * p(i) - 2 * p(i - m_numOfJoints);
		}
		else
		{
			grad(i) += 4 * p(i) - 2 * p(i - m_numOfJoints) - 2 * p(i + m_numOfJoints);
		}
	}
}


Basic3::Basic3(const VectorXd& startPos, const VectorXd& finalPos, std::vector<double> weights, Robot* robot)
{
	objectives.push_back(new StartFromFirstObjective(startPos, weights.at(0)));
	objectives.push_back(new FinishAtLastObjective(finalPos, weights.at(1)));
	objectives.push_back(new EqualDistributionObjective(startPos.rows()));//weights.at(2)
	//objectives.push_back(new CollisionObjective(startPos.rows(), 1.0, P3D(0, 0, 0), robot));
}


Basic3::~Basic3()
{
}

double Basic3::computeValue(const dVector & p)
{
	double res=0;
	for (ObjectiveFunction* objective : objectives)
		res += objective->computeValue(p);
	return res;
}

void Basic3::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{
}

void Basic3::addGradientTo(dVector & grad, const dVector & p)
{
	for (ObjectiveFunction* objective : objectives)
		objective->addGradientTo(grad,p);
}