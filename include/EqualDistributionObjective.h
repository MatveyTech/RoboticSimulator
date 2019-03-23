#pragma once
#include "ObjectiveFunction.h"
using namespace Eigen;

class EqualDistributionObjective : public ObjectiveFunction
{
public:
	EqualDistributionObjective(int NumOfJoints);
	~EqualDistributionObjective();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	int m_numOfJoints;
};


class Basic3 : public ObjectiveFunction
{	
	std::vector<ObjectiveFunction*> objectives;
public:
	Basic3(const VectorXd& startPos, const VectorXd& finalPos,std::vector<double> weights);
	~Basic3();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	int m_numOfJoints;
};

