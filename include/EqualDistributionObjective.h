#pragma once
#include "ObjectiveFunction.h"
using namespace Eigen;

class EqualDistributionObjective : ObjectiveFunction
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

