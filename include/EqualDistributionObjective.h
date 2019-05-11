#pragma once
#include "ObjectiveFunction.h"
#include "..\include\Robot.h"
using namespace Eigen;

class EqualDistributionObjective : public ObjectiveFunction
{
public:
	EqualDistributionObjective(int NumOfJoints, int weight);
	~EqualDistributionObjective();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	static bool UseBaseAddGradient;

private:
	int m_numOfJoints;
};
