#pragma once

#include "ObjectiveFunction.h"

using namespace Eigen;

class FinishAtLastObjective : public ObjectiveFunction
{
public:
	FinishAtLastObjective(const VectorXd & endPos, double weight);
	~FinishAtLastObjective();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	VectorXd m_endPos;
	int m_numOfJoints;
};

