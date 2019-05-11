#pragma once
#include "ObjectiveFunction.h"

using namespace Eigen;

class StartFromFirstObjective : public ObjectiveFunction
{
public:
	StartFromFirstObjective(const VectorXd& startPos, int weight);
	virtual ~StartFromFirstObjective(void);

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	const VectorXd& m_startPos;
	int m_numOfJoints;
};

