#pragma once
#include "ObjectiveFunction.h"
#include "..\include\Robot.h"
using namespace Eigen;

class SmoothnessObjective : public ObjectiveFunction
{
public:
	SmoothnessObjective(int numOfJoints, int numOfPoints, int weight);
	~SmoothnessObjective();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

	static bool UseBaseAddGradient;

private:
	int m_numOfJoints;
	MatrixXd A;
};
