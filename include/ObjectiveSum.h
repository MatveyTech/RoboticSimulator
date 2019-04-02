#pragma once
#include "ObjectiveFunction.h"
#include "..\include\Robot.h"
using namespace Eigen;

class ObjectiveSum : public ObjectiveFunction
{	
	std::vector<ObjectiveFunction*> objectives;
public:
	ObjectiveSum(const VectorXd& startPos, const VectorXd& finalPos,std::vector<double> weights, Robot* robot);
	~ObjectiveSum();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	int m_numOfJoints;
};

