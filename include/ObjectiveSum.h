#pragma once
#include "ObjectiveFunction.h"
#include "..\include\Robot.h"
#include <vector>
using namespace Eigen;

class ObjectiveSum : public ObjectiveFunction
{	
	std::vector<ObjectiveFunction*> objectives;
public:
	ObjectiveSum(const VectorXd& startPos, const VectorXd& finalPos,std::vector<double> weights, Robot* robot, 
		P3D finalCart, bool onlyFinalCart, std::vector<CollisionSphere> obstacles);
	~ObjectiveSum();

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	int m_numOfJoints;
};

