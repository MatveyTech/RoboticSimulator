#pragma once
#include "ObjectiveFunction.h"
#include "..\include\Robot.h"
#include "..\include\CollisionObjective.h"
#include <vector>
using namespace Eigen;

class ObjectiveSum : public ObjectiveFunction
{	
	std::vector<ObjectiveFunction*> objectives;
	CollisionObjective* m_collisionObjective = nullptr;

public:
	ObjectiveSum(const VectorXd& startPos, const VectorXd& finalPos,std::vector<int> weights, Robot* robot, 
		P3D finalCart, bool onlyFinalCart, std::vector<CollisionSphere> obstacles);
	~ObjectiveSum();

	CollisionObjective* GetCollisionObjective() { return m_collisionObjective; }

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);

private:
	int m_numOfJoints;
};

