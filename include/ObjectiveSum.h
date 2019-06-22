#pragma once
#include "ObjectiveFunction.h"
#include "..\include\Robot.h"
#include "..\include\CollisionObjective.h"
#include "..\include\DraggableSphere.h"
#include <vector>
using namespace Eigen;

class ObjectiveSum : public ObjectiveFunction
{	
protected:
	std::vector<ObjectiveFunction*> objectives;	

public:	
	
	void UpdateWeights(std::vector<int> weights);
	//template <class T>
	ObjectiveFunction* GetObjective(int i);
	

	virtual double computeValue(const dVector& p);
	virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);
	virtual void setWeight(double w);


private:
	int m_numOfJoints;
};

class PathObjectivesSum : public ObjectiveSum
{
public:
	PathObjectivesSum(const VectorXd& startPos, const VectorXd& finalPos, int numOfPoints, std::vector<int> weights, Robot* robot,
		P3D finalCart, bool onlyFinalCart, std::vector<CollisionSphere*> obstacles);
	~PathObjectivesSum();
};




class CollisionObjectivesSum : public ObjectiveSum
{
public:
	CollisionObjectivesSum(int numOfJoints, int numOfPoints, int weight, std::vector<CollisionSphere*> obstacles, Robot* robot);
};

