#include "..\include\SmoothnessObjective.h"
#include "..\include\StartFromFirstObjective.h"
#include "..\include\FinishAtLastObjective.h"
#include "..\include\CloseToPointObjective.h"
#include "..\include\ObjectiveSum.h"
#include <vector>
#include <stdio.h>




void ObjectiveSum::UpdateWeights(std::vector<int> weights)
{
	for (int i=0; i<objectives.size();++i)
	{
		objectives.at(i)->setWeight(weights.at(i));
	}
}


double ObjectiveSum::computeValue(const dVector & p)
{
	double res=0;
	for (ObjectiveFunction* objective : objectives)
		res += objective->computeValue(p);
	return res;
}

void ObjectiveSum::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector & p)
{
	for (ObjectiveFunction* objective : objectives)
	{
		objective->addHessianEntriesTo(hessianEntries, p);
	}
}

void ObjectiveSum::addGradientTo(dVector & grad, const dVector & p)
{
	for (ObjectiveFunction* objective : objectives)
	{
		objective->addGradientTo(grad, p);
	}
}

void ObjectiveSum::setWeight(double w)
{
	for (int i = 0; i<objectives.size(); ++i)
	{
		objectives.at(i)->setWeight(w);
	}
}

//template<class T>
ObjectiveFunction *ObjectiveSum::GetObjective(int i)
{
	return objectives.at(i);
	/*for (auto obj : objectives)
	{
		if (typeid(obj) == typeid(T))
			return obj;
	}*/
}

PathObjectivesSum::PathObjectivesSum(const VectorXd& startPos, const VectorXd& finalPos, int numOfPoints, std::vector<int> weights, Robot* robot,
	P3D finalCart, bool onlyFinalCart, std::vector<CollisionSphere*> obstacles)
{
	int numOfJoints = startPos.rows();

	objectives.push_back(new StartFromFirstObjective(startPos, weights.at(0)));
	objectives.push_back(new FinishAtLastObjective(finalPos, weights.at(1)));
	objectives.push_back(new SmoothnessObjective(numOfJoints, numOfPoints, weights.at(2)));
	objectives.push_back(new CloseToPointObjective(numOfJoints, weights.at(3), finalCart, robot));
	//CollisionSphere* cs = obstacles.front();
	//objectives.push_back(new CollisionObjective(numOfJoints, weights.at(4), cs->Location, cs->Radius, robot));
	objectives.push_back(new CollisionObjectivesSum(numOfJoints, weights.at(4), obstacles, robot));
}

PathObjectivesSum::~PathObjectivesSum()
{
	//delete all objectives
}

CollisionObjectivesSum::CollisionObjectivesSum(int numOfJoints, int weight, std::vector<CollisionSphere*> obstacles, Robot * robot)
{
	for (auto obst : obstacles)
	{
		objectives.push_back(new CollisionObjective(numOfJoints, weight, obst->Location, obst->Radius, robot));
	}
}
