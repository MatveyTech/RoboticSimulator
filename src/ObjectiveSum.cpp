#include "..\include\EqualDistributionObjective.h"
#include "..\include\StartFromFirstObjective.h"
#include "..\include\FinishAtLastObjective.h"
#include "..\include\CloseToPointObjective.h"
#include "..\include\ObjectiveSum.h"
#include <vector>


ObjectiveSum::ObjectiveSum(const VectorXd& startPos, const VectorXd& finalPos, std::vector<int> weights, Robot* robot, 
	P3D finalCart, bool onlyFinalCart,std::vector<CollisionSphere*> obstacles)
{
	int numOfJoints = startPos.rows();
	
	objectives.push_back(new StartFromFirstObjective(startPos, weights.at(0)));
	objectives.push_back(new FinishAtLastObjective(finalPos, weights.at(1)));
	objectives.push_back(new EqualDistributionObjective(numOfJoints, weights.at(2)));
	objectives.push_back(new CloseToPointObjective(numOfJoints, weights.at(3), finalCart, robot));
	CollisionSphere* cs = obstacles.front();
	m_collisionObjective = new CollisionObjective(numOfJoints, weights.at(4), cs->Location, cs->Radius, robot);
	objectives.push_back(m_collisionObjective);
}


ObjectiveSum::~ObjectiveSum()
{
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
}

void ObjectiveSum::addGradientTo(dVector & grad, const dVector & p)
{
	for (ObjectiveFunction* objective : objectives)
		objective->addGradientTo(grad,p);
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
