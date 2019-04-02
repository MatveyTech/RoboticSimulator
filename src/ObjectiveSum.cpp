#include "..\include\EqualDistributionObjective.h"
#include "..\include\StartFromFirstObjective.h"
#include "..\include\FinishAtLastObjective.h"
#include "..\include\CollisionObjective.h"
#include "..\include\ObjectiveSum.h"


ObjectiveSum::ObjectiveSum(const VectorXd& startPos, const VectorXd& finalPos, std::vector<double> weights, Robot* robot, P3D finalCart, bool onlyFinalCart)
{
	if (onlyFinalCart)
	{
		objectives.push_back(new CollisionObjective(startPos.rows(), 1.0, finalCart, robot));
	}
	else
	{
		objectives.push_back(new StartFromFirstObjective(startPos, weights.at(0)));
		objectives.push_back(new FinishAtLastObjective(finalPos, weights.at(1)));
		objectives.push_back(new EqualDistributionObjective(startPos.rows()));//weights.at(2)
		//objectives.push_back(new CollisionObjective(startPos.rows(), 1.0, finalCart, robot));
	}
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