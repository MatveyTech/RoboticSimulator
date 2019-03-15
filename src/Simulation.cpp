#include "..\include\Simulation.h"
#include "..\include\ObjectiveFunction.h"
#include "..\include\EqualDistributionObjective.h"
#include "..\include\GradientDescentFunctionMinimizer.h"
#include <iostream>

using namespace std;

//void printVector(VectorXd pp, int JointsNum = 7)
//{
//	int numOfPoints = pp.size()/JointsNum;
//	
//	int i = 0;
//	while (i < numOfPoints)
//	{
//		int currentIndex = i;
//		while (currentIndex < pp.size())
//		{
//			cout << pp(currentIndex) << " ";
//			currentIndex += numOfPoints;
//		}
//		i++;
//		cout << endl;
//	}
//	cout << endl << endl;
//	return;
//}

void printVector(VectorXd pp, int JointsNum = 7)
{
	for (size_t i = 0; i < pp.size(); i++)
	{
		cout << pp(i) << " ";
		if (i % JointsNum == JointsNum - 1)
			cout << endl;
	}
}



Simulation::Simulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints)
{
	NumOfJoints = startPoint.size();
	NumOfPoints = numOfPoints;
	//CalculatePath(startPoint, endPoint);
}

VectorXd Simulation::GetCurrent()
{
	return path.segment(CurrentIndex*NumOfJoints, NumOfJoints);
}

int Simulation::IncreaseCurrentIndex()
{
	if (CurrentIndex < NumOfPoints - 1)
	{
		CurrentIndex++;
	}
	return CurrentIndex;
}

int Simulation::DecreaseCurrentIndex()
{
	if (CurrentIndex > 0)
	{
		CurrentIndex--;
	}
	return CurrentIndex;
}

VectorXd Simulation::MoveToNextAndGet()
{
	IncreaseCurrentIndex();
	return GetCurrent();
}

VectorXd Simulation::MoveToIndAndGet(int i)
{
	if (CurrentIndex < path.rows()  && CurrentIndex >= 0)
	{
		CurrentIndex = i;
	}
	return GetCurrent();
}

VectorXd Simulation::MoveToPrevAndGet()
{
	DecreaseCurrentIndex();
	return GetCurrent();
}


void Simulation::Reset()
{
	CurrentIndex = 0;
}



BasicSimulation::BasicSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints)
	:Simulation(startPoint, endPoint, numOfPoints)
{
	CalculatePath(startPoint, endPoint);
}


void BasicSimulation::CalculatePath(VectorXd startPoint, VectorXd endPoint)
{
	//path.resize(numOfPoints, startPoint.size());
	VectorXd step = (endPoint - startPoint) / (NumOfPoints - 1);
	path = VectorXd(NumOfJoints*NumOfPoints);
	for (size_t i = 0; i < NumOfPoints; i++)
	{
		VectorXd roww(NumOfJoints);
		for (size_t j = 0; j < NumOfJoints; j++)
		{
			//roww(j) = startPoint(j) + step(j)*(i);
			path(i*NumOfJoints+j) = startPoint(j) + step(j)*(i);
		}
//		path.row(i) = roww;
	}
	printVector(path);
}

AdvancedSimulation::AdvancedSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints)
	:Simulation(startPoint, endPoint, numOfPoints)
{
	CalculatePath(startPoint, endPoint);
}

void AdvancedSimulation::CalculatePath(VectorXd startPoint, VectorXd endPoint)
{
	VectorXd pp(NumOfJoints*NumOfPoints);
	for (size_t i = 0; i < pp.size(); i++)
	{
		pp(i) = 20;
	}

	ObjectiveFunction* eo = new Basic3(startPoint, endPoint);
	double res;
	GradientDescentFunctionMinimizer gm;
	gm.minimize(eo, pp, res);

	path = pp;
	printVector(path);
}
