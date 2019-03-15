#include "..\include\Simulation.h"
#include "..\include\ObjectiveFunction.h"
#include "..\include\EqualDistributionObjective.h"
#include "..\include\GradientDescentFunctionMinimizer.h"
#include <iostream>

using namespace std;

void printVector(VectorXd pp, int JointsNum = 7)
{
	for (size_t i = 0; i < pp.size(); i++)
	{
		cout << pp(i) << " ";
		if (i % JointsNum == JointsNum - 1)
			cout << endl;
	}
}

void Simulation::CalculatePath(VectorXd startPoint, VectorXd endPoint, int numOfPoints)
{
	path.resize(numOfPoints, startPoint.size());
	VectorXd step = (endPoint - startPoint) / (numOfPoints - 1);
	int numOfJoints = startPoint.size();
	for (size_t i = 0; i < numOfPoints; i++)
	{
		VectorXd roww(numOfJoints);
		for (size_t j = 0; j < numOfJoints; j++)
		{
			roww(j) = startPoint(j) + step(j)*(i);
		}
		path.row(i) = roww;
	}
}

VectorXd Simulation::GetCurrent()
{
	return path.row(CurrentIndex);
}

int Simulation::IncreaseCurrentIndex()
{
	if (CurrentIndex < path.rows() - 1)
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
{
	CalculatePath(startPoint, endPoint, numOfPoints);
}


void BasicSimulation::CalculatePath(VectorXd startPoint, VectorXd endPoint, int numOfPoints)
{
	path.resize(numOfPoints, startPoint.size());
	VectorXd step = (endPoint - startPoint) / (numOfPoints - 1);
	int numOfJoints = startPoint.size();
	for (size_t i = 0; i < numOfPoints; i++)
	{
		VectorXd roww(numOfJoints);
		for (size_t j = 0; j < numOfJoints; j++)
		{
			roww(j) = startPoint(j) + step(j)*(i);
		}
		path.row(i) = roww;
	}
	printVector(path);
}

AdvancedSimulation::AdvancedSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints)
{
	CalculatePath(startPoint, endPoint, numOfPoints);
}

void AdvancedSimulation::CalculatePath(VectorXd startPoint, VectorXd endPoint, int numOfPoints)
{
	int numOfJoints = 7;	

	VectorXd pp(numOfJoints*numOfPoints);
	for (size_t i = 0; i < pp.size(); i++)
	{
		pp(i) = 20;
	}

	ObjectiveFunction* eo = new Basic3(startPoint, endPoint);
	
	printVector(pp, numOfJoints);

	cout << endl << endl;
	double res;
	GradientDescentFunctionMinimizer gm;
	gm.minimize(eo, pp, res);

	printVector(pp, numOfJoints);
}
