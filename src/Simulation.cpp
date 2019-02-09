#include "..\include\Simulation.h"

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

VectorXd Simulation::MoveToNextAndGet()
{
	if (CurrentIndex < path.rows() - 1)
	{
		CurrentIndex++;
	}
	return GetCurrent();
}

VectorXd Simulation::MoveToPrevAndGet()
{
	if (CurrentIndex > 0)
	{
		CurrentIndex--;
	}
	return GetCurrent();
}


void Simulation::Reset()
{
	CurrentIndex = 0;
}

Simulation::Simulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints)
{
	CalculatePath(startPoint, endPoint, numOfPoints);
}