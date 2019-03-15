#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class Simulation
{
protected:
	
	int NumOfJoints;
	int NumOfPoints;
	VectorXd path;
	virtual void CalculatePath(VectorXd startPoint, VectorXd endPoint) = 0;

public:
	Simulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints);
	int CurrentIndex = 0;
	VectorXd GetCurrent();
	int IncreaseCurrentIndex();
	int DecreaseCurrentIndex();
	VectorXd MoveToNextAndGet();
	VectorXd MoveToIndAndGet(int i);
	VectorXd MoveToPrevAndGet();
	void Reset();
};

class BasicSimulation : public Simulation
{
protected:
	void CalculatePath(VectorXd startPoint, VectorXd endPoint);

public:
	BasicSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints);
};


class AdvancedSimulation : public Simulation
{
protected:
	void CalculatePath(VectorXd startPoint, VectorXd endPoint);
public:
	AdvancedSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints);
	
};



