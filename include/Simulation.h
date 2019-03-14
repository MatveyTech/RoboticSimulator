#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class Simulation
{
protected:
	int CurrentIndex = 0;
	MatrixXd path;

	virtual void CalculatePath(VectorXd startPoint, VectorXd endPoint, int numOfPoints) = 0;

public:
	
	VectorXd GetCurrent();
	VectorXd MoveToNextAndGet();
	VectorXd MoveToIndAndGet(int i);
	VectorXd MoveToPrevAndGet();
	void Reset();
};

class BasicSimulation : public Simulation
{
protected:
	void CalculatePath(VectorXd startPoint, VectorXd endPoint, int numOfPoints);

public:
	BasicSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints);
};


class AdvancedSimulation : public Simulation
{
protected:
	void CalculatePath(VectorXd startPoint, VectorXd endPoint, int numOfPoints);
public:
	AdvancedSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints);
	
};



