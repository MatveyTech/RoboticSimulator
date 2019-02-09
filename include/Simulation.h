#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class Simulation
{
	int CurrentIndex = 0;
	MatrixXd path;

public:
	Simulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints);

	void CalculatePath(VectorXd startPoint, VectorXd endPoint, int numOfPoints);
	VectorXd GetCurrent();
	VectorXd MoveToNextAndGet();
	VectorXd MoveToPrevAndGet();
	void Reset();
};

