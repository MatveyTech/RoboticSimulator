#pragma once

#include <Eigen/Dense>
#include "..\include\GradientDescentFunctionMinimizer.h"
#include "..\include\ObjectiveFunction.h"

using namespace Eigen;

class Simulation
{
protected:
	
	int NumOfJoints;
	int NumOfPoints;
	VectorXd path;
	//virtual void CalculatePath(VectorXd startPoint, VectorXd endPoint) = 0;

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
	virtual void MakeStep();
};

class BasicSimulation : public Simulation
{
protected:
	//void CalculatePath(VectorXd startPoint, VectorXd endPoint);

public:
	BasicSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints);
};


class AdvancedSimulation : public Simulation
{
protected:
	ObjectiveFunction* m_objective;
	GradientDescentFunctionMinimizer m_gradientDMinimizer;
	//void CalculatePath(VectorXd startPoint, VectorXd endPoint);
public:
	int IterationNum = 0;
	void MakeStep();
	AdvancedSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints,double weight);
	double ComputeValueInCurrentPoint();
	double ComputeGradientInCurrentPoint();
	int GetLastNumOfIterations();
};



