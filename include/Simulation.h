#pragma once

#include <Eigen/Dense>
#include "..\include\GradientDescentFunctionMinimizer.h"
#include "..\include\BFGSFunctionMinimizer.h"
#include "..\include\ObjectiveFunction.h"
#include "..\include\ObjectiveSum.h"
#include "..\include\CollisionObjective.h"
#include "..\include\Robot.h"
#include "..\include\DraggableSphere.h"

using namespace Eigen;

enum MinimizerType { GD, BFGS };

class Simulation
{
protected:
	
	int NumOfJoints;
	int NumOfPoints;
	
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

	VectorXd path;
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
	ObjectiveSum* m_objective;
	GradientBasedFunctionMinimizer* m_gradientBasedMinimizer;
	
	//void CalculatePath(VectorXd startPoint, VectorXd endPoint);
public:
	int IterationNum = 0;
	MinimizerType MinimizerType;// = MinimizerType::GD;
	void MakeStep();
	AdvancedSimulation(VectorXd& startPoint, VectorXd& endPoint, int numOfPoints, std::vector<int> weights, 
		int mt,Robot* robot, P3D finalCart, bool onlyFinalCart,std::vector<CollisionSphere*> obstacles);
	double ComputeValueAll();
	double ComputeGradientAll();
	double ComputeValueCurrent();
	double ComputeGradientCurrent();
	int GetLastNumOfIterations();
	void testGradient(int i);
};



