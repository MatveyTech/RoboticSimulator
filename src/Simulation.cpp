#include "..\include\Simulation.h"
#include "..\include\ObjectiveSum.h"
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
	cout << endl;
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

void Simulation::MakeStep()
{

}



BasicSimulation::BasicSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints)
	:Simulation(startPoint, endPoint, numOfPoints)
{
	VectorXd step = (endPoint - startPoint) / (NumOfPoints - 1);
	path = VectorXd(NumOfJoints*NumOfPoints);
	for (size_t i = 0; i < NumOfPoints; i++)
	{
		VectorXd roww(NumOfJoints);
		for (size_t j = 0; j < NumOfJoints; j++)
		{
			path(i*NumOfJoints + j) = startPoint(j) + step(j)*(i);
		}
	}
}


//void BasicSimulation::CalculatePath(VectorXd startPoint, VectorXd endPoint)
//{
//}

AdvancedSimulation::AdvancedSimulation(VectorXd startPoint, VectorXd endPoint, int numOfPoints, std::vector<double> weights,
	int mt, Robot* robot, P3D finalCart, bool onlyFinalCart)
	:Simulation(startPoint, endPoint, numOfPoints)
{
	
	m_objective = new ObjectiveSum(startPoint, endPoint, weights, robot,finalCart,onlyFinalCart);
	MinimizerType = mt == 0 ? MinimizerType::GD : MinimizerType::BFGS;
	VectorXd pp(NumOfJoints*NumOfPoints);
	for (size_t i = 0; i < pp.size(); i++)
	{
		pp(i) = 20;
	}
	if (MinimizerType == MinimizerType::GD)
		m_gradientBasedMinimizer = new GradientDescentFunctionMinimizer(1);
	else if (MinimizerType == MinimizerType::BFGS)
		m_gradientBasedMinimizer = new BFGSFunctionMinimizer(1);
	path = pp;
	//printVector(path);
}

double AdvancedSimulation::ComputeValueInCurrentPoint()
{
	return m_objective->computeValue(path);
}

double AdvancedSimulation::ComputeGradientInCurrentPoint()
{
	VectorXd grad(NumOfJoints*NumOfPoints);
	for (size_t i = 0; i < grad.size(); i++)
		grad(i) = 0;
	m_objective->addGradientTo(grad, path);
	return grad.norm();
}

int AdvancedSimulation::GetLastNumOfIterations()
{
	return m_gradientBasedMinimizer->LastNumOfIterations;
}

void AdvancedSimulation::MakeStep()
{
	double res;
	IterationNum++;
	m_gradientBasedMinimizer->minimize(m_objective, path, res);
}



//void AdvancedSimulation::CalculatePath(VectorXd startPoint, VectorXd endPoint)
//{
//}
