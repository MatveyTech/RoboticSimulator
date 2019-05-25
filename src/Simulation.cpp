#include "..\include\Simulation.h"
#include "..\include\ObjectiveSum.h"
#include "..\include\CloseToPointObjective.h"
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

AdvancedSimulation::AdvancedSimulation(VectorXd& startPoint, VectorXd& endPoint, int numOfPoints, std::vector<int> weights,
	int mt, Robot* robot, P3D finalCart, bool onlyFinalCart, vector<CollisionSphere*> obstacles)
	:Simulation(startPoint, endPoint, numOfPoints)
{
	
	m_objective = new ObjectiveSum(startPoint, endPoint, numOfPoints, weights, robot,finalCart,onlyFinalCart, obstacles);
	MinimizerType = mt == 0 ? MinimizerType::GD : mt == 1 ? MinimizerType::BFGS : MinimizerType::NW;
	VectorXd pp(NumOfJoints*NumOfPoints);
	pp.setConstant(RAD(20));
	if (MinimizerType == MinimizerType::GD)
		m_gradientBasedMinimizer = new GradientDescentFunctionMinimizer(1);
	else if (MinimizerType == MinimizerType::BFGS)
		m_gradientBasedMinimizer = new BFGSFunctionMinimizer(1);
	else if (MinimizerType == MinimizerType::NW)
		m_gradientBasedMinimizer = new NewtonFunctionMinimizer(1);
	path = pp;
	//printVector(path);
}

double AdvancedSimulation::ComputeValueAll()
{
	return m_objective->computeValue(path);
}

double AdvancedSimulation::ComputeGradientAll()
{
	VectorXd grad(NumOfJoints*NumOfPoints);
	for (size_t i = 0; i < grad.size(); i++)
		grad(i) = 0;
	m_objective->addGradientTo(grad, path);
	return grad.norm();
}

double AdvancedSimulation::ComputeValueCurrent()
{
	//return m_objective->computeValue(GetCurrent());
	return m_objective->GetCollisionObjective()->computeValue(GetCurrent());
}

double AdvancedSimulation::ComputeGradientCurrent()
{
	VectorXd grad(7);
	grad.setZero();	
	//m_objective->addGradientTo(grad, GetCurrent());
	m_objective->GetCollisionObjective()->computeValue(GetCurrent());
	return grad.norm();
}

int AdvancedSimulation::GetLastNumOfIterations()
{
	return m_gradientBasedMinimizer->LastNumOfIterations;
}

void AdvancedSimulation::testGradient(int i)
{
	if (i==0)
		m_objective->testGradientWithFD(path);
	else
	{
		ObjectiveFunction* d = m_objective->GetObjective(i-1);
		d->testGradientWithFD(path);
	}

}

void AdvancedSimulation::TestHessian(int i)
{
	if (i == 0)
		m_objective->testHessianWithFD(path);
	else
	{
		ObjectiveFunction* d = m_objective->GetObjective(i - 1);
		d->testHessianWithFD(path);
	}
}

void AdvancedSimulation::UpdateCloseToPoint(P3D point)
{
	CloseToPointObjective* d = (CloseToPointObjective*)m_objective->GetObjective(3);//bad.To be solved
	d->UpdatePoint(point);
}

void AdvancedSimulation::UpdateWeights(std::vector<int> weights)
{
	m_objective->UpdateWeights(weights);
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
