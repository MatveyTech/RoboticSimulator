#pragma once

#include "../include/ObjectiveFunction.h"
#include "../include/Timer.h"
#include "../include/GradientBasedFunctionMinimizer.h"

/**
	use Newton's method to optimize a function. p will store the final value that minimizes the function, and its initial value
	is used to start the optimization method.

	Task: find p that minimize f(p). This means that df/dp(p) = 0.
	df/dp(p+dp) ~ df/dp(p) + d/dp(df/dp) * dp = 0 ==> -df/dp(p) = d/dp(df/dp) * dp
	Iterating the above, will hopefully get p that minimizes f.
*/
class NewtonFunctionMinimizer : public GradientBasedFunctionMinimizer {
public:
	NewtonFunctionMinimizer(int p_maxIterations = 100, double p_solveResidual = 0.0001, int p_maxLineSearchIterations = 15, bool p_printOutput = false) : GradientBasedFunctionMinimizer(p_maxIterations, p_solveResidual, p_maxLineSearchIterations, p_printOutput) {
		optName = "Newton\'s method";
	}

	// TODO: ambiguos call
	//NewtonFunctionMinimizer() {
	//	optName = "Newton\'s method";
	//}

	virtual ~NewtonFunctionMinimizer() {}

	// The search direction is given by -Hinv * g
	virtual void computeSearchDirection(ObjectiveFunction *function, const dVector &p, dVector& dp);

public:
	Timer timerN;
	ESparseMatrix H;
	DynamicArray<MTriplet> hessianEntries;

	int nMaxStabSteps = 10;		// maximum number of stabilization steps
	double stabValue = 1e-4;	// value that gets added to hessian diagonal during stabilization step
	enum HessCorrectionMethod {
		None = 0,
		DynamicRegularization,
		Projection
	};
	HessCorrectionMethod hessCorrectionMethod = None;
};
