#pragma once

#include "ObjectiveFunction.h"
#include "Minimizer.h"
#include <OptLogger.h>

class GradientDescentFixedStep : public Minimizer {
public:
	GradientDescentFixedStep(int maxIterations=200, double solveResidual=1e-5)
		: maxIterations(maxIterations), solveResidual(solveResidual) {
	}

	int getLastIterations() { return lastIterations; }

	// Main minimize function entry point
	bool minimize(const ObjectiveFunction *function, VectorXd &x, bool log) const override {

		bool optimizationConverged = false;

		VectorXd dx(x.size());
		
		int i=0;
		for(; i < maxIterations; i++) {
			// Compute dx (serach direction) dx = function->getGradient(x);
			dx.setZero();
			computeSearchDirection(function, x, dx);
			//std::cout << "GradientDescentFixedStep "<< i << " Residual Energy:  " << dx.norm() << endl;

			//Determine if residual energy is converged
			//if (dx.norm() < solveResidual){
			//	std::cout << "dx.norm() < solveResidual, Optimization Converged" << dx.norm() << endl;
			//	optimizationConverged = true;
			//	break;
			//}

			//Perform one step function to update x: x -= stepSize * dx;
			step(function, dx, x);

			//Determine if residual energy is converged
			if (function->evaluate(x) < solveResidual) {
				//std::cout << "dx.norm() < solveResidual, Optimization Converged" << function->evaluate(x) << " < " << solveResidual<< endl;
				optimizationConverged = true;
				break;
			}

			function->eachConstraintReport();

			//Log the value of x
			if (log) OptLogger::log(x);
			
		}

		lastIterations = i;
		//std::cout << "Minimization ran for " << i << " steps, Residual = " << function->evaluate(x) << endl;
		return optimizationConverged;
	}

	int minimize_stat(const ObjectiveFunction *function, VectorXd &x, bool log) const  {

		bool optimizationConverged = false;

		VectorXd dx(x.size());

		int i = 0;
		for (; i < maxIterations; i++) {
			// Compute dx (serach direction) dx = function->getGradient(x);
			dx.setZero();
			computeSearchDirection(function, x, dx);
			//std::cout << "GradientDescentFixedStep "<< i << " Residual Energy:  " << dx.norm() << endl;

			//Perform one step function to update x: x -= stepSize * dx;
			step(function, dx, x);

			//Determine if residual energy is converged
			if (function->evaluate(x) < solveResidual) {
				//std::cout << "dx.norm() < solveResidual, Optimization Converged" << function->evaluate(x) << " < " << solveResidual<< endl;
				optimizationConverged = true;
				break;
			}

			function->eachConstraintReport();

			//Log the value of x
			if (log) OptLogger::log(x);

		}

		lastIterations = i;
		//std::cout << "Minimization ran for " << i << " steps, Residual = " << function->evaluate(x) << endl;
		return lastIterations; //  optimizationConverged;
	}

protected:
	virtual void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) const {
		dx = function->getGradient(x);
	}

	// Given the objective `function` and the search direction `x`, update the candidate `x`
	virtual void step(const ObjectiveFunction *function, const VectorXd& dx, VectorXd& x) const	{
		// for fixed-step gradient descent, the step size is independent of the objective function
		x -= stepSize * dx;
		std::cout << "GradientDescentFixedStep step Function Here" << endl;
	}

public:
	double solveResidual = 1e-5;
	int maxIterations = 1;
	double stepSize = 0.001;

	// some stats about the last time `minimize` was called
	mutable int lastIterations = -1;
};


class GradientDescentVariableStep : public GradientDescentFixedStep {
public:
	GradientDescentVariableStep(int maxIterations=500, double solveResidual=1e-5, int maxLineSearchIterations=30)
		: GradientDescentFixedStep (maxIterations, solveResidual), maxLineSearchIterations(maxLineSearchIterations){
	}

protected:
	void step(const ObjectiveFunction *function, const VectorXd& dx, VectorXd& x) const override
	{
		//std::cout << "GradientDescentVariableStep step Function Here" << endl;
		// line search
		double alpha = 1.0; // initial step size
		VectorXd xc(x);
		double initialValue = function->evaluate(xc);

		for(int j = 0; j < maxLineSearchIterations; j++) {

			// let's take a step of size `alpha`
			x = xc - dx * alpha;

			// if the new function value is greater than initial,
			// we want to reduce alpha and take a smaller step
			if(function->evaluate(x) > initialValue)
				alpha /= 2.0;
			else
				return;
		}

	}

protected:
	int maxLineSearchIterations = 50;
};

class GradientDescentMomentum : public GradientDescentVariableStep {
public:
	GradientDescentMomentum(int maxIterations=200, double solveResidual=1e-5, int maxLineSearchIterations=15)
		: GradientDescentVariableStep(maxIterations, solveResidual, maxLineSearchIterations){
	}


protected:
	void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) const override {

		// if gradient hasn't been set yet, set it to zero
		if(gradient.size() == 0){
			gradient.resize(x.size());
			gradient.setZero();
		}
		// compute new gradient
		VectorXd newGradient = function->getGradient(x);
		// search direction is augmented with old gradient
		dx = newGradient + alpha*gradient;
		// save gradient for next step
		gradient = newGradient;
		std::cout << "Momentum Function Here" << endl;
	}

public:
	double alpha = 0.5;
	mutable VectorXd gradient;
};
