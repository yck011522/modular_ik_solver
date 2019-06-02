#pragma once

#include <utility>
#include "RigidBodySimulation.h"
#include <RandomMinimizer.h>
//#include <RandomDecendMinimizer.h>

#include <iostream>
using namespace std;

class MatchTrajectoryObjective : public ObjectiveFunction
{
public:
    MatchTrajectoryObjective(RigidBodySimulation sim, Matrix<double, -1, 2> targetPath)
        : sim(std::move(sim)), targetTrajectory(std::move(targetPath)) {}

     double evaluate(const VectorXd& p) const override {

         ////////////////////////
         // Task 2.2
         //return 0; // return the correct value!
         ////////////////////////

		 sim.setDesignParameters(p);
		 //cout << "p: " << p << endl;
		 Matrix<double, -1, 2> simPath = sim.recordTrajectory();

		 //cout << "simPath: " << simPath << endl;
		 //cout << "targetTrajectory: " << targetTrajectory << endl;
		 //cout << ".squaredNorm(): " << (simPath - targetTrajectory).squaredNorm() << endl;

		 // return (simPath - targetTrajectory).squaredNorm(); // This is not correct
		 return (simPath - targetTrajectory).rowwise().squaredNorm().sum();
	}

public:
	mutable RigidBodySimulation sim;
    Matrix<double, -1, 2> targetTrajectory;
};

class MechanismOptimizer
{
public:
	MechanismOptimizer() = default;

	MechanismOptimizer(const RigidBodySimulation &sim) : sim(sim) {
		p = sim.getDesignParameters();
	}

    void optimizeTrajectory() {
        MatchTrajectoryObjective obj(sim, targetPath);
        minimizer.searchDomainMin = 0.98 * p;
        minimizer.searchDomainMax = 1.02 * p;
		minimizer.minimize(&obj, p, false);
	}

public:
	RigidBodySimulation sim;
	Matrix<double, -1, 2> targetPath;

	RandomMinimizer minimizer;
	//RandomDecendMinimizer minimizer;
	VectorXd p;

};
