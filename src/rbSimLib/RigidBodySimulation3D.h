#pragma once

#include "KinematicEnergy3D.h"
#include <NewtonFunctionMinimizer.h>

#include <Eigen/StdVector>


#ifndef RBSIM_BASIC_DEF
#define RBSIM_BASIC_DEF

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix2d)
struct TrackRBPoint {
	int rbIdx = -1;
	Vector2d local;
};

#endif

using Eigen::Matrix;

class RigidBodySimulation3D
{
public:
	void run() {
		// std::cout << "Constraints situation:" << endl;
		// std::cout << energy.computeConstraints(x) << endl;

		//Create Logger
		if (!param_logFile.empty()) {
			OptLogger::begin(param_logFile); 
		}

		stat_finalSteps = stat_progresiveSteps = 0; //reset counter

		//Progressive optimization phase
		newton.maxIterations = param_progresiveSteps;
		newton.solveResidual = param_progressiveEnergyThreshold;
		for (int i = 0; i < 1000; i++) {
			bool endPointsAllReached = true;
			for (ProgressiveTarget3Pt &t : energy.Targets3Pt) {
				if (!t.setProgressiveWorldPt(x, energy.rigidbodies))endPointsAllReached = false;
			}
			if (endPointsAllReached) break;
			stat_progresiveSteps += newton.minimize_stat(&energy, x, !param_logFile.empty());
		}

		//Final optimization phase, final target point is set
		newton.maxIterations = param_finalSteps;
		newton.solveResidual = param_finalEnergyThreshold;
		stat_finalSteps += newton.minimize_stat(&energy, x, !param_logFile.empty());

		if (!param_logFile.empty()) OptLogger::end();
		
		// std::cout << "Constraints situation:" << endl;
		// std::cout << energy.computeConstraints(x) << endl;

	}

	void reset() {
		x = x0;
	}
	
	// for easier access
	const std::vector<RigidBody3D> &rigidbodies() const { return energy.rigidbodies;	}
	std::vector<RigidBody3D> &rigidbodies() { return energy.rigidbodies;	}

	const std::vector<HingeJoint2Pt> &HingeJoint2Pts() const { return energy.HingeJoint2Pts; }
	std::vector<HingeJoint2Pt> &HingeJoint2Pts() { return energy.HingeJoint2Pts; }

	const std::vector<PointOnLineJoint> &PointOnLineJoints() const { return energy.PointOnLineJoints; }
	std::vector<PointOnLineJoint> &PointOnLineJoints() { return energy.PointOnLineJoints; }

	const std::vector<FixedJoint3Pt> &FixedJoint3Pts() const { return energy.FixedJoint3Pts; }
	std::vector<FixedJoint3Pt> &FixedJoint3Pts() { return energy.FixedJoint3Pts; }

	const std::vector<FixedBody3Pt> &FixedBody3Pts() const { return energy.FixedBody3Pts; }
	std::vector<FixedBody3Pt> &FixedBody3Pts() { return energy.FixedBody3Pts; }

	const std::vector<ProgressiveTarget3Pt> &Targets3Pt() const { return energy.Targets3Pt; }
	std::vector<ProgressiveTarget3Pt> &Targets3Pt() { return energy.Targets3Pt; }

public:
	NewtonFunctionMinimizer newton;
	KinematicEnergy3D energy;
	VectorXd x, x0;

	double param_progressiveEnergyThreshold = 0.0001;
	double param_finalEnergyThreshold = 0.000001;
	int param_progresiveSteps = 50;
	int param_finalSteps = 400;
	int param_verbose = 2;
	int stat_progresiveSteps = 0;
	int stat_finalSteps = 0;
	string param_logFile = "";

};
