#pragma once

#include <RigidBody3D.h>
#include <Joint3D.h>

#include <vector>

#include <ObjectiveFunction.h>

class KinematicEnergy3D : public ObjectiveFunction
{
public:
	virtual double evaluate(const VectorXd& x) const {
		VectorXd c = computeConstraints(x);
		return 0.5 * c.dot(c); //0.5 * ||C||
	}

	virtual void addGradientTo(const VectorXd& x, VectorXd& grad) const {
		grad += computeJacobian(x).transpose() * computeConstraints(x);
	}

	virtual void addHessianEntriesTo(const VectorXd& x, std::vector<Triplet<double>>& hessianEntries) const {
		MatrixXd hess = computeJacobian(x).transpose() * computeJacobian(x);
		for (int i = 0; i < hess.rows(); ++i) {
			for (int j = 0; j < hess.cols(); ++j) {
				hessianEntries.push_back({i, j, hess(i, j)}); 
			}
		}
	}

	VectorXd computeConstraints(const VectorXd &x) const {
		VectorXd c(getNumConstraints());

		int i = 0;  // keep track of how many constraints were processed

		for (const FixedBody3Pt &j : FixedBody3Pts) {
			c.segment<j.NumConstraints>(i) = j.computeConstraints(x, rigidbodies);
			i += j.NumConstraints; 
		}
		
		for (const HingeJoint2Pt &j : HingeJoint2Pts) {
			c.segment<j.NumConstraints>(i) = j.computeConstraints(x, rigidbodies);
			i += j.NumConstraints;
		}

		for (const PointOnLineJoint &j : PointOnLineJoints) {
			c.segment<j.NumConstraints>(i) = j.computeConstraints(x, rigidbodies);
			i += j.NumConstraints;
		}

		for(const FixedJoint3Pt &j : FixedJoint3Pts) {
			c.segment<j.NumConstraints>(i) = j.computeConstraints(x, rigidbodies);
			i += j.NumConstraints; 
		}

		for (const ProgressiveTarget3Pt &t : Targets3Pt) {
			c.segment<t.NumConstraints>(i) = t.computeConstraints(x, rigidbodies);
			i += t.NumConstraints; 
		}
		return c;
	}

	virtual void eachConstraintReport(const VectorXd &x) const {

		for (const FixedBody3Pt &j : FixedBody3Pts) {
			VectorXd c = j.computeConstraints(x, rigidbodies);
			std::cout << "FixedBody3Pt Energy=" << .5 * c.dot(c) << endl;
			std::cout << c << endl;
		}

		for (const HingeJoint2Pt &j : HingeJoint2Pts) {
			VectorXd c = j.computeConstraints(x, rigidbodies);
			std::cout << "HingeJoint2Pt Energy=" << .5 * c.dot(c) << endl;
			std::cout << c << endl;
		}

		for (const PointOnLineJoint &j : PointOnLineJoints) {
			VectorXd c = j.computeConstraints(x, rigidbodies);
			std::cout << "PointOnLineJoint Energy=" << .5 * c.dot(c) << endl;
			std::cout << c << endl;
		}

		for (const FixedJoint3Pt &j : FixedJoint3Pts) {
			VectorXd c = j.computeConstraints(x, rigidbodies);
			std::cout << "FixedJoint3Pt Energy=" << .5 * c.dot(c) << endl;
			std::cout << c << endl;
		}

		for (const ProgressiveTarget3Pt &t : Targets3Pt) {
			VectorXd c = t.computeConstraints(x, rigidbodies);
			std::cout << "Targets3Pt Energy=" << .5 * c.dot(c) << endl;
			std::cout << c << endl;
		}
	}

	//TODO: Add the new joint
	MatrixXd computeJacobian(const VectorXd &x) const {
		MatrixXd dCdx(getNumConstraints(), x.size()); 
		// Jacobian Configurations
		// Vertical Dimension: Constraints
		// Horizontal Dimension: State x size
		dCdx.setZero();

        int i = 0; // keep track of how many constraints were processed

		for (const FixedBody3Pt &j : FixedBody3Pts) {
			Matrix<double, j.NumConstraints, 6> jac = j.computeJacobian(x, rigidbodies);
			//Matrix<double, j.NumConstraints, 6> jac = j.computeJacobian(x, rigidbodies[j.rb0Idx]);
			//<NumConstraint,NumStateSize>
			dCdx.block<j.NumConstraints, 6>(i, j.rb0Idx*6) = jac.block<j.NumConstraints, 6>(0, 0);
			i += j.NumConstraints; // a FixedBody3Pt constraint is of size 9
		}

		for (const HingeJoint2Pt &j : HingeJoint2Pts) {
			Matrix<double, j.NumConstraints, 12> jac = j.computeJacobian(x, rigidbodies);
			//Matrix<double, j.NumConstraints, 12> jac = j.computeJacobian(x, rigidbodies[j.rb0Idx], rigidbodies[j.rb1Idx]);
			//<NumConstraint,NumStateSize>
			dCdx.block<j.NumConstraints, 6>(i, j.rb0Idx * 6) = jac.block<j.NumConstraints, 6>(0, 0);
			dCdx.block<j.NumConstraints, 6>(i, j.rb1Idx * 6) = jac.block<j.NumConstraints, 6>(0, 6);
			i += j.NumConstraints; // a HingeJoint2Pt constraint is of size 6
		}

		for (const PointOnLineJoint &j : PointOnLineJoints) {
			Matrix<double, j.NumConstraints, 12> jac = j.computeJacobian(x, rigidbodies);
			//<NumConstraint,NumStateSize>
			dCdx.block<j.NumConstraints, 6>(i, j.rb0Idx * 6) = jac.block<j.NumConstraints, 6>(0, 0);
			dCdx.block<j.NumConstraints, 6>(i, j.rb1Idx * 6) = jac.block<j.NumConstraints, 6>(0, 6);
			i += j.NumConstraints; // a PointOnLineJoint joint constraint is of size 1 (only)
		}

		for (const FixedJoint3Pt &j : FixedJoint3Pts) {
			Matrix<double, j.NumConstraints, 12> jac = j.computeJacobian(x, rigidbodies);
			//Matrix<double, j.NumConstraints, 12> jac = j.computeJacobian(x, rigidbodies[j.rb0Idx], rigidbodies[j.rb1Idx]);
			//<NumConstraint,NumStateSize>
			dCdx.block<j.NumConstraints, 6>(i, j.rb0Idx*6) = jac.block<j.NumConstraints, 6>(0, 0);
			dCdx.block<j.NumConstraints, 6>(i, j.rb1Idx*6) = jac.block<j.NumConstraints, 6>(0, 6);
			i += j.NumConstraints; // a FixedJoint3Pt constraint is of size 9
		}

		for (const ProgressiveTarget3Pt &t : Targets3Pt) {
			Matrix<double, t.NumConstraints, 6> jac = t.computeJacobian(x, rigidbodies);
			dCdx.block<t.NumConstraints, 6>(i, t.rb0Idx * 6) = jac.block<t.NumConstraints, 6>(0, 0);
			i += t.NumConstraints; // a ProgressiveTarget3Pt constraint is of size 9
		}

		return dCdx;
	}

	int getNumConstraints() const {
		int numOfConstraints = 0;
		numOfConstraints += HingeJoint2Pts.size() * HingeJoint2Pt::NumConstraints;
		numOfConstraints += PointOnLineJoints.size() * PointOnLineJoint::NumConstraints;
		numOfConstraints += FixedJoint3Pts.size() * FixedJoint3Pt::NumConstraints;
		numOfConstraints += FixedBody3Pts.size() * FixedBody3Pt::NumConstraints;
		numOfConstraints += Targets3Pt.size() * ProgressiveTarget3Pt::NumConstraints;
		return numOfConstraints;
	}

public:
	std::vector<RigidBody3D> rigidbodies;
	std::vector<HingeJoint2Pt> HingeJoint2Pts;
	std::vector<PointOnLineJoint> PointOnLineJoints;
	std::vector<FixedJoint3Pt> FixedJoint3Pts;
	std::vector<FixedBody3Pt> FixedBody3Pts;
	std::vector<ProgressiveTarget3Pt> Targets3Pt;

};
