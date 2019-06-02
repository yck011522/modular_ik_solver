#pragma once

#include "RigidBody.h"
#include "Joint.h"

#include <vector>

#include <ObjectiveFunction.h>

class KinematicEnergy : public ObjectiveFunction
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

		int i = 0;
		for (const auto &j : hingeJoints) {
			c.segment<2>(i) = j.computeConstraints(x, rigidbodies[j.rbIdx[0]], rigidbodies[j.rbIdx[1]]);
			i += 2; // a hinge joint constraint is of size 2
		}

		for(const auto &f : fixed) {
			c.segment<2>(i) = f.computeConstraints(x, rigidbodies[f.rbIdx]);
			i += 2; // a fixed joint constraint is of size 2
		}

		for (const auto &pol : pointOnLineJoints) {
			c.segment<1>(i) = pol.computeConstraints(x, rigidbodies[pol.rbIdx[0]], rigidbodies[pol.rbIdx[1]]);
			i += 1; // a Point on Line joint constraint is of size 1
		}

        if(fixedAngleEnabled)
            for(const auto &f : fixedAngle) {
                c.segment<1>(i) = f.computeConstraints(x, rigidbodies[f.rbIdx]);
                i += 1; // a fixed angle joint constraint is of size 1
            }

		return c;
	}

	//TODO: Add the new joint
	MatrixXd computeJacobian(const VectorXd &x) const {
		MatrixXd dCdx(getNumConstraints(), x.size());
		dCdx.setZero();

        int i = 0; // keep track of how many constraints were processed
		for (const auto &j : hingeJoints) {
			const RigidBody &rb0 = rigidbodies[j.rbIdx[0]];
			const RigidBody &rb1 = rigidbodies[j.rbIdx[1]];
			Matrix<double, 2, 6> jac = j.computeJacobian(x, rb0, rb1);

            ////////////////////////
            // Task 1.3
            ////////////////////////
			dCdx.block<2, 3>(i, rb0.dofIdx) = jac.block<2, 3>(0, 0);
			dCdx.block<2, 3>(i, rb1.dofIdx) = jac.block<2, 3>(0, 3);
			i += 2; // a hinge joint constraint is of size 2
		}

		for(const auto &f : fixed) {
			const RigidBody &rb = rigidbodies[f.rbIdx];
            Matrix<double, 2, 3> jac = f.computeJacobian(x, rb);

            ////////////////////////
            // Task 1.3
            ////////////////////////
			dCdx.block<2, 3>(i, rb.dofIdx) = jac;

            i += 2; // a fixed joint constraint is of size 2
		}

		for (const auto &pol : pointOnLineJoints) {   //Added pointOnLineConstraint
			const RigidBody &rb0 = rigidbodies[pol.rbIdx[0]];
			const RigidBody &rb1 = rigidbodies[pol.rbIdx[1]];
			Matrix<double, 1, 6> jac = pol.computeJacobian(x, rb0, rb1);

			dCdx.block<1, 3>(i, rb0.dofIdx) = jac.block<1, 3>(0, 0);
			dCdx.block<1, 3>(i, rb1.dofIdx) = jac.block<1, 3>(0, 3);

			i += 1; // a point On Line joint constraint is of size 1
		}

        if(fixedAngleEnabled)
            for(const auto &f : fixedAngle) {
                const RigidBody &rb = rigidbodies[f.rbIdx];
                Matrix<double, 1, 3> jac = f.computeJacobian(x, rb);

                ////////////////////////
                // Task 1.3
                ////////////////////////
				dCdx.block<1, 3>(i, rb.dofIdx) = jac;

                i += 1; // a fixed angle joint constraint is of size 1
            }

		return dCdx;
	}

	int getNumConstraints() const {
		//Added pointOnLineConstraint
        return hingeJoints.size()*2 + fixed.size() * 2 + pointOnLineJoints.size()
                + ((fixedAngleEnabled) ? fixedAngle.size()*1 : 0);
	}

public:
	std::vector<RigidBody> rigidbodies;
	std::vector<HingeJoint> hingeJoints;
	std::vector<FixedPtJoint> fixed;
	std::vector<PointOnLineJoint> pointOnLineJoints; //Added pointOnLineConstraint
	std::vector<FixedAngleJoint> fixedAngle;

    bool fixedAngleEnabled = false;
};
