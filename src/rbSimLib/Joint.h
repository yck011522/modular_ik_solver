#pragma once

//TODO Refactor this file to separate Joints 2D and Joints 3D

#include "RigidBody.h"
#include <vector>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class HingeJoint {
public:

	HingeJoint(int rb0Idx, int rb1Idx, Vector2d local0, Vector2d local1)
		: rbIdx({rb0Idx, rb1Idx}), local({local0, local1}) {
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody &rb0, const RigidBody &rb1) const {
		Vector2d p0 = rb0.pWorld(x, local[0]);
		Vector2d p1 = rb1.pWorld(x, local[1]);
        return (p1-p0);
	}

	MatrixXd computeJacobian(const VectorXd x, const RigidBody &rb0, const RigidBody &rb1) const {
		MatrixXd dCdx(2, 6);

        ////////////////////////
        // Task 1.2
        ////////////////////////
		Matrix<double, 2, 3> dpWorld_dxrb0 = rb0.dpWorld_dx(x, local[0]);
		Matrix<double, 2, 3> dpWorld_dxrb1 = rb1.dpWorld_dx(x, local[1]);

		//cout << "local[0]: " << local[0] << endl;
		//cout << "local[1]: " << local[1] << endl;

		dCdx.block<2, 3>(0, 0) = dpWorld_dxrb0 * -1.0;
		dCdx.block<2, 3>(0, 3) = dpWorld_dxrb1;

		return dCdx;
	}

public:
	std::array<int, 2> rbIdx;
	std::array<Vector2d, 2> local;
};

class PointOnLineJoint {
public:

	PointOnLineJoint(int rb0Idx, int rb1Idx, Vector2d _local0Point, Vector2d _local0Vector, Vector2d _local1Point)
		: rbIdx({ rb0Idx, rb1Idx }) {
		local0Point = _local0Point;
		local0Vector = _local0Vector;
		local1Point = _local1Point;
	}

	//Implementation of two different constraints , no sqrt involved.
	VectorXd computeConstraints(const VectorXd &x, const RigidBody &rb0, const RigidBody &rb1) const {
		// Point on Line Constraint is 1D only
		VectorXd c(1);
		Vector3d vector_StartToPoint(0, 0, 0);
		vector_StartToPoint.head<2>() = (rb1.pWorld(x, local1Point) - rb0.pWorld(x, local0Point));
		Vector3d vector_StartToEnd(0, 0, 0);

		vector_StartToEnd.head<2>() = rb0.vWorld(x, local0Vector);

		double c_Limit = vector_StartToPoint.dot(vector_StartToEnd) / vector_StartToEnd.squaredNorm();
		if (c_Limit < 0) c_Limit = -c_Limit;
		else if (c_Limit <= 1) c_Limit = 0;

		double c_OnLine = vector_StartToPoint.cross(vector_StartToEnd).squaredNorm();

		c << (c_Limit + c_OnLine );
		return c;
	}

	//TODO: Finish this
	MatrixXd computeJacobian(const VectorXd x, const RigidBody &rb0, const RigidBody &rb1) const {
		MatrixXd dCdx(1, 6);

		//Numerical Gradient to form Jobian
		const double dx = 1e-8;
		for (int i = 0; i < 3; ++i) {
			VectorXd xp = x;
			xp[rb0.dofIdx + i] += dx;
			VectorXd xm = x;
			xm[rb0.dofIdx + i] -= dx;

			VectorXd cp = computeConstraints(xp, rb0, rb1);
			VectorXd cm = computeConstraints(xm, rb0, rb1);
			dCdx.block<1, 1>(0, i) = (cp - cm) / (2 * dx);
		}

		for (int i = 0; i < 3; ++i) {
			VectorXd xp = x;
			xp[rb1.dofIdx + i] += dx;
			VectorXd xm = x;
			xm[rb1.dofIdx + i] -= dx;

			VectorXd cp = computeConstraints(xp, rb0, rb1);
			VectorXd cm = computeConstraints(xm, rb0, rb1);
			dCdx.block<1, 1>(0, i + 3) = (cp - cm) / (2 * dx);
		}


		return dCdx;
	}

public:
	std::array<int, 2> rbIdx;
	Vector2d local0Point;
	Vector2d local0Vector;
	Vector2d local1Point;

};

// Point on rb1 is constraint on line on rb0
class PointOnLineJointDist : public PointOnLineJoint {
public:

	PointOnLineJointDist(int rb0Idx, int rb1Idx, Vector2d _local0Point, Vector2d _local0Vector, Vector2d _local1Point)
		: PointOnLineJoint(rb0Idx, rb1Idx, _local0Point, _local0Vector, _local1Point) {
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody &rb0, const RigidBody &rb1) const {
		// Point on Line Constraint is 1D only
		VectorXd c(1);
		Vector2d vector_pointToStart = (rb1.pWorld(x, local1Point) - rb0.pWorld(x, local0Point));
		double dist_pointToStart = vector_pointToStart.norm();
		Vector2d vector_pointToEnd = (rb1.pWorld(x, local1Point) - rb0.pWorld(x, local0Point) - rb0.vWorld(x, local0Vector));
		double dist_pointToEnd = vector_pointToEnd.norm();
		double dist_limitLength = rb0.vWorld(x, local0Vector).norm();

		c << (dist_pointToStart + dist_pointToEnd - dist_limitLength);
		return c;
	}

	////TODO: Finish this
	//MatrixXd computeJacobian(const VectorXd x, const RigidBody &rb0, const RigidBody &rb1) const {
	//	MatrixXd dCdx(1, 6);

		////////////////////////
		// Task 1.2
		////////////////////////
		//Matrix<double, 2, 3> dpWorld_dxrb0 = rb0.dpWorld_dx(x, local0Point);
		//Matrix<double, 2, 3> dpWorld_dxrb1 = rb1.dpWorld_dx(x, local1Point);

		//cout << "local[0]: " << local[0] << endl;
		//cout << "local[1]: " << local[1] << endl;

		//dCdx.block<2, 3>(0, 0) = dpWorld_dxrb0 * -1.0;
		//dCdx.block<2, 3>(0, 3) = dpWorld_dxrb1;

		//Numerical Gradient to form Jobian
		//const double dx = 1e-8;

		//double x = 
		//for (int i = 0; i < 3; ++i) {
		//	VectorXd xp = x;
		//	xp[rb0.dofIdx + i] += dx;
		//	VectorXd xm = x;
		//	xm[rb0.dofIdx + i] -= dx;

		//	VectorXd cp = computeConstraints(xp, rb0, rb1);
		//	VectorXd cm = computeConstraints(xm, rb0, rb1);
		//	dCdx.block<1, 1>(0, i) = (cp - cm) / (2 * dx);
		//}

		//for (int i = 0; i < 3; ++i) {
		//	VectorXd xp = x;
		//	xp[rb1.dofIdx + i] += dx;
		//	VectorXd xm = x;
		//	xm[rb1.dofIdx + i] -= dx;

		//	VectorXd cp = computeConstraints(xp, rb0, rb1);
		//	VectorXd cm = computeConstraints(xm, rb0, rb1);
		//	dCdx.block<1, 1>(0, i + 3) = (cp - cm) / (2 * dx);
		//}


	//	return dCdx;
	//}


};

class FixedPtJoint
{
public:
	VectorXd computeConstraints(const VectorXd &x, const RigidBody &rb) const {
        return (pos - rb.pWorld(x, localPos));
	}

	MatrixXd computeJacobian(const VectorXd x, const RigidBody &rb) const {
        MatrixXd dCdx(2, 3);

		dCdx = rb.dpWorld_dx(x, localPos) * -1.0;
		return dCdx;
	}

public:
    Vector2d pos;       // world coordinates
    int rbIdx;          // index of rigid body
    Vector2d localPos;  // position in rigid body coordinates
};

class FixedAngleJoint
{
public:
	VectorXd computeConstraints(const VectorXd &x, const RigidBody &rb) const {
		VectorXd c(1);
		c << (rb.theta(x) - angle);
		return c;
	}

	MatrixXd computeJacobian(const VectorXd x, const RigidBody &rb) const {
		MatrixXd dCdx(1, 3);

        ////////////////////////
        // Task 1.2
        ////////////////////////
		dCdx.setZero();
		dCdx(0, 2) = 1;

		return dCdx;
	}

public:
	int rbIdx;
	mutable double angle;
};