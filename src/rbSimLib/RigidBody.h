#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
using namespace std;

using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Matrix;

Matrix2d rotationMatrix(const double &theta) {
	double sinA = sin(theta);
	double cosA = cos(theta);
   return (Matrix2d() << cosA, -sinA, sinA, cosA).finished();
}

Matrix2d drotationMatrix_dtheta(const double &theta) {
	double sinA = sin(theta);
	double cosA = cos(theta);
   return (Matrix2d() << -sinA, -cosA, cosA, -sinA).finished();
}


class RigidBody {
public:
	RigidBody(int dofIdx, double length, std::string name = "", double width = 0.4) : dofIdx(dofIdx), length(length), width(width), name(name) {
	}

	// Returns the world coordinates of a local point `p` given the global state x.
	// `x` contains the states of all rigid bodies. The member `dofIdx` indicates
	// at which index in `x` the DOFs for this rigid body start.
    Vector2d pWorld(const VectorXd &x, const Vector2d &p) const {
		return pos(x) + rotationMatrix(theta(x)) * p;
	}

	// Returns the world vector of a local vector `v` given the global state x.
	Vector2d vWorld(const VectorXd &x, const Vector2d &v) const {
		return rotationMatrix(theta(x)) * v;
	}

	// Returns the Jacobian `dp_world / d_xi` given the global state `x` and the 
	// local point `p`.
	Matrix<double, 2, 3> dpWorld_dx(const VectorXd &x, const Vector2d &p) const {
		Matrix<double, 2, 3> m;
		m.setZero();

        ////////////////////////
        // Task 1.1
        ////////////////////////
		m(0, 0) = 1;
		m(1 ,1) = 1;
		Vector2d localPosition = p;
		//cout << "localPosition: " << localPosition << endl;
		//cout << "drotationMatrix_dtheta(theta(x)): " << drotationMatrix_dtheta(theta(x)) << endl;
		//cout << "drotationMatrix_dtheta(theta(x)) * localPosition: " << drotationMatrix_dtheta(theta(x)) * localPosition << endl;

		m.block<2, 1>(0, 2) = drotationMatrix_dtheta(theta(x)) * localPosition;

		return m;
	}

	// Returns the position of this rigid body given the global state `x`.
    inline const Vector2d pos(const VectorXd &x) const {
		return x.segment<2>(dofIdx);
	}

	// Returns the rotation angle of this rigid body given the global state `x`.
	inline const double &theta(const VectorXd &x) const {
		return x[dofIdx + 2];
	}

    // Returns the rotation angle of this rigid body given the global state `x`.
    inline double &theta(VectorXd &x) const {
        return x[dofIdx + 2];
    }

public:
	// Each rigid body has a field dofIdx which is the start index of the rigid body
	// in the global state vector .
	int dofIdx;

	double length, width;
	std::string name;
};
