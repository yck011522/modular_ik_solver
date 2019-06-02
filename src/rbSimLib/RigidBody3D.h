#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
using namespace std;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Matrix;
using Eigen::AngleAxisd;


class RigidBody3D {
public:
	
	RigidBody3D(int _rbIdx, std::string _name = "") : rbIdx(_rbIdx), name(_name) {
		dofIdx = _rbIdx * 6;
		std::cout << "RigidBody3D constructor rbIdx: " << rbIdx << endl ;
	}
	
	//State of the component is defined by its position and rotation. The rotation is defined by Euler angles. 
	//To avoid gimbal locks, the rotation axes of the last two euler angles can be changed as needed. 
	Vector3d n_alpha = Vector3d(0, 0, 1), n_beta = Vector3d(0, 1, 0), n_gamma = Vector3d(1, 0, 0);
	
	// Returns the world coordinates of a local point `p` given the global state x.
	// `x` contains the states of all rigid bodies. The member `dofIdx` indicates
	// at which index in `x` the DOFs for this rigid body start.
	Vector3d pWorld(const VectorXd &x, const Vector3d &p) const {
		Eigen::Quaterniond q = quat(eulerAngle(x));
		return pos(x) + quat(x) * p;
	}

	// Returns the world vector of a local vector `v` given the global state x.
	Vector3d vWorld(const VectorXd &x, const Vector3d &v) const {
		return quat(x) * v;
	}

	// Returns the Jacobian `dp_world / d_x` given the global state `x` and the 
	// local point `p`.
	Matrix<double, 3, 6> dpWorld_dx(const VectorXd &x, const Vector3d &p) const {
		Matrix<double, 3, 6> m;
		m.setZero();

		m(0, 0) = 1;
		m(1, 1) = 1;
		m(2, 2) = 1;

		Vector3d dw_dgamma = n_gamma.cross(R_gamma(x)*(R_beta(x)*(R_alpha(x)*p)));
		Vector3d dw_dbeta = R_gamma(x)*(n_beta.cross(R_beta(x)*(R_alpha(x)*p)));
		Vector3d dw_dalpha = R_gamma(x) *(R_beta(x)*(n_alpha.cross(R_alpha(x)*p)));

		for (int i = 0; i < 3; i++) {
			m(i, 3) = dw_dgamma[i];
			m(i, 4) = dw_dbeta[i];
			m(i, 5) = dw_dalpha[i];
		}
		return m;
	}

	// Returns the position of this rigid body given the global state `x`.
	inline const Vector3d pos(const VectorXd &x) const {
		return x.segment<3>(dofIdx);
	}

	// Returns the euler angle (RX,RY,RZ) of this rigid body given the global state `x`.
	inline const Vector3d eulerAngle(const VectorXd &x) const {
		//Euler Rotation ZYX
		return x.segment<3>(dofIdx + 3);
	}

	inline const AngleAxisd R_alpha(const VectorXd &x) const {
		return Eigen::AngleAxisd(x(dofIdx + 5), Eigen::Vector3d::UnitZ());
	}
	inline const AngleAxisd R_beta(const VectorXd &x) const {
		return Eigen::AngleAxisd(x(dofIdx + 4), Eigen::Vector3d::UnitY());
	}
	inline const AngleAxisd R_gamma(const VectorXd &x) const {
		return Eigen::AngleAxisd(x(dofIdx + 3), Eigen::Vector3d::UnitX());
	}

	// Returns the Quaternion given the euler angles (RX,RY,RZ). Euler Rotation XYZ is used
	inline const  Quaterniond quat(const Vector3d &euler) const {
		Eigen::AngleAxisd Rx(euler[0], Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd Ry(euler[1], Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd Rz(euler[2], Eigen::Vector3d::UnitZ());
		// Euler Rotation XYZ (Same definition as https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html)
		// Results confirmed by https://www.andre-gaschler.com/rotationconverter/)
		Eigen::Quaterniond q = Rx * Ry * Rz;
		return q;
	}

	inline const Quaterniond quat(const VectorXd &x) const {
		Vector3d euler = eulerAngle(x);
		return quat(euler);
	}



	
public:
	// Each rigid body has a field dofIdx which is the start index of the rigid body
	// in the global state vector .
	int rbIdx;
	int dofIdx;
	std::string name;
};
#pragma once
