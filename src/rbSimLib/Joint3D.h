#pragma once

//TODO Refactor this file to separate Joints 2D and Joints 3D

#include "RigidBody3D.h"
#include <vector>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class HingeJoint2Pt {
public:

	const static int NumConstraints = 6;
	const static int NumStates = 12; //1 RigidBody is 6 states. 2 RigidBody is 12 states.

	HingeJoint2Pt(){
	}

	HingeJoint2Pt(int rb0Idx, int rb1Idx, Vector3d rb0Pt0, Vector3d rb0Pt1, Vector3d rb1Pt0, Vector3d rb1Pt1)
		: rb0Idx(rb0Idx), rb1Idx(rb1Idx), rb0Pt0(rb0Pt0), rb0Pt1(rb0Pt1), rb1Pt0(rb1Pt0), rb1Pt1(rb1Pt1){
	}

	VectorXd computeConstraints(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeConstraints(x, rigidBodies[rb0Idx], rigidBodies[rb1Idx]);
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody3D &rb0, const RigidBody3D &rb1) const {
		Vector3d errorPt0 = rb1.pWorld(x, rb1Pt0) - rb0.pWorld(x, rb0Pt0);
		Vector3d errorPt1 = rb1.pWorld(x, rb1Pt1) - rb0.pWorld(x, rb0Pt1);
		Eigen::VectorXd c(6);
		c.segment<3>(0) = errorPt0;
		c.segment<3>(3) = errorPt1;
		return c;
	}

	Matrix <double, NumConstraints, NumStates> computeJacobian(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeJacobian(x, rigidBodies[rb0Idx], rigidBodies[rb1Idx]);
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd x, const RigidBody3D &rb0, const RigidBody3D &rb1) const {
		MatrixXd dCdx(NumConstraints, NumStates);
		//Jacobian for Pt0
		Matrix<double, 3, 6> pt0_dpWorld_dxrb0 = rb0.dpWorld_dx(x, rb0Pt0);
		Matrix<double, 3, 6> pt0_dpWorld_dxrb1 = rb1.dpWorld_dx(x, rb1Pt0);

		dCdx.block<3, 6>(0, 0) = pt0_dpWorld_dxrb0 * -1.0;
		dCdx.block<3, 6>(0, 6) = pt0_dpWorld_dxrb1;

		//Jacobian for Pt1
		Matrix<double, 3, 6> pt1_dpWorld_dxrb0 = rb0.dpWorld_dx(x, rb0Pt1);
		Matrix<double, 3, 6> pt1_dpWorld_dxrb1 = rb1.dpWorld_dx(x, rb1Pt1);

		dCdx.block<3, 6>(3, 0) = pt1_dpWorld_dxrb0 * -1.0;
		dCdx.block<3, 6>(3, 6) = pt1_dpWorld_dxrb1;

		return dCdx;
	}



public:
	int rb0Idx, rb1Idx;
	Vector3d rb0Pt0, rb0Pt1, rb1Pt0, rb1Pt1;
	int constraintID;
};

class PointOnLineJoint {
public:

	const static int NumConstraints = 3;
	const static int NumStates = 12; //1 RigidBody is 6 states. 2 RigidBody is 12 states.

	PointOnLineJoint() {
	}

	VectorXd computeConstraints(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeConstraints(x, rigidBodies[rb0Idx], rigidBodies[rb1Idx]);
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody3D &rb0, const RigidBody3D &rb1) const {
		Eigen::VectorXd c(3);

		//Point on Line Constraint by Cross Product 
		Vector3d VectorFixed = rb0.pWorld(x, rb0Pt1) - rb0.pWorld(x, rb0Pt0);
		Vector3d VectorMoving = rb1.pWorld(x, rb1Pt0) - rb0.pWorld(x, rb0Pt0);
		c.segment<3>(0) = VectorFixed.cross(VectorMoving);

		return c;
	}

	Matrix <double, NumConstraints, NumStates> computeJacobian(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeJacobian(x, rigidBodies[rb0Idx], rigidBodies[rb1Idx]);
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd x, const RigidBody3D &rb0, const RigidBody3D &rb1) const {
		MatrixXd dCdx(NumConstraints, NumStates);
		
		Vector3d U = rb0.pWorld(x, rb0Pt1) - rb0.pWorld(x, rb0Pt0); //VectorFixed
		Vector3d V = rb1.pWorld(x, rb1Pt0) - rb0.pWorld(x, rb0Pt0); //VectorMoving

		// Deravitive of the two vectors.
		Matrix<double, 3, 6> du_drb0 = rb0.dpWorld_dx(x, rb0Pt1) - rb0.dpWorld_dx(x, rb0Pt0);
		Matrix<double, 3, 6> dv_drb0 = -1 * rb0.dpWorld_dx(x, rb0Pt0);
		Matrix<double, 3, 6> dv_drb1 =  rb1.dpWorld_dx(x, rb1Pt0);

		//Partial derivative of Cross Product between the two vectors
		for (int s = 0; s < 6; s++) // Concerning RB0
			dCdx.block<3, 1>(0, s) = du_drb0.block<3, 1>(0, s).cross(V) + U.cross(dv_drb0.block<3, 1>(0, s));
		for (int s = 0; s < 6; s++) // Converning RB1
			dCdx.block<3, 1>(0, s+6) = U.cross(dv_drb1.block<3, 1>(0, s));

		return dCdx;
	}

public:
	int rb0Idx, rb1Idx;
	Vector3d rb0Pt0, rb0Pt1, rb1Pt0;
	int constraintID;
};

class FixedJoint3Pt {
public:

	const static int NumConstraints = 9;
	const static int NumStates = 12; //1 RigidBody is 6 states. 2 RigidBody is 12 states.

	FixedJoint3Pt(int rb0Idx, int rb1Idx, Vector3d rb0Pt0, Vector3d rb0Pt1, Vector3d rb0Pt2, Vector3d rb1Pt0, Vector3d rb1Pt1, Vector3d rb1Pt2)
		: rb0Idx(rb0Idx), rb1Idx(rb1Idx), rb0Pts({ rb0Pt0, rb0Pt1, rb0Pt2 }), rb1Pts({ rb1Pt0, rb1Pt1, rb1Pt2 }) {
	}

	//Construct object from the relative location of rb1's coordinate frame, expressed in rb0's coordinates.
	FixedJoint3Pt(int rb0Idx, int rb1Idx, Vector3d rb1Origin, Vector3d rb1X, Vector3d rb1Y)
		: rb0Idx(rb0Idx), rb1Idx(rb1Idx), rb0Pts({ rb1Origin, rb1X, rb1Y }){
		rb1Pts = { Vector3d(0,0,0), Vector3d(1,0,0), Vector3d(0,1,0) };
		
	}

	VectorXd computeConstraints(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeConstraints(x, rigidBodies[rb0Idx], rigidBodies[rb1Idx]);
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody3D &rb0, const RigidBody3D &rb1) const {
		Eigen::VectorXd c(rb0Pts.size() * 3);
		for (size_t i = 0; i < rb0Pts.size(); ++i) {
			c.segment<3>(i * 3) = rb1.pWorld(x, rb1Pts[i]) - rb0.pWorld(x, rb0Pts[i]);
		}
		return c;
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeJacobian(x, rigidBodies[rb0Idx], rigidBodies[rb1Idx]);
	}
	   
	Matrix <double, NumConstraints, NumStates> computeJacobian(const VectorXd x, const RigidBody3D &rb0, const RigidBody3D &rb1) const {
		Matrix <double, NumConstraints, NumStates> dCdx;
		//Jacobian for Pt0
		for (size_t i = 0; i < rb0Pts.size(); ++i) {
			dCdx.block<3, 6>(i*3, 0) = rb0.dpWorld_dx(x, rb0Pts[i]) * -1.0;
			dCdx.block<3, 6>(i*3, 6) = rb1.dpWorld_dx(x, rb1Pts[i]);
		}
		return dCdx;
	}



public:
	int rb0Idx, rb1Idx;
	std::array<Vector3d, 3> rb0Pts;
	std::array<Vector3d, 3> rb1Pts;
	int constraintID;

};

class FixedBody3Pt {
public:
	const static int NumConstraints = 9;
	const static int NumStates = 6; //1 RigidBody is 6 states. 2 RigidBody is 12 states.

	FixedBody3Pt(int rb0Idx, Vector3d rb0Pt0, Vector3d rb0Pt1, Vector3d rb0Pt2, Vector3d worldPt0, Vector3d worldPt1, Vector3d worldPt2)
		: rb0Idx(rb0Idx), rb0Pt0(rb0Pt0), rb0Pt1(rb0Pt1), rb0Pt2(rb0Pt2), worldPt0(worldPt0), worldPt1(worldPt1), worldPt2(worldPt2) {
	}

	//Construct object from the relative location of rb0's coordinate frame, expressed in world coordinates.
	FixedBody3Pt(int rb0Idx, Vector3d rb0Origin, Vector3d rb0X, Vector3d rb0Y)
		: rb0Idx(rb0Idx), worldPt0(rb0Origin), worldPt1(rb0X), worldPt2(rb0Y) {
		rb0Pt0 = Vector3d(0, 0, 0);
		rb0Pt1 = Vector3d(1, 0, 0);
		rb0Pt2 = Vector3d(0, 1, 0);
		//cout << "rb0Idx: " << endl << rb0Idx << endl;
		//cout << "rb0Pts0: " << endl << rb0Pts[0] << endl;
		//cout << "rb0Pts1: " << endl << rb0Pts[1] << endl;
		//cout << "rb0Pts2: " << endl << rb0Pts[2] << endl;
		//cout << "worldPt0: " << endl << worldPts[0] << endl;
		//cout << "worldPt1: " << endl << worldPts[1] << endl;
		//cout << "worldPt2: " << endl << worldPts[2] << endl;
	}

	FixedBody3Pt(const RigidBody3D &rb, const VectorXd &x) {
		Vector3d rb0Origin = rb.pWorld(x, Vector3d(0, 0, 0));
		Vector3d rb0X = rb.pWorld(x, Vector3d(1, 0, 0));
		Vector3d rb0Y = rb.pWorld(x, Vector3d(0, 1, 0));
		std::cout << "FixedBody3Pt constructor rb0Idx: " << rb.rbIdx << endl;
		FixedBody3Pt(rb.rbIdx, rb0Origin, rb0X, rb0Y);
	}

	FixedBody3Pt(){}

public:

	void inline fixHere(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) {
		rb0Pt0 = Vector3d(0, 0, 0);
		rb0Pt1 = Vector3d(1, 0, 0);
		rb0Pt2 = Vector3d(0, 1, 0);
		worldPt0 = rigidBodies[rb0Idx].pWorld(x, rb0Pt0);
		worldPt1 = rigidBodies[rb0Idx].pWorld(x, rb0Pt1);
		worldPt2 = rigidBodies[rb0Idx].pWorld(x, rb0Pt2);
	}

	VectorXd computeConstraints(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeConstraints(x, rigidBodies[rb0Idx]);
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody3D &rb0) const {
		Eigen::VectorXd c(NumConstraints);
		//for (size_t i = 0; i < rb0Pts.size(); ++i) {
			//std::cout << "FixedBody3Pt computeConstraints worldPts : " << worldPts[i] << endl;
			//std::cout << "FixedBody3Pt computeConstraints rb0.pWorld : " << rb0.pWorld(x, rb0Pts[i]) << endl;
		c.segment<3>(0) = worldPt0 - rb0.pWorld(x, rb0Pt0);
		c.segment<3>(3) = worldPt1 - rb0.pWorld(x, rb0Pt1);
		c.segment<3>(6) = worldPt2 - rb0.pWorld(x, rb0Pt2);
		return c;
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd x, const std::vector<RigidBody3D> &rigidBodies) const {
		//std::cout << "FixedBody3Pt computeConstraints rb0Idx: " << rb0Idx;
		//return computeJacobian(x, rigidBodies[rb0Idx]);
		//Test to see if this is the problem

		Matrix <double, NumConstraints, NumStates> dCdx;
		dCdx.block<3, 6>(0, 0) = rigidBodies[rb0Idx].dpWorld_dx(x, rb0Pt0) * -1.0;
		dCdx.block<3, 6>(3, 0) = rigidBodies[rb0Idx].dpWorld_dx(x, rb0Pt1) * -1.0;
		dCdx.block<3, 6>(6, 0) = rigidBodies[rb0Idx].dpWorld_dx(x, rb0Pt2) * -1.0;

		return dCdx;
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd x, const RigidBody3D &rb0) const {
		Matrix <double, NumConstraints, NumStates> dCdx;
		dCdx.block<3, 6>(0, 0) = rb0.dpWorld_dx(x, rb0Pt0) * -1.0;
		dCdx.block<3, 6>(3, 0) = rb0.dpWorld_dx(x, rb0Pt1) * -1.0;
		dCdx.block<3, 6>(6, 0) = rb0.dpWorld_dx(x, rb0Pt2) * -1.0;
		return dCdx;
	}

public:
	int rb0Idx;
	Vector3d rb0Pt0;
	Vector3d rb0Pt1;
	Vector3d rb0Pt2;
	Vector3d worldPt0;
	Vector3d worldPt1;
	Vector3d worldPt2;
	int constraintID;

};

class Target1Pt {
public:

	const static int NumConstraints = 3;	// 1 Point Coincide is 3 Constraints, 2Pt = 6Constraints, 3Pt = 9Constraints
	const static int NumStates = 6;			// 1 RigidBody is 6 states. 2 RigidBody is 12 states.

	Target1Pt(int rb0Idx,  Vector3d rb0Pt0,  Vector3d worldPt0)
		: rb0Idx(rb0Idx), worldPt0(worldPt0) {
	}

	VectorXd computeConstraints(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeConstraints(x, rigidBodies[rb0Idx]);
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody3D &rb0) const {
		Vector3d errorPt0 = worldPt0 - rb0.pWorld(x, rb0Pt0);
		Eigen::VectorXd c(3);
		c.segment<3>(0) = errorPt0;
		return c;
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeJacobian(x, rigidBodies[rb0Idx]);
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd x, const RigidBody3D &rb0) const {
		Matrix <double, NumConstraints, NumStates> dCdx;

		Matrix<double, 3, 6> pt0_dpWorld_dxrb0 = rb0.dpWorld_dx(x, rb0Pt0); 

		dCdx.block<3, 6>(0, 0) = pt0_dpWorld_dxrb0 * -1.0;
		return dCdx;
	}




public:
	int rb0Idx;
	Vector3d rb0Pt0, worldPt0;
	int constraintID;

};

class ProgressiveTarget1Pt {
public:

	const static int NumConstraints = 3;	// 1 Point Coincide is 3 Constraints, 2Pt = 6Constraints, 3Pt = 9Constraints
	const static int NumStates = 6;			// 1 RigidBody is 6 states. 2 RigidBody is 12 states.


	ProgressiveTarget1Pt(int rb0Idx, Vector3d rb0Pt0, Vector3d worldPt0, double _progressiveAlpha)
		: rb0Idx(rb0Idx), finalWorldPt0(worldPt0), progressiveAlpha(_progressiveAlpha) {
	}

	VectorXd computeConstraints(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeConstraints(x, rigidBodies[rb0Idx]);
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody3D &rb0) const {
		Vector3d errorPt0 = worldPt0 - rb0.pWorld(x, rb0Pt0);
		//std::cout << "errorPt0.norm()= " << errorPt0.norm() << endl;
		Eigen::VectorXd c(3);
		c.segment<3>(0) = errorPt0;

		return c;
	}

	bool setProgressiveWorldPt(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) {
		return setProgressiveWorldPt(x, rigidBodies[rb0Idx]);
	}

	bool setProgressiveWorldPt(const VectorXd &x, const RigidBody3D &rb0) {
		Vector3d targetVector = finalWorldPt0 - rb0.pWorld(x, rb0Pt0);
		double error = targetVector.norm();
		if (error > progressiveAlpha) {
			targetVector = targetVector.normalized() * progressiveAlpha;
			worldPt0 = rb0.pWorld(x, rb0Pt0) + targetVector;
			//std::cout << "ProgressiveWorldPt: finalWorldPt0 =" << finalWorldPt0[0] << "," << finalWorldPt0[1] << "," << finalWorldPt0[2];
			//std::cout << " worldPt0 = " << worldPt0[0] << "," << worldPt0[1] << "," << worldPt0[2];
			//std::cout << " errorNorm = " << error << endl;
			return false;
		}
		else {
			worldPt0 = finalWorldPt0;
			//std::cout << "ProgressiveWorldPt: Final Target is set" << endl;

			return true; //If the final target is set.
		}
	}


	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeJacobian(x, rigidBodies[rb0Idx]);
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd x, const RigidBody3D &rb0) const {
		Matrix <double, NumConstraints, NumStates> dCdx;

		Matrix<double, 3, 6> pt0_dpWorld_dxrb0 = rb0.dpWorld_dx(x, rb0Pt0);

		dCdx.block<3, 6>(0, 0) = pt0_dpWorld_dxrb0 * -1.0;
		//std::cout << "computeJacobian" << endl;
		return dCdx;
	}

public:
	int rb0Idx;
	Vector3d rb0Pt0, worldPt0, finalWorldPt0;
	double progressiveAlpha = 100;
	int constraintID;

};

class ProgressiveTarget3Pt {
public:

	const static int NumConstraints = 9;	// 1 Point Coincide is 3 Constraints, 2Pt = 6Constraints, 3Pt = 9Constraints
	const static int NumStates = 6;			// 1 RigidBody is 6 states. 2 RigidBody is 12 states.

	ProgressiveTarget3Pt() {
	}

	ProgressiveTarget3Pt(int rb0Idx) //, Vector3d rb0Pt0, Vector3d rb0Pt1, Vector3d rb0Pt2, Vector3d worldPt0, Vector3d worldPt1, Vector3d worldPt2, int _numTargetPoints, double _progressiveAlpha)
		: rb0Idx(rb0Idx){
		//rb0Pt0(rb0Pt0), rb0Pt1(rb0Pt1), rb0Pt2(rb0Pt2),
		//finalWorldPt0(worldPt0), finalWorldPt1(worldPt1), finalWorldPt2(worldPt2),
		//numTargetPoints(_numTargetPoints), progressiveAlpha(_progressiveAlpha) {
	}

	VectorXd computeConstraints(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeConstraints(x, rigidBodies[rb0Idx]);
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody3D &rb0) const {
		Vector3d errorPt0 = worldPt0 - rb0.pWorld(x, rb0Pt0);
		Vector3d errorPt1 = worldPt1 - rb0.pWorld(x, rb0Pt1);
		Vector3d errorPt2 = worldPt2 - rb0.pWorld(x, rb0Pt2);
		
		//std::cout << "errorPt0.norm()= " << errorPt0.norm() << endl;
		Eigen::VectorXd c(9);
		c.setZero();
		c.segment<3>(0) = errorPt0;
		if (numTargetPoints > 1)c.segment<3>(3) = errorPt1;
		if (numTargetPoints > 2)c.segment<3>(6) = errorPt2;
		return c;
	}

	bool setProgressiveWorldPt(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) {
		return setProgressiveWorldPt(x, rigidBodies[rb0Idx]);
	}

	//This will move the WorldPoint towards the final worldPoint by the amount set by progressiveAlpha
	bool setProgressiveWorldPt(const VectorXd &x, const RigidBody3D &rb0) {
		Vector3d targetVector = finalWorldPt0 - rb0.pWorld(x, rb0Pt0);
		double error = targetVector.norm();
		//std::cout << " Distance to Target (errorNorm) = " << error << endl;
		if (error > progressiveAlpha) {
			//Set vector length according to one Vector derived from Point 0 
			targetVector = targetVector.normalized() * progressiveAlpha; 
			worldPt0 = rb0.pWorld(x, rb0Pt0) + targetVector;
			worldPt1 = rb0.pWorld(x, rb0Pt1) + targetVector;
			worldPt2 = rb0.pWorld(x, rb0Pt2) + targetVector;
			//std::cout << "ProgressiveWorldPt: finalWorldPt0 =" << finalWorldPt0[0] << "," << finalWorldPt0[1] << "," << finalWorldPt0[2] << "; ";
			//std::cout << " worldPt0 = " << worldPt0[0] << "," << worldPt0[1] << "," << worldPt0[2] << endl; 
			return false;
		}
		else {
			worldPt0 = finalWorldPt0;
			worldPt1 = finalWorldPt1;
			worldPt2 = finalWorldPt2;
			//std::cout << "ProgressiveWorldPt: Final Target is set" << endl;
			//std::cout << "ProgressiveWorldPt: finalWorldPt0 =" << finalWorldPt0[0] << "," << finalWorldPt0[1] << "," << finalWorldPt0[2] << endl;
			return true; //If the final target is set.
		}
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd &x, const std::vector<RigidBody3D> &rigidBodies) const {
		return computeJacobian(x, rigidBodies[rb0Idx]);
	}

	Matrix <double, NumConstraints, NumStates>  computeJacobian(const VectorXd x, const RigidBody3D &rb0) const {
		Matrix <double, NumConstraints, NumStates> dCdx;
		dCdx.setZero();

		Matrix<double, 3, 6> pt0_dpWorld_dxrb0 = rb0.dpWorld_dx(x, rb0Pt0);
		Matrix<double, 3, 6> pt1_dpWorld_dxrb0 = rb0.dpWorld_dx(x, rb0Pt1);
		Matrix<double, 3, 6> pt2_dpWorld_dxrb0 = rb0.dpWorld_dx(x, rb0Pt2);

		dCdx.block<3, 6>(0, 0) = pt0_dpWorld_dxrb0 * -1.0;
		if (numTargetPoints > 1) dCdx.block<3, 6>(3, 0) = pt1_dpWorld_dxrb0 * -1.0;
		if (numTargetPoints > 2) dCdx.block<3, 6>(6, 0) = pt2_dpWorld_dxrb0 * -1.0;
		//std::cout << "computeJacobian" << endl;
		return dCdx;
	}

public:
	int rb0Idx;
	int numTargetPoints;							// This constraint can be programmed to constraint 1 or 2 or 3 points. Achieving Positional, CoAxle and Frame Targets.
	Vector3d rb0Pt0, rb0Pt1, rb0Pt2;		// This the local coordinate on Rigidbody
	Vector3d worldPt0, worldPt1, worldPt2;	// This is the world coordinate on the immediate target (If progressive Alpha is engaged)
	Vector3d finalWorldPt0, finalWorldPt1, finalWorldPt2;	// This is the final World Coordinate where target should finally reach.
	double progressiveAlpha = 100;			// This is the amount of move per progressive step for point0 to move forward.
	int constraintID;
};
