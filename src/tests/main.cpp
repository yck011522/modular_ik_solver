
#define _USE_MATH_DEFINES

//DEFINES to use ASIO as a standalone import without BOOST
#define ASIO_STANDALONE 
#define _WEBSOCKETPP_CPP11_TYPE_TRAITS_
// Includes for Websockets
#include <iostream>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <string>

#include <math.h>
#include <RigidBody3D.h>
#include <Joint3D.h>
#include <KinematicEnergy3D.h>
#include <RigidBodySimulation3D.h>

#include <fstream>
#include <vector>
#include <OptLogger.h>



//typedef websocketpp::server<websocketpp::config:asio> server;
//
//void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
//	std::cout << msg->get_payload() << std::endl;
//}

using namespace std;

struct TestResult {
	bool passed = true;
	std::string error = "";
};

struct OptimizationResult {
	bool converged = true;
};


//struct SimParameters {
//	std::string setupFilePath = "";
//	OptimizationLogger * logger = 0;
//
//};
inline int extractInt(char *& line) {
	int val = atoi(line);
	line = strstr(line, ",") + 1; //Locate delimiter ,
	return val;
}

inline double extractDouble(char *& line) {
	double val = atof(line);
	line = strstr(line, ",") + 1; //Locate delimiter ,
	return val;
}

inline Vector3d extractThreeDouble(char *& line) {
	Vector3d val;
	val[0] = extractDouble(line);
	val[1] = extractDouble(line);
	val[2] = extractDouble(line);
	return val;
}

// New Test Functions for testing New Rigid Body / New joints / New goals and targets / New Sim

TestResult testRigidBody3D() {

	RigidBody3D rb(0, "Test1");

	//State of the rigid body
	VectorXd x(6);
	x.setZero();

	//^ Values of the states are position (X,Y,Z), euler angle (RX,RY,RZ)
	x << 0, 50, 20, M_PI / 2, M_PI / 4, M_PI / 4;

	//Create a test point p at local coordinate 1,1
	const Vector3d p(15, 5, 1);
	//Compute world coordinate of p using pWorld
	//Vector3d pWorld = rb.pWorld(x, p);

	//Test result
	//cout << "testRigidBody3D - pWorld: " << pWorld << endl;
	//cout << "test 1 - Vector2d(0, sqrt(2)): " << Vector2d(0, sqrt(2)) << endl;
	//if ((pWorld - Vector2d(0, sqrt(2))).norm() > 1e-8) {
	//	return TestResult({ false, "p world wrong" });
	//}


	//Compute the dpWorld_dx using dpWorld_dx function of test point p
	Matrix<double, 3, 6> dpWorld_dx = rb.dpWorld_dx(x, p);

	//Create controled test using numerical method
	Matrix<double, 3, 6> dpWorld_dx_FD;
	dpWorld_dx_FD.setZero();

	//Create a small delta
	const double dx = 1e-8;
	for (int i = 0; i < x.size(); ++i) {
		VectorXd xp = x;
		xp[i] += dx;
		VectorXd xm = x;
		xm[i] -= dx;

		Vector3d pWorld_p = rb.pWorld(xp, p);
		Vector3d pWorld_m = rb.pWorld(xm, p);

		dpWorld_dx_FD.block<3, 1>(0, i) = (pWorld_p - pWorld_m) / (2 * dx);
	}

	//cout << "dpWorld_dx" << endl << dpWorld_dx << endl;

	//cout << "test 2 - dpWorld_dx: " << dpWorld_dx << endl;
	//cout << "test 2 - dpWorld_dx_FD: " << dpWorld_dx_FD << endl;
	if ((dpWorld_dx - dpWorld_dx_FD).norm() > 1e-6) {
		cout << "dpWorld_dx" << endl << dpWorld_dx << endl;
		cout << "dpWorld_dx_FD" << endl << dpWorld_dx_FD << endl;
		cout << "difference:" << endl << (dpWorld_dx - dpWorld_dx_FD).norm() << endl;
		return TestResult({ false, "dpWorld/dx wrong" });
	}

	return TestResult();
}

TestResult testHingeJoint3D() {

	//State of the rigid body
	VectorXd x(12);
	x.setZero();

	//^ Values of the states are position (X,Y,Z), euler angle (RX,RY,RZ)
	x << 0, 10, 20, M_PI / 4, M_PI / 4, M_PI / 4, 0, 50, 20, M_PI / 2, M_PI / 4, M_PI / 4;

	RigidBody3D rb1(0, "Test1");
	RigidBody3D rb2(1, "Test2");



	//Create Joint object from the two rigidbody, joint location is at {0.5, 0} and {-0.5, 0.}
	HingeJoint2Pt joint(0, 6, { 0.5, 0, 1 }, { 0.5, 0, -1 }, { 15, 5, 1 }, { 15, 5, -1 });

	//Compute dC_dx using computeJacobian 
	MatrixXd dC_dx = joint.computeJacobian(x, rb1, rb2);

	MatrixXd dC_dx_FD(6, 12);
	const double dx = 1e-8;
	for (int i = 0; i < x.size(); ++i) {
		VectorXd xp = x;
		xp[i] += dx;
		VectorXd xm = x;
		xm[i] -= dx;

		VectorXd cp = joint.computeConstraints(xp, rb1, rb2);
		VectorXd cm = joint.computeConstraints(xm, rb1, rb2);
		dC_dx_FD.block<6, 1>(0, i) = (cp - cm) / (2 * dx);
	}

	if ((dC_dx - dC_dx_FD).norm() > 1e-5) {
		cout << "dC_dx" << endl << dC_dx << endl;
		cout << "dC_dx_FD" << endl << dC_dx_FD << endl;
		cout << "difference:" << endl << (dC_dx - dC_dx_FD) << endl;
		cout << "difference norm:" << endl << (dC_dx - dC_dx_FD).norm() << endl;
		return TestResult({ false, "dC/dx wrong" });
	}
	return TestResult();
}

TestResult testFixedJoint3Pt3D() {

	//State of the rigid body
	VectorXd x(12); //6DOF per Rigidbody
	x.setZero();

	//^ Values of the states are position (X,Y,Z), euler angle (RX,RY,RZ)
	x << 0, 10, 20, M_PI / 4, M_PI / 4, M_PI / 4, 0, 50, 20, M_PI / 2, M_PI / 4, M_PI / 4;

	RigidBody3D rb1(0, "Rigid Body 1");
	RigidBody3D rb2(1, "Rigid Body 2");

	
	//Create Joint object from the two rigidbody, joint location is at {0.5, 0} and {-0.5, 0.}
	//FixedJoint3Pt joint(0, 6, { 0.5, 0, 1 }, { 0.5, 0, -1 }, { 1.5, 0, -1 }, { 15, 5, 1 }, { 15, 5, -1 }, { 14, 5, -1 });
	FixedJoint3Pt joint(0, 6, { 0.5, 0, 1 }, { 0.5, 0, 2 }, { 1.5, 1, 1 });

	//Compute dC_dx using computeJacobian 
	MatrixXd dC_dx = joint.computeJacobian(x, rb1, rb2);

	MatrixXd dC_dx_FD(9, 12);
	const double dx = 1e-8;
	for (int i = 0; i < x.size(); ++i) {
		VectorXd xp = x;
		xp[i] += dx;
		VectorXd xm = x;
		xm[i] -= dx;

		VectorXd cp = joint.computeConstraints(xp, rb1, rb2);
		VectorXd cm = joint.computeConstraints(xm, rb1, rb2);
		dC_dx_FD.block<9, 1>(0, i) = (cp - cm) / (2 * dx);
	}

	if ((dC_dx - dC_dx_FD).norm() > 1e-5) {
		cout << "dC_dx" << endl << dC_dx << endl;
		cout << "dC_dx_FD" << endl << dC_dx_FD << endl;
		cout << "difference:" << endl << (dC_dx - dC_dx_FD) << endl;
		cout << "difference norm:" << endl << (dC_dx - dC_dx_FD).norm() << endl;
		return TestResult({ false, "dC/dx wrong" });
	}
	return TestResult();
}

TestResult testFixedBody3Pt3D() {

	//State of the rigid body
	VectorXd x(6); //6DOF per Rigidbody
	x.setZero();

	//^ Values of the states are position (X,Y,Z), euler angle (RX,RY,RZ)
	x << 0, 10, 20, M_PI / 4, M_PI / 4, M_PI / 4;

	RigidBody3D rb1(0, "Rigid Body 1");


	//Create Joint object from the two rigidbody, joint location is at {0.5, 0} and {-0.5, 0.}
	//FixedJoint3Pt joint(0, 6, { 0.5, 0, 1 }, { 0.5, 0, -1 }, { 1.5, 0, -1 }, { 15, 5, 1 }, { 15, 5, -1 }, { 14, 5, -1 });
	FixedBody3Pt joint(0, { 20, 0, 1 }, { 155, 0, 2 }, { 300, 10, 558 });

	//Compute dC_dx using computeJacobian 
	MatrixXd dC_dx = joint.computeJacobian(x, rb1);

	MatrixXd dC_dx_FD(9, 6);
	const double dx = 1e-8;
	for (int i = 0; i < x.size(); ++i) {
		VectorXd xp = x;
		xp[i] += dx;
		VectorXd xm = x;
		xm[i] -= dx;

		VectorXd cp = joint.computeConstraints(xp, rb1);
		VectorXd cm = joint.computeConstraints(xm, rb1);
		dC_dx_FD.block<9, 1>(0, i) = (cp - cm) / (2 * dx);
	}

	if ((dC_dx - dC_dx_FD).norm() > 1e-5) {
		cout << "dC_dx" << endl << dC_dx << endl;
		cout << "dC_dx_FD" << endl << dC_dx_FD << endl;
		cout << "difference:" << endl << (dC_dx - dC_dx_FD) << endl;
		cout << "difference norm:" << endl << (dC_dx - dC_dx_FD).norm() << endl;
		return TestResult({ false, "dC/dx wrong" });
	}
	return TestResult();
}

typedef TestResult (*func_t)();
inline void test_function(func_t f, const char *name) {
	TestResult tr = f();
	if(!tr.passed){
		std::cerr << name << " failed: `" << tr.error << "`" << std::endl;
	}
	else {
		std::cout << name << ": [PASSED]" << std::endl;
	}
}

typedef TestResult(*func_t2)(const char *);
inline void test_opt_function(func_t2 f, const char *filePath,  const char *name) {
	TestResult tr = f(filePath);
	if (!tr.passed) {
		std::cerr << name << " failed: `" << tr.error << "`" << std::endl;
	}
	else {
		std::cout << name << ": [PASSED]" << std::endl;
	}
}



void inverseKinematics(const char *filePath) {

	//Confirm file exist
	std::cout << "filePath" << std::endl;
	std::cout << filePath << std::endl;

	//Complain if cannot open file
	ifstream in(filePath);
	if (!in) {
		return;// TestResult({ false, "Cannot open input file." });
	}

	// Parse file - first line to see how many Rigid Bodies we have
	char str[2048];
	in.getline(str, 255);
	char * pch = strstr(str, "=") + 1; //Locate delimiter =
	int numberOfRigidBodies = atoi(pch);

	cout << "numberOfRigidBodies = " << numberOfRigidBodies << endl << endl;
	if (numberOfRigidBodies == 0) {
		return;//  TestResult({ false, "numberOfRigidBodies = 0 , nothing to do." });
	}

	// Create Simulation package object
	RigidBodySimulation3D sim;

	// Create All Rigid Body objects in Sim
	for (int i = 0; i < numberOfRigidBodies; i++) {
		sim.rigidbodies().emplace_back(i, "RB"+ to_string(i));
	}

	// Create state vector
	VectorXd x(numberOfRigidBodies * 6);
	for (int i = 0; i < numberOfRigidBodies; i++) {
		in.getline(str, 255);  // Get a line from file
		//cout << "Rigid Body Encode: " << str << endl;
		char * line = str;
		// Store first 3 Values as X Y Z state
		for (int j = 0; j < 3; j++) {
			line = strstr(line, ",") + 1; //Locate delimiter ,
			double val = atof(line);
			//cout << "   " << val << endl;
			x[i * 6 + j] = val;
		}

		// Grab next 4 Values as W X Y Z quaternion values and 
		// convert them to euler angle (RX,RY,RZ) to be stored in state
		double qValues[4];
		for (int j = 0; j < 4; j++) {
			line = strstr(line, ",") + 1; //Locate delimiter ,
			double val = atof(line);
			//cout << "   " << val << endl;
			qValues[j] = val;
		}
		// Rhino values are quaternion = a + bi + cj + dk
		// Eigen values quaternion = w + xi + yj + zk
		Eigen::Quaterniond q(qValues[0], qValues[1], qValues[2], qValues[3]);

		//Conversion of Quaternion to Euler
		Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2,1,0); //eulerAngles(2,1,0) is ZYX convention
		x[i * 6 + 3] = euler[2]; // euler[2] = Rx
		x[i * 6 + 4] = euler[1]; // euler[1] = Ry
		x[i * 6 + 5] = euler[0]; // euler[0] = Rz

		std::cout << "Decoded Rigid Body State: ";
		for (int j = 0; j < 6; j++) {
			std::cout << x[i * 6 + j] << ", ";
		}
		std::cout << endl;
	}
	sim.x = x;
	sim.x0 = sim.x;

	// Create constraints
	while (in) {
		in.getline(str, 1024);  // delim defaults to '\n'
		if (string(str).find("gnd,") == 0) {
			char * line = str;
			line = strstr(line, ",") + 1; // Go past first delimiter ,
			int rb0Idx = extractInt(line);
			FixedBody3Pt constraint = FixedBody3Pt (sim.rigidbodies()[rb0Idx], x);
			//The following lines is a hot fix !!! I dont know why this properity is nnot stored after being moved into the vector.
			constraint.rb0Idx = rb0Idx; 
			constraint.rb0Pt0 = Vector3d(0, 0, 0);
			constraint.rb0Pt1 = Vector3d(1, 0, 0);
			constraint.rb0Pt2 = Vector3d(0, 1, 0);
			constraint.worldPt0 = sim.rigidbodies()[rb0Idx].pWorld(x, Vector3d(0, 0, 0));
			constraint.worldPt1 = sim.rigidbodies()[rb0Idx].pWorld(x, Vector3d(1, 0, 0));
			constraint.worldPt2 = sim.rigidbodies()[rb0Idx].pWorld(x, Vector3d(0, 1, 0));

			sim.FixedBody3Pts().push_back(constraint);
			cout << "Created FixedBody3Pt (Ground) Constraint: For RB " << rb0Idx << endl;
		}

		if (string(str).find("hin,") == 0) {
			char * line = str;
			line = strstr(line, ",") + 1; // Go past first delimiter ,
			int rb0Idx = extractInt(line);
			int rb1Idx = extractInt(line);
			//cout << "  rb0Idx " << rb0Idx << endl;
			//cout << "  rb1Idx " << rb1Idx << endl;
			Vector3d rb0Pt0 = extractThreeDouble(line);
			Vector3d rb0Pt1 = extractThreeDouble(line);
			Vector3d rb1Pt0 = extractThreeDouble(line);
			Vector3d rb1Pt1 = extractThreeDouble(line);
			//cout << "  rb0Pt0 " << rb0Pt0 << endl;
			//cout << "  rb0Pt1 " << rb0Pt1 << endl;
			//cout << "  rb1Pt0 " << rb1Pt0 << endl;
			//cout << "  rb1Pt1 " << rb1Pt1 << endl;
			HingeJoint2Pt constraint(rb0Idx, rb1Idx, rb0Pt0, rb0Pt1, rb1Pt0, rb1Pt1);
			sim.HingeJoint2Pts().push_back(constraint);
			cout << "Created HingeJoint2Pt Constraint: Between RB " << rb0Idx << " and RB " << rb1Idx << endl;

		}

		if (string(str).find("endPt,") == 0) {

			char * line = str;
			line = strstr(line, ",") + 1; // Go past first delimiter ,
			int rb0Idx = extractInt(line);
			//cout << "  rb0Idx " << rb0Idx << endl;
			Vector3d rb0Pt0 = extractThreeDouble(line);
			Vector3d rb0Pt1 = extractThreeDouble(line);
			Vector3d rb0Pt2 = extractThreeDouble(line);
			Vector3d tgtPt0 = extractThreeDouble(line);
			Vector3d tgtPt1 = extractThreeDouble(line);
			Vector3d tgtPt2 = extractThreeDouble(line);
			double progressiveAlpha = extractDouble(line);
			int numTargetPoints = extractInt(line);

			// Create constraint
			ProgressiveTarget3Pt constraint(rb0Idx);
			
			constraint.rb0Idx = rb0Idx;
			constraint.rb0Pt0 = rb0Pt0;
			constraint.rb0Pt1 = rb0Pt1;
			constraint.rb0Pt2 = rb0Pt2;
			constraint.finalWorldPt0 = tgtPt0;
			constraint.finalWorldPt1 = tgtPt1;
			constraint.finalWorldPt2 = tgtPt2;
			//constraint.worldPt0 = tgtPt0;
			//constraint.worldPt1 = tgtPt1;
			//constraint.worldPt2 = tgtPt2;
			constraint.progressiveAlpha = progressiveAlpha;
			constraint.numTargetPoints = numTargetPoints;
			//constraint.setProgressiveWorldPt(x,sim.rigidbodies()[rb0Idx]);
			sim.Targets3Pt().push_back(constraint);
			cout << "Created Targets3Pt (Target) Constraint: On RB " << rb0Idx; // << endl;
			//cout << "Target Distance is: " << constraint.computeConstraints(x,sim.rigidbodies()) << endl;
		}
		
	}

	in.close();

	// Debug Print X State:
	if (false) {
		cout << "State x:" << endl;
		for (int i = 0; i < numberOfRigidBodies * 6; i++) {
			cout << "   " << x[i] << endl;
		}
	}
	
	std::cout << "Energy Before: " << sim.energy.evaluate(sim.x) << endl;
	std::cout << "State Before: " <<  endl;
	for (int i = 0; i < numberOfRigidBodies * 6; i++) {
		cout << "   " << sim.x[i] << endl;
	}

	sim.run();
	std::cout << "Energy After: " << sim.energy.evaluate(sim.x) << endl;
	std::cout << "State After: " << endl;
	for (int i = 0; i < numberOfRigidBodies * 6; i++) {
		cout << "   " << sim.x[i] << endl;
	}
	//return TestResult({ true, "Sim Completed" });

}


//Simulation setup and state are kept at high level to maintain their values
RigidBodySimulation3D simulation;

#include <sstream>
#include <iterator>

//Reset the states and clears out all constraints
void simulationReset(const string setupString) {
	std::stringstream ss(setupString);
	std::string token;

	//Clear the arrays containing the rigidbodies and the constraints
	simulation.energy.rigidbodies.clear();
	simulation.energy.FixedBody3Pts.clear();
	simulation.energy.FixedJoint3Pts.clear();
	simulation.energy.HingeJoint2Pts.clear();
	simulation.energy.Targets3Pt.clear();

	//Find out how many rigid bodies in total
	std::getline(ss, token, ';');
	char newline[1024];
	strcpy(newline, token.c_str());
	char * line = newline;
	line = strstr(line, "=") + 1; //Locate delimiter =
	int numberOfRigidBodies = atoi(line);
	cout << "numberOfRigidBodies: " << numberOfRigidBodies << endl;

	// Create All Rigid Body objects in Sim
	for (int i = 0; i < numberOfRigidBodies; i++) {
		simulation.rigidbodies().emplace_back(i, "RB" + to_string(i));
	}

	// Create state vector
	VectorXd x(numberOfRigidBodies * 6);
	for (int i = 0; i < numberOfRigidBodies; i++) {
		std::getline(ss, token, ';');
		strcpy(newline, token.c_str());
		char * line = newline;
		cout << "Rigid Body Encode: " << line << endl;

		// Store first 3 Values as X Y Z state
		for (int j = 0; j < 3; j++) {
			line = strstr(line, ",") + 1; //Locate delimiter ,
			double val = atof(line);
			//cout << "   " << val << endl;
			x[i * 6 + j] = val;
		}

		// Grab next 4 Values as W X Y Z quaternion values and 
		// convert them to euler angle (RX,RY,RZ) to be stored in state
		double qValues[4];
		for (int j = 0; j < 4; j++) {
			line = strstr(line, ",") + 1; //Locate delimiter ,
			double val = atof(line);
			//cout << "   " << val << endl;
			qValues[j] = val;
		}
		// Rhino values are quaternion = a + bi + cj + dk
		// Eigen values quaternion = w + xi + yj + zk
		Eigen::Quaterniond q(qValues[0], qValues[1], qValues[2], qValues[3]);

		//Conversion of Quaternion to Euler
		Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0); //eulerAngles(2,1,0) is ZYX convention
		x[i * 6 + 3] = euler[2]; // euler[2] = Rx
		x[i * 6 + 4] = euler[1]; // euler[1] = Ry
		x[i * 6 + 5] = euler[0]; // euler[0] = Rz

		//std::cout << "Decoded Rigid Body State: ";
		for (int j = 0; j < 6; j++) {
			//std::cout << x[i * 6 + j] << ", ";
		}
		std::cout << endl;
	}
	simulation.x = x;
	simulation.x0 = simulation.x;

	// Create constraints and set other parametres
	while (std::getline(ss, token, ';')) {

		if (token.find("gnd,") == 0) {
			// Copying the line of string into a modifyable character array 
			char newline[1024];
			strcpy(newline, token.c_str());
			char * line = newline;
			line = strstr(line, ",") + 1; // Go past first delimiter ,

			// Create constraint
			FixedBody3Pt constraint;
			constraint.rb0Idx = extractInt(line);
			constraint.constraintID = extractInt(line);
			
			constraint.fixHere(x, simulation.rigidbodies());

			//Add constraint to the list
			simulation.FixedBody3Pts().push_back(constraint);
			cout << "Created FixedBody3Pt (Ground) Constraint: For RB " << constraint.rb0Idx << endl;
		}

		if (token.find("hin,") == 0) {
			// Copying the line of string into a modifyable character array 
			char newline[1024];
			strcpy(newline, token.c_str());
			char * line = newline;
			line = strstr(line, ",") + 1; // Go past first delimiter ,

			// Create constraint
			HingeJoint2Pt constraint;
			constraint.rb0Idx = extractInt(line);
			constraint.rb1Idx = extractInt(line);
			constraint.rb0Pt0 = extractThreeDouble(line);
			constraint.rb0Pt1 = extractThreeDouble(line);
			constraint.rb1Pt0 = extractThreeDouble(line);
			constraint.rb1Pt1 = extractThreeDouble(line);
			constraint.constraintID = extractInt(line);

			//Add constraint to the list
			simulation.HingeJoint2Pts().push_back(constraint);
			cout << "Created HingeJoint2Pt Constraint: Between RB " << constraint.rb0Idx << " and RB " << constraint.rb1Idx << endl;
		}

		if (token.find("ptOnLine,") == 0) {
			// Copying the line of string into a modifyable character array 
			char newline[1024];
			strcpy(newline, token.c_str());
			char * line = newline;
			line = strstr(line, ",") + 1; // Go past first delimiter ,

			// Create constraint
			PointOnLineJoint constraint;
			constraint.rb0Idx = extractInt(line);
			constraint.rb1Idx = extractInt(line);
			constraint.rb0Pt0 = extractThreeDouble(line);
			constraint.rb0Pt1 = extractThreeDouble(line);
			constraint.rb1Pt0 = extractThreeDouble(line);
			constraint.constraintID = extractInt(line);

			//Add constraint to the list
			simulation.PointOnLineJoints().push_back(constraint);
			cout << "Created PointOnLineJoint Constraint: Between RB " << constraint.rb0Idx << " and RB " << constraint.rb1Idx << endl;

		}

		if (token.find("target,") == 0) {
			// Copying the line of string into a modifyable character array 
			char newline[1024];
			strcpy(newline, token.c_str());
			char * line = newline;

			line = strstr(line, ",") + 1; // Go past first delimiter ,

			// Create constraint
			ProgressiveTarget3Pt constraint;
			constraint.rb0Idx = extractInt(line);
			constraint.rb0Pt0 = extractThreeDouble(line);
			constraint.rb0Pt1 = extractThreeDouble(line);
			constraint.rb0Pt2 = extractThreeDouble(line);
			constraint.finalWorldPt0 = extractThreeDouble(line);
			constraint.finalWorldPt1 = extractThreeDouble(line);
			constraint.finalWorldPt2 = extractThreeDouble(line);
			constraint.progressiveAlpha = extractDouble(line);
			constraint.numTargetPoints = extractInt(line);
			constraint.constraintID = extractInt(line);
			constraint.setProgressiveWorldPt(simulation.x, simulation.rigidbodies());

			//Add it to the list
			simulation.Targets3Pt().push_back(constraint);

			cout << "Created Targets3Pt (Target) Constraint: On RB " << constraint.rb0Idx; // << endl;
			//cout << "Target Distance is: " << constraint.computeConstraints(x,sim.rigidbodies()) << endl;
		}

		if (token.find("param_") == 0) {
			string keyString = token.substr(0,token.find('='));
			string valString = token.substr(token.find('=') + 1);

			if (keyString == "param_progressiveEnergyThreshold") simulation.param_progressiveEnergyThreshold = std::stod(valString);
			if (keyString == "param_finalEnergyThreshold") simulation.param_finalEnergyThreshold = std::stod(valString);
			if (keyString == "param_progresiveSteps") simulation.param_progresiveSteps = std::stoi(valString);
			if (keyString == "param_finalSteps") simulation.param_finalSteps = std::stoi(valString);
			if (keyString == "param_logFile") simulation.param_logFile = (valString);
			if (keyString == "param_verbose") simulation.param_verbose = std::stoi(valString);

			std::cout << "Parameter Setting: " << keyString << " = " << valString << endl;
		}

	} //End of message reading while loop
}

// Update only selected Constraints (inc Targets)
void simulationUpdate(const string setupString) {
	std::stringstream ss(setupString);
	std::string token;

	//Update Constraints

	// Create constraints
	while (std::getline(ss, token, ';')) {

		if (token.find("gnd,") == 0) {
			// Copying the line of string into a modifyable character array 
			char newline[1024];
			strcpy(newline, token.c_str());
			char * line = newline;
			line = strstr(line, ",") + 1; // Go past first delimiter ,

			// Find constraint
			FixedBody3Pt * constraint;
			int constraintID = std::stoi(token.substr(token.find_last_of(',') + 1)); //ID is the last value in sequence

			for (std::size_t i = 0; i < simulation.energy.FixedBody3Pts.size(); ++i) {
				if (simulation.energy.FixedBody3Pts[i].constraintID == constraintID)
					constraint = &simulation.energy.FixedBody3Pts[i];
			}

			// Update Content
			constraint->rb0Idx = extractInt(line);
			constraint->fixHere(simulation.x0, simulation.rigidbodies());

			if (simulation.param_verbose >= 1) cout << "Updated FixedBody3Pt (Ground) Constraint: For RB " << constraint->rb0Idx << endl;
		}

		if (token.find("hin,") == 0) {
			// Copying the line of string into a modifyable character array 
			char newline[1024];
			strcpy(newline, token.c_str());
			char * line = newline;
			line = strstr(line, ",") + 1; // Go past first delimiter ,

			// Find constraint
			HingeJoint2Pt * constraint;
			int constraintID = std::stoi(token.substr(token.find_last_of(',') + 1)); //ID is the last value in sequence

			for (std::size_t i = 0; i < simulation.energy.HingeJoint2Pts.size(); ++i) {
				if (simulation.energy.HingeJoint2Pts[i].constraintID == constraintID)
					constraint = &simulation.energy.HingeJoint2Pts[i];
			}

			// Update Content
			constraint->rb0Idx = extractInt(line);
			constraint->rb1Idx = extractInt(line);
			constraint->rb0Pt0 = extractThreeDouble(line);
			constraint->rb0Pt1 = extractThreeDouble(line);
			constraint->rb1Pt0 = extractThreeDouble(line);
			constraint->rb1Pt1 = extractThreeDouble(line);

			if (simulation.param_verbose >= 1) cout << "Updated HingeJoint2Pt Constraint: Between RB " << constraint->rb0Idx << " and RB " << constraint->rb1Idx << endl;

		}

		if (token.find("ptOnLine,") == 0) {
			// Copying the line of string into a modifyable character array 
			char newline[1024];
			strcpy(newline, token.c_str());
			char * line = newline;
			line = strstr(line, ",") + 1; // Go past first delimiter ,

			// Find constraint
			PointOnLineJoint * constraint;
			int constraintID = std::stoi(token.substr(token.find_last_of(',') + 1)); //ID is the last value in sequence

			for (std::size_t i = 0; i < simulation.energy.PointOnLineJoints.size(); ++i) {
				if (simulation.energy.PointOnLineJoints[i].constraintID == constraintID)
					constraint = &simulation.energy.PointOnLineJoints[i];
			}

			// Update Content
			constraint->rb0Idx = extractInt(line);
			constraint->rb1Idx = extractInt(line);
			constraint->rb0Pt0 = extractThreeDouble(line);
			constraint->rb0Pt1 = extractThreeDouble(line);
			constraint->rb1Pt0 = extractThreeDouble(line);

			if (simulation.param_verbose >= 1) cout << "Updated PointOnLineJoint Constraint: Between RB " << constraint->rb0Idx << " and RB " << constraint->rb1Idx << endl;

		}

		if (token.find("target,") == 0) {
			// Copying the line of string into a modifyable character array 
			char newline[1024];
			strcpy(newline, token.c_str());
			char * line = newline;
			line = strstr(line, ",") + 1; // Go past first delimiter ,

			// Find constraint
			ProgressiveTarget3Pt * constraint;
			int constraintID = std::stoi(token.substr(token.find_last_of(',') + 1)); //ID is the last value in sequence

			for (std::size_t i = 0; i < simulation.energy.Targets3Pt.size(); ++i) {
				if (simulation.energy.Targets3Pt[i].constraintID == constraintID)
					constraint = &simulation.energy.Targets3Pt[i];
			}

			// Update content
			constraint->rb0Idx = extractInt(line);
			constraint->rb0Pt0 = extractThreeDouble(line);
			constraint->rb0Pt1 = extractThreeDouble(line);
			constraint->rb0Pt2 = extractThreeDouble(line);
			constraint->finalWorldPt0 = extractThreeDouble(line);
			constraint->finalWorldPt1 = extractThreeDouble(line);
			constraint->finalWorldPt2 = extractThreeDouble(line);
			constraint->progressiveAlpha = extractDouble(line);
			constraint->numTargetPoints = extractInt(line);
			constraint->setProgressiveWorldPt(simulation.x, simulation.rigidbodies());

			if (simulation.param_verbose >= 1) cout << "Updated Targets3Pt (Target) Constraint: On RB " << constraint->rb0Idx << endl;
		}

	} //End of message reading while loop

	// Debug Print Energy and State:
	if (simulation.param_verbose >= 1) {
		std::cout << " " << endl;
		std::cout << "Energy Before: " << simulation.energy.evaluate(simulation.x0) << endl;
		/*std::cout << "State Before: " << endl;
		for (int i = 0; i < simulation.x0.size(); i++) {
			cout << "   " << simulation.x0[i] << endl;
		}*/
	}

	//Run Optimization
	simulation.run();

	// Debug Print Energy and State:
	if (simulation.param_verbose >= 1) {
		std::cout << "Steps Used (Progressive): " << simulation.stat_progresiveSteps << endl;
		std::cout << "Steps Used (Final): " << simulation.stat_finalSteps << endl;
		VectorXd energyResidule = simulation.energy.computeConstraints(simulation.x);
		std::cout << "Energy After: " << 0.5 * energyResidule.dot(energyResidule) << endl;
		 for (int i = 0; i < energyResidule.size(); i++) {
			cout << "   " << energyResidule(i) << endl;
		}
		 simulation.energy.eachConstraintReport(simulation.x);
		//std::cout << "State After: " << endl;
		//for (int i = 0; i < simulation.x.size(); i++) {
		//	cout << "   " << simulation.x[i] << endl;
		//}
	}
}

// Websocketpp server object for opening websocket
typedef websocketpp::server<websocketpp::config::asio> server;
server websocket;

// On reception of message, parse header and call simulation setup script / Sim Constraint Update 
void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
	//std::cout << "Message Received: "  << std::endl;
	string message = msg->get_payload();
	//std::cout << message << std::endl << std::endl;

	if (message.substr(0, 6).compare("reset;") == 0) {
		string trimmedMessage = message.substr(message.find_first_of(';') + 1);
		simulationReset(trimmedMessage);
	}

	if (message.substr(0, 7).compare("update;") == 0) {
		string trimmedMessage = message.substr(message.find_first_of(';') + 1);		
		simulationUpdate(trimmedMessage);
		string reply = "";
		for(std::size_t i = 0; i < simulation.x.size(); ++i) {
			reply.append(std::to_string(simulation.x[i]));
			if (i < simulation.x.size() - 1) reply.append(",");
		}

		websocket.send(hdl, reply, websocketpp::frame::opcode::TEXT);
	}

}	

int main(int, char**)
{

	websocket.clear_access_channels(websocketpp::log::alevel::all);
	websocket.set_message_handler(&on_message);
	websocket.init_asio();
	websocket.listen(9002);
	websocket.start_accept();
	websocket.run();
	
	return 0;
}
