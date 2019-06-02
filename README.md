*!! this code is not production ready !!*
# [modular] Inverse Kinematics Solver
This project is a optimization-based, headless, modular, Inverse Kinematics solver.


The solver encodes machine parts such as revolute joints, linear modules and linkages as **rigidbodies(RB)** that is decribed with 6-DOF state. The internal representation is 6 State variables (X,Y,Z for position, Rx, Ry, Rz for Eular ZYX rotation) that describes the local frame with respect to the global frame.

Service based IK solution for any mechanisms is described by **constraints** that relates one more more RB. The following constraints are supported:
- 2 Point Constraint: (Revolute joint or hinge)
- Point on Line Constraint (Linear pistons, prismatic joints etc) 
- Ground Constraint (Components that are fixed to the world frame, aka. fixed pieces)
- Target Constraint (Three different types of target behaviour can be chosen)
  - Point to Point
  - Point to Point + 1 Axis (Useful for cylindrical end effectors such as milling)
  - Point to Point + 2 Axis (aka. frame match) 
  
This provides a back-end that communicates to any frontend via **Websockets** (default port 9002). That has only two behaviour:
- "Setup Call" that configures a kinematic setup and set an initial state. 
- "Update Call" that updates any selected constraint, rerun the optimization using the previous state, and return the result to sender.

The expected frontend(FE) to backend(BE) communication is for the FE to make a setup call. Followed by an empty Update call, FE then waits for the BE for the resurned result. Because it is very common for robots to move between a list of waypoints that are closely spaced apart, the update call can be used to find a solution by updating only the target position. This saves all the computational overhead to setup the  kinematic relationships and preserves the state of the previous optimization for a smoother transition from the previous state.

## Compilation Instruction
1. Pull all the dependent libraries
1. Create a "build" folter at root folder.
1. Change Directory to the `build` folder and run `cmake ..`. 
1. Open Solution build/modularIK.sln with Visual Studio 2017.
1. Set Startup project as tests
1. Build using `Release` settings will result in much faster library.

## Known Issue / Limitations
1. This software is not fast enough for real time IK. (Perhaps a limitation)
1. This software returns only the state of the rb, instead of joint angles. (Not implemented yet)
1. This software currently works for unconstrained revolute and prismatic joints and does not respect any limit on the joints. This is limited by the fact that a discontinuous joints creates discontinuous solution spaces that may causes the optimization to stuck in local minma. (Not implemented / perhaps a limitation)

**Credits:** This project started as an assignment from ["Physics-based Modeling for Computational Fabrication and Robotics - 2019"](http://crl.ethz.ch/teaching/computational-fab-19/index.html) in ETHZ by Stelian Coros, Moritz Bächer, Kristina Shea. The code base comes from the starter code written by Moritz Geilinger and others in [Computational Robotics Lab](http://crl.ethz.ch/). 
**Issues and questions:** Please use the issue tracker of the repository if you have any general issues or questions.This is a not an actively maintained project, limited support cannot be expected. 
