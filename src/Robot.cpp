
#include "../include/Robot.h"
#include <stdio.h>
#include "../include/SimpleLimb.h"

#include "../include/HingeJoint.h"
#include "../include/UniversalJoint.h"
#include "../include/BallAndSocketJoint.h"
#include <iostream>

/**
	the constructor
*/
Robot::Robot(RigidBody* root){
	this->root = root;
	if (root == NULL)
		throwError("Can't build a robot without a root!\n");

	//gather the list of jointList for the robot
	jointList.clear();
	DynamicArray<RigidBody*> bodies;
	bodies.push_back(root);

	while (bodies.size()>0) {
		if (bodies[0]->pJoints.size() > 1) 
			throwError("Possible kinematic loop detected in robot definition. Not currently allowed...\n");
		//add all the children jointList to the list
		for (uint i = 0;i<bodies[0]->cJoints.size();i++) {
			jointList.push_back(bodies[0]->cJoints[i]);
			bodies.push_back(bodies[0]->cJoints[i]->child);
		}
		bodies.erase(bodies.begin());
	}

	//index the jointList properly...
	for (uint i = 0;i<jointList.size();i++)
		jointList[i]->jIndex = i;

	for (uint i = 0; i<auxiliaryJointList.size(); i++)
		auxiliaryJointList[i]->jIndex = i;

	//compute the mass of the robot
	mass = root->rbProperties.mass;
	for (uint i = 0; i < jointList.size(); i++)
		mass += jointList[i]->child->rbProperties.mass;

	bFrame = new BodyFrame(this);
}

/**
	the destructor
*/
Robot::~Robot(void){
	delete bFrame;
}

/**
	uses the state of the robot to populate the input
*/
void Robot::populateState(RobotState* state, bool useDefaultAngles) {
	//we'll push the root's state information - ugly code....
	state->setPosition(root->state.position);
	state->setOrientation(root->state.orientation);
	state->setVelocity(root->state.velocity);
	state->setAngularVelocity(root->state.angularVelocity);
	state->setHeadingAxis(Globals::worldUp);

	state->setJointCount(jointList.size());
	state->setAuxiliaryJointCount(auxiliaryJointList.size());

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is revolute!!!

	for (uint i=0;i<jointList.size();i++){
		if (!useDefaultAngles){
			state->setJointRelativeOrientation(getRelativeOrientationForJoint(jointList[i]), i);
			state->setJointRelativeAngVelocity(getRelativeLocalCoordsAngularVelocityForJoint(jointList[i]), i);
		}
		else {
			state->setJointRelativeOrientation(QuaternionR(), i);
			state->setJointRelativeAngVelocity(V3D(), i);
			HingeJoint* hj = dynamic_cast<HingeJoint*>(jointList[i]);
			if (hj)
				state->setJointRelativeOrientation(getRotationQuaternion(hj->defaultAngle, hj->rotationAxis), i);
		}
	}

	for (uint i = 0; i<auxiliaryJointList.size(); i++) {
		if (!useDefaultAngles) {
			state->setAuxiliaryJointRelativeOrientation(getRelativeOrientationForJoint(auxiliaryJointList[i]), i);
			state->setAuxiliaryJointRelativeAngVelocity(getRelativeLocalCoordsAngularVelocityForJoint(auxiliaryJointList[i]), i);
		}
		else {
			state->setAuxiliaryJointRelativeOrientation(QuaternionR(), i);
			state->setAuxiliaryJointRelativeAngVelocity(V3D(), i);
		}
	}
}

/**
	This method populates the state of the current robot with the values that are passed
	in the dynamic array. The same conventions as for the getState() method are assumed.
*/
void Robot::setState(RobotState* state){
	//kinda ugly code....
	root->state.position = state->getPosition();
	root->state.orientation = state->getOrientation();
	root->state.orientation.toUnit();
	root->state.velocity = state->getVelocity();
	root->state.angularVelocity = state->getAngularVelocity();

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is revolute!!!
	for (uint j=0;j<jointList.size();j++){
		setRelativeOrientationForJoint(jointList[j], state->getJointRelativeOrientation((int)j).toUnit());
		setRelativeLocalCoordsAngularVelocityForJoint(jointList[j], state->getJointRelativeAngVelocity((int)j));
		//and now set the linear position and velocity
		jointList[j]->fixJointConstraints(true, true, true, true);
	}

	for (uint j = 0; j<auxiliaryJointList.size(); j++) {
		setRelativeOrientationForJoint(auxiliaryJointList[j], state->getAuxiliaryJointRelativeOrientation((int)j).toUnit());
		setRelativeLocalCoordsAngularVelocityForJoint(auxiliaryJointList[j], state->getAuxiliaryJointRelativeAngVelocity((int)j));
		//and now set the linear position and velocity
		auxiliaryJointList[j]->fixJointConstraints(true, true, true, true);
	}

}

/**
	makes sure the state of the robot is consistent with all the joint types...
*/
void Robot::fixJointConstraints() {
	for (size_t j = 0; j<jointList.size(); j++)
		jointList[j]->fixJointConstraints(true, true, true, true);

	for (size_t j = 0; j<auxiliaryJointList.size(); j++)
		auxiliaryJointList[j]->fixJointConstraints(true, true, true, true);
}

/**
	This method is used to compute the center of mass of the articulated figure.
*/
P3D Robot::computeCOM(){
	P3D COM = root->getCMPosition() * root->rbProperties.mass;
	double totalMass = root->rbProperties.mass;

	for (uint i=0; i <jointList.size(); i++){
		totalMass += jointList[i]->child->rbProperties.mass;
		COM += jointList[i]->child->getCMPosition() * jointList[i]->child->rbProperties.mass;
	}

	return COM / totalMass;
}

/**
	This method is used to compute the velocity of the center of mass of the articulated figure.
*/
V3D Robot::computeCOMVelocity(){
	V3D COMVel = root->getCMVelocity() * root->rbProperties.mass;
	double totalMass = root->rbProperties.mass;

	for (uint i=0; i <jointList.size(); i++){
		totalMass += jointList[i]->child->rbProperties.mass;
		COMVel += jointList[i]->child->getCMVelocity() * jointList[i]->child->rbProperties.mass;
	}

	return COMVel / totalMass;
}

/**
	this method is used to rotate the robot (well, the robot whose state is passed in as a parameter) 
	about the vertical axis, so that it's default heading has the value that is given as a parameter
*/
void Robot::setHeading(double val){
	RobotState state(this);
	populateState(&state);
	state.setHeading(val);
	setState(&state);
}

/**
	this method is used to read the reduced state of the robot from the file
*/
void Robot::loadReducedStateFromFile(const char* fName){
	RobotState state(this);
	state.readFromFile(fName);
	setState(&state);
}

/**
	this method is used to write the reduced state of the robot to a file
*/
void Robot::saveReducedStateToFile(const char* fName){
	RobotState state(this);
	state.writeToFile(fName, this);
}

void setupSimpleRobotStructure(Robot* robot) {
	for (int i = 0; i<robot->getJointCount(); i++) {
		if (robot->getJoint(i)->child->rbProperties.getEndEffectorPointCount() > 0) {
			//this rigid body needs to be the end segment of a limb...
			GenericLimb* limb = new SimpleLimb(robot->getJoint(i)->child->name.c_str(), robot->getJoint(i)->child, robot->getRoot());
			robot->bFrame->addLimb(limb);
		}
	}
	// for (uint i = 0; i<robot->jointList.size(); i++)
		// robot->bFrame->addBodyLink(robot->jointList[i]->child);
}

/**
this method is used to return a reference to the articulated figure's rigid body whose name is passed in as a parameter,
or NULL if it is not found.
*/
RigidBody* Robot::getRBByName(const char* jName) {
	for (uint i = 0; i<jointList.size(); i++) {
		if (strcmp(jointList[i]->parent->name.c_str(), jName) == 0)
			return jointList[i]->parent;
		if (strcmp(jointList[i]->child->name.c_str(), jName) == 0)
			return jointList[i]->child;
	}
	return NULL;
}

bool Robot::MoveByJointsR(Eigen::VectorXd vec, bool isDeg/* = false*/)
{
	//quick and dirty
	DynamicArray<double> newJoints;
	newJoints.resize(14, 0);
	for (size_t i = 0; i < 7; i++)
	{
		newJoints[2 * i] = vec(i);
	}
	return MoveByJoints(newJoints,isDeg);
}

bool Robot::MoveByJoints(DynamicArray<double> newJoints, bool isDeg/* = false*/)
{
	if (jointList.size() != newJoints.size())
		return false;

	RobotState rs(this);
	for (uint i = 0; i < jointList.size(); i++) {
		HingeJoint* joint = dynamic_cast<HingeJoint*>(jointList[i]);
		double ang = isDeg ? RAD(newJoints[i]) : newJoints[i];
		QuaternionR jointOrientation = getRotationQuaternion(ang, joint->rotationAxis);
		rs.setJointRelativeOrientation(jointOrientation, i);
	}
	setState(&rs);
	return true;
}

void Robot::PrintJointsValues()
{
	std::cout << "Joints:{ ";

	for (uint i = 0; i < jointList.size(); i++) {
		if (i % 2 != 0) continue;
		HingeJoint* joint = dynamic_cast<HingeJoint*>(jointList[i]);
		double rotAngle = joint->computeRelativeOrientation().getRotationAngle(joint->rotationAxis);
		std::cout << DEG(rotAngle) << ", ";
	}
	std::cout << "}" << std::endl;
}

Eigen::VectorXd Robot::GetQ()
{
	Eigen::VectorXd res = Eigen::VectorXd::Zero(20);
	for (uint i = 0; i < jointList.size(); ++i)
	{
		HingeJoint* joint = dynamic_cast<HingeJoint*>(jointList[i]);
		//joint->computeRelativeOrientation().getAxisAngle(rotAxis, rotAngle);
		double rotAngle = joint->computeRelativeOrientation().getRotationAngle(joint->rotationAxis);
		res(i+6) = /*DEG*/(rotAngle);
	}
	return res;
}

double Robot::GetJointValueR(int i)
{
	if (i < 0 || i > 13)
		return -999.999;
	HingeJoint* joint = dynamic_cast<HingeJoint*>(jointList[2*i]);
	double rotAngle = joint->computeRelativeOrientation().getRotationAngle(joint->rotationAxis);
	return DEG(rotAngle);
}