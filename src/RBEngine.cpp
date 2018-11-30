#include "../include/RBEngine.h"

void RBEngine::applyForceTo(RigidBody* b, const V3D& f, const P3D& p)
{
}


void RBEngine::applyTorqueTo(RigidBody* b, const V3D& t)
{
}

void RBEngine::applyRelativeTorqueTo(RigidBody* b, const V3D& t)
{
}

DynamicArray<ContactForce> RBEngine::getContactForceOnRB(RigidBody* b)
{
	vector<ContactForce> v;
	return v;
}
