#pragma once
#include "../include/AbstractRBEngine.h"

using namespace std;

/*--------------------------------------------------------------------------------------*
 * This class is used as a wrapper for the Open Dynamics Engine.                        *
 *--------------------------------------------------------------------------------------*/
class RBEngine : public AbstractRBEngine
{
	virtual void applyForceTo(RigidBody* b, const V3D& f, const P3D& p);
	virtual void applyTorqueTo(RigidBody* b, const V3D& t);
	virtual void applyRelativeTorqueTo(RigidBody* b, const V3D& t);
};
