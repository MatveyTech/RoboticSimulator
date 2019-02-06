#pragma once

#include "MathLib.h"
#include "V3D.h"
#include "../include/Logger.h"
#include <string>

/*=====================================================================================================================================*
 	Quaternion class. Rather than a general implementation, this class is geared towards representing rotations.                                                                                                                                                 |
 *=====================================================================================================================================*/
class QuaternionR  {
public:
	//the scalar part of the quaternion
	double s;
	//the vector part of the quaternion
	V3D v;
public:

/**
	Constructors
*/

	//	A constructor. Intuitive...
	QuaternionR(double s, const V3D& v);

	// A copy constructor
	QuaternionR(const QuaternionR& other);

	// Default constructor - no rotation
	QuaternionR();

	// Another constructor.
	QuaternionR(double w, double x, double y, double z);
	
	// destructor.
	~QuaternionR(void);

/**
	Operators
*/
	// A copy operator
	QuaternionR& operator = (const QuaternionR &rhs);

	// returns the result of multiplying the current quaternion by rhs
	QuaternionR operator * (const QuaternionR &rhs) const;

	// multiply the current quaternion by the rhs one.
	QuaternionR& operator *= (const QuaternionR &rhs);

	// multiplies the quaternion by a scalar
	QuaternionR& operator *= (double scalar);

	// multiplies the quaternion by a scalar
	QuaternionR& operator /= (double scalar);

	// return a copy of the quaternion, multiplied by a scalar
	QuaternionR operator * (double scalar) const;

	// returns the result of dividing the current quaternion by rhs
	QuaternionR operator / (double scalar) const;

	// returns sum of *this and rhs
	QuaternionR operator + (const QuaternionR &rhs) const;

	// adds rhs to this quaternion
	QuaternionR& operator += (const QuaternionR &rhs);
	

	bool operator==(const QuaternionR& _q);
	bool operator!=(const QuaternionR& _q);

	// rotates vector v by this quaternion
	V3D operator * (const V3D &v) const;

	double& operator[](const int _id);

/**
	useful methods
*/

	// normalize the quaternion
	QuaternionR& toUnit();

	// returns the rotation angle represented by this quaternion - in the range -pi to pi. Since q and -q are the same rotation, we need to know which side is "up" explicitly
	double getRotationAngle(const V3D& positiveRotAxis);

	// Returns the complex conjugate of the current quaternion.
	QuaternionR getComplexConjugate() const;

	//	Returns the inverse of the current quaternion: q * q^-1 = identity quaternion: s = 1, v = (0,0,0)
	QuaternionR getInverse() const;

	//	Returns the length of this quaternion.
	double length() const;

	// return the squared length of this quaternion.
	double length2() const;

	// Computes the dot product between the current quaternion and the one given as parameter.
	double dot(const QuaternionR &other) const;

	// linear interpolation (don't do it!). If t = 0, the result is *this. If t = 1, result is other.
    QuaternionR linearlyInterpolateWith(const QuaternionR &other, double t) const;

	// SLERP (spherical linear interpolation). If t = 0, the result is *this. If t = 1, result is other.
	QuaternionR sphericallyInterpolateWith(const QuaternionR &other, double t) const;
	
	// SLERP (spherical linear interpolation). If t = 0, the result is *this. If t = 1, result is other.
	void sphericallyInterpolateWith(const QuaternionR &other, double t, QuaternionR &result) const;

	// return a rotated version of u
	V3D rotate(const V3D& u) const;

	// rotate u by the inverse of the quaternion
	V3D inverseRotate(const V3D& u) const;

	// target will be set to u rotated by this quaternion. 'target' cannot point at u
	void rotate(const V3D& u, V3D* target) const;

	// target will be set to u rotated by this quaternion. 'target' cannot point at u
	void inverseRotate(const V3D& u, V3D *target) const;

	// sets value of rotation quaternion from arguments
	void setRotationFrom(double angle, const V3D& axis);

	// sets value of rotation quaternion from arguments - the magnitude of the vector encodes the rotation angle
	void setRotationFrom(const V3D& vec);

	//sets value of rotation quaternion from coordinate frame passed in as arguments. The vectors passed in are the x, y and z axis of a local 
	//coordinate frame expressed in the global coordinate frame. The resulting quaternion will convert vectors from the local frame to the global frame
	void setRotationFrom(const V3D& xRot, const V3D& yRot, const V3D& zRot);

	//sets value of rotation quaternion from the matrix that is passed in as a parameter
	void setRotationFrom(const Matrix3x3& mat);

	//sets the 3x3 top-left corner of a matrix with a rotation cooresponding to this quaternion
	Matrix3x3 getRotationMatrix() const;

	// This method returns the quaternion in axis-angle representation.
	void getAxisAngle(V3D &axis, double &angle) const;

	static const QuaternionR	SetRotationFrom(const double _a, const V3D& _axis);
	static const QuaternionR	SetRotationFrom(const V3D& _axis);
	
	static QuaternionR				ExpMap(const V3D& _x) { return QuaternionR::SetRotationFrom(_x); }
	
	double							logMap() const { return 2.0*atan2(v.length(), s); }
	Vector3d						toAxisAngle() const;

	///< modulates rotation
	QuaternionR						partial(double _dt) const;	
	
	std::string ToString();
};

//	returns a quaternion that represents a rotation of angle radians around the axis (unit vector!) provided as a parameter.
inline QuaternionR getRotationQuaternion(double angle, const V3D &axis) {
//	assert(IS_EQUAL(axis.length(), 1));
	QuaternionR result(cos(angle / 2), axis.unit() * sin(angle / 2));
	return result;
}


/**
	Assume that the current quaternion represents the relative orientation between two coordinate frames P and C (i.e. q
	rotates vectors from the child/local frame C into the parent/global frame P).

	With v specified in frame C's coordinates, this method decomposes the current relative rotation, such that:

	PqC = qA * qB, where qB represents a rotation about axis v.

	This can be thought of as a decomposition into a twist about axis v (qB), and a swing (qA).
	Note that qB is a rotation from the C frame into a tmp, twist-free frame T, and qA a rotation from T into P.
	In the T coordinate frame, v is the same as in C, and qA is a rotation that aligns v from P to that from T.
*/
inline void decomposeRotation(const QuaternionR& PqC, QuaternionR* Pq, QuaternionR* qC, const V3D& v) {
	//we need to compute v in P's coordinates
	V3D vP = PqC * v;
	//compute the rotation that alligns the vector v in the two coordinate frames (P and T - remember that v has the same coordinates in C and in T)
	V3D rotAxis;
	rotAxis = vP.cross(v);
	rotAxis.toUnit();
	double rotAngle = -safeACOS(vP.dot(v));

	Pq->setRotationFrom(rotAngle, rotAxis);
	//now qB = qAinv * PqC
	*qC = Pq->getComplexConjugate() * PqC;
}

/**
Same as above. This method directly returns the twist rotation
*/
inline QuaternionR decomposeRotation(const QuaternionR& PqC, const V3D& v) {
	QuaternionR Pq, qC;
	decomposeRotation(PqC, &Pq, &qC, v);
	return qC;
}

/**
decomposes the current quaternion as follows: *this = qHeading * qOther, where qHeading is a rotation about the
axis passed in as a parameter.
*/
inline QuaternionR computeHeading(const QuaternionR& q, const V3D& axis) {
	return decomposeRotation(q.getComplexConjugate(), axis).getComplexConjugate();
}

//compute a set of euler angle axes given the q - there are many choices here, so one that minimizes the rotation angles is chosen
inline void computeEulerAxesFromQuaternion(const QuaternionR &q, V3D& a, V3D& b, V3D& c) {
	//assume q is a rotation transforming vectors from coordinate frame A to B (i.e. B_q_A). We want a sequence of Euler rotations that take you from B to A (i.e. B_q_A = rot(c) * rot(b) * rot(a). There are many options, so just choose one of them...
	a = q.v.unit();
	//then b and c should be orthogonal to a - find a rotation between a and the x-axis, and then apply the same rotation to the y and z axes to get b and c...
	V3D rotAxis = a.cross(V3D(1, 0, 0)).unit();
	double rotAngle = a.angleWith(V3D(1, 0, 0));
	if (rotAxis.length() < 0.000001) {
		rotAxis = a = V3D(1, 0, 0);
		rotAngle = 0;
	}

	//	V3D testV = V3D(1,0,0).rotate(-rotAngle, rotAxis);
	assert((V3D(1, 0, 0).rotate(-rotAngle, rotAxis) - a).length() < 0.000001);

	a = V3D(1, 0, 0).rotate(-rotAngle, rotAxis);
	b = V3D(0, 1, 0).rotate(-rotAngle, rotAxis);
	c = V3D(0, 0, 1).rotate(-rotAngle, rotAxis);
}

//decompose the quaternion q as: q = R(c, gamma) * R(b, beta) * R(a, alpha). Unknowns are: alpha, beta, gamma
inline void computeEulerAnglesFromQuaternion(const QuaternionR &q, const V3D &a, const V3D &b, const V3D &c, double& alpha, double& beta, double& gamma) {
	//the idea here is that the a axis only gets rotated about b and c, which are assumed to be orthogonal to each other. Based on this info, we can first compute the angles beta and gamma
	assert(IS_ZERO(a.dot(b)) && IS_ZERO(b.dot(c)));
	assert(IS_ZERO(a.length() - 1) && IS_ZERO(b.length() - 1) && IS_ZERO(c.length() - 1));

	V3D aRot = q.rotate(a);

	if (IS_ZERO(a.dot(c))) {
		bool circular = a.cross(b).dot(c) > 0;
		//the three axes form an orthonormal basis (i.e. Tait-Bryan)... singularity around beta = -IK_PI/2 or IK_PI/2
		if (circular) {
			beta = -safeASIN(aRot.dot(c));
			gamma = atan2(aRot.dot(b), aRot.dot(a));
		}
		else {
			beta = safeASIN(aRot.dot(c));
			gamma = atan2(-aRot.dot(b), aRot.dot(a));
		}
	}
	else if (IS_ZERO(a.dot(c) - 1)) {
		//these are "proper" euler axes, where the first and the last one are the same... singularity around beta = 0 or IK_PI
		V3D lastAxis = a.cross(b);
		beta = safeACOS(aRot.dot(a));
		gamma = atan2(aRot.dot(b), -aRot.dot(lastAxis));
	}
	else {
		//dunno what this is.... freak out...
		alpha = beta = gamma = 0;
		assert(false);
		return;
	}

	QuaternionR qLeft = getRotationQuaternion(-beta, b) * getRotationQuaternion(-gamma, c) * q;
	alpha = qLeft.getRotationAngle(a);

//	Quaternion residual = (getRotationQuaternion(gamma, c) * getRotationQuaternion(beta, b) * getRotationQuaternion(alpha, a) * q.getComplexConjugate());
//	printf("residual: %lf (%lf %lf %lf %lf) --> %lf %lf %lf\n", residual.v.length(), residual.s, residual.v[0], residual.v[1], residual.v[2], alpha, beta, gamma);
	assert(IS_ZERO((getRotationQuaternion(gamma, c) * getRotationQuaternion(beta, b) * getRotationQuaternion(alpha, a) * q.getComplexConjugate()).v.length() / 10e5));
}

//compute the rotation angle about axis a such as to most closely match the orientation in q
inline void computeRotationAngleFromQuaternion(const QuaternionR &q, const V3D &a, double& alpha) {
	alpha = decomposeRotation(q, a).getRotationAngle(a);
}

//decompose the quaternion q as: q = R(b, beta) * R(a, alpha). Unknowns are: alpha and beta. In general, there will be some residual 
inline void computeEulerAnglesFromQuaternion(const QuaternionR &q, const V3D &a, const V3D &b, double &alpha, double &beta) {
	//we assume the quaternion corresponds to a rotation about the two axes only... therefore the axis a gets rotated about b by angle beta...
	assert(IS_ZERO(a.dot(b)));
	assert(IS_ZERO(a.length() - 1) && IS_ZERO(b.length() - 1));

	QuaternionR qA, qB;
	decomposeRotation(q, &qB, &qA, a);
	alpha = qA.getRotationAngle(a);

	computeRotationAngleFromQuaternion(qB, b, beta);

	//	Quaternion residual = (Quaternion::getRotationQuaternion(beta, b) * Quaternion::getRotationQuaternion(alpha, a) * q.getComplexConjugate());
	//	Logger::printStatic("residual: %lf (%lf %lf %lf %lf) --> %lf %lf\n", residual.v.length(), residual.s, residual.v.x, residual.v.y, residual.v.z, alpha, beta);
	//	assert( IS_ZERO((Quaternion::getRotationQuaternion(beta, b) * Quaternion::getRotationQuaternion(alpha, a) * q.getComplexConjugate()).v.length()));
}

QuaternionR getRotationAxisThatAlignsVectors(const V3D& a, const V3D& b);


/**
	returns the orientation obtained by applying a constant-angular-velocity rotation starting from q
*/
inline QuaternionR updateRotationGivenAngularVelocity(const QuaternionR& q, const V3D& angularVelocity, double dt) {
	//q_p = rot(w, dt) * q
	double angularVelocityMagnitude = angularVelocity.length();
	//avoid divide by close to zero...
	if (angularVelocityMagnitude > TINY) {
		//compute the rotation that would have happened during dt, and add it (pre) to the current orientation
		double netRotationAngle = angularVelocityMagnitude * dt;
		V3D rotationAxis = angularVelocity.unit();
		return getRotationQuaternion(netRotationAngle, rotationAxis) * q;
	}
	return q;
}


/**
	returns the angular velocity that explains how we got from qStart to qEnd in dt time (qEnd = qDueToAngVelOverDT * qStart)
*/
inline V3D estimateAngularVelocity(const QuaternionR& qStart, const QuaternionR& qEnd, double dt) {
	//qEnd = rot(w_p, dt) * qStart
	QuaternionR qRot = qEnd * qStart.getComplexConjugate();

	V3D rotAxis = qRot.v;
	if (rotAxis.length() < TINY)
		return V3D();
	rotAxis.toUnit();
	double rotAngle = qRot.getRotationAngle(rotAxis);

	//this rotation angle is the result of applying the angular velocity for some time dt... 
	return rotAxis * rotAngle / dt;
}

//tests decompose rotation methods. In conjunction with the tests done for decomposition along 3 euler angles, all is good for the three
//computeEulerAnglesFromQuaternion and the two decomposeRotation methods...
inline void testQuaternionDecomposition() {
	for (int i = 0; i < 100; i++) {
		V3D v = getRandomUnitVector();
		V3D a, b;
		v.getOrthogonalVectors(a, b);
		double aVal = getRandomNumberInRange(-IK_PI*0.98, IK_PI*0.98);
		double bVal = getRandomNumberInRange(-IK_PI*0.98, IK_PI*0.98);
		double aValTest = 0, bValTest = 0;
		QuaternionR qTest = getRotationQuaternion(bVal, b) * getRotationQuaternion(aVal, a);
		computeEulerAnglesFromQuaternion(qTest, a, b, aValTest, bValTest);
		QuaternionR qErr = getRotationQuaternion(bValTest, b) * getRotationQuaternion(aValTest, a) * qTest.getComplexConjugate();
		assert(qErr.v.length() < 0.000001);
		assert(IS_EQUAL(aVal, aValTest) && IS_EQUAL(bVal, bValTest));
	}
}

inline Matrix3x3 getRotationMatrixFromEulerAngles(const V3D& eulerAngles, const V3D& axis_1, const V3D& axis_2, const V3D& axis_3)
{
	QuaternionR q0 = getRotationQuaternion(eulerAngles[0], axis_1) *
		getRotationQuaternion(eulerAngles[1], axis_2) *
		getRotationQuaternion(eulerAngles[2], axis_3);

	return q0.getRotationMatrix();
}
