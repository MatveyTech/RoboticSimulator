#include "../include/Quaternion.h"
#include "../include/Utils.h"
#include <sstream>

//	A constructor. Intuitive...
QuaternionR::QuaternionR(double s, const V3D& v) {
	this->s = s;
	this->v = v;
}

// A copy constructor
QuaternionR::QuaternionR(const QuaternionR& other) {
	this->s = other.s;
	this->v = other.v;
}

// Default constructor - no rotation
QuaternionR::QuaternionR() {
	this->s = 1;
	this->v = V3D(0, 0, 0);
}

// Another constructor.
QuaternionR::QuaternionR(double w, double x, double y, double z) {
	this->s = w;
	this->v = V3D(x, y, z);
}

// destructor.
QuaternionR::~QuaternionR(void) {
}

// A copy operator
QuaternionR& QuaternionR::operator = (const QuaternionR &rhs) {
	this->s = rhs.s;
	this->v = rhs.v;
	return *this;
}

// returns the result of multiplying the current quaternion by rhs
QuaternionR QuaternionR::operator * (const QuaternionR &rhs) const{
	QuaternionR q = *this;
	q *= rhs;
	return q;
}

// multiply the current quaternion by the rhs one.
QuaternionR& QuaternionR::operator *= (const QuaternionR &rhs) {
	double newS = this->s * rhs.s - this->v.dot(rhs.v);
	V3D newV = rhs.v * this->s + this->v * rhs.s + this->v.cross(rhs.v);
	this->s = newS;
	this->v = newV;
	return *this;
}

// multiplies the quaternion by a scalar
QuaternionR& QuaternionR::operator *= (double scalar) {
	this->s *= scalar;
	this->v *= scalar;
	return *this;
}

// divides the quaternion by a scalar
QuaternionR& QuaternionR::operator /= (double scalar) {
	this->s /= scalar;
	this->v /= scalar;
	return *this;
}

// return a copy of the quaternion, multiplied by a scalar
QuaternionR QuaternionR::operator * (double scalar) const {
	return QuaternionR(s * scalar, v * scalar);
}

// returns the result of dividing the current quaternion by rhs
QuaternionR QuaternionR::operator / (double scalar) const {
	return QuaternionR(s / scalar, v * scalar);
}

// returns sum of *this and rhs
QuaternionR QuaternionR::operator + (const QuaternionR &rhs) const {
	return QuaternionR(s + rhs.s, v + rhs.v);
}

double& QuaternionR::operator[](const int _id)
{
	assert(_id >= 0 && _id <= 3);
	if (_id == 0)
		return s;
	else
		return v[_id - 1];
}

bool QuaternionR::operator==(const QuaternionR& _q)
{
	bool bAns = true;
	bAns &= abs(_q.s - s)<TINY;
	bAns &= (_q.v - v).length()<TINY;
	return bAns;
}

bool QuaternionR::operator!=(const QuaternionR& _q)
{
	return !(*this == _q);
}

// adds rhs to this quaternion
QuaternionR& QuaternionR::operator += (const QuaternionR &rhs) {
	this->s += rhs.s;
	this->v += rhs.v;
	return *this;
}

// rotates vector v by this quaternion
V3D QuaternionR::operator * (const V3D &v) const {
	return rotate(v);
}

// normalize the quaternion
QuaternionR& QuaternionR::toUnit() {
	*this /= this->length();
	return *this;
}

// returns the rotation angle represented by this quaternion - in the range -pi to pi. Since q and -q are the same rotation, we need to know which side is "up" explicitly
double QuaternionR::getRotationAngle(const V3D& positiveRotAxis) {
	int sinSign = SGN(positiveRotAxis.dot(v));
	double result = 2 * safeACOS(s);
	if (sinSign < 0)
		result = -result;
	if (result > IK_PI) result -= 2 * IK_PI;
	if (result < -IK_PI) result += 2 * IK_PI;
	return result;
}

// Returns the complex conjugate of the current quaternion.
QuaternionR QuaternionR::getComplexConjugate() const {
	return QuaternionR(s, -v);
}

//	Returns the inverse of the current quaternion: q * q^-1 = identity quaternion: s = 1, v = (0,0,0)
QuaternionR QuaternionR::getInverse() const {
	return this->getComplexConjugate() / length2();
}

//	Returns the length of this quaternion.
double QuaternionR::length() const {
	return sqrt(s*s + v.dot(v));
}

double QuaternionR::length2() const {
	return s*s + v.dot(v);
}

// Computes the dot product between the current quaternion and the one given as parameter.
double QuaternionR::dot(const QuaternionR &other) const {
	return (this->s * other.s + this->v.dot(other.v));
}

// linear interpolation (don't do it!). If t = 0, the result is *this. If t = 1, result is other.
QuaternionR QuaternionR::linearlyInterpolateWith(const QuaternionR &other, double t) const {
	if (t<0) t = 0;
	if (t>1) t = 1;
	QuaternionR result = (*this)*(1 - t) + other*t;
	return result * (1 / result.length());
}

// SLERP (spherical linear interpolation). If t = 0, the result is *this. If t = 1, result is other.
QuaternionR QuaternionR::sphericallyInterpolateWith(const QuaternionR &other, double t) const {
	QuaternionR result;
	sphericallyInterpolateWith(other, t, result);
	return result;
}

// SLERP (spherical linear interpolation). If t = 0, the result is *this. If t = 1, result is other.
void QuaternionR::sphericallyInterpolateWith(const QuaternionR &other, double t, QuaternionR &result) const {
	//make sure that we return the same value if either of the quaternions involved is q or -q 
	if (this->dot(other) < 0) {
		QuaternionR temp;
		temp.s = -other.s;
		temp.v = -other.v;
		this->sphericallyInterpolateWith(temp, t, result);
		return;
	}

	if (t<0) t = 0;
	if (t>1) t = 1;
	double dotProduct = this->dot(other);
	double sinTheta = sqrt(MAX(0, 1 - dotProduct*dotProduct));
	if (sinTheta == 0)
		result = *this;
	else{
		double invSinTheta = 1.0 / sinTheta;
		double theta = safeACOS(dotProduct);
		double fN = sin(theta * (1 - t)) * invSinTheta;
		double fP = sin(theta * t) * invSinTheta;

		result.s = this->s*fN + other.s*fP;
		result.v = this->v*fN + other.v*fP;
	}
}

// return a rotated version of u
V3D QuaternionR::rotate(const V3D& u) const {
	V3D res;
	rotate(u, &res);
	return res;
}

// rotate u by the inverse of the quaternion
V3D QuaternionR::inverseRotate(const V3D& u) const {
	V3D res;
	inverseRotate(u, &res);
	return res;
}

// target will be set to u rotated by this quaternion. 'target' cannot point at u
void QuaternionR::rotate(const V3D& u, V3D* target) const {
	//uRot = q * (0, u) * q' = (s, v) * (0, u) * (s, -v)
	//working it out we get:
	V3D t = v.cross(u) + u * s;
	*target = v.cross(t) + t*s + v*u.dot(v);
}

// target will be set to u rotated by this quaternion. 'target' cannot point at u
void QuaternionR::inverseRotate(const V3D& u, V3D *target) const {
	//uRot = q * (0, u) * q' = (s, -v) * (0, u) * (s, v)
	//working it out we get:
	V3D t = u.cross(v) + u * s;
	*target = t.cross(v) + t * s + v * u.dot(v);
}

// This method returns the quaternion in axis-angle representation.
void QuaternionR::getAxisAngle(V3D &axis, double &angle) const {
	if (fabs(fabs(s) - 1.0) < TINY){
		angle = 0.0;
		axis = V3D(1,0,0);
	}else{
		angle = 2 * safeACOS(s);
		axis = v * (1.0 / sqrt(1 - s*s));
		axis.toUnit();
//		assert(IS_ZERO(axis.length() - 1));
	}
}

// sets value of rotation quaternion from arguments
void QuaternionR::setRotationFrom(double angle, const V3D& axis) {
	s = cos(angle / 2.0);
	v = axis * sin(angle / 2.0);
}

// sets value of rotation quaternion from arguments - the magnitude of the vector encodes the rotation angle
void QuaternionR::setRotationFrom(const V3D& vec) {
	double len = vec.length();
	if (IS_ZERO(len)) {
		s = 1;
		v = V3D();
	}
	else
		setRotationFrom(len, vec / len);
}

QuaternionR QuaternionR::partial(double _dt) const
{
	return SetRotationFrom(logMap()*_dt, v.normalized());
}

string QuaternionR::ToString()
{
	V3D axis;
	double angle;
	getAxisAngle(axis, angle);

	ostringstream oss;
	oss << "The axis is:" << axis << " . The angle is : " << DEG(angle);
	return oss.str();
}

const QuaternionR QuaternionR::SetRotationFrom(const double _a, const V3D& _axis)
{
	QuaternionR q;
	q.setRotationFrom(_a, _axis);
	return q;
}

const QuaternionR QuaternionR::SetRotationFrom(const V3D& _axis)
{
	QuaternionR q;
	q.setRotationFrom(_axis);
	return q;
}

// sets value of rotation quaternion from coordinate frame passed in as arguments. The vectors passed in are the x, y and z axis of a local 
//coordinate frame expressed in the global coordinate frame. The resulting quaternion will convert vectors from the local frame to the global frame
void QuaternionR::setRotationFrom(const V3D& xRot, const V3D& yRot, const V3D& zRot) {
	assert(IS_ZERO(xRot.length()-1) && IS_ZERO(yRot.length()-1) && IS_ZERO(zRot.length()-1) && IS_ZERO(xRot.dot(yRot)) && IS_ZERO(yRot.dot(zRot)) && IS_ZERO(zRot.dot(xRot)));

	double mat[9] = {xRot[0], yRot[0], zRot[0], 
					 xRot[1], yRot[1], zRot[1],
					 xRot[2], yRot[2], zRot[2]};

#define MAT(i, j) (mat[i*3+j])

	double tr = MAT(0, 0) + MAT(1, 1) + MAT(2, 2);

	if (tr > 0) {
		double S = sqrt(tr + 1) * 2;
		this->s = 0.25*S;
		this->v[0] = (MAT(2, 1) - MAT(1, 2)) / S;
		this->v[1] = (MAT(0, 2) - MAT(2, 0)) / S;
		this->v[2] = (MAT(1, 0) - MAT(0, 1)) / S;
	}
	else if (MAT(0, 0)>MAT(1, 1) && MAT(0, 0)>MAT(2, 2)) {
		double S = sqrt(1 + MAT(0, 0) - MAT(1, 1) - MAT(2, 2)) * 2;
		this->s = (MAT(2, 1) - MAT(1, 2)) / S;
		this->v[0] = 0.25*S;
		this->v[1] = (MAT(0, 1) + MAT(1, 0)) / S;
		this->v[2] = (MAT(0, 2) + MAT(2, 0)) / S;
	}
	else if (MAT(1, 1) > MAT(2, 2)) {
		double S = sqrt(1 + MAT(1, 1) - MAT(0, 0) - MAT(2, 2)) * 2;
		this->s = (MAT(0, 2) - MAT(2, 0)) / S;
		this->v[0] = (MAT(0, 1) + MAT(1, 0)) / S;
		this->v[1] = 0.25*S;
		this->v[2] = (MAT(1, 2) + MAT(2, 1)) / S;
	}
	else {
		double S = sqrt(1 + MAT(2, 2) - MAT(0, 0) - MAT(1, 1)) * 2;
		this->s = (MAT(1, 0) - MAT(0, 1)) / S;
		this->v[0] = (MAT(0, 2) + MAT(2, 0)) / S;
		this->v[1] = (MAT(1, 2) + MAT(2, 1)) / S;
		this->v[2] = 0.25*S;
	}

	//now test the result...
	assert(this->rotate(V3D(1, 0, 0)) == xRot && this->rotate(V3D(0, 1, 0)) == yRot && this->rotate(V3D(0, 0, 1)) == zRot);
}

//sets the 3x3 top-left corner of a matrix with a rotation cooresponding to this quaternion
Matrix3x3 QuaternionR::getRotationMatrix() const{
	double w = s, x = v[0], y = v[1], z = v[2];

	Matrix3x3 res;
	res << 1 - 2 * y*y - 2 * z*z,	2 * x*y - 2 * w*z,	2 * x*z + 2 * w*y,
		2 * x*y + 2 * w*z,	1 - 2 * x*x - 2 * z*z,	2 * y*z - 2 * w*x,
		2 * x*z - 2 * w*y,	2 * y*z + 2 * w*x,	1 - 2 * x*x - 2 * y*y;
	return res;
}

Vector3d QuaternionR::toAxisAngle() const
{
	double l = v.length();
	if (IS_ZERO(l))
		return Vector3d(0, 0, 0);
	else
		return (v / l)*logMap();
}

//sets value of rotation quaternion from the matrix that is passed in as a parameter
void QuaternionR::setRotationFrom(const Matrix3x3& mat){
	double tr = mat(0, 0) + mat(1, 1) + mat(2, 2);

	if (tr > 0) {
		double S = sqrt(tr + 1) * 2;
		s = 0.25*S;
		v[0] = (mat(2, 1) - mat(1, 2)) / S;
		v[1] = (mat(0, 2) - mat(2, 0)) / S;
		v[2] = (mat(1, 0) - mat(0, 1)) / S;
	}
	else if (mat(0, 0)>mat(1, 1) && mat(0, 0)>mat(2, 2)) {
		double S = sqrt(1 + mat(0, 0) - mat(1, 1) - mat(2, 2)) * 2;
		s = (mat(2, 1) - mat(1, 2)) / S;
		v[0] = 0.25*S;
		v[1] = (mat(0, 1) + mat(1, 0)) / S;
		v[2] = (mat(0, 2) + mat(2, 0)) / S;
	}
	else if (mat(1, 1) > mat(2, 2)) {
		double S = sqrt(1 + mat(1, 1) - mat(0, 0) - mat(2, 2)) * 2;
		s = (mat(0, 2) - mat(2, 0)) / S;
		v[0] = (mat(0, 1) + mat(1, 0)) / S;
		v[1] = 0.25*S;
		v[2] = (mat(1, 2) + mat(2, 1)) / S;
	}
	else {
		double S = sqrt(1 + mat(2, 2) - mat(0, 0) - mat(1, 1)) * 2;
		s = (mat(1, 0) - mat(0, 1)) / S;
		v[0] = (mat(0, 2) + mat(2, 0)) / S;
		v[1] = (mat(1, 2) + mat(2, 1)) / S;
		v[2] = 0.25*S;
	}

	toUnit();
}

QuaternionR getRotationAxisThatAlignsVectors(const V3D& a, const V3D& b) {
	V3D aUnit = a.unit();
	V3D bUnit = b.unit();
	//we first need a rotation that gets dir to be aligned with the y-axis...
	V3D rotAxis = aUnit.cross(bUnit);
	if (IS_ZERO(rotAxis.length()))
		return QuaternionR();
	rotAxis.toUnit();
	double rotAngle = a.angleWith(b, rotAxis);
	return getRotationQuaternion(rotAngle, rotAxis);
}

