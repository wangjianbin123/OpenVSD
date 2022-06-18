#pragma once

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif

#ifndef _VECTOR3X_H_
#define _VECTOR3X_H_
#include "Vector3x.h"
#endif

// EulerAngle ÃüÁîÄ¿±ê
// Euler Angles commonly used in vehicle simulations. ZXY(3->1->2) successive rotations.
// Version 1.0.1 by Jianbin Wang

class EulerAngle : public CObject
{
public:
	EulerAngle();
	EulerAngle(double Yaw, double Roll, double Pitch);
	EulerAngle(double *pAry);
	EulerAngle(VectorN &vecn);
	EulerAngle(Vector3x &vec3);
	EulerAngle(Matrix &mx);
	EulerAngle(const EulerAngle &angle);

	void Serialize(CArchive &ar);
	DECLARE_SERIAL(EulerAngle)

	double getValue(int i) const { return m_Angvec.getValue(i);}
    
	double& operator[](int dim);
	EulerAngle& operator=(const EulerAngle &angle);
	EulerAngle& operator+=(EulerAngle &angle);
	EulerAngle& operator-=(EulerAngle &angle);
	EulerAngle& operator*=(double k);

	void Empty(void);
	Vector3x getAngle(void);
	double getYaw(void);
	double getPitch(void);
	double getRoll(void);
	void reSet(double psi,double phi,double theta);
	Matrix getRotMat(void);
	Matrix getGMat(void);
	Matrix getGconjMat(void);
	VectorN toVectorN(void);
	Vector3x toVector3x(void);

	virtual ~EulerAngle();
private:
	Vector3x m_Angvec;
};


