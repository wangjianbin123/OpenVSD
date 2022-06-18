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

// EulerPara 命令目标
// Eulerparameters used in Newton-Euler dynamic formulation construction
// Version 1.0.1 by Jianbin Wang

class EulerPara : public CObject
{
public:
	EulerPara();
	EulerPara(double theta0,double theta1,double theta2,double theta3);
	EulerPara(VectorN &vecN);
	EulerPara(double theta,Vector3x &vec3);
	EulerPara(const EulerPara &ep); //090404 ->add const para

	void Serialize(CArchive &ar);
	DECLARE_SERIAL(EulerPara)

	// 运算符重载
	double & operator[](int dim);
	EulerPara & operator=(const EulerPara &ep); //090404 ->add const para

	double getValue(int i) const {return m_nEP.getValue(i);} //090404 -> add const retriving function
	VectorN getVecN(void) const { return m_nEP;}

	// 欧拉参数函数
	void Empty(void);
	BOOL IsEmpty(void);
	void setEPValue(double t0, double t1, double t2, double t3);
	Matrix getRotMat(void);
	Matrix toMatrix(void);
	VectorN toVectorN(void);
	Matrix getEMat(void);
	Matrix getEconjMat(void);
	Vector3x getRotAxial(void);
	double getRotAngle(void);
	Matrix getGMat(void);
	Matrix getGconjMat(void);

	friend Matrix operator*(Matrix &mx,EulerPara &ep);

	virtual ~EulerPara();
private:
	VectorN m_nEP; 
};
