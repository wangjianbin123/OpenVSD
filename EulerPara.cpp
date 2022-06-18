// EulerPara.cpp : 实现文件
//
#include "pch.h"
#include "EulerPara.h"

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

#ifndef _MATH_H_
#define _MATH_H_
#include "math.h"
#endif

IMPLEMENT_SERIAL(EulerPara,CObject,VERSIONABLE_SCHEMA|2)

// EulerPara

//构造函数
EulerPara::EulerPara()
{
	m_nEP.SetVectorN(4);
}
EulerPara::EulerPara(double theta0, double theta1, double theta2, double theta3)
{
	m_nEP.SetVectorN(4);
    (*this)[0]=theta0;
    (*this)[1]=theta1;
    (*this)[2]=theta2;
    (*this)[3]=theta3;
}
EulerPara::EulerPara(VectorN &vecN)
{
	m_nEP.SetVectorN(4);
	for (int i=0;i<4;i++)
	{
		(*this)[i]=vecN[i];
	}
}
EulerPara::EulerPara(double theta, Vector3x &vec3)
{
	m_nEP.SetVectorN(4);
	(*this)[0]=theta;
	for (int i=0;i<3;i++)
	{
		(*this)[i+1]=vec3[i];
	}
}
EulerPara::EulerPara(const EulerPara &ep)
{
	m_nEP.SetVectorN(4);
	for (int i=0;i<4;i++)
	{
		(*this)[i]=ep.getValue(i);
	}
}

//析构函数
EulerPara::~EulerPara()
{
	this->Empty();
}

//文件流
void EulerPara::Serialize(CArchive &ar)
{
	CObject::Serialize(ar);
	m_nEP.Serialize(ar);
}

//操作符重载
double & EulerPara::operator [](int dim)
{
	return m_nEP[dim];
}
EulerPara & EulerPara::operator =(const EulerPara &ep)
{
	if (this==&ep)
	{
		return *this;
	}
	for (int i=0;i<4;i++)
	{
		(*this)[i]=ep.getValue(i);
	}
	return *this;
}
// EulerPara 成员函数
void EulerPara::Empty(void)
{
	m_nEP.Empty();
}
BOOL EulerPara::IsEmpty(void)
{
	return m_nEP.IsEmpty();
}
void EulerPara::setEPValue(double t0, double t1, double t2, double t3)
{
	(*this)[0]=t0;
    (*this)[1]=t1;
    (*this)[2]=t2;
    (*this)[3]=t3;
}
Matrix EulerPara::toMatrix(void)
{
	Matrix mx(4,1);
	for (int i=0;i<4;i++)
	{
		mx[i][0]=(*this)[i];
	}
	return mx;
}
VectorN EulerPara::toVectorN(void)
{
	return this->getVecN();
}
Matrix EulerPara::getRotMat(void)
{
	Matrix mx(3,3);
	double t0=(*this)[0];
	double t1=(*this)[1];
	double t2=(*this)[2];
	double t3=(*this)[3];
	mx[0][0]=1-2*t2*t2-2*t3*t3;
	mx[0][1]=2*(t1*t2-t0*t3);
	mx[0][2]=2*(t1*t3+t0*t2);
	mx[1][0]=2*(t1*t2+t0*t3);
	mx[1][1]=1-2*t1*t1-2*t3*t3;
	mx[1][2]=2*(t2*t3-t0*t1);
	mx[2][0]=2*(t1*t3-t0*t2);
	mx[2][1]=2*(t2*t3+t0*t1);
	mx[2][2]=1-2*t1*t1-2*t2*t2;
	return mx;
}
Matrix EulerPara::getEMat(void)
{
	Matrix mx(3,4);
	double t0=(*this)[0];
	double t1=(*this)[1];
	double t2=(*this)[2];
	double t3=(*this)[3];
	mx[0][0]=(-1)*t1;
	mx[0][1]=t0;
	mx[0][2]=(-1)*t3;
	mx[0][3]=t2;
	mx[1][0]=(-1)*t2;
	mx[1][1]=t3;
	mx[1][2]=t0;
	mx[1][3]=(-1)*t1;
	mx[2][0]=(-1)*t3;
	mx[2][1]=(-1)*t2;
	mx[2][2]=t1;
	mx[2][3]=t0;
	return mx;
}
Matrix EulerPara::getEconjMat(void)
{
	Matrix mx(3,4);
	double t0=(*this)[0];
	double t1=(*this)[1];
	double t2=(*this)[2];
	double t3=(*this)[3];
	mx[0][0]=(-1)*t1;
	mx[0][1]=t0;
	mx[0][2]=t3;
	mx[0][3]=(-1)*t2;
	mx[1][0]=(-1)*t2;
	mx[1][1]=(-1)*t3;
	mx[1][2]=t0;
	mx[1][3]=t1;
	mx[2][0]=(-1)*t3;
	mx[2][1]=t2;
	mx[2][2]=(-1)*t1;
	mx[2][3]=t0;
	return mx;
}
Matrix EulerPara::getGMat(void)
{
	Matrix Gmx(3,4);
	Gmx=this->getEMat()*2;
	return Gmx;
}
Matrix EulerPara::getGconjMat(void)
{
	Matrix Gconjmx(3,4);
	Gconjmx=this->getEconjMat()*2;
	return Gconjmx;
}
Matrix operator*(Matrix &mx,EulerPara &ep)
{
	Matrix tmp=mx*ep.toMatrix();
	return tmp;
}



