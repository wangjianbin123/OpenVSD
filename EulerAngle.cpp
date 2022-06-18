// EulerAngle.cpp : 实现文件
//
#include "pch.h"
#include "EulerAngle.h"

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


// EulerAngle

IMPLEMENT_SERIAL(EulerAngle,CObject,VERSIONABLE_SCHEMA|2)

//构造函数
EulerAngle::EulerAngle()
{
	for (int i=0;i<3;i++)
	{
		m_Angvec[i]=0;
	}
}
EulerAngle::EulerAngle(double Yaw,double Roll,double Pitch)
{
	m_Angvec[0]=Yaw;
	m_Angvec[1]=Roll;
	m_Angvec[2]=Pitch;
}
EulerAngle::EulerAngle(double *pAry)
{
	for (int i=0;i<3;i++)
	{
		m_Angvec[i]=pAry[i];
	}
}
EulerAngle::EulerAngle(Vector3x &vec3)
{
	for (int i=0;i<3;i++)
	{
		m_Angvec[i]=vec3[i];
	}
}
EulerAngle::EulerAngle(VectorN &vecn)
{
	for (int i=0;i<3;i++)
	{
		m_Angvec[i]=vecn[i];
	}
}
EulerAngle::EulerAngle(Matrix &mx)
{
	ASSERT((mx.Row()==3&&mx.Col()==1)||(mx.Row()==1&&mx.Col()==3));
	if (mx.Row()==3)
	{
		for (int i=0;i<3;i++)
		{
			m_Angvec[i]=mx[i][0];
		}
	}
	else
	{
		for (int i=0;i<3;i++)
		{
			m_Angvec[i]=mx[0][i];
		}
	}
}
EulerAngle::EulerAngle(const EulerAngle &angle)
{
	for (int i=0;i<3;i++)
	{
		(*this)[i]=angle.getValue(i);
	}
}
//单目运算符重载
double& EulerAngle::operator [](int dim)
{
	return m_Angvec[dim];
}
EulerAngle& EulerAngle::operator =(const EulerAngle &angle)
{
	if (this==&angle)
	{
		return (*this);
	}
	for (int i=0;i<3;i++)
	{
		(*this)[i]=angle.getValue(i);
	}
	return (*this);
}
EulerAngle& EulerAngle::operator +=(EulerAngle &angle)
{
	for (int i=0;i<3;i++)
	{
		(*this)[i]+=angle[i];
	}
	return (*this);
}
EulerAngle& EulerAngle::operator -=(EulerAngle &angle)
{
	for (int i=0;i<3;i++)
	{
		(*this)[i]-=angle[i];
	}
	return (*this);
}
EulerAngle& EulerAngle::operator *=(double k)
{
	for (int i=0;i<3;i++)
	{
		(*this)[i]*=k;
	}
	return (*this);
}

//文件流操作
void EulerAngle::Serialize(CArchive &ar)
{
	CObject::Serialize(ar);
	m_Angvec.Serialize(ar);
}

EulerAngle::~EulerAngle()
{
	this->Empty();
	//m_Angvec.Empty();
}

// EulerAngle 成员函数
void EulerAngle::Empty(void)
{
	m_Angvec.Empty();
}
Vector3x EulerAngle::getAngle(void)
{
	return m_Angvec;
}
double EulerAngle::getYaw(void)
{
	return m_Angvec[0];
}
double EulerAngle::getRoll(void)
{
	return m_Angvec[1];
}
double EulerAngle::getPitch(void)
{
	return m_Angvec[2];
}
void EulerAngle::reSet(double psi, double phi, double theta)
{
	m_Angvec[0]=psi;
	m_Angvec[1]=phi;
	m_Angvec[2]=theta;
}
Matrix EulerAngle::getRotMat(void)
{
	Matrix mxTmp(3,3);
	mxTmp[0][0]=cos(m_Angvec[0])*cos(m_Angvec[2])-sin(m_Angvec[0])*sin(m_Angvec[1])*sin(m_Angvec[2]);
	mxTmp[0][1]=(-1)*sin(m_Angvec[0])*cos(m_Angvec[1]);
	mxTmp[0][2]=cos(m_Angvec[0])*sin(m_Angvec[2])+sin(m_Angvec[0])*sin(m_Angvec[1])*cos(m_Angvec[2]);
	mxTmp[1][0]=sin(m_Angvec[0])*cos(m_Angvec[2])+cos(m_Angvec[0])*sin(m_Angvec[1])*sin(m_Angvec[2]);
	mxTmp[1][1]=cos(m_Angvec[0])*cos(m_Angvec[1]);
	mxTmp[1][2]=sin(m_Angvec[0])*sin(m_Angvec[2])-cos(m_Angvec[0])*sin(m_Angvec[1])*cos(m_Angvec[2]);
	mxTmp[2][0]=(-1)*cos(m_Angvec[1])*sin(m_Angvec[2]);
	mxTmp[2][1]=sin(m_Angvec[1]);
	mxTmp[2][2]=cos(m_Angvec[1])*cos(m_Angvec[2]);
	return mxTmp;
}
Matrix EulerAngle::getGMat(void)
{
	Matrix mxTmp(3,3);
	mxTmp[0][0]=0;
	mxTmp[0][1]=cos(m_Angvec[0]);
	mxTmp[0][2]=(-1)*sin(m_Angvec[0])*cos(m_Angvec[1]);
	mxTmp[1][0]=0;
	mxTmp[1][1]=sin(m_Angvec[0]);
	mxTmp[1][2]=cos(m_Angvec[0])*cos(m_Angvec[1]);
	mxTmp[2][0]=1.0;
	mxTmp[2][1]=0;
	mxTmp[2][2]=sin(m_Angvec[1]);
	return mxTmp;
}
Matrix EulerAngle::getGconjMat(void)
{
	Matrix mxTmp(3,3);
	mxTmp[0][0]=(-1)*cos(m_Angvec[1])*sin(m_Angvec[2]);
	mxTmp[0][1]=cos(m_Angvec[2]);
	mxTmp[0][2]=0;
	mxTmp[1][0]=sin(m_Angvec[1]);
	mxTmp[1][1]=0;
	mxTmp[1][2]=1;
	mxTmp[2][0]=cos(m_Angvec[1])*cos(m_Angvec[2]);
	mxTmp[2][1]=sin(m_Angvec[2]);
	mxTmp[2][2]=0;
	return mxTmp;
}
VectorN EulerAngle::toVectorN(void)
{
	return m_Angvec.toVectorN();
}
Vector3x EulerAngle::toVector3x(void)
{
	return m_Angvec;
}





