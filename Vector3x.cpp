// Vector3x.cpp : 实现文件
//
#include "pch.h"
#include "Vector3x.h"

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif

#ifndef _MATH_H_
#define _MATH_H_
#include <math.h>
#endif


IMPLEMENT_SERIAL(Vector3x,CObject,VERSIONABLE_SCHEMA|2)

//构造函数
Vector3x::Vector3x()
{
	m_vec.SetVectorN(3);
}
Vector3x::Vector3x(double Rx, double Ry, double Rz)
{
	m_vec.SetVectorN(3);
	m_vec[0]=Rx;
	m_vec[1]=Ry;
	m_vec[2]=Rz;
}
Vector3x::Vector3x(VectorN &vec)
{
	m_vec.SetVectorN(3);
	for (int i=0;i<3;i++)
	{
		m_vec[i]=vec[i];
	}
}

Vector3x::Vector3x(const Vector3x &sc)
{
	m_vec.SetVectorN(3);
	for (int i=0;i<3;i++)
	{
		m_vec[i]=sc.getValue(i);
	}
}

//单目操作符重载
Vector3x& Vector3x::operator =(const Vector3x &vec3)
{
	if (this==&vec3)
	{
		return *this;
	}
	else
	{
		m_vec=vec3.getVecN();
		return (*this);
	}
}
Vector3x& Vector3x::operator +=(Vector3x &vec3)
{
	m_vec+=vec3.getVecN();
	return (*this);
}
Vector3x& Vector3x::operator -=(Vector3x &vec3)
{
	m_vec-=vec3.getVecN();
	return (*this);
}
Vector3x& Vector3x::operator *=(double k)
{
	m_vec*=k;
	return (*this);
}

double& Vector3x::operator [](int dim)
{
	return m_vec[dim];
}

//文件流操作
void Vector3x::Serialize(CArchive &ar)
{
	CObject::Serialize(ar);
	m_vec.Serialize(ar);
}
	
//析构函数
Vector3x::~Vector3x()
{
	this->Empty();
}

// Vector3x 成员函数
void Vector3x::Empty(void)
{
	m_vec.Empty();
}
double Vector3x::getRx(void)
{
	return m_vec[0];
}
double Vector3x::getRy(void)
{
	return m_vec[1];
}
double Vector3x::getRz(void)
{
	return m_vec[2];
}
double Vector3x::getNorm(void)
{
	return m_vec.getNorm();
}
void Vector3x::reSet(double x,double y,double z)
{
	m_vec[0]=x;
	m_vec[1]=y;
	m_vec[2]=z;
}
void Vector3x::toUnit(void)
{
	double norm=m_vec.getNorm();
	for (int i=0;i<3;i++)
	{
		(*this)[i]/=norm;
	}
}
Matrix Vector3x::toSkewMatrix(void)
{
	Matrix tmpMx(3,3);
	tmpMx[0][1]=m_vec[2]*(-1);
	tmpMx[1][0]=m_vec[2];
	tmpMx[0][2]=m_vec[1];
	tmpMx[2][0]=m_vec[1]*(-1);
	tmpMx[1][2]=m_vec[0]*(-1);
	tmpMx[2][1]=m_vec[0];

	return tmpMx;
}
VectorN Vector3x::toVectorN(void)
{
	return m_vec;
}
Matrix Vector3x::toMatrix(void)
{

	return m_vec.toMatrix();
}
double Vector3x::dotProduct(Vector3x &vec3)
{
	double tmp=0.0;
	for (int i=0;i<3;i++)
	{
		tmp+=(*this)[i]*vec3[i];
	}

	return tmp;
}
Vector3x Vector3x::crossProduct(Vector3x &vec3)
{
	Vector3x tmpVec((m_vec[1]*vec3[2]-m_vec[2]*vec3[1]),(m_vec[2]*vec3[0]-m_vec[0]*vec3[2]),(m_vec[0]*vec3[1]-m_vec[1]*vec3[0]));
	return tmpVec;
}
Vector3x operator +(Vector3x &vec,double k)
{
	Vector3x vec3;
	for (int i=0;i<3;i++)
	{
		vec3[i]=vec[i]+k;
	}
	return vec3;
}

Vector3x operator +(double k,Vector3x &vec)
{
	Vector3x vec3;
	for (int i=0;i<3;i++)
	{
		vec3[i]=vec[i]+k;
	}
	return vec3;
}

Vector3x operator +(Vector3x &vec1, Vector3x &vec2)
{
	Vector3x vec3;
	for (int i=0;i<3;i++)
	{
		vec3[i]=vec1[i]+vec2[i];
	}
	return vec3;
}


Vector3x operator -(Vector3x &vec,double k)
{
	Vector3x vec3;
	for (int i=0;i<3;i++)
	{
		vec3[i]=vec[i]-k;
	}
	return vec3;
}

Vector3x operator -(double k,Vector3x &vec)
{
	Vector3x vec3;
	for (int i=0;i<3;i++)
	{
		vec3[i]=k-vec[i];
	}
	return vec3;
}

Vector3x operator -(Vector3x &vec1, Vector3x &vec2)
{
	Vector3x vec3;
	for (int i=0;i<3;i++)
	{
		vec3[i]=vec1[i]-vec2[i];
	}
	return vec3;
}

Vector3x operator *(Vector3x &vec,double k)
{
	Vector3x vec3;
	for (int i=0;i<3;i++)
	{
		vec3[i]=vec[i]*k;
	}
	return vec3;
}

Vector3x operator *(double k,Vector3x &vec)
{
	Vector3x vec3;
	for (int i=0;i<3;i++)
	{
		vec3[i]=k*vec[i];
	}
	return vec3;
}

double operator *(Vector3x &vec1, Vector3x &vec2)
{
	double tmp=0;
	for (int i=0;i<3;i++)
	{
		tmp+=vec1[i]*vec2[i];
	}
	return tmp;
}
Vector3x operator*(Matrix &mx,Vector3x &vec)
{
	Vector3x vecTmp;
	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			vecTmp[i]+=mx[i][j]*vec[j];
		}
	}
	return vecTmp;
}









