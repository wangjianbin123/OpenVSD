// Marker.cpp : 实现文件
//
#include "pch.h"
#include "Marker.h"

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

#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_
#include "CubicSpline.h"
#endif

#ifndef _EULERANGLE_H_
#define _EULERANGLE_H_
#include "EulerAngle.h"
#endif

// Marker
IMPLEMENT_SERIAL(Marker,CObject,VERSIONABLE_SCHEMA|2)

Marker::Marker()
{
	num=0;
	Origin.Empty();
	Orientation.Empty();
}
Marker::Marker(int n, Vector3x &vec, EulerAngle &ea):num(n),Origin(vec),Orientation(ea)
{
}
Marker::Marker(int n, double a1, double a2, double a3, double b1, double b2, double b3):num(n),Origin(a1,a2,a3),Orientation(b1,b2,b3)
{
}
Marker::Marker(const Marker &mk)
{
	num=mk.getNum();
	Origin=mk.getOrigin();
	Orientation=mk.getOrientation();
}
void Marker::Serialize(CArchive &ar)
{
	CObject::Serialize(ar);
	if (ar.IsStoring()==TRUE)
	{
		ar<<num;
		Origin.Serialize(ar);
		Orientation.Serialize(ar);
	}
	else
	{
		int tmp;
		ar>>tmp;
		num=tmp;
		Origin.Serialize(ar);
		Orientation.Serialize(ar);
	}
}
Marker& Marker::operator =(const Marker &mk)
{
	if (this==&mk)
	{
		return *this;
	}
	else
	{
		num=mk.getNum();
		Origin=mk.getOrigin();
		Orientation=mk.getOrientation();
		return *this;
	}
}
void Marker::update(Vector3x &vec,EulerAngle &ea)
{
	Origin.reSet(vec[0],vec[1],vec[2]);
	Orientation.reSet(ea[0],ea[1],ea[2]);
}
void Marker::update(double a1, double a2, double a3, double b1, double b2, double b3)
{
	Origin.reSet(a1,a2,a3);
	Orientation.reSet(b1,b2,b3);
}
Marker::~Marker()
{
}


// Marker 成员函数
