// InertiaElem.cpp : 实现文件
//
#include "pch.h"
#include "InertiaElem.h"

#ifndef _MATH_H_
#define _MATH_H_
#include <math.h>
#endif

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

#ifndef _EULERPARA_H_
#define _EULERPARA_H_
#include "EulerPara.h"
#endif

#ifndef _MARKER_H_
#define _MARKER_H_
#include "Marker.h"
#endif

#include <vector>

// InertiaElem
//IMPLEMENT_SERIAL(InertiaElem,CObject,VERSIONABLE_SCHEMA|2)

InertiaElem::InertiaElem()
{
	SN=0;
	m_name="";
	FLEX=FALSE;
	NG=FALSE;
	m=0.0;
	principle_I.Empty();
	principle_mat.Empty();
	g.reSet(0.0,0.0,-9.81);
	wsmarker.clear();
}
InertiaElem::InertiaElem(int number,CString name,BOOL flexible,BOOL NotGround,double mass,Vector3x &I)
{
	SN=number;
	m_name=name;
	FLEX=flexible;
	NG=NotGround;
	m=mass;
	principle_I=I;
	g.reSet(0.0,0.0,-9.81);
	principle_mat.SetMatrix(3,3);
	for (int i=0;i<3;i++)
	{
		principle_mat[i][i]=principle_I[i];
	}
	Vector3x r(0.0,0.0,0.0);
	EulerAngle ea(0.0,0.0,0.0);
	Marker CG(0,r,ea);
	this->addMarker(CG);
}
InertiaElem::InertiaElem(const InertiaElem &iner)
{
	SN=iner.getSN();
	m_name=iner.getName();
	FLEX=iner.getFlexible();
	NG=iner.getGround();
	m=iner.getMass();
	principle_I=iner.getPrincipleI();
	principle_mat=iner.getPrincipleMat();
	g=iner.getGravity();
	wsmarker=iner.getMarkerVector();
}
InertiaElem& InertiaElem::operator=(const InertiaElem &iner)
{
	if (this==&iner)
	{
		return *this;
	}
	else
	{
		SN=iner.getSN();
		m_name=iner.getName();
		FLEX=iner.getFlexible();
		NG=iner.getGround();
		m=iner.getMass();
		principle_I=iner.getPrincipleI();
		principle_mat=iner.getPrincipleMat();
		g=iner.getGravity();
		wsmarker=iner.getMarkerVector();
		return *this;
	}
}
Marker InertiaElem::getMarker(int num)
{
	return wsmarker[num];
}
void InertiaElem::addMarker(Marker &mk)
{
	wsmarker.push_back(mk);
}
Vector3x InertiaElem::getMarker_Pos(int num,Vector3x &r,EulerAngle &theta)
{
	Vector3x TMP;
	return TMP;
}
Vector3x InertiaElem::getMarker_Vel(int num,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad)
{
	Vector3x TMP;
	return TMP;
}
EulerAngle InertiaElem::getEulerAngle(void)
{
	EulerAngle TMP;
	return TMP;
}
Vector3x InertiaElem::getMarkerLocalVec(int num)
{
	Vector3x TMP;
	return TMP;
}
Matrix InertiaElem::AssMassMat(EulerAngle &theta)
{
	Matrix TMP;
	return TMP;
}
VectorN InertiaElem::AssQuadForceVec(EulerAngle &theta,EulerAngle &thetad)
{
	Matrix TMP;
	return TMP;
}
VectorN InertiaElem::AssExternalForce(EulerAngle &theta,double splineindex)
{
	Matrix TMP;
	return TMP;
}
VectorN InertiaElem::AssExternalTorque(EulerAngle &theta,double splineindex)
{
	Matrix TMP;
	return TMP;
}
VectorN InertiaElem::AssembleForceVector(Vector3x &r,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad,double splineindex)
{
	VectorN TMP;
	return TMP;
}
VectorN InertiaElem::getCurrentKinematicPara(void)
{
	VectorN TMP;
	return TMP;
}
VectorN InertiaElem::getCurrentForcePara(void)
{
	VectorN TMP;
	return TMP;
}
InertiaElem::~InertiaElem()
{
}
// InertiaElem 成员函数
void InertiaElem::SetName(CString name)
{
	m_name=name;
}
