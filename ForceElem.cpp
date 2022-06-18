// ForceElem.cpp : 实现文件
//
#include "pch.h"
#include "ForceElem.h"

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

#ifndef _INERTIAELEM_H_
#define _INERTIAELEM_H_
#include "InertiaElem.h"
#endif

// ForceElem

ForceElem::ForceElem():sn(0),m_name("")
{
	from=NULL;
	marker_from=0;
	to=NULL;
	marker_to=0;
}
ForceElem::ForceElem(int n,CString name,InertiaElem* f,int fmarker,InertiaElem* t,int tmarker):sn(n),m_name(name)
{
	from=f;
	marker_from=fmarker;
	to=t;
	marker_to=tmarker;
}
ForceElem::ForceElem(const ForceElem &fe)
{
	sn=fe.getSN();
	m_name=fe.getName();
	from=fe.from;
	marker_from=fe.marker_from;
	to=fe.to;
	marker_to=fe.marker_to;
}
ForceElem& ForceElem::operator =(const ForceElem& fe)
{
	if (this==&fe)
		return *this;
	else
	{
		sn=fe.getSN();
		m_name=fe.getName();
		from=fe.from;
		marker_from=fe.marker_from;
		to=fe.to;
		marker_to=fe.marker_to;
		return *this;
	}
}
void ForceElem::setName(CString name)
{
	m_name=name;
}
std::vector<VectorN> ForceElem::AssForceVec(Vector3x &fr,Vector3x &frd,EulerAngle &ftheta,EulerAngle &fthetad,Vector3x &tr,Vector3x &trd,EulerAngle &ttheta,EulerAngle &tthetad,double splineindex)
{
	std::vector<VectorN> TMP;
	return TMP;
}
ForceElem::~ForceElem()
{
	//delete from;
	//delete to;
	from=NULL;
	to=NULL;
}

// ForceElem 成员函数
