// TPFElem.cpp : 实现文件
//
#include "pch.h"
#include "TPFElem.h"


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

#ifndef _EULERANGLE_H_
#define _EULERANGLE_H_
#include "EulerAngle.h"
#endif

#ifndef _EULERPARA_H_
#define _EULERPARA_H_
#include "EulerPara.h"
#endif

#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_
#include "CubicSpline.h"
#endif

#ifndef _MARKER_H_
#define _MARKER_H_
#include "Marker.h"
#endif


// TPFElem

TPFElem::TPFElem()
{
}
TPFElem::TPFElem(int flag, CubicSpline &f, Marker &from, Marker &to, Marker &at):FLAG(flag),
force(f),i(from),j(to),a(at)
{
}
TPFElem::TPFElem(const TPFElem &tpf)
{
	FLAG=tpf.getflag();
	force=tpf.getforce();
	i=tpf.getfrom();
	j=tpf.getto();
	a=tpf.getat();
}
TPFElem& TPFElem::operator =(const TPFElem &tpf)
{
	if (this==&tpf)
	{
		return *this;
	}
	else
	{
		FLAG=tpf.getflag();
		force=tpf.getforce();
		i=tpf.getfrom();
		j=tpf.getto();
		a=tpf.getat();
		return *this;
	}
}
Vector3x TPFElem::getForceVec(double splineindex)
{
	Vector3x orientation=j.getOrigin()-i.getOrigin();
	orientation.toUnit();
	double f=force.ValueAt(splineindex);
	Vector3x tmp=orientation*f;
	return tmp;
}
TPFElem::~TPFElem()
{
}


// TPFElem 成员函数
