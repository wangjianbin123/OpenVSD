#pragma once

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

// DOFConstraint 命令目标

class DOFConstraint : public CObject
{
public:
	DOFConstraint(void);
	DOFConstraint(int n, CString name,InertiaElem *bodyi,InertiaElem *bodyj);
	DOFConstraint(const DOFConstraint& dofcons);
	DOFConstraint& operator=(const DOFConstraint& dofcons);
	void setName(CString name) { m_name=name; }

	//void Serialize(CArchive &ar);
	//DECLARE_SERIAL(DOFConstraint)
	InertiaElem* getbodyi(void) const {return body_i;};//20130715 added
	InertiaElem* getbodyj(void) const {return body_j;};//20130715 added


	//虚函数
	virtual Matrix AssJacobian(Vector3x &Ri,EulerAngle &Thetai,Vector3x &Rj,EulerAngle &Thetaj);
	virtual Matrix AssConstraintEquations(Vector3x &Ri, EulerAngle &Thetai, Vector3x &Rj, EulerAngle &Thetaj);
	virtual int getType(void) const {return 0;}
	virtual int getDecrDofs(void) const {return 0;}; 

	//克隆操作
	virtual DOFConstraint* clone(void) const { return new DOFConstraint(*this); }
	virtual ~DOFConstraint();
protected:
	int sn;
	CString m_name;
	InertiaElem *body_i;
	InertiaElem *body_j;
};


