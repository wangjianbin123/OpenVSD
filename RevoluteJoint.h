#pragma once
#include "DOFConstraint.h"

#pragma once
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

#ifndef _INERTIAELEM_H_
#define _INERTIAELEM_H_
#include "InertiaElem.h"
#endif


class RevoluteJoint :
	public DOFConstraint
{
public:
	RevoluteJoint(void);
	RevoluteJoint(int n,CString name,InertiaElem* i,int connecti,int v1,int v2,InertiaElem* j,int connectj,int vj);
	RevoluteJoint(const RevoluteJoint& rj);
	
	RevoluteJoint& operator=(const RevoluteJoint& rj);
	RevoluteJoint* clone(void) const { return new RevoluteJoint(*this);}
	int getType(void) const {return m_type;};
	int getDecrDofs(void) const {return decr_dofs;};
	
	//void Serialize(CArchive &ar);
	//DECLARE_SERIAL(RevoluteJoint)

	//Ðéº¯ÊýÖØÐ´
	Matrix AssJacobian(Vector3x &Ri,EulerAngle &Thetai,Vector3x &Rj,EulerAngle &Thetaj);//20130712 modified
	Matrix AssConstraintEquations(Vector3x &Ri, EulerAngle &Thetai, Vector3x &Rj, EulerAngle &Thetaj);

	virtual ~RevoluteJoint(void);
private:
	int m_type;//type of dof constraints
	int decr_dofs;//number of decreased dofs
	int iconnect_pnt;
	int v1_pnt;
	int v2_pnt;
	int jconnect_pnt;
	int vj_pnt;
};
