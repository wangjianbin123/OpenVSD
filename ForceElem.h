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

#include <vector>

// ForceElem 命令目标

class ForceElem : public CObject
{
public:
	ForceElem();
	ForceElem(int n,CString name,InertiaElem* f,int fmarker,InertiaElem* t,int tmarker);
	ForceElem(const ForceElem &fe);

	ForceElem& operator=(const ForceElem &fe);
	void setName( CString name);

	int getSN(void) const { return sn; }
	CString getName(void) const { return m_name;}
	InertiaElem* getfrom(void) const {return from;};
	InertiaElem* getto(void) const {return to;};
	int getfrommarker(void) const {return marker_from;};
	int gettomarker(void) const {return marker_to;};


	//要实现的虚函数-----------------------------------------------------------------------
	virtual ForceElem* clone(void) const { return new ForceElem(*this); }
	virtual std::vector<VectorN> AssForceVec(Vector3x &fr,Vector3x &frd,EulerAngle &ftheta,EulerAngle &fthetad,Vector3x &tr,Vector3x &trd,EulerAngle &ttheta,EulerAngle &tthetad,double splineindex);
	virtual ~ForceElem();
protected:
	int sn;
	CString m_name;
	InertiaElem* from;
	int marker_from;
	InertiaElem* to;
	int marker_to;
};


