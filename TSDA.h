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

#include <vector>

class TSDA :
	public ForceElem
{
public:
	TSDA(void);
	TSDA(int n,CString name,InertiaElem* f,int fmarker,InertiaElem* t,int tmarker,CubicSpline& stiff,CubicSpline& damper,double length,CubicSpline& ac);
	TSDA(const TSDA& sr);
	TSDA& operator=(const TSDA& sr);
	
	//ÖØÐ´Ðéº¯Êý
	TSDA* clone(void) const { return new TSDA(*this);}
	std::vector<VectorN> AssForceVec(Vector3x &fr,Vector3x &frd,EulerAngle &ftheta,EulerAngle &fthetad,Vector3x &tr,Vector3x &trd,EulerAngle &ttheta,EulerAngle &tthetad,double splineindex);
public:
	CubicSpline getStiff(void) const {return k;};
	CubicSpline getDamp(void) const {return c;};
	double getL0(void) const {return l0;};
	CubicSpline getActuator(void) const {return actuator;};
public:
	~TSDA(void);
private:
	CubicSpline k;
	CubicSpline c;
	double l0;
	CubicSpline actuator;
};
