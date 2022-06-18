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

// TPFElem ÃüÁîÄ¿±ê

class TPFElem : public CObject
{
public:
	TPFElem();
	TPFElem(int flag,CubicSpline &f,Marker &from,Marker &to,Marker &at);
	TPFElem(const TPFElem &tpf);

	TPFElem& operator=(const TPFElem &tpf);

	int getflag(void) const { return FLAG; }
	CubicSpline getforce(void) const { return force; }
	Marker getfrom(void) const { return i; }
	Marker getto(void) const { return j; }
	Marker getat(void) const { return a; }

	Vector3x getForceVec(double splineindex);

	virtual ~TPFElem();
private:
	int FLAG;//(0:force),(1,torque)
	CubicSpline force;
	Marker i;
	Marker j;
	Marker a;
};


