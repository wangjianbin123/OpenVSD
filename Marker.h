#pragma once

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


// Marker ÃüÁîÄ¿±ê

class Marker : public CObject
{
public:
	Marker();
	Marker(int n,Vector3x &vec,EulerAngle &ea); 
	Marker(int n,double a1,double a2,double a3,double b1,double b2,double b3);
	Marker(const Marker &mk);
	
	int getNum(void) const { return num;}
	Vector3x getOrigin(void) const { return Origin;}
	EulerAngle getOrientation(void) const { return Orientation;}

	void Serialize(CArchive &ar);
	DECLARE_SERIAL(Marker)

	Marker& operator=(const Marker &mk);
	void update(Vector3x &vec,EulerAngle &ea);
	void update(double a1,double a2,double a3,double b1,double b2,double b3);

	virtual ~Marker();
private:
	int num;
	Vector3x Origin;
	EulerAngle Orientation;

};


