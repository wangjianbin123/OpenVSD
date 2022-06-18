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

#ifndef _TRACK_H_
#define _TRACK_H_
#include "Track.h"
#endif

#ifndef _WHEELGEOMETRY_H_
#define _WHEELGEOMETRY_H_
#include "WheelGeometry.h"
#endif

#include <fcntl.h>
#include<stdio.h>
#include<io.h>
#include <iostream>
#include <Windows.h>
// AEContGeo 命令目标

class AEContGeo : public CObject
{
public:
	//构造函数
	AEContGeo();
	AEContGeo(WheelGeometry &wg,BOOL bl);
	AEContGeo(const AEContGeo &aecg);
	AEContGeo& operator=(const AEContGeo &aecg);
	//const functions
	WheelGeometry getWheelGeo(void) const { return m_wg; }
	BOOL beLeft(void) const { return Left; }

	//文件流
	//void Serialize(CArchive &ar);
	//DECLARE_SERIAL(AEContGeo)

	//shabana's methods tr1 and tr2 point product with Rwr
	VectorN AssFunVal(Track &tk,VectorN &surpara,Vector3x &profpos,Matrix &rotmat);
	Matrix AssJocob(Track &tk,VectorN &surpara,Vector3x &profpos,Matrix &rotmat);
	Matrix AssHessian(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat);

	//Gauss-Newton Step
	VectorN AssGaussNewtonStep(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat);
	//Jacobian Step
	VectorN AssJacobianStep(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat);

	//proposed by Shabana Newton downhill method
	VectorN AssConPara(Track &tk,VectorN &surpara,Vector3x &profpos,Matrix &rotmat);
	VectorN AssConPara2nd(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat);
	double Penetration(Track &tk,VectorN &surpara,Vector3x &profpos,Matrix &rotmat);
	Matrix IteractonProcess(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat);
	//return iteraction evolution of contact points and residual norms
	Matrix IteractionConvergence(Track &tk, Vector3x profpos, int size, int maxiter, double rmin, double rmax, double wmin, double wmax);
	int IteractionNum(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat,int maxiter);

	void setParameter(WheelGeometry &wg, BOOL bl);
	void Empty(void);
	void InitConsolWnd(void);

	virtual ~AEContGeo();
private:
	WheelGeometry m_wg;
	BOOL Left;
};


