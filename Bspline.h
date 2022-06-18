#pragma once

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif

// Bspline 命令目标

class Bspline : public CObject
{
public:
	Bspline();
	Bspline(Matrix &mx);
	Bspline(const Bspline &bsp);
	void SetEps(double Eps);
	double GetEps(void) const;
	Matrix GetMatrix(void) const;
	inline int MBiger(double a, double b){
		return a > b || a < -b;
	}
	double N(int i, int k, double su[], double u);
	double BSL1(int k, double su[], double d[], int n, double u);
	//
	inline double alpha(int I, int j, int k, double su[], double u){
		if ((su[j + k + 1 - I] - su[j]<eps) && (su[j + k + 1 - I] - su[j]>(-1.0*eps)))
			return 0.0;
		else
			return (u - su[j]) / (su[j + k + 1 - I] - su[j]);
	}
	//
	double d1(int I, int j, int k, double su[], double d[], double u);
	double BSL(int k, double su[], double d[], int n, double u);
	double d2(int I, int j, int k, double su[], double d[], double u);
	double GetDerValue(int k, double su[], double d[], int n, int r, double u);
	double BSS(int k, double su[], int I, double sv[], double **d, int m, double u, int n, double v);
	double NURBSL(int k, double su[], double d[], double w[], int n, double u);
	double NURBSS(int k, double su[], int I, double sv[], double **d, double **w, int m, double u, int n, double v);
	double UBSL(int k, double d[], int n, double u);
	double QUBSL(int k, double d[], int n, double u);
	double UBSS(int k, int I, double **d, int m, double u, int n, double v);
	double QUBSS(int k, int I, double **d, int m, double u, int n, double v);
	//非均匀三次B样条曲线
	void RiBSL(double *dx, double *dy, int n, double u, double *x, double *y);
	//
	Matrix RiBSL(Matrix &mx, int output);
	double ValueAt(double x);
	double SlopeAt(double x);
	
	virtual ~Bspline();
public:
	double eps;
	Matrix mx0;
};


