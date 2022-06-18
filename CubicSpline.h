#pragma once

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif

// CubicSpline 命令目标
// version 1.0.2 修正了参数计算错误
// version 1.1.0 更新为三次B样条曲线
// version 2.0.1 更新B样条曲线取值向量

class CubicSpline : public CObject
{
public:
	//构造函数
	CubicSpline();
	CubicSpline(VectorN &vecx,VectorN &vecy);
	CubicSpline(double x[],double y[],int n);
	CubicSpline(Matrix &mx);
	CubicSpline(const CubicSpline &sp);
	virtual ~CubicSpline();
	
	//运算符重载
	CubicSpline& operator=(const CubicSpline &sp);

	//文件流操作
	void Serialize(CArchive &ar);
	DECLARE_SERIAL(CubicSpline)

	//三次样条曲线函数
	void Empty(void);
	int getIntervals(void) const;
	Matrix getCoff(void) const;
	Matrix getPoints(void) const;
	Matrix getDerivative(void) const;
	Matrix getDerivative2nd(void) const;
	//20200821
	VectorN getmsu0(void) const;
	VectorN getmsu1(void) const;
	VectorN getmsu2(void) const;
	VectorN getmd0(void) const;
	VectorN getmd1(void) const;
	VectorN getmd2(void) const;
	double getml0(void) const;
	double getml1(void) const;
	double getml2(void) const;
	//三次样条曲线
	double ValueAtC(double x);
	double SlopeAtC(double x);
	CubicSpline DerivativeC(void);

	//smothing functions
	VectorN gaussian_kernal(int n, double w);
	Matrix gaussian_filter1d(Matrix &mx, int n, double w);
	//smothing operation for original and derivative lines
	void gaussian_smothing_original(int n, double w);
	void gaussian_smothing_derivative(int n, double w);
	void gaussian_smothing_derivative2nd(int n, double w);

	//非均匀有理B样条
	int MBiger(double a, double b);
	double alpha(int I, int j, int k, VectorN &su, double u);
	double N(int i, int k, VectorN &su, double u);
	//
	double BSL(int k, VectorN &su, VectorN &d, int n, double u);
	double BSL1(int k, VectorN &su, VectorN &d, int n, double u);
	//20210604
	double BSL(VectorN &su, VectorN &d, int n, double u);
	double BSL1(VectorN &su, VectorN &d, int n, double u);
	//
	double d1(int I, int j, int k, VectorN &su, VectorN &d, double u);
	double d2(int I, int j, int k, VectorN &su, VectorN &d, double u);
	double GetDerValue(int k, VectorN &su, VectorN &d, int n, int r, double u);

	double ValueAt(double x);
	double SlopeAt(double x);
	double Slope2ndAt(double x);

	CubicSpline Derivative(void);
	CubicSpline Derivative2nd(void);
	double GetDer1st(double x);
	double GetDer2nd(double x);

	//文件保存读取
	BOOL SaveCubicSpline(void);
	BOOL LoadCubicSpline(void);

	//Initalize function
	void initialize();
	//set matrix paramerers
	void setParameters(Matrix &mx);
	void InitCubicCoffMat(void);
	void InitDer1stMat(void);
	void InitDer2ndMat(void);
	//
	void InitNurbsVec0(void);
	void InitNurbsVec1(void);
	void InitNurbsVec2(void);
	
private:
	// basic inputs or initial parameters
	Matrix m_nPoints;
	int m_nIntervals;
	// cubic spline coefficient matrix
	Matrix m_nCoff;
	// nurbs
	VectorN m_su0;
	VectorN m_d0;
	double m_L0;
	// first derivative nurbs
	Matrix m_Derivative;
	VectorN m_su1;
	VectorN m_d1;
	double m_L1;
	// second derivative nurbs
	Matrix m_Derivative2nd;
	VectorN m_su2;
	VectorN m_d2;
	double m_L2;
};


