#pragma once

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif
// CSpline 命令目标

class CSpline : public CObject
{
public:
	//构造函数
	CSpline();
	CSpline(VectorN &vecx, VectorN &vecy);
	CSpline(double x[], double y[], int n);
	CSpline(Matrix &mx);
	CSpline(const CSpline &sp);
	void setParameters(Matrix &mx);
	virtual ~CSpline();

	//运算符重载
	CSpline& operator=(const CSpline &sp);

	void Serialize(CArchive &ar);
	DECLARE_SERIAL(CSpline)

	void Empty(void);
	int getIntervals(void) const;
	Matrix getCoff(void) const;
	Matrix getPoints(void) const;

	void initialize(void);
	void InitCubicCoffMat(void);

	double ValueAtC(double x);
	double SlopeAtC(double x);
private:
	Matrix m_nPoints;
	int m_nIntervals;
	Matrix m_nCoff;
};


