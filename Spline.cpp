// Spline.cpp : 实现文件
//
#include "pch.h"
#include "Spline.h"

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif

#ifndef _MATH_H_
#define _MATH_H_
#include <math.h>
#endif

#ifndef _EPS_
#define eps 1e-8
#endif

#ifndef _M_PI_
#define M_PI 3.14159265358979323846
#endif

IMPLEMENT_SERIAL(CSpline, CObject, VERSIONABLE_SCHEMA | 2)

// CSpline

CSpline::CSpline()
{
	m_nPoints.Empty();
	m_nCoff.Empty();
	m_nIntervals = 0;
}
CSpline::CSpline(VectorN &vecx, VectorN &vecy)
{
	m_nIntervals = vecx.Dim() - 1;
	m_nPoints.SetMatrix(vecx.Dim(), 2);
	m_nCoff.SetMatrix(vecx.Dim() - 1, 4);
	for (int i = 0; i<vecx.Dim(); i++)
	{
		m_nPoints[i][0] = vecx[i];
		m_nPoints[i][1] = vecy[i];
	}
	this->initialize();
}
CSpline::CSpline(double x[], double y[], int n)
{
	m_nIntervals = n - 1;
	m_nPoints.SetMatrix(n, 2);
	m_nCoff.SetMatrix(n - 1, 4);

	for (int i = 0; i<n; i++)
	{
		m_nPoints[i][0] = x[i];
		m_nPoints[i][1] = y[i];
	}
	this->initialize();
}
CSpline::CSpline(Matrix &mx)
{
	int m = mx.Row();
	m_nPoints = mx;
	m_nIntervals = m - 1;
	m_nCoff.SetMatrix(m - 1, 4);
	this->initialize();
}
CSpline::CSpline(const CSpline &sp)
{
	m_nPoints = sp.getPoints();
	m_nCoff = sp.getCoff();
	m_nIntervals = sp.getIntervals();
}
CSpline::~CSpline()
{
	m_nPoints.Empty();
	m_nCoff.Empty();
	m_nIntervals = 0;
}

CSpline& CSpline::operator =(const CSpline &sp)
{
	if (this == &sp)
	{
		return (*this);
	}
	else
	{
		this->Empty();
		m_nPoints = sp.getPoints();
		m_nCoff = sp.getCoff();
		m_nIntervals = sp.getIntervals();
	}
	return (*this);
}

//文件流操作
void CSpline::Serialize(CArchive &ar)
{
	CObject::Serialize(ar);
	if (ar.IsStoring())
	{
		ar << m_nIntervals;
		m_nPoints.Serialize(ar);
		m_nCoff.Serialize(ar);
	}
	else
	{
		int nIntervals;
		Matrix nPoints, nCoff;
		ar >> nIntervals;
		nPoints.Serialize(ar);
		nCoff.Serialize(ar);
		m_nIntervals = nIntervals;
		m_nPoints = nPoints;
		m_nCoff = nCoff;
	}
}
// CSpline 成员函数

void CSpline::Empty(void)
{
	m_nCoff.Empty();
	m_nPoints.Empty();
	m_nIntervals = 0;
}

Matrix CSpline::getPoints(void) const
{
	return m_nPoints;
}

int CSpline::getIntervals(void) const
{
	return m_nIntervals;
}

Matrix CSpline::getCoff(void) const
{
	return m_nCoff;
}

double CSpline::ValueAtC(double x)
{
	if (x<m_nPoints[0][0])
	{
		return m_nPoints[0][1];
	}
	int m = m_nPoints.Row();
	if (x>m_nPoints[m - 1][0])
	{
		return m_nPoints[m - 1][1];
	}
	//
	int low = 0;
	int high = m_nIntervals - 1;
	int mid = 0;
	while (low <= high)
	{
		mid = (low + high) / 2;
		if ((m_nPoints[mid + 1][0] >= x) && (m_nPoints[mid][0] <= x))
		{
			return m_nCoff[mid][0] * pow((m_nPoints[mid + 1][0] - x), 3.0) + m_nCoff[mid][1] * pow((x - m_nPoints[mid][0]), 3.0) + m_nCoff[mid][2] * (m_nPoints[mid + 1][0] - x) + m_nCoff[mid][3] * (x - m_nPoints[mid][0]);
		}
		if (m_nPoints[mid + 1][0]<x)
			low = mid + 1;
		else
			high = mid - 1;
	}
	return 0;
}

double CSpline::SlopeAtC(double x)
{
	int low = 0;
	int high = m_nIntervals - 1;
	int mid = 0;
	while (low <= high)
	{
		mid = (low + high) / 2;
		if ((m_nPoints[mid + 1][0] >= x) && (m_nPoints[mid][0] <= x))
		{
			return m_nCoff[mid][0] * pow((m_nPoints[mid + 1][0] - x), 2)*(-3) + 3 * m_nCoff[mid][1] * pow((x - m_nPoints[mid][0]), 2) - m_nCoff[mid][2] + m_nCoff[mid][3];
		}
		if (m_nPoints[mid + 1][0]<x)
			low = mid + 1;
		else
			high = mid - 1;
	}
	return 0;
}

void CSpline::InitCubicCoffMat(void)
{
	int k;
	int m = m_nPoints.Row();
	double *e = new double[m + 1];
	double *f = new double[m + 1];
	double *g = new double[m + 1];
	double *r = new double[m + 1];
	double *w = new double[m + 1];
	e[0] = 0; 
	e[1] = 0; 
	f[0] = 0; 
	g[0] = 0; 
	r[0] = 0; 
	g[m - 2] = 0;
	for (k = 1; k <= m - 2; k++)
	{
		e[k + 1] = m_nPoints[k + 1][0] - m_nPoints[k][0];
		f[k] = 2 * (m_nPoints[k + 1][0] - m_nPoints[k - 1][0]);
		g[k] = m_nPoints[k + 1][0] - m_nPoints[k][0];
		r[k] = 6 * ((m_nPoints[k + 1][1] - m_nPoints[k][1]) / (m_nPoints[k + 1][0] - m_nPoints[k][0]) + (m_nPoints[k - 1][1] - m_nPoints[k][1]) / (m_nPoints[k][0] - m_nPoints[k - 1][0]));
	}
	w[0] = 0;
	w[m - 1] = 0;
	for (k = 2; k <= m - 2; k++)
	{
		e[k] /= f[k - 1];
		f[k] -= e[k] * g[k - 1];
	}
	for (k = 2; k <= m - 2; k++)
	{
		r[k] -= e[k] * r[k - 1];
	}
	w[m - 2] = r[m - 2] / f[m - 2];
	for (k = m_nPoints.Row() - 3; k >= 1; k--)
	{
		w[k] = (r[k] - g[k] * w[k + 1]) / f[k];
	}
	for (k = 1; k <= m - 1; k++)
	{
		m_nCoff[k - 1][0] = w[k - 1] / (6 * (m_nPoints[k][0] - m_nPoints[k - 1][0]));
		m_nCoff[k - 1][1] = w[k] / (6 * (m_nPoints[k][0] - m_nPoints[k - 1][0]));
		m_nCoff[k - 1][2] = m_nPoints[k - 1][1] / (m_nPoints[k][0] - m_nPoints[k - 1][0]) - w[k - 1] * (m_nPoints[k][0] - m_nPoints[k - 1][0]) / 6;
		m_nCoff[k - 1][3] = m_nPoints[k][1] / (m_nPoints[k][0] - m_nPoints[k - 1][0]) - w[k] * (m_nPoints[k][0] - m_nPoints[k - 1][0]) / 6;
	}
	delete[] e, f, g, r, w;
}

void CSpline::initialize()
{
	this->InitCubicCoffMat();
}

void CSpline::setParameters(Matrix &mx)
{
	int m = mx.Row();
	m_nIntervals = m - 1;
	m_nCoff.SetMatrix(m - 1, 4);
	m_nPoints = mx;
	this->initialize();
}