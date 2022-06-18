// CubicSpline.cpp : 实现文件
//
#include "pch.h"
#include "CubicSpline.h"

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

IMPLEMENT_SERIAL(CubicSpline,CObject,VERSIONABLE_SCHEMA|2)

// 构造函数

CubicSpline::CubicSpline()
{
	m_nPoints.Empty();
	m_Derivative.Empty();
	m_Derivative2nd.Empty();
	m_nCoff.Empty();
	m_nIntervals=0;
	m_su0.Empty();
	m_d0.Empty();
	m_L0 = 0;
	m_su1.Empty();
	m_d1.Empty();
	m_L1 = 0;
	m_su2.Empty();
	m_d2.Empty();
	m_L2 = 0;
}

CubicSpline::CubicSpline(VectorN &vecx,VectorN &vecy)
{
	m_nIntervals=vecx.Dim()-1;
	m_nPoints.SetMatrix(vecx.Dim(),2);
	m_nCoff.SetMatrix(vecx.Dim()-1,4);
	//
	for (int i=0;i<vecx.Dim();i++)
	{
		m_nPoints[i][0]=vecx[i];
		m_nPoints[i][1]=vecy[i];
	}
	//
	this->initialize();
}

CubicSpline::CubicSpline(double x[], double y[], int n)
{
	m_nIntervals=n-1;
	m_nPoints.SetMatrix(n,2);
	m_nCoff.SetMatrix(n-1,4);
	//
	for (int i=0;i<n;i++)
	{
		m_nPoints[i][0]=x[i];
		m_nPoints[i][1]=y[i];
	}
	//
	this->initialize();
}

CubicSpline::CubicSpline(Matrix &mx)
{
	int m=mx.Row();
	m_nPoints=mx;
	m_nIntervals=m-1;
	m_nCoff.SetMatrix(m-1,4);
	//
	this->initialize();
}

CubicSpline::CubicSpline(const CubicSpline &sp)
{
	m_nPoints=sp.getPoints();
	m_Derivative = sp.getDerivative();
	m_Derivative2nd = sp.getDerivative2nd();
	m_nCoff=sp.getCoff();
	m_nIntervals = sp.getIntervals();
	//20200821
	m_su0 = sp.getmsu0();
	m_su1 = sp.getmsu1();
	m_su2 = sp.getmsu2();
	m_d0 = sp.getmd0();
	m_d1 = sp.getmd1();
	m_d2 = sp.getmd2();
	m_L0 = sp.getml0();
	m_L1 = sp.getml1();
	m_L2 = sp.getml2();
}
//析构函数
CubicSpline::~CubicSpline()
{
	m_nPoints.Empty();
	m_Derivative.Empty();
	m_Derivative2nd.Empty();
	m_nCoff.Empty();
	m_nIntervals = 0;
	//20200821
	m_su0.Empty();
	m_d0.Empty();
	m_L0 = 0;
	m_su1.Empty();
	m_d1.Empty();
	m_L1 = 0;
	m_su2.Empty();
	m_d2.Empty();
	m_L2 = 0;
}
void CubicSpline::initialize()
{
	this->InitCubicCoffMat();
	//
	this->InitNurbsVec0();
	//
	this->InitDer1stMat();
	this->InitNurbsVec1();
	//
	this->InitDer2ndMat();
	this->InitNurbsVec2();
}
//运算符重载
CubicSpline& CubicSpline::operator =(const CubicSpline &sp)
{
	if (this==&sp)
	{
		return (*this);
	}
	else
	{
		this->Empty();
		m_nPoints=sp.getPoints();
		m_Derivative = sp.getDerivative();
		m_Derivative2nd = sp.getDerivative2nd();
		m_nCoff = sp.getCoff();
		m_nIntervals = sp.getIntervals();
		//20200821
		m_su0 = sp.getmsu0();
		m_su1 = sp.getmsu1();
		m_su2 = sp.getmsu2();
		m_d0 = sp.getmd0();
		m_d1 = sp.getmd1();
		m_d2 = sp.getmd2();
		m_L0 = sp.getml0();
		m_L1 = sp.getml1();
		m_L2 = sp.getml2();
	}
	return (*this);
}
//文件流操作
void CubicSpline::Serialize(CArchive &ar)
{
	CObject::Serialize(ar);
	if (ar.IsStoring())
	{
		ar<<m_nIntervals;
		m_nPoints.Serialize(ar);
		m_nCoff.Serialize(ar);
		m_Derivative.Serialize(ar);
		m_Derivative2nd.Serialize(ar);
		//20200821
		m_su0.Serialize(ar);
		m_d0.Serialize(ar);
		ar<<m_L0;
		m_su1.Serialize(ar);
		m_d1.Serialize(ar);
		ar<<m_L1;
		m_su2.Serialize(ar);
		m_d2.Serialize(ar);
		ar<<m_L2;
	}
	else
	{
		int nIntervals;
		Matrix nPoints,nCoff,mDer,mDer2nd;
		double ml0, ml1, ml2;
		VectorN msu0, msu1, msu2, md0, md1, md2;
		ar>>nIntervals;
		nPoints.Serialize(ar);
		nCoff.Serialize(ar);
		mDer.Serialize(ar);
		mDer2nd.Serialize(ar);
		msu0.Serialize(ar);
		md0.Serialize(ar);
		ar >> ml0;
		msu1.Serialize(ar);
		md1.Serialize(ar);
		ar >> ml1;
		msu2.Serialize(ar);
		md2.Serialize(ar);
		ar >> ml2;
		m_nIntervals=nIntervals;
		m_nPoints=nPoints;
		m_nCoff=nCoff;
		m_Derivative = mDer;
		m_Derivative2nd = mDer2nd;
		m_su0 = msu0;
		m_d0 = md0;
		m_L0 = ml0;
		m_su1 = msu1;
		m_d1 = md1;
		m_L1 = ml1;
		m_su2 = msu2;
		m_d2 = md2;
		m_L2 = ml2;
	}
}

//样条曲线函数
void CubicSpline::Empty(void)
{
	m_nCoff.Empty();
	m_Derivative.Empty();
	m_Derivative2nd.Empty();
	m_nPoints.Empty();
	m_nIntervals = 0;
	m_su0.Empty();
	m_d0.Empty();
	m_L0 = 0;
	m_su1.Empty();
	m_d1.Empty();
	m_L1 = 0;
	m_su2.Empty();
	m_d2.Empty();
	m_L2 = 0;
}

Matrix CubicSpline::getPoints(void) const
{
	return m_nPoints;
}

Matrix CubicSpline::getDerivative(void) const
{
	return m_Derivative;
}

Matrix CubicSpline::getDerivative2nd(void) const
{
	return m_Derivative2nd;
}

VectorN CubicSpline::getmsu0(void) const
{
	return m_su0;
}

VectorN CubicSpline::getmsu1(void) const
{
	return m_su1;
}

VectorN CubicSpline::getmsu2(void) const
{
	return m_su2;
}

VectorN CubicSpline::getmd0(void) const
{
	return m_d0;
}

VectorN CubicSpline::getmd1(void) const
{
	return m_d1;
}

VectorN CubicSpline::getmd2(void) const
{
	return m_d2;
}

double CubicSpline::getml0(void) const
{
	return m_L0;
}

double CubicSpline::getml1(void) const
{
	return m_L1;
}
double CubicSpline::getml2(void) const
{
	return m_L2;
}

int CubicSpline::getIntervals(void) const
{
	return m_nIntervals;
}

Matrix CubicSpline::getCoff(void) const
{
	return m_nCoff;
}

double CubicSpline::ValueAtC(double x)
{
	int m = m_nPoints.Row();
	if (x<m_nPoints[0][0])
	{
		return m_nPoints[0][1];
	}
	if (x>m_nPoints[m-1][0])
	{
		return m_nPoints[m-1][1];
	}
	//
	int low=0;
	int high=m_nIntervals-1;
	int mid=0;
	while(low<=high)
	{
		mid=(low+high)/2;
		if ((m_nPoints[mid+1][0]>=x)&&(m_nPoints[mid][0]<=x))
		{
			return m_nCoff[mid][0]*pow((m_nPoints[mid+1][0]-x),3.0)+m_nCoff[mid][1]*pow((x-m_nPoints[mid][0]),3.0)+m_nCoff[mid][2]*(m_nPoints[mid+1][0]-x)+m_nCoff[mid][3]*(x-m_nPoints[mid][0]); 
		}
		if (m_nPoints[mid+1][0]<x)
			low=mid+1;
		else
			high=mid-1;
	}
	return 0;
}

double CubicSpline::SlopeAtC(double x)
{
	int low=0;
	int high=m_nIntervals-1;
	int mid=0;
	while(low<=high)
	{
		mid=(low+high)/2;
		if ((m_nPoints[mid+1][0]>=x)&&(m_nPoints[mid][0]<=x))
		{
			return m_nCoff[mid][0]*pow((m_nPoints[mid+1][0]-x),2)*(-3)+3*m_nCoff[mid][1]*pow((x-m_nPoints[mid][0]),2)-m_nCoff[mid][2]+m_nCoff[mid][3]; 
		}
		if (m_nPoints[mid+1][0]<x)
			low=mid+1;
		else
			high=mid-1;
	}
	return 0;
}

CubicSpline CubicSpline::Derivative(void)
{
	CubicSpline tmpCubicSpline(this->getDerivative());
	return tmpCubicSpline;
}

CubicSpline CubicSpline::Derivative2nd(void)
{
	CubicSpline tmpCubicSpline(this->getDerivative2nd());
	return tmpCubicSpline;
}

CubicSpline CubicSpline::DerivativeC(void)
{
	int nrow = this->getPoints().Row();
	Matrix mx(nrow, 2);
	for (int i = 0; i < nrow; i++)
	{
		mx[i][0] = m_nPoints[i][0];
		mx[i][1] = this->SlopeAtC(mx[i][0]);
	}
	CubicSpline sp(mx);
	return sp;
}
void CubicSpline::gaussian_smothing_original(int n, double w)
{
	Matrix mx = gaussian_filter1d(this->m_nPoints,n, w);
	m_nPoints = mx;
	m_nIntervals = mx.Row() - 1;
	m_nCoff.SetMatrix(mx.Row() - 1, 4);
	// 
	this->InitCubicCoffMat();
	this->InitNurbsVec0();
	this->InitDer1stMat();
	this->InitNurbsVec1(); 
	this->InitDer2ndMat();
	this->InitNurbsVec2();
}

void CubicSpline::gaussian_smothing_derivative(int n, double w)
{
	Matrix mx = this->gaussian_filter1d(this->m_Derivative, n, w);
	m_Derivative = mx;
	this->InitNurbsVec1();
	this->InitDer2ndMat();
	this->InitNurbsVec2();
}

void CubicSpline::gaussian_smothing_derivative2nd(int n, double w)
{
	Matrix mx = this->gaussian_filter1d(this->m_Derivative2nd, n, w);
	m_Derivative2nd = mx;
	this->InitNurbsVec2();
}
VectorN CubicSpline::gaussian_kernal(int n, double w)
{
	int inRadius = n;
	double inWeight = w;
	int mem_amount = (inRadius * 2) + 1;
	VectorN gaussian_kernal(mem_amount);
	double twoRadiusSquareRecip = 1.0 / (2.0*inRadius*inRadius);
	double sqrtTwoPiTimesRadiusRecip = 1.0 / (sqrt(2.0*M_PI)*inRadius);
	double radiusModifier = inWeight;
	int r = -1 * inRadius;
	double sum = 0.0;
	for (int i = 0; i < mem_amount; i++)
	{
		double x = r*radiusModifier;
		x *= x;
		double v = sqrtTwoPiTimesRadiusRecip*exp(-x*twoRadiusSquareRecip);
		gaussian_kernal[i] = v;
		sum += v;
		r++;
	}
	double div = sum;
	for (int i = 0; i < mem_amount; i++)
	{
		gaussian_kernal[i] /= div;
	}
	return gaussian_kernal;
}

Matrix CubicSpline::gaussian_filter1d(Matrix &mx, int n, double w)
{
	VectorN gaussian_kernal = this->gaussian_kernal(n, w);
	int mem_amount = (n * 2) + 1;
	int nrow = mx.Row();
	Matrix tmp(nrow, 2);
	for (int ii = 0; ii < nrow; ii++)
	{
		for (int jj = 0; jj < mem_amount; jj++)
		{
			if (ii - jj < 0)
			{
				tmp[ii][0] += 0.0;
				tmp[ii][1] += 0.0;
			}
			else
			{
				tmp[ii][0] += mx[ii - jj][0] * gaussian_kernal[jj];
				tmp[ii][1] += mx[ii - jj][1] * gaussian_kernal[jj];
			}
		}
	}
	Matrix tm1(nrow - mem_amount, 2);
	for (int kk = 0; kk < nrow - mem_amount; kk++)
	{
		tm1[kk][0] = tmp[kk + mem_amount][0];
		tm1[kk][1] = tmp[kk + mem_amount][1];
	}
	return tm1;
}

void CubicSpline::setParameters(Matrix &mx)
{
	int m = mx.Row();
	m_nIntervals = m - 1;
	m_nCoff.SetMatrix(m - 1, 4);
	m_nPoints = mx;
	this->initialize();
}
void CubicSpline::InitCubicCoffMat(void)
{
	int k;
	int m = m_nPoints.Row();
	double *e = new double[m + 1];
	double *f = new double[m + 1];
	double *g = new double[m + 1];
	double *r = new double[m + 1];
	double *w = new double[m + 1];
	e[0] = 0; e[1] = 0; f[0] = 0; g[0] = 0; r[0] = 0; g[m - 2] = 0;
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
void CubicSpline::InitDer1stMat(void)
{
	int m = m_nPoints.Row();
	Matrix mx(m, 2);
	for (int ii = 0; ii < m-1; ii++)
	{
		mx[ii][0] = m_nPoints[ii][0];
		mx[ii][1] = this->GetDer1st(m_nPoints[ii][0]);
	}
	//20200920
	mx[m - 1][0] = m_nPoints[m - 1][0];
	mx[m - 1][1] = mx[m - 2][1];
	m_Derivative = mx;
}

void CubicSpline::InitDer2ndMat(void)
{
	int m = m_Derivative.Row();
	Matrix mx(m, 2);
	for (int ii = 0; ii < m-1; ii++)
	{
		mx[ii][0] = m_Derivative[ii][0];
		mx[ii][1] = this->GetDer2nd(m_Derivative[ii][0]);
	}
	//20200920
	mx[m - 1][0] = m_Derivative[m - 1][0];
	mx[m - 1][1] = mx[m - 2][1];
	this->m_Derivative2nd = mx;
}

void CubicSpline::InitNurbsVec0(void)
{
	int nrow = m_nPoints.Row();
	m_d0.SetVectorN(nrow);
	for (int ii = 0; ii < nrow; ii++)
	{
		m_d0[ii] = m_nPoints[ii][1];
	}
	// cubic B spline where k ==3
	m_su0.SetVectorN(nrow + 3);
	double *I = new double[nrow];
	double L = 0.0, I1;
	I[0] = 0.0;
	int i = 0;
	for (i = 1; i < nrow; i++){
		I[i] = m_nPoints[i][0] - m_nPoints[i - 1][0];
		L = L + I[i];
	}
	m_su0[0] = 0.0;
	m_su0[1] = 0.0; 
	m_su0[2] = 0.0; 
	m_su0[3] = 0.0;
	I1 = I[1];
	for (i = 4; i < nrow - 1; i++){
		I1 = I1 + I[i - 2];
		m_su0[i] = I1 / L;
	}
	m_su0[nrow] = 1.0;
	m_su0[nrow + 1] = 1.0;
	m_su0[nrow + 2] = 1.0;
	m_su0[nrow - 1] = 1.0;
	m_L0 = L;
	delete[]I;
}

void CubicSpline::InitNurbsVec1(void)
{
	int nrow = m_Derivative.Row();
	m_d1.SetVectorN(nrow);
	for (int ii = 0; ii < nrow; ii++)
	{
		m_d1[ii] = m_Derivative[ii][1];
	}
	// cubic B spline where k ==3
	m_su1.SetVectorN(nrow + 3);
	double *I = new double[nrow];
	double L = 0.0, I1;
	I[0] = 0.0;
	int i = 0;
	for (i = 1; i < nrow; i++){
		I[i] = m_Derivative[i][0] - m_Derivative[i - 1][0];
		L = L + I[i];
	}
	m_su1[0] = 0.0; 
	m_su1[1] = 0.0; 
	m_su1[2] = 0.0; 
	m_su1[3] = 0.0;
	I1 = I[1];
	for (i = 4; i < nrow - 1; i++){
		I1 = I1 + I[i - 2];
		m_su1[i] = I1 / L;
	}
	m_su1[nrow] = 1.0;
	m_su1[nrow + 1] = 1.0;
	m_su1[nrow + 2] = 1.0;
	m_su1[nrow - 1] = 1.0;
	m_L1 = L;
	delete[]I;
}

void CubicSpline::InitNurbsVec2(void)
{
	int nrow = m_Derivative2nd.Row();
	m_d2.SetVectorN(nrow);
	for (int ii = 0; ii < nrow; ii++)
	{
		m_d2[ii] = m_Derivative2nd[ii][1];
	}
	// cubic B spline where k ==3
	m_su2.SetVectorN(nrow + 3);
	double *I = new double[nrow];
	double L = 0.0, I1;
	I[0] = 0.0;
	int i = 0;
	for (i = 1; i < nrow; i++){
		I[i] = m_Derivative2nd[i][0] - m_Derivative2nd[i - 1][0];
		L = L + I[i];
	}
	m_su2[0] = 0.0; 
	m_su2[1] = 0.0; 
	m_su2[2] = 0.0; 
	m_su2[3] = 0.0;
	I1 = I[1];
	for (i = 4; i < nrow - 1; i++){
		I1 = I1 + I[i - 2];
		m_su2[i] = I1 / L;
	}
	m_su2[nrow] = 1.0;
	m_su2[nrow + 1] = 1.0;
	m_su2[nrow + 2] = 1.0;
	m_su2[nrow - 1] = 1.0;
	m_L2 = L;
	delete[]I;
}

int CubicSpline::MBiger(double a, double b)
{
	return a > b || a < -b;
}

double CubicSpline::alpha(int I, int j, int k, VectorN &su, double u)
{
	if ((su[j + k + 1 - I] - su[j]<eps) && (su[j + k + 1 - I] - su[j]>(-1.0*eps)))
		return 0.0;
	else
		return (u - su[j]) / (su[j + k + 1 - I] - su[j]);
}

double CubicSpline::N(int i, int k, VectorN &su, double u)
{
	if (k == 0)
	{
		if (u - su[0] < eps)
		{
			if (i == 0)
				return 1.0;
			else
				return 0.0;
		}
		if (u >= su[i] && u < su[i + 1])
			return 1.0;
		else
			return 0.0;
	}
	else if (k>0)
	{
		if (MBiger(su[i + k] - su[i], eps) && MBiger(su[i + k + 1] - su[i + 1], eps))
		{
			return ((u - su[i]) * N(i, k - 1, su, u) / (su[i + k] - su[i])) + (su[i + k + 1] - u)*N(i + 1, k - 1, su, u) / (su[i + k + 1] - su[i + 1]);
		}
		else if (!MBiger(su[i + k] - su[i], eps) && MBiger(su[i + k + 1] - su[i + 1], eps))
		{
			return (su[i + k + 1] - u)*N(i + 1, k - 1, su, u) / (su[i + k + 1] - su[i + 1]);
		}
		else if (MBiger(su[i + k] - su[i], eps) && !MBiger(su[i + k + 1] - su[i + 1], eps))
		{
			return (u - su[i])*N(i, k - 1, su, u) / (su[i + k] - su[i]);
		}
		else
			return 0.0;
	}
	else
		return 0.0;
}

double CubicSpline::d1(int I, int j, int k, VectorN &su, VectorN &d, double u)
{
	if (I <= 0)
		return d[j];
	double a = this->alpha(I, j, k, su, u);
	return (1 - a)*d1(I - 1, j - 1, k, su, d, u) + a*d1(I - 1, j, k, su, d, u);
}

double CubicSpline::d2(int I, int j, int k, VectorN &su, VectorN &d, double u)
{
	if (I <= 0)
		return d[j];
	double z = su[j + k + 1 - I] - su[j];
	if (z<eps&&z>-eps)
		return 0.0;
	return (k + 1 - I)*(d2(I - 1, j, k, su, d, u) - d2(I - 1, j - 1, k, su, d, u)) / z;
}

double CubicSpline::GetDerValue(int k, VectorN &su, VectorN &d, int n, int r, double u)
{
	int i, j;
	double s = 0.0;
	for (i = 1; i < n + k; i++)
	{
		if (u < su[i])
			break;
		if (i>k&&u == su[i])
			break;
	}
	i--;
	for (j = i - k + r; j <= i; j++)
	{
		s += d2(r, j, k, su, d, u)*N(j, k - r, su, u);
	}
	return s;
}

double CubicSpline::BSL(int k, VectorN &su, VectorN &d, int n, double u)
{
	int i;
	for (i = 1; i < n + k; i++)
	{
		if (u < su[i])
			break;
		if (i>k&&u == su[i])
			break;
	}
	i--;
	return d1(k, i, k, su, d, u);
}

double CubicSpline::BSL1(int k, VectorN &su, VectorN &d, int n, double u)
{
	int i = 0;
	double s = 0.0;
	for (i = 0; i < n; i++)
	{
		s = s + d[i] * N(i, k, su, u);
	}
	return s;
}

//
double CubicSpline::BSL(VectorN &su, VectorN &d, int n, double u)
{
	int num = su.Dim();
	int low = 3;
	int high = num - 4;
	int mid = 0;
	while (low <= high)
	{
		mid = (low + high) / 2;
		if ((su[mid + 1] >= u) && (su[mid] <= u))
		{
			break;
		}
		else if (su[mid + 1] < u)
		{
			low = mid + 1;
		}
		else
		{
			high = mid - 1;
		}
	}
	return d1(3, mid, 3, su, d, u);
}

double CubicSpline::BSL1(VectorN &su, VectorN &d, int n, double u)
{
	int i = 0;
	double s = 0.0;
	for (i = 0; i < n; i++)
	{
		s = s + d[i] * N(i, 3, su, u);
	}
	return s;
}
//

double CubicSpline::GetDer1st(double x)
{
	int nrow = m_nPoints.Row();
	double u = (x - m_nPoints[0][0]) / m_L0;
	double v = this->GetDerValue(3, m_su0, m_d0, nrow, 1, u) / m_L0;
	return v;
}

double CubicSpline::GetDer2nd(double x)
{
	int nrow = m_Derivative.Row();
	double u = (x - m_Derivative[0][0]) / m_L1;
	double v = this->GetDerValue(3, m_su1, m_d1, nrow, 1, u) / m_L1;
	return v;
}

double CubicSpline::ValueAt(double x)
{
	int nrow = m_nPoints.Row();
	double v = 0.0;

	//20200930
	if (x <= m_nPoints[0][0])
	{
		return m_nPoints[0][1];
	}
	else if (x >= m_nPoints[nrow - 1][0])
	{
		return m_nPoints[nrow - 1][1];
	}
	else
	{
		double u = (x - m_nPoints[0][0]) / m_L0;
		//v = this->BSL(3, m_su0, m_d0, nrow, u);
		//20210604
		v = this->BSL(m_su0, m_d0, nrow, u);
	}
	return  v;
}

double CubicSpline::SlopeAt(double x)
{
	int nrow = m_Derivative.Row();
	double v = 0.0;

	//20200930
	if (x <= m_Derivative[0][0])
	{
		return m_Derivative[0][1];
	}
	else if (x >= m_Derivative[nrow - 1][0])
	{
		return m_Derivative[nrow - 1][1];
	}
	else{
		double u = (x - m_Derivative[0][0]) / m_L1;
		//v = this->BSL(3, m_su1, m_d1, nrow, u);
		//20210604
		v = this->BSL(m_su1, m_d1, nrow, u);
	}
	return v;
}
double CubicSpline::Slope2ndAt(double x)
{
	int nrow = m_Derivative2nd.Row();
	double v = 0.0;

	//20200930
	if (x <= m_Derivative2nd[0][0])
	{
		return m_Derivative2nd[0][1];
	}
	else if (x >= m_Derivative2nd[nrow - 1][0])
	{
		return m_Derivative2nd[nrow - 1][1];
	}
	else{
		double u = (x - m_Derivative2nd[0][0]) / m_L2;
		//v = this->BSL(3, m_su2, m_d2, nrow, u);
		//20210604
		v = this->BSL(m_su2, m_d2, nrow, u);
	}
	return v;
}
//文件保存与读写
BOOL CubicSpline::SaveCubicSpline(void)
{
	CString szFilter = _T("BINARY FILES(*.CSP)|*.CSP");
	CFileDialog dlg(FALSE, _T("CubicSpline.csp"), NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, szFilter);
	if (dlg.DoModal() != IDOK)
	{
		return FALSE;
	}
	CFile file(dlg.GetFileName(), CFile::modeCreate | CFile::modeReadWrite);
	CArchive ar(&file, CArchive::store);
	Serialize(ar);
	return TRUE;
}
BOOL CubicSpline::LoadCubicSpline(void)
{
	CString szFilter = _T("BINARY FILES(*.CSP)|*.CSP");
	CFileDialog dlg(TRUE, _T("CubicSpline.csp"), NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, szFilter);
	if (dlg.DoModal() != IDOK)
	{
		return FALSE;
	}
	CFile file(dlg.GetFileName(), CFile::modeRead);
	CArchive ar(&file, CArchive::load);
	Serialize(ar);
	return TRUE;
}





			

