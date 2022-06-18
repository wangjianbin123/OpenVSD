// AEContGeo.cpp : 实现文件
//
#include "pch.h"
#include "AEContGeo.h"

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

#ifndef _EPSINON_V_
#define EPSINON_V 1E-6
#endif

#ifndef _EPSINON_ABS_
#define EPSINON_ABS 1E-6
#endif

#ifndef _EPSINON_RATIO_
#define EPSINON_RATIO 1E-3
#endif

#ifndef _ITERACTION_NUM_
#define ITERACTION_NUM 1000
#endif

// AEContGeo
//IMPLEMENT_SERIAL(AEContGeo,CObject,VERSIONABLE_SCHEMA|2)

AEContGeo::AEContGeo()
{
	m_wg.Empty();
	Left=0;
}
AEContGeo::AEContGeo(WheelGeometry &wg,BOOL bl)
{
	m_wg=wg;
	Left=bl;
}
void AEContGeo::setParameter(WheelGeometry &wg, BOOL bl)
{
	m_wg = wg;
	Left = bl;
}
AEContGeo::AEContGeo(const AEContGeo &aecg)
{
	m_wg=aecg.getWheelGeo();
	Left=aecg.beLeft();
}

AEContGeo& AEContGeo::operator =(const AEContGeo &aecg)
{
	if (this==&aecg)
	{
		return *this;
	}
	else
	{
		m_wg=aecg.getWheelGeo();
		Left=aecg.beLeft();
		return *this;
	}	
}

VectorN AEContGeo::AssFunVal(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat)
{
	VectorN tmp(4);
	Vector3x tr1,tr2,tw1,tw2,Rwr,Nr;
	if (this->beLeft()==TRUE)
	{
		tr1=tk.LeftRail_T1(surpara[2],surpara[3]);
		tr2=tk.LeftRail_T2(surpara[2],surpara[3]);
		tw1=rotmat*m_wg.getLocal_T1(surpara[0],surpara[1]);
		tw2=rotmat*m_wg.getLocal_T2(surpara[0],surpara[1]);
		Rwr=profpos+(rotmat*m_wg.getLocalPos(surpara[0],surpara[1]))-tk.LeftPositon(surpara[2],surpara[3]);
		Nr=tk.LeftRail_N(surpara[2],surpara[3]);
	}
	else
	{
		tr1=tk.RightRail_T1(surpara[2],surpara[3]);
		tr2=tk.RightRail_T2(surpara[2],surpara[3]);
		tw1=rotmat*m_wg.getLocal_T1(surpara[0],surpara[1]);
		tw2=rotmat*m_wg.getLocal_T2(surpara[0],surpara[1]);
		Rwr=profpos+(rotmat*m_wg.getLocalPos(surpara[0],surpara[1]))-tk.RightPosition(surpara[2],surpara[3]);
		Nr=tk.RightRail_N(surpara[2],surpara[3]);
	}
	tmp[0]=tr1*Rwr;
	tmp[1]=tr2*Rwr;
	tmp[2]=tw1*Nr;
	tmp[3]=tw2*Nr;
	return tmp;
}

Matrix AEContGeo::AssJocob(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat)
{
	Matrix mx(4,4);
	Vector3x tr1,tr2,tw1,tw2,Rwr,Nr,tr1ds1,tr1ds2,tr2ds1,tr2ds2,tw1ds1,tw1ds2,tw2ds1,tw2ds2,Nrds1,Nrds2;
	if (this->beLeft()==1)
	{
		tr1=tk.LeftRail_T1(surpara[2],surpara[3]);
		tr2=tk.LeftRail_T2(surpara[2],surpara[3]);
		tw1=rotmat*m_wg.getLocal_T1(surpara[0],surpara[1]);
		tw2=rotmat*m_wg.getLocal_T2(surpara[0],surpara[1]);
		Rwr=profpos+(rotmat*m_wg.getLocalPos(surpara[0],surpara[1]))-tk.LeftPositon(surpara[2],surpara[3]);
		Nr=tk.LeftRail_N(surpara[2],surpara[3]);
		tr1ds1=tk.LeftRail_T1DS1(surpara[2],surpara[3]);
		tr1ds2=tk.LeftRail_T1DS2(surpara[2],surpara[3]);
		tr2ds1=tk.LeftRail_T2DS1(surpara[2],surpara[3]);
		tr2ds2=tk.LeftRail_T2DS2(surpara[2],surpara[3]);
		Nrds1=tk.LeftRail_NDS1(surpara[2],surpara[3]);
		Nrds2=tk.LeftRail_NDS2(surpara[2],surpara[3]);
		tw1ds1=rotmat*m_wg.getLocal_T1DS1(surpara[0],surpara[1]);
		tw1ds2=rotmat*m_wg.getLocal_T1DS2(surpara[0],surpara[1]);
		tw2ds1=rotmat*m_wg.getLocal_T2DS1(surpara[0],surpara[1]);
		tw2ds2=rotmat*m_wg.getLocal_T2DS2(surpara[0],surpara[1]);
	}
	else
	{
		tr1=tk.RightRail_T1(surpara[2],surpara[3]);
		tr2=tk.RightRail_T2(surpara[2],surpara[3]);
		tw1=rotmat*m_wg.getLocal_T1(surpara[0],surpara[1]);
		tw2=rotmat*m_wg.getLocal_T2(surpara[0],surpara[1]);
		Rwr=profpos+(rotmat*m_wg.getLocalPos(surpara[0],surpara[1]))-tk.RightPosition(surpara[2],surpara[3]);
		Nr=tk.RightRail_N(surpara[2],surpara[3]);
		tr1ds1=tk.RightRail_T1DS1(surpara[2],surpara[3]);
		tr1ds2=tk.RightRail_T1DS2(surpara[2],surpara[3]);
		tr2ds1=tk.RightRail_T2DS1(surpara[2],surpara[3]);
		tr2ds2=tk.RightRail_T2DS2(surpara[2],surpara[3]);
		Nrds1=tk.RightRail_NDS1(surpara[2],surpara[3]);
		Nrds2=tk.RightRail_NDS2(surpara[2],surpara[3]);
		tw1ds1=rotmat*m_wg.getLocal_T1DS1(surpara[0],surpara[1]);
		tw1ds2=rotmat*m_wg.getLocal_T1DS2(surpara[0],surpara[1]);
		tw2ds1=rotmat*m_wg.getLocal_T2DS1(surpara[0],surpara[1]);
		tw2ds2=rotmat*m_wg.getLocal_T2DS2(surpara[0],surpara[1]);
	}
	mx[0][0] = tr1*tw1; mx[0][1] = tr1*tw2; mx[0][2] = tr1ds1*Rwr - tr1*tr1; mx[0][3] = tr1ds2*Rwr - tr1*tr2;
	mx[1][0] = tr2*tw1; mx[1][1] = tr2*tw2; mx[1][2] = tr2ds1*Rwr - tr2*tr1; mx[1][3] = tr2ds2*Rwr - tr2*tr2;
	mx[2][0] = tw1ds1*Nr; mx[2][1] = tw1ds2*Nr; mx[2][2] = Nrds1*tw1; mx[2][3] = Nrds2*tw1;
	mx[3][0] = tw2ds1*Nr; mx[3][1] = tw2ds2*Nr; mx[3][2] = Nrds1*tw2; mx[3][3] = Nrds2*tw2;
	return mx;
}

Matrix AEContGeo::AssHessian(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat)
{
	Matrix mx(4, 4);
	Vector3x tr1, tr2, tw1, tw2, Rwr, Nr, tr1ds1, tr1ds2, tr2ds1, tr2ds2, tw1ds1, tw1ds2, tw2ds1, tw2ds2, Nrds1, Nrds2;
	Vector3x Nrds1ds1, Nrds2ds2, Nrds1ds2;
	if (this->beLeft() == 1)
	{
		tr1 = tk.LeftRail_T1(surpara[2], surpara[3]);
		tr2 = tk.LeftRail_T2(surpara[2], surpara[3]);
		tw1 = rotmat*m_wg.getLocal_T1(surpara[0], surpara[1]);
		tw2 = rotmat*m_wg.getLocal_T2(surpara[0], surpara[1]);
		Rwr = profpos + (rotmat*m_wg.getLocalPos(surpara[0], surpara[1])) - tk.LeftPositon(surpara[2], surpara[3]);
		Nr = tk.LeftRail_N(surpara[2], surpara[3]);
		tr1ds1 = tk.LeftRail_T1DS1(surpara[2], surpara[3]);
		tr1ds2 = tk.LeftRail_T1DS2(surpara[2], surpara[3]);
		tr2ds1 = tk.LeftRail_T2DS1(surpara[2], surpara[3]);
		tr2ds2 = tk.LeftRail_T2DS2(surpara[2], surpara[3]);
		Nrds1 = tk.LeftRail_NDS1(surpara[2], surpara[3]);
		Nrds2 = tk.LeftRail_NDS2(surpara[2], surpara[3]);
		tw1ds1 = rotmat*m_wg.getLocal_T1DS1(surpara[0], surpara[1]);
		tw1ds2 = rotmat*m_wg.getLocal_T1DS2(surpara[0], surpara[1]);
		tw2ds1 = rotmat*m_wg.getLocal_T2DS1(surpara[0], surpara[1]);
		tw2ds2 = rotmat*m_wg.getLocal_T2DS2(surpara[0], surpara[1]);
		Nrds1ds1 = tk.LeftRail_NDS1DS1(surpara[2], surpara[3]);
		Nrds2ds2 = tk.LeftRail_NDS2DS2(surpara[2], surpara[3]);
		Nrds1ds2 = tk.LeftRail_NDS1DS2(surpara[2], surpara[3]);
	}
	else
	{
		tr1 = tk.RightRail_T1(surpara[2], surpara[3]);
		tr2 = tk.RightRail_T2(surpara[2], surpara[3]);
		tw1 = rotmat*m_wg.getLocal_T1(surpara[0], surpara[1]);
		tw2 = rotmat*m_wg.getLocal_T2(surpara[0], surpara[1]);
		Rwr = profpos + (rotmat*m_wg.getLocalPos(surpara[0], surpara[1])) - tk.RightPosition(surpara[2], surpara[3]);
		Nr = tk.RightRail_N(surpara[2], surpara[3]);
		tr1ds1 = tk.RightRail_T1DS1(surpara[2], surpara[3]);
		tr1ds2 = tk.RightRail_T1DS2(surpara[2], surpara[3]);
		tr2ds1 = tk.RightRail_T2DS1(surpara[2], surpara[3]);
		tr2ds2 = tk.RightRail_T2DS2(surpara[2], surpara[3]);
		Nrds1 = tk.RightRail_NDS1(surpara[2], surpara[3]);
		Nrds2 = tk.RightRail_NDS2(surpara[2], surpara[3]);
		tw1ds1 = rotmat*m_wg.getLocal_T1DS1(surpara[0], surpara[1]);
		tw1ds2 = rotmat*m_wg.getLocal_T1DS2(surpara[0], surpara[1]);
		tw2ds1 = rotmat*m_wg.getLocal_T2DS1(surpara[0], surpara[1]);
		tw2ds2 = rotmat*m_wg.getLocal_T2DS2(surpara[0], surpara[1]);
		Nrds1ds1 = tk.RightRail_NDS1DS1(surpara[2], surpara[3]);
		Nrds2ds2 = tk.RightRail_NDS2DS2(surpara[2], surpara[3]);
		Nrds1ds2 = tk.RightRail_NDS1DS2(surpara[2], surpara[3]);
	}
	mx[0][0] = tr1*tw1ds1; mx[0][1] = tr1*tw2ds1; mx[0][2] = tr1ds1*tw1; mx[0][3] = tr1ds2*tw1;
	mx[1][0] = tr2*tw1ds2; mx[1][1] = tr2*tw2ds2; mx[1][2] = tr2ds1*tw2; mx[1][3] = tr2ds2*tw2;
	mx[2][0] = tw1ds1*Nrds1; mx[2][1] = tw1ds2*Nrds1; mx[2][2] = Nrds1ds1*tw1; mx[2][3] = Nrds1ds2*tw1;
	mx[3][0] = tw2ds1*Nrds2; mx[3][1] = tw2ds2*Nrds2; mx[3][2] = Nrds1ds2*tw2; mx[3][3] = Nrds2ds2*tw2;
	return mx;
}

VectorN AEContGeo::AssGaussNewtonStep(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat)
{
	VectorN s = !(~(this->AssJocob(tk, surpara, profpos, rotmat))*this->AssJocob(tk, surpara, profpos, rotmat))*(~(this->AssJocob(tk, surpara, profpos, rotmat)))*this->AssFunVal(tk, surpara, profpos, rotmat);
	return s;
}

VectorN AEContGeo::AssJacobianStep(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat)
{
	VectorN s = !(this->AssJocob(tk, surpara, profpos, rotmat))*this->AssFunVal(tk, surpara, profpos, rotmat);
	return s;
}

VectorN AEContGeo::AssConPara(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat)
{
	//this->InitConsolWnd();
	double k = 0.5;
	int num = 0;
	num += 1;
	VectorN x0 = surpara;
	VectorN Fx0 = this->AssFunVal(tk, x0, profpos, rotmat);
	VectorN dx = this->AssJacobianStep(tk, x0, profpos, rotmat);
	VectorN x1 = x0 - k*dx;
	VectorN Fx1 = this->AssFunVal(tk, x1, profpos, rotmat);
	double residual = Fx1.getNorm();
	//
	while (residual >= EPSINON_V && Fx1.getNorm() / Fx0.getNorm()>EPSINON_RATIO)
	{
		x0 = x1;
		Fx0 = Fx1;
		dx = this->AssJacobianStep(tk, x0, profpos, rotmat);
		x1 = x0 - k*dx;
		Fx1 = this->AssFunVal(tk, x1, profpos, rotmat);
		residual = Fx1.getNorm();
		num++;
		//printf("%12.10f,%6d\n", Fx1.getNorm(), num);
		// 20210510 adaptive k setting
		if (num >= ITERACTION_NUM)
		{
			k = k*0.5;
			x0 = surpara;
			Fx0 = this->AssFunVal(tk, x0, profpos, rotmat);
			dx = this->AssJacobianStep(tk, x0, profpos, rotmat);
			x1 = x0 - k*dx;
			Fx1 = this->AssFunVal(tk, x1, profpos, rotmat);
			residual = Fx1.getNorm();
			num = 1;
			continue;
		}
	}
	//printf("%6d\n", num);
	return x1;
	//FreeConsole();
}

VectorN AEContGeo::AssConPara2nd(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat)
{
	//this->InitConsolWnd();
	double k = 0.1;
	int num = 0;
	VectorN x0 = surpara;
	VectorN Fx0 = this->AssFunVal(tk, x0, profpos, rotmat);
	VectorN x1 = x0 - k*this->AssGaussNewtonStep(tk, x0, profpos, rotmat);
	VectorN Fx1 = this->AssFunVal(tk, x1, profpos, rotmat);
	double residual = Fx1.getNorm();

	while (residual >= EPSINON_V && Fx1.getNorm() / Fx0.getNorm()>EPSINON_RATIO)
	{
		x0 = x1;
		Fx0 = Fx1;
		x1 = x0 - k*this->AssGaussNewtonStep(tk, x0, profpos, rotmat);
		Fx1 = this->AssFunVal(tk, x1, profpos, rotmat);
		residual = Fx1.getNorm();
		num++;
		//printf("%12.10f,%6d\n", Fx1.getNorm(), num);
	}
	return x1;
}
double AEContGeo::Penetration(Track &tk,VectorN &surpara,Vector3x &profpos,Matrix &rotmat)
{
	double tmp;
	Vector3x Rwr,Nr;
	if (this->beLeft()==TRUE)
	{
		Rwr=profpos+(rotmat*m_wg.getLocalPos(surpara[0],surpara[1]))-tk.LeftPositon(surpara[2],surpara[3]);
		Nr=tk.LeftRail_N(surpara[2],surpara[3]);
	}
	else
	{
		Rwr=profpos+(rotmat*m_wg.getLocalPos(surpara[0],surpara[1]))-tk.RightPosition(surpara[2],surpara[3]);
		Nr=tk.RightRail_N(surpara[2],surpara[3]);
	}
	tmp=Rwr*Nr;
	return tmp;
}
Matrix AEContGeo::IteractonProcess(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat)
{
	std::vector<double> residualnorm;
	std::vector<double> wheelpnt;
	std::vector<double> railpnt;
	std::vector<double> pen;

	// push back iteraction start point
	residualnorm.push_back(this->AssFunVal(tk, surpara, profpos, rotmat).getNorm());
	wheelpnt.push_back(surpara[0]);
	railpnt.push_back(surpara[3]);
	pen.push_back(this->Penetration(tk, surpara, profpos, rotmat));
	// 
	this->InitConsolWnd();
	int num = 0;
	double k = 0.1;

	VectorN x0 = surpara;
	VectorN Fx0 = this->AssFunVal(tk, x0, profpos, rotmat);
	// initial residual
	double r0 = Fx0.getNorm();
	VectorN dx = this->AssJacobianStep(tk, x0, profpos, rotmat);

	VectorN x1 = x0 - k*dx;
	VectorN Fx1 = this->AssFunVal(tk, x1, profpos, rotmat);

	double r1 = Fx1.getNorm();
	double residual = Fx1.getNorm();
	num += 1;
	residualnorm.push_back(r1);
	wheelpnt.push_back(x1[0]);
	railpnt.push_back(x1[3]);
	pen.push_back(this->Penetration(tk, x1, profpos, rotmat));
	//
	double ratio = r1 / r0;
	while (ratio>EPSINON_RATIO && r1>EPSINON_ABS)
	{
		x0 = x1;
		Fx0 = Fx1;
		dx = this->AssJacobianStep(tk, x0, profpos, rotmat);
		x1 = x0 - k*dx;
		Fx1 = this->AssFunVal(tk, x1, profpos, rotmat);
		residual = Fx1.getNorm();
		r1 = residual;
		ratio = r1 / r0;
		num += 1;
		residualnorm.push_back(Fx1.getNorm());
		wheelpnt.push_back(x1[0]);
		railpnt.push_back(x1[3]);
		pen.push_back(this->Penetration(tk, x1, profpos, rotmat));
		printf("%6d, %12.10f\n", num, Fx1.getNorm());
	}
	int n = static_cast<int>(residualnorm.size());
	Matrix mx(n, 4);
	for (int i = 0; i < n; i++)
	{
		mx[i][0] = residualnorm[i];
		mx[i][1] = wheelpnt[i];
		mx[i][2] = railpnt[i];
		mx[i][3] = pen[i];
	}
	return mx;
}

Matrix AEContGeo::IteractionConvergence(Track &tk, Vector3x profpos, int size, int maxiter, double rmin, double rmax, double wmin, double wmax)
{
	//
	this->InitConsolWnd();
	Matrix m(size, size);
	EulerAngle ea(0.0, 0.0, 0.0);
	Matrix rotmat = ea.getRotMat();
	VectorN surpara(4);
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			double rr = rmin + ((rmax - rmin) / static_cast<double>(size))*static_cast<double>(i);
			double ww = wmin + ((wmax - wmin) / static_cast<double>(size))*static_cast<double>(j);
			surpara[0] = ww;
			surpara[1] = 0.0;
			surpara[2] = 0.0;
			surpara[3] = rr;
			/*m[i][j] = maxiter+1-this->IteractionNum(tk, surpara, profpos, rotmat, maxiter);*/
			m[i][j] = this->IteractionNum(tk, surpara, profpos, rotmat, maxiter);
			printf("%6f\n", m[i][j]);
		}
	}
	return m;
}

int AEContGeo::IteractionNum(Track &tk, VectorN &surpara, Vector3x &profpos, Matrix &rotmat, int maxiter)
{
	int num = 0;
	double k = 0.2;
	VectorN x0 = surpara;
	VectorN x1 = x0 - k*this->AssJacobianStep(tk, x0, profpos, rotmat);
	VectorN Fx0 = this->AssFunVal(tk, x0, profpos, rotmat);
	VectorN Fx1 = this->AssFunVal(tk, x1, profpos, rotmat);
	double residual = Fx1.getNorm();
	num += 1;
	while (residual >= EPSINON_V && Fx1.getNorm() / Fx0.getNorm()>EPSINON_RATIO)
	{
		x0 = x1;
		Fx0 = Fx1;
		x1 = x0 - k*this->AssJacobianStep(tk, x0, profpos, rotmat);
		Fx1 = this->AssFunVal(tk, x1, profpos, rotmat);
		residual = Fx1.getNorm();
		num += 1;
		if (num >= maxiter)
			break;
	}
	return num;
}

void AEContGeo::Empty(void)
{
	m_wg.Empty();
}

void AEContGeo::InitConsolWnd(void)
{
	int nCrt = 0;
	FILE* fp;
	AllocConsole();
	nCrt = _open_osfhandle((intptr_t)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
	fp = _fdopen(nCrt, "w");
	*stdout = *fp;
	setvbuf(stdout, NULL, _IONBF, 0);
}
// AEContGeo 成员函数
AEContGeo::~AEContGeo()
{
	this->Empty();
}
