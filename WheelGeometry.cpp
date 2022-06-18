// WheelGeometry.cpp : 实现文件
//
#include "pch.h"
#include "WheelGeometry.h"

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

#ifndef _EULERPARA_H_
#define _EULERPARA_H_
#include "EulerPara.h"
#endif

#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_
#include "CubicSpline.h"
#endif

#ifndef _MATH_H_
#define _MATH_H_
#include <math.h>
#endif
#include <vector>

//IMPLEMENT_SERIAL(WheelGeometry,CObject,VERSIONABLE_SCHEMA|2)
// WheelGeometry

WheelGeometry::WheelGeometry()
{
	radius=0;
	DisMat.Empty();
	Profile.Empty();
	Profile_1st.Empty();
	Profile_2nd.Empty();
	LPcurMat.Empty();
	lrp.clear();
	lpdeltas.Empty();
	lps.Empty();
	lcur.clear();
	xindic.clear();
}

WheelGeometry::WheelGeometry(double r, Matrix &mx,int smothindex,int smothindex1st,int smothindex2nd)
{
	radius=r;
	DisMat=mx;
	Matrix mxTmp=mx;
	for (int i=0;i<mx.Row();i++)
	{
		mxTmp[i][1]+=r;
	}
	Profile.setParameters(mxTmp);
	//20200916
	Profile.gaussian_smothing_original(smothindex, 0.99);
	Profile.gaussian_smothing_derivative(smothindex1st, 0.99);
	Profile.gaussian_smothing_derivative2nd(smothindex2nd, 0.99);
	Profile_1st = Profile.Derivative();
	Profile_2nd = Profile.Derivative2nd();

	this->InitCurvature();
}

WheelGeometry::WheelGeometry(double r, Matrix &mx, int smothindex, int smothindex1st, int smothindex2nd, int sm1, int sm2, int sm3)
{
	radius = r;
	DisMat = mx;
	Matrix mxTmp = mx;
	for (int i = 0; i<mx.Row(); i++)
	{
		mxTmp[i][1] += r;
	}
	Profile.setParameters(mxTmp);
	//20200916
	Profile.gaussian_smothing_original(smothindex, 0.99);
	Profile.gaussian_smothing_derivative(smothindex1st, 0.99);
	Profile.gaussian_smothing_derivative2nd(smothindex2nd, 0.99);
	Profile_1st = Profile.Derivative();
	Profile_2nd = Profile.Derivative2nd();

	this->InitCurvature(sm1, sm2, sm3);
}

void WheelGeometry::setParameter(double r, Matrix &mx, int smothindex, int smothindex1st, int smothindex2nd)
{
	radius = r;
	DisMat = mx;
	Matrix mxTmp = mx;
	for (int i = 0; i<mx.Row(); i++)
	{
		mxTmp[i][1] += r;
	}

	Profile.setParameters(mxTmp);
	Profile.gaussian_smothing_original(smothindex, 0.99);
	Profile.gaussian_smothing_derivative(smothindex1st, 0.99);
	Profile.gaussian_smothing_derivative2nd(smothindex2nd, 0.99);
	Profile_1st = Profile.Derivative();
	Profile_2nd = Profile.Derivative2nd();

	this->InitCurvature();
}

void WheelGeometry::setParameter(double r, Matrix &mx, int smothindex, int smothindex1st, int smothindex2nd, int sm1, int sm2, int sm3)
{
	radius = r;
	DisMat = mx;
	Matrix mxTmp = mx;
	for (int i = 0; i<mx.Row(); i++)
	{
		mxTmp[i][1] += r;
	}
	Profile.setParameters(mxTmp);
	Profile.gaussian_smothing_original(smothindex, 0.99);
	Profile.gaussian_smothing_derivative(smothindex1st, 0.99);
	Profile.gaussian_smothing_derivative2nd(smothindex2nd, 0.99);
	Profile_1st = Profile.Derivative();
	Profile_2nd = Profile.Derivative2nd();

	this->InitCurvature(sm1, sm2, sm3);
}
WheelGeometry::WheelGeometry(const WheelGeometry &wg)
{
	radius=wg.get_radius();
	DisMat=wg.get_DisMat();
	Profile=wg.get_Profile();
	Profile_1st=wg.get_Profile_1st();
	Profile_2nd=wg.get_Profile_2nd();
	LPcurMat=wg.getLPcurMat();
	lrp=wg.getlrp();
	lpdeltas=wg.getlpdeltas();
	lps=wg.getlps();
	lcur=wg.getlcur();
	xindic=wg.getxindic();
}
//操作符重载
WheelGeometry& WheelGeometry::operator =(const WheelGeometry &wg)
{
	if (this==&wg)
	{
		return *this;
	}
	else
	{
		radius=wg.get_radius();
		DisMat=wg.get_DisMat();
		Profile=wg.get_Profile();
		Profile_1st=wg.get_Profile_1st();
		Profile_2nd=wg.get_Profile_2nd();
		LPcurMat=wg.getLPcurMat();
		lrp=wg.getlrp();
		lpdeltas=wg.getlpdeltas();
		lps=wg.getlps();
		lcur=wg.getlcur();
		xindic=wg.getxindic();
		return *this;
	}
}
void WheelGeometry::setPcurMat(void)
{
	LPcurMat.SetMatrix(DisMat.Row(),3);
	double j=0.0;
	for (int i=0;i<DisMat.Row();i++)
	{
		LPcurMat[i][0]=j;
		LPcurMat[i][1]=DisMat[i][0];
		LPcurMat[i][2]=DisMat[i][1]+radius;
		j+=1.0;
	}
}

void WheelGeometry::setSPcur(void)
{
	//index of geometry file colum1(x)
	CubicSpline lpx(LPcurMat.getDoubleCols(0,1));
	//index of geometry file colum2(y)
	CubicSpline lpy(LPcurMat.getDoubleCols(0,2));
	CubicSpline lpxd=lpx.Derivative();
	CubicSpline lpyd=lpy.Derivative();
	lrp.push_back(lpx);
	lrp.push_back(lpy); 
	lrp.push_back(lpxd);
	lrp.push_back(lpyd);
}

void WheelGeometry::setcurDeltaS(void)
{
	lpdeltas.SetMatrix(LPcurMat.Row()-1,2);
	double conven_delta=1.0e-8; 
	double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};
	double intpnt[5]={-0.90610,-0.538469,0,0.538469,0.90610};
	for (int i=1;i<LPcurMat.Row();i++)
	{
		double inte_interval=LPcurMat[i][0]-LPcurMat[i-1][0];
		double inte1=0.0;
		double inte2=0.0;
		double low,high,tmp,coff;
		int sub=2;
		for (int k=0;k<sub;k++)
		{
			low=LPcurMat[i-1][0]+k*inte_interval/sub;
			high=LPcurMat[i-1][0]+(k+1)*inte_interval/sub;
			tmp=0.0;
			for (int j=0;j<5;j++)
			{
				coff=((high-low)/2)*intpnt[j]+((high+low)/2);
				tmp+=weight[j]*((high-low)/2)*sqrt(pow(lrp[2].ValueAt(coff),2.0)+pow(lrp[3].ValueAt(coff),2.0));
			}
			inte2+=tmp;
		}
		while (abs(inte2-inte1)>conven_delta)
		{
			inte1=inte2;
			sub*=2;
			inte2=0.0;
			for (int k=0;k<sub;k++)
			{
				low=LPcurMat[i-1][0]+k*inte_interval/sub;
				high=LPcurMat[i-1][0]+(k+1)*inte_interval/sub;
				tmp=0.0;
				for (int j=0;j<5;j++)
				{
					coff=((high-low)/2)*intpnt[j]+((high+low)/2);
					tmp+=weight[j]*((high-low)/2)*sqrt(pow(lrp[2].ValueAt(coff),2.0)+pow(lrp[3].ValueAt(coff),2.0));
				}
				inte2+=tmp;
			}
		}
		lpdeltas[i-1][0]=i-1;
		lpdeltas[i-1][1]=inte2;
	}
	lps.SetMatrix(LPcurMat.Row(),4);
	for (int i=0;i<LPcurMat.Row();i++)
	{
		lps[i][0]=i;
		if (i>0)
		{
			lps[i][1]=lpdeltas[i-1][1]+lps[i-1][1];
		}
		lps[i][2]=LPcurMat[i][1];
		lps[i][3]=LPcurMat[i][2];
	}
}

void WheelGeometry::setcurSp(void)
{
	CubicSpline csx(lps.getDoubleCols(1,2));
	CubicSpline csy(lps.getDoubleCols(1,3));

	CubicSpline csxds = csx.Derivative();
	csxds.gaussian_smothing_original(10, 0.99);
	CubicSpline csyds=csy.Derivative();
	csyds.gaussian_smothing_original(10, 0.99);
	CubicSpline csxdss=csxds.Derivative();
	csxdss.gaussian_smothing_original(15, 0.99);
	CubicSpline csydss=csyds.Derivative();
	csydss.gaussian_smothing_original(15, 0.99);
	//
	lcur.push_back(csx);
	lcur.push_back(csy);
	lcur.push_back(csxds);
	lcur.push_back(csyds);
	lcur.push_back(csxdss);
	lcur.push_back(csydss);
	//
	CubicSpline lxindic(lps.getDoubleCols(2,1));
	xindic.push_back(lxindic);
}

void WheelGeometry::setcurSp(int sm1, int sm2, int sm3)
{
	CubicSpline csx(lps.getDoubleCols(1, 2));
	CubicSpline csy(lps.getDoubleCols(1, 3));
	//
	csx.gaussian_smothing_original(sm1, 0.99);
	csy.gaussian_smothing_original(sm1, 0.99);
	csx.gaussian_smothing_derivative(sm2, 0.99);
	csy.gaussian_smothing_derivative(sm2, 0.99);
	csx.gaussian_smothing_derivative2nd(sm3, 0.99);
	csy.gaussian_smothing_derivative2nd(sm3, 0.99);

	CubicSpline csxds = csx.Derivative();
	CubicSpline csxdss = csx.Derivative2nd();
	CubicSpline csyds = csy.Derivative();
	CubicSpline csydss = csy.Derivative2nd();
	lcur.push_back(csx);
	lcur.push_back(csy);
	lcur.push_back(csxds);
	lcur.push_back(csyds);
	lcur.push_back(csxdss);
	lcur.push_back(csydss);
	//
	CubicSpline lxindic(lps.getDoubleCols(2, 1));
	xindic.push_back(lxindic);
}
void WheelGeometry::InitCurvature(void)
{
	this->setPcurMat();
	this->setSPcur();
	this->setcurDeltaS();
	this->setcurSp();
}

void WheelGeometry::InitCurvature(int sm1, int sm2, int sm3)
{
	this->setPcurMat();
	this->setSPcur();
	this->setcurDeltaS();
	this->setcurSp(sm1, sm2, sm3);
}

//成员函数
Vector3x WheelGeometry::getLocalPos(double s1, double s2)
{
	double gs1=Profile.ValueAt(s1);
	// -1 means reverse of the wheel profiles
	Vector3x vec(gs1*sin(s2),s1,(-1)*gs1*cos(s2));
	return vec;
}
Vector3x WheelGeometry::getLocal_T1(double s1,double s2)
{
	double dgs1s1=Profile_1st.ValueAt(s1);
	Vector3x vec(dgs1s1*sin(s2),1.0,(-1)*dgs1s1*cos(s2));
	vec.toUnit();
	return vec;
}
Vector3x WheelGeometry::getLocal_T2(double s1, double s2)
{
	double gs1=Profile.ValueAt(s1);
	Vector3x vec(gs1*cos(s2),0.0,gs1*sin(s2));
	vec.toUnit();
	return vec;
}
Vector3x WheelGeometry::getLocal_N(double s1, double s2)
{
	double gs1=Profile.ValueAt(s1);
	double dgs1s1=Profile_1st.ValueAt(s1);
	Vector3x vec(gs1*sin(s2),(-1)*gs1*dgs1s1,(-1)*gs1*cos(s2));
	vec.toUnit();
	return vec;
}
Vector3x WheelGeometry::getLocal_T1DS1(double s1, double s2)
{
	double dg2s12=Profile_2nd.ValueAt(s1);
	Vector3x vec(dg2s12*sin(s2),0,(-1)*dg2s12*cos(s2));
	return vec;
}
Vector3x WheelGeometry::getLocal_T1DS2(double s1, double s2)
{
	double dgs1s1=Profile_1st.ValueAt(s1);
	Vector3x vec(dgs1s1*cos(s2),0,dgs1s1*sin(s2));
	return vec;
}
Vector3x WheelGeometry::getLocal_T2DS1(double s1, double s2)
{
	double dgs1s1=Profile_1st.ValueAt(s1);
	Vector3x vec(dgs1s1*cos(s2),0,dgs1s1*sin(s2));
	return vec;
}
Vector3x WheelGeometry::getLocal_T2DS2(double s1,double s2)
{
	double gs1=Profile.ValueAt(s1);
	Vector3x vec((-1)*gs1*sin(s2),0,gs1*cos(s2));
	return vec;
}
Vector3x WheelGeometry::getLocal_NDS1(double s1, double s2)
{
	double gs1=Profile.ValueAt(s1);
	double dgs1s1=Profile_1st.ValueAt(s1);
	double dg2s12=Profile_2nd.ValueAt(s1);
	Vector3x vec(dgs1s1*sin(s2),(-1)*dgs1s1*dgs1s1-gs1*dg2s12,(-1)*dgs1s1*cos(s2));
	return vec;
}
Vector3x WheelGeometry::getLocal_NDS2(double s1, double s2)
{
	double gs1=Profile.ValueAt(s1);
	Vector3x vec(gs1*cos(s2),0,gs1*sin(s2));
	return vec;
}
double WheelGeometry::getCurvatureP2(double s1)
	//s1 means wheel profile coordinates x
{
	double s=xindic[0].ValueAt(s1);
	Vector3x vec(lcur[4].ValueAt(s),lcur[5].ValueAt(s),0);
	if (vec.getRy()>=0)
	{
		return (-1.0)*vec.getNorm();
	}
	else
	{
		return vec.getNorm();
	}
}
Vector3x WheelGeometry::getCurvatureVec(double s1)
{	
	double s=xindic[0].ValueAt(s1);
	Vector3x vec(lcur[4].ValueAt(s),lcur[5].ValueAt(s),0);
	return vec;
}

double WheelGeometry::getCurvatureP1(double s1)
{
	return 1.0/(Profile.ValueAt(s1));
}
double WheelGeometry::getRadius(double s1)
{
	return Profile.ValueAt(s1);
}
double WheelGeometry::getProfileLow(void)
{
	return this->get_DisMat()[0][0];
}
double WheelGeometry::getProfileHigh(void)
{
	int pntnum = this->get_DisMat().Row();
	return this->get_DisMat()[pntnum - 1][0];
}

Matrix WheelGeometry::getCurvatureMat(void)
{
	int nrow = DisMat.Row();
	Matrix mx(nrow, 2);
	for (int i = 0; i < nrow; i++)
	{
		mx[i][0] = DisMat[i][0];
		mx[i][1] = this->getCurvatureP2(mx[i][0]);
	}
	return mx;
}

void WheelGeometry::Empty(void)
{
	radius=0;
	DisMat.Empty();
	Profile.Empty();
	Profile_1st.Empty();
	Profile_2nd.Empty();
	LPcurMat.Empty();
	lrp.clear();
	lpdeltas.Empty();
	lps.Empty();
	lcur.clear();
	xindic.clear();
}
WheelGeometry::~WheelGeometry()
{
	this->Empty();
}
// WheelGeometry 成员函数
