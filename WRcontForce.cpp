// WRcontForce.cpp : 实现文件
//
#include "pch.h"
#include "WRcontForce.h"

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

#ifndef _CSPLINE_H_
#define _CSPLINE_H_
#include "Spline.h"
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

#include <vector>

#ifndef _PI_
#define PI 3.14159267
#endif

#ifndef _Ena_
#define Ena 2.7182818285
#endif

// WRcontForce
IMPLEMENT_SERIAL(WRcontForce,CObject,VERSIONABLE_SCHEMA|2)

WRcontForce::WRcontForce()
{
	Er=2.07e11;
	Ew=2.07e11;
	Miur=0.27;
	Miuw=0.27;
	friction=0.3;
	this->InitMaterial();
	this->InitMatrix();
	this->InitCSpline();
}
WRcontForce::WRcontForce(double ER, double EW, double MIUR, double MIUW, double FRICTION)
{
	Er=ER;
	Ew=EW;
	Miur=MIUR;
	Miuw=MIUW;
	friction=FRICTION;
	this->InitMaterial(ER,EW,MIUR,MIUW);
	this->InitMatrix();
	this->InitCSpline();
}
WRcontForce::WRcontForce(const WRcontForce &wf)
{
	Er=wf.getER();
	Ew=wf.getEW();
	Miur=wf.getMIUR();
	Miuw=wf.getMIUW();
	friction=wf.getFriction();
	//20210412
	G = wf.getG();
	Miu = wf.getMiu();
	coff=wf.getCoff();
	beta=wf.getBeta();
	kalker1_c11=wf.getkalkerc11();
	kalker1_c22=wf.getkalkerc22();
	kalker1_c23=wf.getkalkerc23();
	kalker1_c33=wf.getkalkerc33();
	betacoff=wf.getBetacoff();
	kkcoff1_c11=wf.getkalkercoff111();
	kkcoff1_c22=wf.getkalkercoff122();
	kkcoff1_c23=wf.getkalkercoff123();
	kkcoff1_c33=wf.getkalkercoff133();
}
WRcontForce& WRcontForce::operator=(const WRcontForce &wf)
{
	if(this==&wf)
	{
		return *this;
	}
	else
	{
		Er=wf.getER();
		Ew=wf.getEW();
		Miur=wf.getMIUR();
		Miuw=wf.getMIUW();
		friction=wf.getFriction();
		//20210412
		G = wf.getG();
		Miu = wf.getMiu();
		coff=wf.getCoff();
		beta=wf.getBeta();
		kalker1_c11=wf.getkalkerc11();
		kalker1_c22=wf.getkalkerc22();
		kalker1_c23=wf.getkalkerc23();
		kalker1_c33=wf.getkalkerc33();

		betacoff=wf.getBetacoff();
		kkcoff1_c11=wf.getkalkercoff111();
		kkcoff1_c22=wf.getkalkercoff122();
		kkcoff1_c23=wf.getkalkercoff123();
		kkcoff1_c33=wf.getkalkercoff133();
		return *this;
	}
}
//构造初始化函数
void WRcontForce::InitMaterial(void)
{
	double Gw=Ew/(2.0*(1.0+Miuw));
	double Gr=Er/(2.0*(1.0+Miur));
	G=2.0/((1.0/Gr)+(1.0/Gw));
	Miu=0.5*G*(Miuw/Gw+Miur/Gr);
}

void WRcontForce::InitMaterial(double er, double ew, double miur, double miuw)
{
	double gw = ew / (2.0*(1.0 + miuw));
	double gr = er / (2.0*(1.0 + miur));
	G = 2.0 / ((1.0 / gr) + (1.0 / gw));
	Miu = 0.5*G*(Miuw / gw + Miur / gr);
}
void WRcontForce::InitMatrix(void)
{
	coff.SetMatrix(4,2);
	beta.SetMatrix(9,2);

	kalker1_c11.SetMatrix(19, 4);
	kalker1_c22.SetMatrix(19, 4);
	kalker1_c23.SetMatrix(19, 4);
	kalker1_c33.SetMatrix(19, 4);

	coff[0][0]=-1.086419052477; coff[0][1]=-0.773444080706;
	coff[1][0]=-0.106496432832; coff[1][1]=0.256695354565;
	coff[2][0]=1.350000000000;  coff[2][1]=0.200000000000;
	coff[3][0]=1.057885958251;  coff[3][1]=-0.280958376499;

	beta[8][0]=1.00000; beta[8][1]=0.3180;
	beta[7][0]=0.70410; beta[7][1]=0.3215;
	beta[6][0]=0.49030; beta[6][1]=0.3322;
	beta[5][0]=0.33330; beta[5][1]=0.3505;
	beta[4][0]=0.21740; beta[4][1]=0.3819;
	beta[3][0]=0.13250; beta[3][1]=0.4300;
	beta[2][0]=0.07180; beta[2][1]=0.5132;
	beta[1][0]=0.03110; beta[1][1]=0.6662;
	beta[0][0]=0.00765; beta[0][1]=1.1450;

	//C11
	kalker1_c11[0][0]=0.1; kalker1_c11[0][1]=2.51; kalker1_c11[0][2]=3.31; kalker1_c11[0][3]=4.85;
	kalker1_c11[1][0]=0.2; kalker1_c11[1][1]=2.59; kalker1_c11[1][2]=3.37; kalker1_c11[1][3]=4.81;
	kalker1_c11[2][0]=0.3; kalker1_c11[2][1]=2.68; kalker1_c11[2][2]=3.44; kalker1_c11[2][3]=4.80;
	kalker1_c11[3][0]=0.4; kalker1_c11[3][1]=2.78; kalker1_c11[3][2]=3.53; kalker1_c11[3][3]=4.82;
	kalker1_c11[4][0]=0.5; kalker1_c11[4][1]=2.88; kalker1_c11[4][2]=3.62; kalker1_c11[4][3]=4.83;
	kalker1_c11[5][0]=0.6; kalker1_c11[5][1]=2.98; kalker1_c11[5][2]=3.72; kalker1_c11[5][3]=4.91;
	kalker1_c11[6][0]=0.7; kalker1_c11[6][1]=3.09; kalker1_c11[6][2]=3.81; kalker1_c11[6][3]=4.97;
	kalker1_c11[7][0]=0.8; kalker1_c11[7][1]=3.19; kalker1_c11[7][2]=3.91; kalker1_c11[7][3]=5.05;
	kalker1_c11[8][0]=0.9; kalker1_c11[8][1]=3.29; kalker1_c11[8][2]=4.01; kalker1_c11[8][3]=5.12;

	kalker1_c11[9][0] = 1.0 / 1.0; kalker1_c11[9][1] = 3.40; kalker1_c11[9][2] = 4.12; kalker1_c11[9][3] = 5.20;
	kalker1_c11[10][0] = 1.0 / 0.9; kalker1_c11[10][1] = 3.51; kalker1_c11[10][2] = 4.22; kalker1_c11[10][3] = 5.30;
	kalker1_c11[11][0] = 1.0 / 0.8; kalker1_c11[11][1] = 3.65; kalker1_c11[11][2] = 4.36; kalker1_c11[11][3] = 5.42;
	kalker1_c11[12][0] = 1.0 / 0.7; kalker1_c11[12][1] = 3.82; kalker1_c11[12][2] = 4.54; kalker1_c11[12][3] = 5.58;
	kalker1_c11[13][0] = 1.0 / 0.6; kalker1_c11[13][1] = 4.06; kalker1_c11[13][2] = 4.78; kalker1_c11[13][3] = 5.80;
	kalker1_c11[14][0] = 1.0 / 0.5; kalker1_c11[14][1] = 4.37; kalker1_c11[14][2] = 5.10; kalker1_c11[14][3] = 6.11;
	kalker1_c11[15][0] = 1.0 / 0.4; kalker1_c11[15][1] = 4.84; kalker1_c11[15][2] = 5.57; kalker1_c11[15][3] = 6.57;
	kalker1_c11[16][0] = 1.0 / 0.3; kalker1_c11[16][1] = 5.57; kalker1_c11[16][2] = 6.34; kalker1_c11[16][3] = 7.34;
	kalker1_c11[17][0] = 1.0 / 0.2; kalker1_c11[17][1] = 6.96; kalker1_c11[17][2] = 7.78; kalker1_c11[17][3] = 8.82;
	kalker1_c11[18][0] = 1.0 / 0.1; kalker1_c11[18][1] = 10.7; kalker1_c11[18][2] = 11.7; kalker1_c11[18][3] = 12.9;
	
	//C22
	kalker1_c22[0][0]=0.1; kalker1_c22[0][1]=2.51; kalker1_c22[0][2]=2.52; kalker1_c22[0][3]=2.53;
	kalker1_c22[1][0]=0.2; kalker1_c22[1][1]=2.59; kalker1_c22[1][2]=2.63; kalker1_c22[1][3]=2.66;
	kalker1_c22[2][0]=0.3; kalker1_c22[2][1]=2.68; kalker1_c22[2][2]=2.75; kalker1_c22[2][3]=2.81;
	kalker1_c22[3][0]=0.4; kalker1_c22[3][1]=2.78; kalker1_c22[3][2]=2.88; kalker1_c22[3][3]=2.98;
	kalker1_c22[4][0]=0.5; kalker1_c22[4][1]=2.88; kalker1_c22[4][2]=3.01; kalker1_c22[4][3]=3.14;
	kalker1_c22[5][0]=0.6; kalker1_c22[5][1]=2.98; kalker1_c22[5][2]=3.14; kalker1_c22[5][3]=3.31;
	kalker1_c22[6][0]=0.7; kalker1_c22[6][1]=3.09; kalker1_c22[6][2]=3.28; kalker1_c22[6][3]=3.48;
	kalker1_c22[7][0]=0.8; kalker1_c22[7][1]=3.19; kalker1_c22[7][2]=3.41; kalker1_c22[7][3]=3.65;
	kalker1_c22[8][0]=0.9; kalker1_c22[8][1]=3.29; kalker1_c22[8][2]=3.54; kalker1_c22[8][3]=3.82;

	kalker1_c22[9][0] = 1.0 / 1.0; kalker1_c22[9][1] = 3.40; kalker1_c22[9][2] = 3.67; kalker1_c22[9][3] = 3.98;
	kalker1_c22[10][0] = 1.0 / 0.9; kalker1_c22[10][1] = 3.51; kalker1_c22[10][2] = 3.81; kalker1_c22[10][3] = 4.16;
	kalker1_c22[11][0] = 1.0 / 0.8; kalker1_c22[11][1] = 3.65; kalker1_c22[11][2] = 3.99; kalker1_c22[11][3] = 4.39;
	kalker1_c22[12][0] = 1.0 / 0.7; kalker1_c22[12][1] = 3.82; kalker1_c22[12][2] = 4.21; kalker1_c22[12][3] = 4.67;
	kalker1_c22[13][0] = 1.0 / 0.6; kalker1_c22[13][1] = 4.06; kalker1_c22[13][2] = 4.50; kalker1_c22[13][3] = 5.04;
	kalker1_c22[14][0] = 1.0 / 0.5; kalker1_c22[14][1] = 4.37; kalker1_c22[14][2] = 4.90; kalker1_c22[14][3] = 5.56;
	kalker1_c22[15][0] = 1.0 / 0.4; kalker1_c22[15][1] = 4.84; kalker1_c22[15][2] = 5.48; kalker1_c22[15][3] = 6.31;
	kalker1_c22[16][0] = 1.0 / 0.3; kalker1_c22[16][1] = 5.57; kalker1_c22[16][2] = 6.40; kalker1_c22[16][3] = 7.51;
	kalker1_c22[17][0] = 1.0 / 0.2; kalker1_c22[17][1] = 6.96; kalker1_c22[17][2] = 8.14; kalker1_c22[17][3] = 9.79;
	kalker1_c22[18][0] = 1.0 / 0.1; kalker1_c22[18][1] = 10.7; kalker1_c22[18][2] = 12.8; kalker1_c22[18][3] = 16.0;
	
	//C23
	kalker1_c23[0][0]=0.1; kalker1_c23[0][1]=0.334; kalker1_c23[0][2]=0.473; kalker1_c23[0][3]=0.731;
	kalker1_c23[1][0]=0.2; kalker1_c23[1][1]=0.483; kalker1_c23[1][2]=0.603; kalker1_c23[1][3]=0.809;
	kalker1_c23[2][0]=0.3; kalker1_c23[2][1]=0.607; kalker1_c23[2][2]=0.715; kalker1_c23[2][3]=0.889;
	kalker1_c23[3][0]=0.4; kalker1_c23[3][1]=0.720; kalker1_c23[3][2]=0.823; kalker1_c23[3][3]=0.977;
	kalker1_c23[4][0]=0.5; kalker1_c23[4][1]=0.827; kalker1_c23[4][2]=0.929; kalker1_c23[4][3]=1.070;
	kalker1_c23[5][0]=0.6; kalker1_c23[5][1]=0.930; kalker1_c23[5][2]=1.030; kalker1_c23[5][3]=1.180;
	kalker1_c23[6][0]=0.7; kalker1_c23[6][1]=1.030; kalker1_c23[6][2]=1.140; kalker1_c23[6][3]=1.290;
	kalker1_c23[7][0]=0.8; kalker1_c23[7][1]=1.130; kalker1_c23[7][2]=1.250; kalker1_c23[7][3]=1.400;
	kalker1_c23[8][0]=0.9; kalker1_c23[8][1]=1.230; kalker1_c23[8][2]=1.360; kalker1_c23[8][3]=1.510;

	kalker1_c23[9][0] = 1.0 / 1.0; kalker1_c23[9][1] = 1.33; kalker1_c23[9][2] = 1.47; kalker1_c23[9][3] = 1.63;
	kalker1_c23[10][0] = 1.0 / 0.9; kalker1_c23[10][1] = 1.44; kalker1_c23[10][2] = 1.57; kalker1_c23[10][3] = 1.77;
	kalker1_c23[11][0] = 1.0 / 0.8; kalker1_c23[11][1] = 1.58; kalker1_c23[11][2] = 1.75; kalker1_c23[11][3] = 1.94;
	kalker1_c23[12][0] = 1.0 / 0.7; kalker1_c23[12][1] = 1.76; kalker1_c23[12][2] = 1.95; kalker1_c23[12][3] = 2.18;
	kalker1_c23[13][0] = 1.0 / 0.6; kalker1_c23[13][1] = 2.01; kalker1_c23[13][2] = 2.23; kalker1_c23[13][3] = 2.50;
	kalker1_c23[14][0] = 1.0 / 0.5; kalker1_c23[14][1] = 2.35; kalker1_c23[14][2] = 2.62; kalker1_c23[14][3] = 2.96;
	kalker1_c23[15][0] = 1.0 / 0.4; kalker1_c23[15][1] = 2.88; kalker1_c23[15][2] = 3.24; kalker1_c23[15][3] = 3.70;
	kalker1_c23[16][0] = 1.0 / 0.3; kalker1_c23[16][1] = 3.79; kalker1_c23[16][2] = 4.32; kalker1_c23[16][3] = 5.01;
	kalker1_c23[17][0] = 1.0 / 0.2; kalker1_c23[17][1] = 5.72; kalker1_c23[17][2] = 6.63; kalker1_c23[17][3] = 7.89;
	kalker1_c23[18][0] = 1.0 / 0.1; kalker1_c23[18][1] = 12.2; kalker1_c23[18][2] = 14.6; kalker1_c23[18][3] = 18.0;
	
	//C33
	kalker1_c33[0][0]=0.1; kalker1_c33[0][1]=6.42; kalker1_c33[0][2]=8.28; kalker1_c33[0][3]=11.7;
	kalker1_c33[1][0]=0.2; kalker1_c33[1][1]=3.46; kalker1_c33[1][2]=4.27; kalker1_c33[1][3]=5.66;
	kalker1_c33[2][0]=0.3; kalker1_c33[2][1]=2.49; kalker1_c33[2][2]=2.96; kalker1_c33[2][3]=3.72;
	kalker1_c33[3][0]=0.4; kalker1_c33[3][1]=2.02; kalker1_c33[3][2]=2.32; kalker1_c33[3][3]=2.77;
	kalker1_c33[4][0]=0.5; kalker1_c33[4][1]=1.74; kalker1_c33[4][2]=1.93; kalker1_c33[4][3]=2.22;
	kalker1_c33[5][0]=0.6; kalker1_c33[5][1]=1.56; kalker1_c33[5][2]=1.68; kalker1_c33[5][3]=1.86;
	kalker1_c33[6][0]=0.7; kalker1_c33[6][1]=1.43; kalker1_c33[6][2]=1.50; kalker1_c33[6][3]=1.60;
	kalker1_c33[7][0]=0.8; kalker1_c33[7][1]=1.34; kalker1_c33[7][2]=1.37; kalker1_c33[7][3]=1.42;
	kalker1_c33[8][0]=0.9; kalker1_c33[8][1]=1.27; kalker1_c33[8][2]=1.27; kalker1_c33[8][3]=1.27;

	kalker1_c33[9][0] = 1.0 / 1.0; kalker1_c33[9][1] = 1.210; kalker1_c33[9][2] = 1.190; kalker1_c33[9][3] = 1.160;
	kalker1_c33[10][0] = 1.0 / 0.9; kalker1_c33[10][1] = 1.160; kalker1_c33[10][2] = 1.110; kalker1_c33[10][3] = 1.060;
	kalker1_c33[11][0] = 1.0 / 0.8; kalker1_c33[11][1] = 1.100; kalker1_c33[11][2] = 1.040; kalker1_c33[11][3] = 0.954;
	kalker1_c33[12][0] = 1.0 / 0.7; kalker1_c33[12][1] = 1.050; kalker1_c33[12][2] = 0.965; kalker1_c33[12][3] = 0.852;
	kalker1_c33[13][0] = 1.0 / 0.6; kalker1_c33[13][1] = 1.010; kalker1_c33[13][2] = 0.892; kalker1_c33[13][3] = 0.751;
	kalker1_c33[14][0] = 1.0 / 0.5; kalker1_c33[14][1] = 0.958; kalker1_c33[14][2] = 0.819; kalker1_c33[14][3] = 0.650;
	kalker1_c33[15][0] = 1.0 / 0.4; kalker1_c33[15][1] = 0.912; kalker1_c33[15][2] = 0.747; kalker1_c33[15][3] = 0.549;
	kalker1_c33[16][0] = 1.0 / 0.3; kalker1_c33[16][1] = 0.868; kalker1_c33[16][2] = 0.674; kalker1_c33[16][3] = 0.446;
	kalker1_c33[17][0] = 1.0 / 0.2; kalker1_c33[17][1] = 0.828; kalker1_c33[17][2] = 0.601; kalker1_c33[17][3] = 0.341;
	kalker1_c33[18][0] = 1.0 / 0.1; kalker1_c33[18][1] = 0.795; kalker1_c33[18][2] = 0.526; kalker1_c33[18][3] = 0.228;
}
void WRcontForce::InitCSpline(void)
{
	betacoff.setParameters(beta);
	std::vector<CSpline> kk1_c11;

	//interpolated by miu
	for (int i=0;i<19;i++)
	{
		Matrix mx(3,2);
		mx[0][0]=0.0;
		mx[1][0]=0.25; 
		mx[2][0]=0.5;
		mx[0][1]=kalker1_c11[i][1]; 
		mx[1][1]=kalker1_c11[i][2];
		mx[2][1]=kalker1_c11[i][3];
		CSpline cs(mx);
		kk1_c11.push_back(cs);
	}
	std::vector<CSpline> kk1_c22;
	for (int i=0;i<19;i++)
	{
		Matrix mx(3,2);
		mx[0][0]=0.0; 
		mx[1][0]=0.25;
		mx[2][0]=0.5;
		mx[0][1]=kalker1_c22[i][1]; 
		mx[1][1]=kalker1_c22[i][2]; 
		mx[2][1]=kalker1_c22[i][3];
		CSpline cs(mx);
		kk1_c22.push_back(cs);
	}
	std::vector<CSpline> kk1_c23;
	for (int i=0;i<19;i++)
	{
		Matrix mx(3,2);
		mx[0][0]=0.0; 
		mx[1][0]=0.25; 
		mx[2][0]=0.5;
		mx[0][1]=kalker1_c23[i][1]; 
		mx[1][1]=kalker1_c23[i][2]; 
		mx[2][1]=kalker1_c23[i][3];
		CSpline cs(mx);
		kk1_c23.push_back(cs);
	}
	std::vector<CSpline> kk1_c33;
	for (int i=0;i<19;i++)
	{
		Matrix mx(3,2);
		mx[0][0]=0.0; 
		mx[1][0]=0.25; 
		mx[2][0]=0.5;
		mx[0][1]=kalker1_c33[i][1]; 
		mx[1][1]=kalker1_c33[i][2]; 
		mx[2][1]=kalker1_c33[i][3];
		CSpline cs(mx);
		kk1_c33.push_back(cs);
	}
	
	Matrix k1_c11(19,2);
	Matrix k1_c22(19,2);
	Matrix k1_c23(19,2);
	Matrix k1_c33(19,2);

	//20210409

	k1_c11[0][0] = 0.1; k1_c11[1][0] = 0.2; k1_c11[2][0] = 0.3; k1_c11[3][0] = 0.4; k1_c11[4][0] = 0.5;
	k1_c11[5][0] = 0.6; k1_c11[6][0] = 0.7; k1_c11[7][0] = 0.8; k1_c11[8][0] = 0.9; k1_c11[9][0] = 1.0;
	k1_c11[10][0] = 1.0 / 0.9; k1_c11[11][0] = 1.0 / 0.8; k1_c11[12][0] = 1.0 / 0.7; k1_c11[13][0] = 1.0 / 0.6; k1_c11[14][0] = 1.0 / 0.5;
	k1_c11[15][0] = 1.0 / 0.4; k1_c11[16][0] = 1.0 / 0.3; k1_c11[17][0] = 1.0/0.2; k1_c11[18][0] = 1.0/0.1;

	k1_c22[0][0] = 0.1; k1_c22[1][0] = 0.2; k1_c22[2][0] = 0.3; k1_c22[3][0] = 0.4; k1_c22[4][0] = 0.5;
	k1_c22[5][0] = 0.6; k1_c22[6][0] = 0.7; k1_c22[7][0] = 0.8; k1_c22[8][0] = 0.9; k1_c22[9][0] = 1.0;
	k1_c22[10][0] = 1.0 / 0.9; k1_c22[11][0] = 1.0 / 0.8; k1_c22[12][0] = 1.0 / 0.7; k1_c22[13][0] = 1.0 / 0.6; k1_c22[14][0] = 1.0 / 0.5;
	k1_c22[15][0] = 1.0 / 0.4; k1_c22[16][0] = 1.0 / 0.3; k1_c22[17][0] = 1.0 / 0.2; k1_c22[18][0] = 1.0 / 0.1;

	k1_c23[0][0] = 0.1; k1_c23[1][0] = 0.2; k1_c23[2][0] = 0.3; k1_c23[3][0] = 0.4; k1_c23[4][0] = 0.5;
	k1_c23[5][0] = 0.6; k1_c23[6][0] = 0.7; k1_c23[7][0] = 0.8; k1_c23[8][0] = 0.9; k1_c23[9][0] = 1.0;
	k1_c23[10][0] = 1.0 / 0.9; k1_c23[11][0] = 1.0 / 0.8; k1_c23[12][0] = 1.0 / 0.7; k1_c23[13][0] = 1.0 / 0.6; k1_c23[14][0] = 1.0 / 0.5;
	k1_c23[15][0] = 1.0 / 0.4; k1_c23[16][0] = 1.0 / 0.3; k1_c23[17][0] = 1.0 / 0.2; k1_c23[18][0] = 1.0 / 0.1;

	k1_c33[0][0] = 0.1; k1_c33[1][0] = 0.2; k1_c33[2][0] = 0.3; k1_c33[3][0] = 0.4; k1_c33[4][0] = 0.5;
	k1_c33[5][0] = 0.6; k1_c33[6][0] = 0.7; k1_c33[7][0] = 0.8; k1_c33[8][0] = 0.9; k1_c33[9][0] = 1.0;
	k1_c33[10][0] = 1.0 / 0.9; k1_c33[11][0] = 1.0 / 0.8; k1_c33[12][0] = 1.0 / 0.7; k1_c33[13][0] = 1.0 / 0.6; k1_c33[14][0] = 1.0 / 0.5;
	k1_c33[15][0] = 1.0 / 0.4; k1_c33[16][0] = 1.0 / 0.3; k1_c33[17][0] = 1.0 / 0.2; k1_c33[18][0] = 1.0 / 0.1;

	for(int i=0;i<19;i++)
	{
		k1_c11[i][1]=kk1_c11[i].ValueAtC(Miu);
		k1_c22[i][1]=kk1_c22[i].ValueAtC(Miu);
		k1_c23[i][1]=kk1_c23[i].ValueAtC(Miu);
		k1_c33[i][1]=kk1_c33[i].ValueAtC(Miu);
	}
	CSpline cs1(k1_c11);
	CSpline cs2(k1_c22); 
	CSpline cs3(k1_c23); 
	CSpline cs4(k1_c33);

	kkcoff1_c11=cs1;
	kkcoff1_c22=cs2; 
	kkcoff1_c23=cs3;
	kkcoff1_c33=cs4;
}
void WRcontForce::setMaterial(double er,double ew,double miur,double miuw,double fri)
{
	Er=er;
	Ew=ew;
	Miur=miur;
	Miuw=miuw;
	friction=fri;
	this->InitMaterial();
	this->InitMatrix();
	this->InitCSpline();
}

void WRcontForce::Serialize(CArchive &ar)
{
	CObject::Serialize(ar);
	if (ar.IsStoring()==TRUE)
	{
		ar<<Er<<Ew<<Miur<<Miuw<<friction<<G<<Miu;
		coff.Serialize(ar); beta.Serialize(ar);
		kalker1_c11.Serialize(ar);kalker1_c22.Serialize(ar);kalker1_c23.Serialize(ar);kalker1_c33.Serialize(ar);
		betacoff.Serialize(ar);
		kkcoff1_c11.Serialize(ar);kkcoff1_c22.Serialize(ar);kkcoff1_c23.Serialize(ar);kkcoff1_c33.Serialize(ar);
	}
	else
	{
		double a1,a2,a3,a4,a5,a6,a7;
		ar>>a1>>a2>>a3>>a4>>a5>>a6>>a7;
		Er=a1;Ew=a2;Miur=a3;Miuw=a4;friction=a5;G=a6;Miu=a7;
		coff.Serialize(ar); beta.Serialize(ar);
		kalker1_c11.Serialize(ar);kalker1_c22.Serialize(ar);kalker1_c23.Serialize(ar);kalker1_c33.Serialize(ar);
		betacoff.Serialize(ar);
		kkcoff1_c11.Serialize(ar);kkcoff1_c22.Serialize(ar);kkcoff1_c23.Serialize(ar);kkcoff1_c33.Serialize(ar);
	}
}
VectorN WRcontForce::AssK(VectorN &cur,double yaw)
{
	VectorN tmp(4);
	tmp[0] = (1.0 - pow(Miuw, 2.0)) / (PI*Ew);
	tmp[1] = (1.0 - pow(Miur, 2.0)) / (PI*Er);
	tmp[2] = 0.5*cur.getSum();
	double t1 = pow((cur[0] - cur[1]), 2.0);
	double t2 = pow((cur[2] - cur[3]), 2.0);
	double t3 = 2.0*(cur[0] - cur[1])*(cur[2] - cur[3])*cos(2.0*yaw);
	tmp[3] = 0.5*sqrt(t1 + t2 + t3);
	return tmp;
}
double WRcontForce::getFn(VectorN &k,double penetration)
{
	if (penetration>=0.0)
	{
		return 0.0;
	}
	else
	{
		double A=(k[2]-k[3])/2.0;
		double B=(k[2]+k[3])/2.0;
		double ab_ratio=A/B;
		//double force = ((4.0*betacoff.ValueAtC(ab_ratio)) / (3.0*(k[0] + k[1])*sqrt(k[2])))*pow(abs(penetration), 1.5);
		//double force = 10e9*pow(abs(penetration), 1.5);
		double force = 1e9 * abs(penetration);
		return force;
	}
}
VectorN WRcontForce::getab(VectorN &cur, double yaw, double penetration)
{
	VectorN tmp(2);
	if (penetration >= 0.0)
	{
		return tmp;
	}
	else
	{
		VectorN k = this->AssK(cur, yaw);
		double fn = this->getFn(k, penetration);
		VectorN t(2);
		VectorN mn(2);
		double theta = acos(abs(k[3] / k[2]));
		mn[0] = coff[0][0] * tan(theta - PI / 2.0) + coff[1][0] / pow(theta, coff[2][0]) + coff[3][0];
		mn[1] = 1.0 / (coff[0][1] * tan(theta - PI / 2.0) + 1.0) + coff[1][1] * pow(theta, coff[2][1]) + coff[3][1] * sin(theta);
		t[0] = mn[0] * pow(3.0*PI*fn*(k[0] + k[1]) / (4.0*k[2]), (1.0 / 3.0));
		t[1] = mn[1] * pow(3.0*PI*fn*(k[0] + k[1]) / (4.0*k[2]), (1.0 / 3.0));
		double longer = 0.0;
		double shoter = 0.0;
		if (t[0] >= t[1])
		{
			longer = t[0];
			shoter = t[1];
		}
		else
		{
			longer = t[1];
			shoter = t[0];
		}
		//
		if ((cur[0] + cur[2]) >= (cur[1] + cur[3]))
		{
			tmp[0] = shoter;
			tmp[1] = longer;
		}
		else
		{
			tmp[0] = longer;
			tmp[1] = shoter;
		}
		return tmp;
	}
}
VectorN WRcontForce::getab(VectorN &cur, double yaw, double Fn, double penetration)
{
	VectorN tmp(2);
	if (Fn <= 0.0)
	{
		return tmp;
	}
	else
	{
		VectorN k = this->AssK(cur, yaw);
		VectorN t(2);
		VectorN mn(2);
		double theta = acos(abs(k[3] / k[2]));
		mn[0] = coff[0][0] * tan(theta - PI / 2.0) + coff[1][0] / pow(theta, coff[2][0]) + coff[3][0];
		mn[1] = 1.0 / (coff[0][1] * tan(theta - PI / 2.0) + 1.0) + coff[1][1] * pow(theta, coff[2][1]) + coff[3][1] * sin(theta);
		t[0] = mn[0] * pow(3.0*PI*Fn*(k[0] + k[1]) / (4.0*k[2]), (1.0 / 3.0));
		t[1] = mn[1] * pow(3.0*PI*Fn*(k[0] + k[1]) / (4.0*k[2]), (1.0 / 3.0));
		double longer = 0.0;
		double shoter = 0.0;
		if (t[0] >= t[1])
		{
			longer = t[0];
			shoter = t[1];
		}
		else
		{
			longer = t[1];
			shoter = t[0];
		}
		//
		if ((cur[0] + cur[2]) >= (cur[1] + cur[3]))
		{
			tmp[0] = shoter;
			tmp[1] = longer;
		}
		else
		{
			tmp[0] = longer;
			tmp[1] = shoter;
		}
		return tmp;
	}
}
VectorN WRcontForce::getGandMiu(void)
{
	VectorN tmp(2);
	tmp[0] = G;
	tmp[1] = Miu;
	return tmp;
}
VectorN WRcontForce::getKalkerCoef(VectorN &ab)
{
	double ab_ratio;
	double c11, c22, c23, c33;
	ab_ratio = ab[0] / ab[1];
	c11 = kkcoff1_c11.ValueAtC(ab_ratio);
	c22 = kkcoff1_c22.ValueAtC(ab_ratio);
	c23 = kkcoff1_c23.ValueAtC(ab_ratio);
	c33 = kkcoff1_c33.ValueAtC(ab_ratio);
	VectorN kcoef(4);
	kcoef[0] = c11;
	kcoef[1] = c22;
	kcoef[2] = c23;
	kcoef[3] = c33;
	return kcoef;
}
VectorN WRcontForce::Kalker_Creepforce(VectorN &cur, VectorN &creepage, double yaw, double penetration)
{
	VectorN ab(2);
	ab=this->getab(cur,yaw,penetration);
	if (penetration>=0.0)
	{
		VectorN tmp(3);
		return tmp;
	}
	else
	{
		double ab_ratio;
		double c11,c22,c23,c33;
		ab_ratio = ab[0] / ab[1];
		c11 = kkcoff1_c11.ValueAtC(ab_ratio);
		c22 = kkcoff1_c22.ValueAtC(ab_ratio);
		c23 = kkcoff1_c23.ValueAtC(ab_ratio);
		c33 = kkcoff1_c33.ValueAtC(ab_ratio);
		Matrix kkcoff(3,3);
		kkcoff[0][0]=c11; 
		kkcoff[0][1]=0.00; 
		kkcoff[0][2]=0.00;
		kkcoff[1][0]=0.00; 
		kkcoff[1][1]=c22; 
		kkcoff[1][2]=sqrt(ab[0]*ab[1])*c23;
		kkcoff[2][0]=0.00; 
		kkcoff[2][1]=(-1.0)*sqrt(ab[0]*ab[1])*c23; 
		kkcoff[2][2]=ab[0]*ab[1]*c33;
		VectorN force=(-1.0)*G*ab[0]*ab[1]*kkcoff*creepage;
		return force;
	}
}

VectorN WRcontForce::SHE_Creepforce(VectorN &cur, VectorN &creepage, double yaw, double penetration)
{
	VectorN K=this->AssK(cur,yaw);
	double nforce=this->getFn(K,penetration);
	if (penetration>=0.0)
	{
		VectorN tmp(3); 
		return tmp;
	}
	else
	{
		double friction_limit=nforce*friction;
		VectorN kkforce=this->Kalker_Creepforce(cur,creepage,yaw,penetration);
		double FL=sqrt(pow(kkforce[0],2.0)+pow(kkforce[1],2.0));
		double mFL;
		if (FL<=(3.0*friction_limit))
		{
			double t=(FL/friction_limit);
			mFL=friction_limit*(t-(1.0/3.0)*pow(t,2)+(1.0/27.0)*pow(t,3));
		}
		else
		{
			mFL=friction_limit;
		}
		double modify=mFL/FL;
		VectorN mForce(2);
		mForce[0]=kkforce[0]; 
		mForce[1]=kkforce[1];
		mForce*=modify;
		VectorN tmp(3);
		tmp[0] = mForce[0];
		tmp[1] = mForce[1];
		tmp[2] = kkforce[2];
		return tmp;
	}
}
VectorN WRcontForce::Polach_Creepforce(VectorN &cur, VectorN &creepage, double yaw, double penetration)
{
	if (penetration>=0.0)
	{
		VectorN tmp(3);
		return tmp;
	}
	else
	{
		VectorN K4=this->AssK(cur,yaw);
		double N=this->getFn(K4,penetration);
		VectorN ab=this->getab(cur,yaw,penetration);
		double ab_ratio;
		double c11,c22,c23,c33;
		ab_ratio = ab[0] / ab[1];
		c11 = kkcoff1_c11.ValueAtC(ab_ratio);
		c22 = kkcoff1_c22.ValueAtC(ab_ratio);
		c23 = kkcoff1_c23.ValueAtC(ab_ratio);
		c33 = kkcoff1_c33.ValueAtC(ab_ratio);
		double zeta_x=creepage[0];
		double zeta_y=creepage[1]; 
		double spin=creepage[2];
		double zeta_yc;
		if (abs(zeta_y+spin*ab[0])<=abs(zeta_y))
		{
			zeta_yc=zeta_y;
		}
		if (abs(zeta_y+spin*ab[0])>abs(zeta_y))
		{
			zeta_yc=zeta_y+spin*ab[0];
		}
		double vc=sqrt(pow(zeta_x,2.0)+pow(zeta_yc,2.0));
		double v=sqrt(pow(zeta_x,2.0)+pow(zeta_y,2.0));
		double Ch=sqrt(pow((c11*zeta_x)/v,2.0)+pow((c22*zeta_y)/v,2.0));
		double epsilon=(1.0/4.0)*G*PI*ab[0]*ab[1]*Ch*vc/(friction*N);
		double Fpx=(-1.0)*(2.0*friction*N)*((epsilon/(1+pow(epsilon,2.0)))+atan(epsilon))/PI;

		double epsilon_y=(8.0/3.0)*(G*ab[1]*sqrt(ab[0]*ab[1])/(friction*N))*(c23*zeta_yc/(1.0+6.3*(1.0-pow(Ena,(-1.0*ab[0]/ab[1])))));
		double delta=(pow(epsilon_y,2.0)-1.0)/(pow(epsilon_y,2.0)+1.0);
		double K=abs(epsilon_y)*(pow(delta,3.0)/3.0-pow(delta,2.0)/2.0+(1.0/6.0))-(1.0/3.0)*sqrt(pow(1-pow(delta,2.0),3.0));
		double Fpy=(-9.0/16.0)*ab[0]*friction*N*K*(1.0+6.3*(1.0-pow(Ena,(-1.0*ab[0]/ab[1]))));

		VectorN kkforce = this->Kalker_Creepforce(cur, creepage, yaw, penetration);

		VectorN PolachForce(3);
		PolachForce[0] = Fpx*zeta_x / vc;
		PolachForce[1] = Fpx*zeta_y / vc + Fpy*spin / vc;
		PolachForce[2] = kkforce[2];
		return PolachForce;
	}
}

VectorN WRcontForce::None_Creepforce(VectorN &cur, VectorN &creepage, double yaw, double penetration)
{
	VectorN tmp(3);
	return tmp;
}
double WRcontForce::LeftNormalForce(VectorN &cur,double penetration,double yaw)
{
	return this->getFn(this->AssK(cur, yaw), penetration);
}
double WRcontForce::RightNormalForce(VectorN &cur,double penetration,double yaw)
{
	return this->getFn(this->AssK(cur, yaw), penetration);
}
double WRcontForce::LeftNormalDampForce(double penetrationvelocity,double penetration)
{
	double pen=0.0;
	if (penetration>=0.0)
	{
		pen=0.0;
	}
	else
	{
		pen=abs(penetration);
	}
	return -100e8*penetrationvelocity*pen;
	//return -10e8*penetrationvelocity*pen;
}
double WRcontForce::RightNormalDampForce(double penetrationvelocity,double penetration)
{
	double pen=0.0;
	if(penetration>=0.0)
	{
		pen=0.0;
	}
	else
	{
		pen=abs(penetration);
	}
	return -100e8*penetrationvelocity*pen;
	//return -10e8*penetrationvelocity*pen;
}
//
void WRcontForce::Empty(void)
{
	Er=0.0;
	Ew=0.0;
	Miur=0.0;
	Miuw=0.0;
	friction=0.0;
	G=0.0;
	Miu=0.0;
	coff.Empty();
	beta.Empty();
	kalker1_c11.Empty();
	kalker1_c22.Empty();
	kalker1_c23.Empty();
	kalker1_c33.Empty();
	betacoff.Empty();
	kkcoff1_c11.Empty();
	kkcoff1_c22.Empty();
	kkcoff1_c23.Empty();
	kkcoff1_c33.Empty();
}
WRcontForce::~WRcontForce()
{
	this->Empty();
}
