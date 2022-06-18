#pragma once

#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "Matrix.h"
#endif

#ifndef _VECTORN_H_
#define _VECTORN_H_
#include "VectorN.h"
#endif

#ifndef _VECTOR3X_H
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

#include <vector>

// WheelGeometry 命令目标

class WheelGeometry : public CObject
{
public:
	//构造函数
	WheelGeometry();
	WheelGeometry(double r,Matrix &mx,int smothindex,int smothindex1st,int smothindex2nd);
	WheelGeometry(double r, Matrix &mx, int smothindex, int smothindex1st, int smothindex2nd, int sm1, int sm2, int sm3);
	WheelGeometry(const WheelGeometry &wg);
	WheelGeometry& operator=(const WheelGeometry &wg);
	//void Serialize(CArchive &ar);
	//DECLARE_SERIAL(WheelGeometry)

	// const functions
	double get_radius(void) const { return radius; }
	Matrix get_DisMat(void) const { return DisMat; }
	CubicSpline get_Profile(void) const { return Profile; }
	CubicSpline get_Profile_1st(void) const { return Profile_1st; }
	CubicSpline get_Profile_2nd(void) const { return Profile_2nd; }
	Matrix getLPcurMat(void) const { return LPcurMat; }
	std::vector<CubicSpline> getlrp(void) const { return lrp; }
	Matrix getlpdeltas(void) const { return lpdeltas; }
	Matrix getlps(void) const { return lps; }
	std::vector<CubicSpline> getlcur(void) const { return lcur; }
	std::vector<CubicSpline> getxindic(void) const { return xindic; }
	// cur functions
	void setParameter(double r, Matrix &mx, int smothindex, int smothindex1st, int smothindex2nd);
	void setParameter(double r, Matrix &mx, int smothindex, int smothindex1st, int smothindex2nd, int sm1, int sm2, int sm3);
	void setPcurMat(void);//set x,z of points respect to points number related to LPcurMat matrix
	void setSPcur(void);
	void setcurDeltaS(void);
	void setcurSp(void);
	//20201010
	void setcurSp(int sm1, int sm2, int sm3);
	void InitCurvature(void);
	//20201010
	void InitCurvature(int sm1, int sm2, int sm3);
	//
	Vector3x getLocalPos(double s1,double s2);
	Vector3x getLocal_T1(double s1,double s2);//return local tangent vector
	Vector3x getLocal_T2(double s1,double s2);//return local tangent vector
	Vector3x getLocal_N(double s1,double s2);
	Vector3x getLocal_T1DS1(double s1,double s2);
	Vector3x getLocal_T1DS2(double s1,double s2);
	Vector3x getLocal_T2DS1(double s1,double s2);
	Vector3x getLocal_T2DS2(double s1,double s2);
	Vector3x getLocal_NDS1(double s1,double s2);
	Vector3x getLocal_NDS2(double s1,double s2);

	double getCurvatureP1(double s1);
	Vector3x getCurvatureVec(double s1);
	double getCurvatureP2(double s1);
	double getRadius(double s1);//20140317added
	double getProfileLow(void);
	double getProfileHigh(void);
	void Empty(void);

	//20201017
	Matrix getCurvatureMat(void);

	virtual ~WheelGeometry();

private:
	double radius;
	Matrix DisMat;//points on the thread
	CubicSpline Profile;
	CubicSpline Profile_1st;
	CubicSpline Profile_2nd;
	//曲率相关
	Matrix LPcurMat;
	std::vector<CubicSpline> lrp;//refer to t 0 x ,1 y ,2 xd ,3 yd
	Matrix lpdeltas;
	Matrix lps;
	std::vector<CubicSpline> lcur;//refer to s 0 x 1 y 2 dx 3 dy 4 dx2 5 dy2
	std::vector<CubicSpline> xindic; // 0 is left
};


