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

#ifndef _PI_
#define PI 3.14159267
#endif

#ifndef _Ena_
#define Ena 2.7182818285
#endif

// WRcontForce 命令目标

class WRcontForce : public CObject
{
public:
	WRcontForce();
	WRcontForce(double ER,double EW,double MIUR,double MIUW,double FRICTION);
	WRcontForce(const WRcontForce &wf);
	WRcontForce& operator=(const WRcontForce &wf);
	// const functions
	double getER(void) const { return Er; }
	double getEW(void) const { return Ew; }
	double getMIUR(void) const { return Miur; }
	double getMIUW(void) const { return Miuw; }
	double getFriction(void) const { return friction; }
	//20210412
	double getG(void) const { return G; }
	double getMiu(void) const { return Miu; }

	Matrix getCoff(void) const { return coff; }
	Matrix getBeta(void) const { return beta; }
	Matrix getkalkerc11(void) const { return kalker1_c11; }
	Matrix getkalkerc22(void) const { return kalker1_c22; }
	Matrix getkalkerc23(void) const { return kalker1_c23; }
	Matrix getkalkerc33(void) const { return kalker1_c33; }

	CSpline getBetacoff(void) const { return betacoff; }
	CSpline  getkalkercoff111(void) const { return kkcoff1_c11; }
	CSpline  getkalkercoff122(void) const { return kkcoff1_c22; }
	CSpline  getkalkercoff123(void) const { return kkcoff1_c23; }
	CSpline  getkalkercoff133(void) const { return kkcoff1_c33; }

	//
	void InitMaterial(void);
	void InitMaterial(double er, double ew, double miur, double miuw);
	void InitMatrix(void); 
	void InitCSpline(void);
	void setMaterial(double er,double ew,double miur,double miuw,double fri);

	//文件流操作
	void Serialize(CArchive &ar);
	DECLARE_SERIAL(WRcontForce)

	VectorN AssK(VectorN &cur,double yaw);
	double getFn(VectorN &k,double penetration); // return m and n in 2 dimentional vector
	VectorN getab(VectorN &cur,double yaw,double penetration); // [0] logitudinal [1] transverse
	VectorN getab(VectorN &cur, double yaw, double Fn,double pentration);//just for test, pentration not used
	
	//20210321
	VectorN getGandMiu(void);
	VectorN getKalkerCoef(VectorN &ab);

	//蠕滑力计算
	VectorN Kalker_Creepforce(VectorN &cur,VectorN &creepage,double yaw,double penetration); //A kalker线性模型
	VectorN SHE_Creepforce(VectorN &cur,VectorN &creepage,double yaw,double penetration);//B SHE修正模型
	VectorN Polach_Creepforce(VectorN &cur,VectorN &creepage,double yaw,double penetration); //C Polach模型
	//20210406
	VectorN None_Creepforce(VectorN &cur, VectorN &creepage, double yaw, double penetration); // Zero creep forces
	//法向力计算
	double LeftNormalForce(VectorN &cur,double penetration,double yaw);
	double RightNormalForce(VectorN &cur,double penetration,double yaw);
	//法向阻尼力计算
	double LeftNormalDampForce(double penetrationvelocity,double penetration);
	double RightNormalDampForce(double penetrationvelocity,double penetration);
	void Empty(void);

	virtual ~WRcontForce();
private:
	double Er,Ew,Miur,Miuw;
	double friction;
	double G;
	double Miu;
	Matrix coff;
	Matrix beta;
	Matrix kalker1_c11, kalker1_c22, kalker1_c23, kalker1_c33;
	CSpline betacoff;
	CSpline kkcoff1_c11, kkcoff1_c22, kkcoff1_c23, kkcoff1_c33;
};


