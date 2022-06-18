#pragma once

// Track 命令目标
//Version 1.1.1 by Jianbin Wang
//modified at 20090506
//Rail Track
//(1) Basic track parameters should be defined including serial number,track name, geometry description matrix,
//    left and right rail surface description matrix,track origin, track orientation, gauge width, track roll angle,
//    data intervals.
//(2) After the required data have been defined, initial function must be called to generate related geometry data.
//    void InitTrackDiffGeo(void);
//    void InitCurvature(void);
//version 1.2.1
//updated as cubicspline class changed

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

#include <vector>

class Track : public CObject
{
public:
	//构造函数
	Track();
	Track(Matrix &mx,Matrix &pmxl,Matrix &pmxr);//Matrix1：轨道中心线几何矩阵 Matrix2：左轨踏面 Matrix3：右轨踏面
	Track(Matrix &mxdescrip,Matrix &leftmx,Matrix &rightmx,Vector3x ori,EulerAngle ang,double gauge,double cant,double interval,int sm,int dersm1st,int dersm2nd);
	//20201018
	Track(Matrix &mxdescrip, Matrix &leftmx, Matrix &rightmx, Vector3x ori, EulerAngle ang, double gauge, double cant, double interval, int sm, int dersm1st, int dersm2nd, int csm1, int csm2, int csm3);
	Track(const Track &tk);
	Track& operator=(const Track &tk);
	//重设置函数
	void setParameters(int num, CString name, Matrix description, Matrix lp, Matrix rp, Vector3x ori, EulerAngle ang, double gauge, double c, double interval);
	//文件流
	//void Serialize(CArchive &ar);
	//DECLARE_SERIAL(Track)
	
	//const return functions
	Matrix getDiscription(void) const { return m_nDiscription;}
	Matrix getLProfile(void) const { return LProfile; }
	Matrix getRProfile(void) const { return RProfile; }
	Vector3x getOrigin(void) const { return m_nOrigin; }
	EulerAngle getOrientation(void) const { return m_nOrientation; }
	double getWidth(void) const { return gauge_width; }
	double getcant(void) const { return cant; }
	double getdpinterval(void) const { return dp_interval; }
	Matrix getConPntMat(void) const { return m_nConPnt;}
	Matrix getCPxyzMat(void) const { return m_nCPxyz;}
	Matrix getArcLenDiffMatLeft(void) const { return m_nArcLenDiffLeft;}
	Matrix getArcLenDiffMatRight(void) const { return m_nArcLenDiffRight;}
	Matrix getLeftRPDATA(void) const  { return LeftRP_Data;}
	Matrix getRightRPDADA(void) const { return RightRP_Data;}
	std::vector<CubicSpline> getLeftRailCS(void) const { return LeftRailCS;}
	std::vector<CubicSpline> getRightRailCS(void) const { return RightRailCS;}
	std::vector<CubicSpline> getLRP_D1st(void) const { return LRProfDeri_1st;}
	std::vector<CubicSpline> getLRP_D2nd(void) const { return LRProfDeri_2nd;}
	std::vector<CubicSpline> getLRP_D3rd(void) const { return LRProfDeri_3rd; }
	std::vector<CubicSpline> getRRP_D1st(void) const { return RRProfDeri_1st;}
	std::vector<CubicSpline> getRRP_D2nd(void) const { return RRProfDeri_2nd;}
	std::vector<CubicSpline> getRRP_D3rd(void) const { return RRProfDeri_3rd; }
 	CubicSpline getLProfileCS(void) const { return LProfileCS; }
	CubicSpline getRProfileCS(void) const { return RProfileCS; }
	CubicSpline getLProfile_D1st(void) const { return LProfile_D1st; }
	CubicSpline getLProfile_D2nd(void) const { return LProfile_D2nd; }
	CubicSpline getRProfile_D1st(void) const { return RProfile_D1st; }
	CubicSpline getRProfile_D2nd(void) const { return RProfile_D2nd; }
	Matrix getLPcurMat(void) const {return LPcurMat;}
	Matrix getRPcurMat(void) const {return RPcurMat;}
	std::vector<CubicSpline> getlrp(void) const { return lrp; }
	std::vector<CubicSpline> getrrp(void) const { return rrp; }
	Matrix getlpdeltas(void) const {return lpdeltas;}
	Matrix getrpdeltas(void) const {return rpdeltas;}
	Matrix getlps(void) const {return lps;}
	Matrix getrps(void) const {return rps;}
	std::vector<CubicSpline> getlcur(void) const { return lcur; }
	std::vector<CubicSpline> getrcur(void) const { return rcur; }
	std::vector<CubicSpline> getxindic(void) const { return xindic; }
    
	//析构函数
	virtual ~Track();

	//文件流
	//void Serialize(CArchive &ar);
	//DECLARE_SERIAL(Track)

	//运算符重载

	//初始化函数,被构造初始化函数调用
	void setConPntMat(void);
	void setCPxyzMat(void);
	void setArcLenDiffMatLeft(void);
	void setArcLenDiffMatRight(void);
	void setLeftRPData(void);
	void setRightRPData(void);
	void setLeftCSs(void);
	void setRightCSs(void);
	void setLeftProfile_Derivate(void);
	void setRightProfile_Derivate(void);
	void setProfileCS(void);
	void setProfileCS(int sm0,int sm1,int sm2);
	//曲率相关矩阵及样条曲线设置
	void setPcurMat(void);
	void setSPcur(void);
	void setcurDeltaS(void);
	void setcurSp(void);
	//20201017
	void setcurSp(int csm1, int csm2, int csm3);

	//构造初始化函数
	void InitTrackDiffGeo(void);
	void InitTrackDiffGeo(int sm0,int sm1,int sm2);
	void InitCurvature(void);
	//20201018
	void InitCurvature(int csm1, int csm2, int csm3);

	//辅助函数
	int LocSegmentS(double S); //定位映射弧长S所在的段数
	int LocSegments(double s); //定位实际弧长s所在的段数
	double TranS2s(double S); // 映射弧长S转换为对应的实际弧长s
	double Trans2S(double s); // 实际弧长s转换为对应的映射弧长S
	double getCH(double S);  //返回映射弧长S对应的水平曲率

	//轨道函数
	double getRPx(double S); //轨道中心线X坐标
	double getRPy(double S); //轨道中心线Y坐标
	double getRPz(double S); //轨道中心线Z坐标
	double getRPYaw(double S); //轨道中心线摇头角 PSI
	double getRPPitch(double S); //轨道中心线点头角 THETA
	double getRPRoll(double S); // 轨道中心线侧滚角 PHI
	Matrix getTrackRotMat(double s); // 返回实际弧长下s的轨道中心线转动矩阵
	Vector3x getTangentVec(double s); // 返回实际弧长s下轨道中心线切线矢量

	//左轨函数
	Vector3x LeftRailRP(double s); //返回左轨空间矢量 s 中心线实际弧长
	Vector3x LeftRailAngle(double s); //返回左轨转动角度 s 中心线实际弧长
	Matrix LeftRailRotMat(double s); //返回左轨参考系转动矩阵 s 中心线实际弧长
	Vector3x LeftRailTanVec(double s); //返回左轨切线矢量 s 中心线实际弧长
	double getDiffArcLenLeft(double s); //返回左轨弧长差 s 中心线实际弧长
	
	Matrix LeftRotationMat(double sl1);
	Vector3x LeftProfile_Local(double sl2);
	Vector3x LeftRP_global(double sl1);
	
	Vector3x LeftPositon(double sl1,double sl2);//R
	Vector3x LeftRail_T1(double sl1,double sl2);//dR/ds1 global tangent vector
	Vector3x LeftRail_T2(double sl1,double sl2);//dR/ds2 global tangent vector
	Vector3x LeftRail_N(double sl1,double sl2); //dR/ds1 x dR/ds2

	Vector3x LeftRail_T1DS1(double sl1,double sl2);
	Vector3x LeftRail_T1DS2(double sl1,double sl2);
	//20191025 added
	Vector3x LeftRail_T1DS1DS1(double s11, double s12);
	Vector3x LeftRail_T1DS1DS2(double s11, double s12);
	Vector3x LeftRail_T1DS2DS2(double sl1, double sl2);
	Vector3x LeftRail_T2DS1(double sl1,double sl2);
	Vector3x LeftRail_T2DS2(double sl1,double sl2);
	//20191025 added
	Vector3x LeftRail_T2DS1DS1(double sl1, double sl2);
	Vector3x LeftRail_T2DS1DS2(double s11, double s12);
	Vector3x LeftRail_T2DS2DS2(double s11, double s12);
	Vector3x LeftRail_NDS1(double sl1,double sl2);
	Vector3x LeftRail_NDS2(double sl1,double sl2);
	//20191024 Hesian Matrix 
	Vector3x LeftRail_NDS1DS2(double sl1, double sl2);
	Vector3x LeftRail_NDS1DS1(double sl1, double sl2);
	Vector3x LeftRail_NDS2DS2(double sl1, double sl2);
	double LeftRail_curvature(double sl2);
	//20201018
	Matrix getLeftCurvatureMat(void);
	
	//右轨函数
	Vector3x RightRailRP(double s); //返回右轨空间矢量 s 中心线实际弧长
	Vector3x RightRailAngle(double s); //返回右轨转动角度 s  中心线实际弧长
	Matrix RightRailRotMat(double s); //返回右轨参考下转动矩阵 s 中心线实际弧长
	Vector3x RightRailTanVec(double s); //返回右轨切线矢量 s 中心线实际弧长
	double getDiffArcLenRight(double s); //返回右轨弧长差 s 中心线实际弧长

	Matrix RightRotationMat(double sr1);
	Vector3x RightProfile_Local(double sr2);
	Vector3x RightRP_global(double sr1);

	Vector3x RightPosition(double sr1,double sr2);
	Vector3x RightRail_T1(double sr1, double sr2);
	Vector3x RightRail_T2(double sr1,double sr2);
	Vector3x RightRail_N(double sr1,double sr2);

	Vector3x RightRail_T1DS1(double sr1,double sr2);
	Vector3x RightRail_T1DS2(double sr1,double sr2);
	//20191025 added
	Vector3x RightRail_T1DS1DS1(double sr1, double sr2);
	Vector3x RightRail_T1DS1DS2(double sr1, double sr2);
	Vector3x RightRail_T1DS2DS2(double sr1, double sr2);
	Vector3x RightRail_T2DS1(double sr1,double sr2);
	Vector3x RightRail_T2DS2(double sr1,double sr2);
	//20191025 added
	Vector3x RightRail_T2DS1DS2(double sr1, double sr2);
	Vector3x RightRail_T2DS2DS2(double sr1, double sr2);
	Vector3x RightRail_T2DS1DS1(double sr1, double sr2);

	Vector3x RightRail_NDS1(double sr1,double sr2);
	Vector3x RightRail_NDS2(double sr1,double sr2);
	//20191024 Hesian Matrix 
	Vector3x RightRail_NDS1DS2(double s11, double s12);
	Vector3x RightRail_NDS1DS1(double s11, double s12);
	Vector3x RightRail_NDS2DS2(double s11, double s12);

	//20191028 Ad Add function added
	Matrix getLeftAd(double sl1, double sl2);
	Matrix getLeftAdd(double sl1, double sl2);
	Matrix getRightAd(double sr1, double sr2);
	Matrix getRightAdd(double sr1, double sr2);

	double RightRail_curvature(double sr2);
	//20201018
	Matrix getRightCurvatureMat(void);
	//轨道曲率
	//轨道总体参数
	//轨道激励函数 垂向 profile 横向 alignment
	//rail geometry plot
private:
	//构造参数，轨道类的主要参数 用于serialize函数读取和存储
	//20100323 private par m_num and m_name added.
	int m_num;
	CString m_name;
	Matrix m_nDiscription;
	Matrix LProfile;
	Matrix RProfile;	
	Vector3x m_nOrigin;//轨道原点
	EulerAngle m_nOrientation;
	double gauge_width;//标准轨距
	double cant; //轨底坡侧滚角
	double dp_interval; //数据点间距
	
	//中间内存矩阵，保存各类中间信息
	//轨道总体几何
	Matrix m_nConPnt;
	Matrix m_nCPxyz;
	Matrix m_nArcLenDiffLeft;
	Matrix m_nArcLenDiffRight;
	Matrix LeftRP_Data;
	Matrix RightRP_Data;
	std::vector<CubicSpline> LeftRailCS;//左轨型面坐标系x0 y1 z2 psi3 theta4 phi5
	std::vector<CubicSpline> RightRailCS;//右轨型面坐标系x0 y1 z2 psi3 theta4 phi5
	std::vector<CubicSpline> LRProfDeri_1st;
	std::vector<CubicSpline> LRProfDeri_2nd;
	//20191025 3rd of the derivarive added
	std::vector<CubicSpline> LRProfDeri_3rd;
	std::vector<CubicSpline> RRProfDeri_1st;
	std::vector<CubicSpline> RRProfDeri_2nd;
	//20191025 3rd of the derivarive added
	std::vector<CubicSpline> RRProfDeri_3rd;

	//轨道型面几何
	CubicSpline LProfileCS; //左轨型面数据
	CubicSpline RProfileCS; //右轨型面数据
	CubicSpline LProfile_D1st; //左轨型面曲线1阶导数
	CubicSpline LProfile_D2nd; //左轨型面曲线2阶导数
	CubicSpline RProfile_D1st; //右轨型面曲线1阶导数
	CubicSpline RProfile_D2nd; //右轨型面曲线2阶导数

	//轨面曲率
	Matrix LPcurMat;
	Matrix RPcurMat;
	std::vector<CubicSpline> lrp;//refer to t 0 x ,1 y ,2 xd ,3 yd
	std::vector<CubicSpline> rrp;//refer to t 0 x ,1 y ,2 xd ,3 yd
	Matrix lpdeltas,lps;
	Matrix rpdeltas,rps;
	std::vector<CubicSpline> lcur;//refer to s 0 x 1 y 2 dx 3 dy 4 dx2 5 dy2
	std::vector<CubicSpline> rcur;//refer to s 0 x 1 y 2 dx 3 dy 4 dx2 5 dy2
	std::vector<CubicSpline> xindic; // 0 is left and 1 is right
};
