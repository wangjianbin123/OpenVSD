#pragma once

// Track ����Ŀ��
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
	//���캯��
	Track();
	Track(Matrix &mx,Matrix &pmxl,Matrix &pmxr);//Matrix1����������߼��ξ��� Matrix2�����̤�� Matrix3���ҹ�̤��
	Track(Matrix &mxdescrip,Matrix &leftmx,Matrix &rightmx,Vector3x ori,EulerAngle ang,double gauge,double cant,double interval,int sm,int dersm1st,int dersm2nd);
	//20201018
	Track(Matrix &mxdescrip, Matrix &leftmx, Matrix &rightmx, Vector3x ori, EulerAngle ang, double gauge, double cant, double interval, int sm, int dersm1st, int dersm2nd, int csm1, int csm2, int csm3);
	Track(const Track &tk);
	Track& operator=(const Track &tk);
	//�����ú���
	void setParameters(int num, CString name, Matrix description, Matrix lp, Matrix rp, Vector3x ori, EulerAngle ang, double gauge, double c, double interval);
	//�ļ���
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
    
	//��������
	virtual ~Track();

	//�ļ���
	//void Serialize(CArchive &ar);
	//DECLARE_SERIAL(Track)

	//���������

	//��ʼ������,�������ʼ����������
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
	//������ؾ���������������
	void setPcurMat(void);
	void setSPcur(void);
	void setcurDeltaS(void);
	void setcurSp(void);
	//20201017
	void setcurSp(int csm1, int csm2, int csm3);

	//�����ʼ������
	void InitTrackDiffGeo(void);
	void InitTrackDiffGeo(int sm0,int sm1,int sm2);
	void InitCurvature(void);
	//20201018
	void InitCurvature(int csm1, int csm2, int csm3);

	//��������
	int LocSegmentS(double S); //��λӳ�仡��S���ڵĶ���
	int LocSegments(double s); //��λʵ�ʻ���s���ڵĶ���
	double TranS2s(double S); // ӳ�仡��Sת��Ϊ��Ӧ��ʵ�ʻ���s
	double Trans2S(double s); // ʵ�ʻ���sת��Ϊ��Ӧ��ӳ�仡��S
	double getCH(double S);  //����ӳ�仡��S��Ӧ��ˮƽ����

	//�������
	double getRPx(double S); //���������X����
	double getRPy(double S); //���������Y����
	double getRPz(double S); //���������Z����
	double getRPYaw(double S); //���������ҡͷ�� PSI
	double getRPPitch(double S); //��������ߵ�ͷ�� THETA
	double getRPRoll(double S); // ��������߲���� PHI
	Matrix getTrackRotMat(double s); // ����ʵ�ʻ�����s�Ĺ��������ת������
	Vector3x getTangentVec(double s); // ����ʵ�ʻ���s�¹������������ʸ��

	//��캯��
	Vector3x LeftRailRP(double s); //�������ռ�ʸ�� s ������ʵ�ʻ���
	Vector3x LeftRailAngle(double s); //�������ת���Ƕ� s ������ʵ�ʻ���
	Matrix LeftRailRotMat(double s); //�������ο�ϵת������ s ������ʵ�ʻ���
	Vector3x LeftRailTanVec(double s); //�����������ʸ�� s ������ʵ�ʻ���
	double getDiffArcLenLeft(double s); //������컡���� s ������ʵ�ʻ���
	
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
	
	//�ҹ캯��
	Vector3x RightRailRP(double s); //�����ҹ�ռ�ʸ�� s ������ʵ�ʻ���
	Vector3x RightRailAngle(double s); //�����ҹ�ת���Ƕ� s  ������ʵ�ʻ���
	Matrix RightRailRotMat(double s); //�����ҹ�ο���ת������ s ������ʵ�ʻ���
	Vector3x RightRailTanVec(double s); //�����ҹ�����ʸ�� s ������ʵ�ʻ���
	double getDiffArcLenRight(double s); //�����ҹ컡���� s ������ʵ�ʻ���

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
	//�������
	//����������
	//����������� ���� profile ���� alignment
	//rail geometry plot
private:
	//�����������������Ҫ���� ����serialize������ȡ�ʹ洢
	//20100323 private par m_num and m_name added.
	int m_num;
	CString m_name;
	Matrix m_nDiscription;
	Matrix LProfile;
	Matrix RProfile;	
	Vector3x m_nOrigin;//���ԭ��
	EulerAngle m_nOrientation;
	double gauge_width;//��׼���
	double cant; //����²����
	double dp_interval; //���ݵ���
	
	//�м��ڴ���󣬱�������м���Ϣ
	//������弸��
	Matrix m_nConPnt;
	Matrix m_nCPxyz;
	Matrix m_nArcLenDiffLeft;
	Matrix m_nArcLenDiffRight;
	Matrix LeftRP_Data;
	Matrix RightRP_Data;
	std::vector<CubicSpline> LeftRailCS;//�����������ϵx0 y1 z2 psi3 theta4 phi5
	std::vector<CubicSpline> RightRailCS;//�ҹ���������ϵx0 y1 z2 psi3 theta4 phi5
	std::vector<CubicSpline> LRProfDeri_1st;
	std::vector<CubicSpline> LRProfDeri_2nd;
	//20191025 3rd of the derivarive added
	std::vector<CubicSpline> LRProfDeri_3rd;
	std::vector<CubicSpline> RRProfDeri_1st;
	std::vector<CubicSpline> RRProfDeri_2nd;
	//20191025 3rd of the derivarive added
	std::vector<CubicSpline> RRProfDeri_3rd;

	//������漸��
	CubicSpline LProfileCS; //�����������
	CubicSpline RProfileCS; //�ҹ���������
	CubicSpline LProfile_D1st; //�����������1�׵���
	CubicSpline LProfile_D2nd; //�����������2�׵���
	CubicSpline RProfile_D1st; //�ҹ���������1�׵���
	CubicSpline RProfile_D2nd; //�ҹ���������2�׵���

	//��������
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
