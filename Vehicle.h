#pragma once

#ifndef _INERTIAELEMHDR_H_
#define _INERTIAELEMHDR_H_
#include "InertiaElemHdr.h"
#endif

#ifndef _FORCEELEMHDR_H_
#define _FORCEELEMHDR_H_
#include "ForceElemHdr.h"
#endif

#ifndef _DOFCONSTRAINTHDR_H_
#define _DOFCONSTRAINTHDR_H_
#include "DOFConstraintHdr.h"
#endif

#ifndef _RIGIDBODY_H_
#define _RIGIDBODY_H_
#include "RigidBody.h"
#endif

#ifndef _RWHEELSET_H_
#define _RWHEELSET_H_
#include "RWheelSet.h"
#endif

#ifndef _TSDA_H_
#define _TSDA_H_
#include "TSDA.h"
#endif

#ifndef _LINEARTSDA_H_
#define _LINEARTSDA_H_
#include "LinearTSDA.h"
#endif

#ifndef _POINTTSDA_H_
#define _POINTTSDA_H_
#include "PointTSDA.h"
#endif

#include <vector>
#include <fcntl.h>
#include<stdio.h>
#include<io.h>
#include <iostream>
#include <Windows.h>

// Vehicle 命令目标

class Vehicle : public CObject
{
public:
	Vehicle();
	Vehicle(const Vehicle& v);
	Vehicle& operator=( const Vehicle& v);

	//void Serialize(CArchive &ar);
	//DECLARE_SERIAL(Vehicle)

	// constant functions
	CString getName(void) const { return m_name; }
	int getSN(void) const { return m_sn; }
	std::vector<VectorN> getq0vec(void) const { return q0vec; }
	std::vector<VectorN> getqd0vec(void) const { return qd0vec; }
	std::vector<VectorN> getqdd0vec(void) const { return qdd0vec; }
	std::vector<InertiaElemHdr> getInertiaElemHdrArray(void) const { return Inertia_array; }
	std::vector<ForceElemHdr> getForceElemHdrArray(void) const { return ForceElem_array; }
	std::vector<DOFConstraintHdr> getDOFConstraintHdrArray(void) const { return DOFConstraint_array; }
	std::vector<CubicSpline> getCubicSplineArray(void) const { return spl; }

	//创建多体动力学系统单元
	void CreateInertiaElemHdr(InertiaElem &ie);
	void CreateForceElemHdr(ForceElem &fe);
	void CreateDOFConstraintHdr(DOFConstraint &cons);
	void CreateSpline(Matrix &mx);//20130724 added

	//20140807modified to add markers for Ground
	void AddGroundMarker(Marker &mk);

	//组建系统动力学方程
	Matrix AssSysMat(std::vector<VectorN> &qvector);
	Matrix AssSysJacobMat(std::vector<VectorN> &q,std::vector<VectorN> &qd);
	VectorN AssSysForVec(std::vector<VectorN> &q,std::vector<VectorN> &qd,double splineindex);
	VectorN AssSysConstraintEquations(std::vector<VectorN> &q, std::vector<VectorN> &qd);
	//20140107modified
	void UpdateKinematics(std::vector<VectorN> &q,std::vector<VectorN> &qd);
	void UpdateKinematicsAcc(std::vector<VectorN> &q, std::vector<VectorN> &qd, std::vector<VectorN> &qdd);
	// 20210506 for equivalent analysis 
	void UpateEquivalent(std::vector<VectorN> &q, std::vector<VectorN> &qd);
	//20210412 update pos for initial configuration
	void UpdateInitialPos(std::vector<VectorN> &q);
	void UpdateForces(std::vector<VectorN> &stdvecn);
	void TriggerIntPntResPushBack(void);
	void TriggerSysSave(void);

	//辅助函数
	std::vector<VectorN> assembleStdVecn(VectorN &vecn);
	VectorN assembleVecn(std::vector<VectorN> &stdvecn);

	std::vector<VectorN> getq0vec(void) {return q0vec;}
	std::vector<VectorN> getqd0vec(void) {return qd0vec;}
	std::vector<VectorN> getqdd0vec(void) {return qdd0vec;}

	std::vector<VectorN> SortbyInertiaIndex(std::vector<VectorN> &stdvec);

	//static analysis
	void static_analysis(void);
	//Update Static Configuration after equivalent analysis, pos, vel, acc updated
	void UpdateStaticConfiguration(void);
	void UpdateVelocityConfiguration(void);
	// Apply golbal velocity for each body
	void SetRunSpeed(double V);

	// ODEs time integration
	void InitBalancedPosition(double stepsize, double time);
	void RK4integration(double stepsize, double time);
	void RK4(double stepsize, double time);
	void ABAM(double stepsize, double time);
	void Newmark(double stepsize, double time,double beta,double garma);
	//std::vector<std::vector<VectorN>> HHTintegration(double stepsize,double time,double alfa);
	void HHT(double stepsize,double time,double alfa);

public:
	void InitConsolWnd(void);
	virtual ~Vehicle();
public:
	CString m_name;
	int m_sn;
	std::vector<int> InertiaElemIndex;
	std::vector<VectorN> q0vec;
	std::vector<VectorN> qd0vec;
	std::vector<VectorN> qdd0vec;
	std::vector<InertiaElemHdr> Inertia_array;
	std::vector<ForceElemHdr> ForceElem_array;
	std::vector<DOFConstraintHdr> DOFConstraint_array;
	std::vector<CubicSpline> spl;
};


