#pragma once

#ifndef _INERTIAELEM_H_
#define _INERTIAELEM_H_
#include "InertiaElem.h"
#endif

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

#ifndef _EULERANGLE_H_
#define _EULERANGLE_H_
#include "EulerAngle.h"
#endif

#ifndef _EULERPARA_H_
#define _EULERPARA_H_
#include "EulerPara.h"
#endif

#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_
#include "CubicSpline.h"
#endif

#ifndef _CSPLINE_H_
#define _CSPLINE_H_
#include "Spline.h"
#endif

#ifndef _TRACK_H_
#define _TRACK_H_
#include "Track.h"
#endif

#ifndef _WHEELGEOMETRY_H_
#define _WHEELGEOMETRY_H_
#include "WheelGeometry.h"
#endif

#ifndef _AECONTGEO_H_
#define _AECONTGEO_H_
#include "AEContGeo.h"
#endif

#ifndef _WRCONTFORCE_H_
#define _WRCONTFORCE_H_
#include "WRcontForce.h"
#endif

#ifndef _MARKER_H_
#define _MARKER_H_
#include "Marker.h"
#endif

#ifndef _TPFELEM_H_
#define _TPFELEM_H_
#include "TPFElem.h"
#endif

#ifndef _OBSERVER_H_
#define _OBSERVER_H_
#include "Observer.h"
#endif

#ifndef _OBSERVERHDR_H_
#define _OBSERVERHDR_H_
#include "ObserverHdr.h"
#endif

#include <fcntl.h>
#include<stdio.h>
#include<io.h>

class RWheelSet :
	public InertiaElem
{
public:

	//构造函数
	RWheelSet(void);
	//20210511 WR ContGeo Method Flag Added
	RWheelSet(int number, CString name, double mass, Vector3x &I, Vector3x &r0, Vector3x &rd0, Vector3x &rdd0, EulerAngle &theta0, EulerAngle &thetad0,
		EulerAngle &thetadd0, double lradius, double rradius, double l, Matrix &lp, Matrix &rp, int flag, int wrcontgeoflag, Track &tk, int sindex1, int sindex2, int sindex3, WRcontForce &wf);
	RWheelSet(int number, CString name, double mass, Vector3x &I, Vector3x &r0, Vector3x &rd0, Vector3x &rdd0, EulerAngle &theta0, EulerAngle &thetad0,
		EulerAngle &thetadd0, double lradius, double rradius, double l, Matrix &lp, Matrix &rp, int flag, int wrcontgeoflag, Track &tk, int sindex1, int sindex2, int sindex3, int csm1, int csm2, int csm3, WRcontForce &wf);

	RWheelSet(const RWheelSet &ws);
	RWheelSet& operator=(const RWheelSet &ws);
	RWheelSet* clone(void) const { return new RWheelSet(*this);}

	//const return functions-----------------
	//positions------------------------------
	Vector3x getR0(void) const { return R0; };
	Vector3x getRD0(void) const { return Rd0; };
	Vector3x getRDD0(void) const { return Rdd0; };
	EulerAngle getTHETA0(void) const { return THETA0; };
	EulerAngle getTHETAD0(void) const { return THETAd0; };
	EulerAngle getTHETADD0(void) const { return THETAdd0; };
	Vector3x getR(void) const { return R; };
	Vector3x getRD(void) const { return Rd; };
	Vector3x getRDD(void) const { return Rdd; };
	EulerAngle getTHETA(void) const { return THETA; };
	EulerAngle getTHETAD(void) const { return THETAd; };
	EulerAngle getTHETADD(void) const { return THETAdd; };
	EulerAngle getEulerAngle(void);
	Vector3x getMarkerLocalVec(int num);
	// wheel geometry----------------------------
	double getLeftRadius(void) const {return l_radius;};
	double getRightRadius(void) const {return r_radius;};
	double getLd(void) const{ return ld;};
	WheelGeometry getLeftWheelGeometry(void) const { return LeftWheelGeometry; };
	WheelGeometry getRightWheelGeometry(void) const { return RightWheelGeometry; };
	// track model-------------------------------
	Track getTrack(void) const {return TK;};
	// wheel rail contact forces model
	int getCreepModel(void) const {return FLAG;};
	// 20210511 contact geometry method flag
	int getContGeoFlag(void) const { return ContGeoFlag; }
	VectorN getLeftConPara0(void) const {return LeftConPara0;};
	VectorN getRightConPara0(void) const {return RightConPara0;};
	VectorN getcurrentLeftContPara(void) const {return currentLeftContPara;};
	VectorN getcurrentRightContPara(void) const {return currentRightContPara;};
	AEContGeo getLeftWheelContact(void) const {return LeftWheelContact;};
	AEContGeo getRightWheelContact(void) const {return RightWheelContact;};
	WRcontForce getWheelForce(void) const {return WheelForce;};
	// 3 pnts forces-----------------------------
	std::vector<TPFElem> getTPF(void) const {return force;};
	// observers---------------------------------
	std::vector<ObserverHdr> getObserverHdr(void) const { return obsvec;}
	// system DAEs related-----------------------
	VectorN getCurrentForceVector(void) const { return currentForceVector;}// MBD forces
	std::vector<VectorN> getleftcurrentcontactforce(void) const{ return leftcurrentcontactforce; }// wheel rail contact forces
	std::vector<VectorN> getrightcurrentcontactforce(void) const { return rightcurrentcontactforce; }// wheel rail contact forces
	// Equivalent conicity computation-----------
	int getConPntFLAG(void) const { return ConPntFLAG; }
	Matrix getRollingRadiusDiff(void) const { return RollingRadiusDiff; }
	Matrix getConsEquContactPnts(void) const { return ConsEquContactPnts; }

	std::vector<CubicSpline> getleftWheelRailContactPntCSVec(void) const { return leftWheelRailContactPntCSVec; }
	std::vector<CubicSpline> getrightWheelRailContactPntCSVec(void) const { return rightWheelRailContactPntCSVec; }
	//
	Matrix getLinearSearchContactPnts(void) const { return LinearSearchContactPnts; }
	Matrix getLinearSearchRadiusDiff(void) const { return LinearSearchRadiusDiff; }
	std::vector<VectorN> getleftLinearSearchContactPnts(void) const { return leftLinearSearchContactPnts; }
	std::vector<VectorN> getrightLinearSearchContactPnts(void) const { return rightLinearSearchContactPnts; }
	//----------------------------------------------
	VectorN getq0(void) {VectorN q0(6); q0[0]=R0[0];q0[1]=R0[1];q0[2]=R0[2];q0[3]=THETA0[0];q0[4]=THETA0[1];q0[5]=THETA0[2]; return q0;}
	VectorN getqd0(void) {VectorN qd0(6); qd0[0]=Rd0[0];qd0[1]=Rd0[1];qd0[2]=Rd0[2];qd0[3]=THETAd0[0];qd0[4]=THETAd0[1];qd0[5]=THETAd0[2]; return qd0;}
	VectorN getqdd0(void) {VectorN qdd0(6); qdd0[0]=Rdd0[0];qdd0[1]=Rdd0[1];qdd0[2]=Rdd0[2];qdd0[3]=THETAdd0[0];qdd0[4]=THETAdd0[1];qdd0[5]=THETAdd0[2]; return qdd0;}
	// current dynamic parameters-------------------
	Vector3x getMarker_Pos(int num, Vector3x &r, EulerAngle &theta);
	Vector3x getPoint_Pos(Vector3x &loc, Vector3x &r, EulerAngle &theta);
	Vector3x getMarker_Vel(int num, Vector3x &rd, EulerAngle &theta, EulerAngle &thetad);
	Vector3x getPoint_Vel(Vector3x &loc, Vector3x &rd, EulerAngle &theta, EulerAngle &thetad);
	VectorN getCurrentKinematicPara(void);
	VectorN getCurrentForcePara(void);
	Vector3x AssGravityForce(void);
	Matrix getGConjMatDer(EulerAngle &theta, EulerAngle &thetad);
	// virtual function-----------------------
	Matrix AssMassMat(EulerAngle &theta);//20130706 modified
	VectorN AssQuadForceVec(EulerAngle &theta,EulerAngle &thetad);
	VectorN AssExternalForce(EulerAngle &theta,double splineindex);//（gravity,TPF）
	VectorN AssExternalTorque(EulerAngle &theta,double splineindex);//（TPF）
	VectorN AssembleForceVector(Vector3x &r,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad,double splineindex);
	// update current states for each integration pnts
	void UpdateLeftWRContPara(VectorN &lcontpara);//update contact points
	void UpdateRightWRContPara(VectorN &rcontpara);//update contact points
	// update status for each integration step
	void UpdateKimematicsPara(VectorN &q, VectorN &qd);//update pos vel
	//20210506
	void UpdateEquivalentPara(VectorN &q, VectorN &qd); // contact para not updated
	void UpdateKinematicsParaAcc(VectorN &q, VectorN &qd, VectorN &qdd);// update acc
	void UpdateForcePara(VectorN &vecn);// update forces parameters
	void UpdateLeftCurrentContactForce(std::vector<VectorN>& lstdvecn) { leftcurrentcontactforce.clear(); leftcurrentcontactforce = lstdvecn; }
	void UpdateRightCurrentContactForce(std::vector<VectorN>& rstdvecn) { rightcurrentcontactforce.clear(); rightcurrentcontactforce = rstdvecn; }
	//20210412
	void UpdateInitialKinematicsParaPos(VectorN &q);
	void UpdateInitialKinematicsParaPosVel(VectorN &q, VectorN &qd);
	void UpdateInitialKinematicsParaPosVelAcc(VectorN &q, VectorN &qd, VectorN &qdd);
	//20210414
	void SetRunSpeed(double V);
	//----------------------------------------
	// framework related
	void addForceVec(TPFElem &tpf);
	void CreateObserverHdr(Observer& obs);
	// tragger functin------------------------
	void TriggeIntPntResPushBackrProcess(void);
	void TriggerSave(void);
	//EN15302 equivalent conicity computations
	Matrix EquivalentConicity(int flag,double ecstartpnt,double ecendpnt,double ECsize);
	
public:
	void InitConsolWnd(void);// open console
	void InitMaterial(double ew, double er, double miuw, double miur,double fr);
	void InitWheelGeometry(Matrix &l, Matrix &r, int somthindex1, int somthindex2, int somthindex3);//initialize wheel profiles splines
	//20201022
	void InitWheelGeometry(Matrix &l, Matrix &r, int somthindex1, int somthindex2, int somthindex3, int csm1, int csm2, int csm3);//initialize wheel profiles splines
	void InitMarker(void);// add markers
	void InitContGeo(void);// computing initial contact points
	//
	// initialize contact pnts based on linear search
	void InitLinearSearchContactGeometry(double startpnt, double endpnt, double stepsize, Track &tk);
	// initialize contact pnts based on algebra equations
	void InitWheelRailContactGeometry(double startpnt, double endpnt, double stepsize, Track &tk);
public:
	std::vector<VectorN> AssContPntsByLookupTable(Track &tk, Vector3x &ws_r, EulerAngle &ws_theta);
	
public:
	~RWheelSet(void);
private:
	Vector3x R0,Rd0,Rdd0;
	EulerAngle THETA0,THETAd0,THETAdd0;
	Vector3x bR0, bRd0, bRdd0;
	EulerAngle bTHETA0, bTHETAd0, bTHETAdd0;
	Vector3x R;
	EulerAngle THETA;
	Vector3x Rd;
	EulerAngle THETAd;
	Vector3x Rdd;
	EulerAngle THETAdd;
	double l_radius,r_radius;
	double ld;	
	//三点力
	std::vector<TPFElem> force;
	std::vector<ObserverHdr> obsvec;
	//Track
	Track TK;
	//creep model0 kalker linear;1 SHE model;2 polach model
	int FLAG;
	// contact geometry computation method: 0 real-time ; 1 lookup table
	int ContGeoFlag;
	// wheel rail contact force
	WRcontForce WheelForce;
	//wheel geometry
	WheelGeometry LeftWheelGeometry;
	WheelGeometry RightWheelGeometry;
	//contact geometry
	AEContGeo LeftWheelContact;
	AEContGeo RightWheelContact;
	
	//initial contact points
	VectorN LeftConPara0;
	VectorN RightConPara0;	

	//current parameters of the wheelset
	VectorN currentLeftContPara;
	VectorN currentRightContPara;
	//current contact forces parameters
	std::vector<VectorN> leftcurrentcontactforce;
	std::vector<VectorN> rightcurrentcontactforce;	
	VectorN currentForceVector;

	//flag for equivalent conicity chosen
	int ConPntFLAG;
	// constraints equations interaction method
	Matrix RollingRadiusDiff;
	Matrix ConsEquContactPnts;
	//20210511
	std::vector<CubicSpline> leftWheelRailContactPntCSVec;
	std::vector<CubicSpline> rightWheelRailContactPntCSVec;

	// linear search method according to EN15302
	Matrix LinearSearchRadiusDiff;
	Matrix LinearSearchContactPnts;
	std::vector<VectorN> leftLinearSearchContactPnts;
	std::vector<VectorN> rightLinearSearchContactPnts;
};
