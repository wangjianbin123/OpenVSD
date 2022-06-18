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

#ifndef _MARKER_H_
#define _MARKER_H_
#include "Marker.h"
#endif

#ifndef _CUBICSPLINE_H_
#define _CUBICSPLINE_H_
#include "CubicSpline.h"
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


#include <vector>

class RigidBody :
	public InertiaElem
{
public:
	RigidBody(void);

	//20130703 updated
	RigidBody(int number,CString name,BOOL FLEX,BOOL NotGround,double mass,Vector3x &I,Vector3x &r0,Vector3x &rd0,Vector3x &rdd0,
		EulerAngle &theta0,EulerAngle &thetad0,EulerAngle &thetadd0);

	RigidBody(const RigidBody& sr);
	RigidBody& operator=(const RigidBody &sr);
	RigidBody* clone(void) const { return new RigidBody(*this); }

	//------------------------------------------------
	EulerAngle getEulerAngle(void);
	Vector3x getMarkerLocalVec(int num);
	
	Matrix AssMassMat(EulerAngle &theta);
	VectorN AssQuadForceVec(EulerAngle &theta,EulerAngle &thetad);
	VectorN AssExternalForce(EulerAngle &theta,double splineindex);
	VectorN AssExternalTorque(EulerAngle &theta,double splineindex);
	VectorN AssembleForceVector(Vector3x &r,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad,double splineindex);
	//-------------------------------------------------
	
	//Marker
	Vector3x getMarker_Pos(int num,Vector3x &r,EulerAngle &theta);
	Vector3x getMarker_Vel(int num,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad);
	
	//Force
	Vector3x AssGravityForce(void);
	void addForceVec(TPFElem &tpf);

	Matrix getGConjMatDer(EulerAngle &theta,EulerAngle &thetad);

	//const get operations
	Vector3x getR0(void) const {return R0;};
	Vector3x getRD0(void) const {return Rd0;};
	Vector3x getRDD0(void) const {return Rdd0;};
	VectorN getq0(void) {VectorN q0(6); q0[0]=R0[0];q0[1]=R0[1];q0[2]=R0[2];q0[3]=THETA0[0];q0[4]=THETA0[1];q0[5]=THETA0[2]; return q0;}
	VectorN getqd0(void) {VectorN qd0(6); qd0[0]=Rd0[0];qd0[1]=Rd0[1];qd0[2]=Rd0[2];qd0[3]=THETAd0[0];qd0[4]=THETAd0[1];qd0[5]=THETAd0[2]; return qd0;}
	VectorN getqdd0(void) {VectorN qdd0(6); qdd0[0]=Rdd0[0];qdd0[1]=Rdd0[1];qdd0[2]=Rdd0[2];qdd0[3]=THETAdd0[0];qdd0[4]=THETAdd0[1];qdd0[5]=THETAdd0[2]; return qdd0;}
	EulerAngle getTHETA0(void) const {return THETA0;};
	EulerAngle getTHETAD0(void) const {return THETAd0;};
	EulerAngle getTHETADD0(void) const {return THETAdd0;};
	std::vector<TPFElem> getTPF(void) const {return force;};
	std::vector<ObserverHdr> getObserverHdr(void) const { return obsvec;}
	Vector3x getR(void) const {return R;};
	Vector3x getRD(void) const {return Rd;};
	Vector3x getRDD(void) const {return Rdd;};
	EulerAngle getTHETA(void) const {return THETA;};
	EulerAngle getTHETAD(void) const {return THETAd;};
	EulerAngle getTHETADD(void) const {return THETAdd;};
	VectorN getCurrentForceVector(void) const { return currentForceVector; }

	//
	void CreateObserverHdr(Observer& obs);
	void TriggeIntPntResPushBackrProcess(void);
	VectorN getCurrentKinematicPara(void);
	void TriggerSave(void);
	void UpdateKimematicsPara(VectorN &q,VectorN &qd);
	//20210506
	void UpdateEquivalentPara(VectorN &q, VectorN &qd);
	void UpdateKinematicsParaAcc(VectorN &q, VectorN &qd, VectorN &qdd);
	//20210412
	void UpdateInitialKinematicsParaPos(VectorN &q);
	void UpdateInitialKinematicsParaPosVel(VectorN &q, VectorN &qd);
	void UpdateInitialKinematicsParaPosVelAcc(VectorN &q, VectorN &qd, VectorN &qdd);
	//20210414
	void SetRunSpeed(double V);

	//20140804 modified
	VectorN getCurrentForcePara(void);
	void UpdateForcePara(VectorN &vecn);
	//
public:
	~RigidBody(void);
private:
	//Initial states
	Vector3x R0,Rd0,Rdd0;
	EulerAngle THETA0,THETAd0,THETAdd0;
	//balanced states
	Vector3x bR0, bRd0, bRdd0;
	EulerAngle bTHETA0, bTHETAd0, bTHETAdd0;
	//kinematic states
	Vector3x R;
	EulerAngle THETA;
	Vector3x Rd;
	EulerAngle THETAd;
	Vector3x Rdd;
	EulerAngle THETAdd;

	std::vector<TPFElem> force;
	std::vector<ObserverHdr> obsvec;
	//20140804modified
	VectorN currentForceVector;
};
