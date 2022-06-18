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

#ifndef _OBSERVER_H_
#define _OBSERVER_H_
#include "Observer.h"
#endif

#ifndef _OBSERVERHDR_H_
#define _OBSERVERHDR_H_
#include "ObserverHdr.h"
#endif

#ifndef _RESOBSERVER_H_
#define _RESOBSERVER_H_
#include "ResObserver.h"
#endif


#include <vector>

// InertiaElem 命令目标

class InertiaElem : public CObject
{
public:
	InertiaElem();
	InertiaElem(int number,CString name,BOOL flexible,BOOL NotGround,double mass,Vector3x &I);
	InertiaElem(const InertiaElem &iner);
	InertiaElem& operator=(const InertiaElem &iner);

	//文件流操作
	//void Serialize(CArchive &ar);
	//DECLARE_SERIAL(InertiaElem)

	//clone fun to support stl
	virtual InertiaElem* clone(void) const { return new InertiaElem(*this); }

	//const functions
	int getSN(void) const { return SN; }
	CString getName(void) const { return m_name; }
	BOOL getFlexible(void) const { return FLEX; }
	BOOL getGround(void) const { return NG; };
	double getMass(void) const { return m; }
	Vector3x getPrincipleI(void) const { return principle_I; }
	Matrix getPrincipleMat(void) const { return principle_mat; }
	Vector3x getGravity(void) const { return g; }
	std::vector<Marker> getMarkerVector(void) const { return wsmarker; }
	
	//MARKER操作
	Marker getMarker(int num);
	void addMarker(Marker &mk);
	// virtual functions
	virtual Vector3x getMarker_Pos(int num,Vector3x &r,EulerAngle &theta);
	virtual Vector3x getMarker_Vel(int num,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad);
	//转动坐标
	virtual EulerAngle getEulerAngle(void);
	//local vector of the marker
	//20210413 add const decorations
	virtual Vector3x getMarkerLocalVec(int num);
	virtual Vector3x getR0(void) const {Vector3x vec;return vec;};
	virtual Vector3x getRD0(void) const { Vector3x vec; return vec; };
	virtual Vector3x getRDD0(void) const { Vector3x vec; return vec; };
	virtual EulerAngle getTHETA0(void) const { EulerAngle ea; return ea; };
	virtual EulerAngle getTHETAD0(void) const { EulerAngle ea; return ea; };
	virtual EulerAngle getTHETADD0(void) const { EulerAngle ea; return ea; };

	virtual VectorN getq0(void) {VectorN vecn;return vecn;}
	virtual VectorN getqd0(void) {VectorN vecnd;return vecnd;}
	virtual VectorN getqdd0(void) {VectorN vecndd;return vecndd;}
	//system functions
	virtual Matrix AssMassMat(EulerAngle &theta);
	virtual VectorN AssQuadForceVec(EulerAngle &theta,EulerAngle &thetad);
	virtual VectorN AssExternalForce(EulerAngle &theta,double splineindex);
	virtual VectorN AssExternalTorque(EulerAngle &theta,double splineindex);
	virtual VectorN AssembleForceVector(Vector3x &r,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad,double splineindex);

	//system integration broadcast functions
	virtual void CreateObserverHdr(Observer &obs) { }
	virtual void TriggeIntPntResPushBackrProcess(void) { }
	virtual void TriggerSave(void) { }

	//20140106modified
	virtual VectorN getCurrentKinematicPara(void);
	//20140804modified
	virtual VectorN getCurrentForcePara(void);
	//20140107modified
	virtual void UpdateKimematicsPara(VectorN &q,VectorN &qd) { }
	//20210506 add new UpdateEquivalentPara function for RWheelset contact points real-time computations
	virtual void UpdateEquivalentPara(VectorN &q, VectorN &qd) { }
	virtual void UpdateKinematicsParaAcc(VectorN &q, VectorN &qd, VectorN &qdd) { }
	virtual void UpdateForcePara(VectorN &vecn) { }
	//20210412
	virtual void UpdateInitialKinematicsParaPos(VectorN &q) { }
	virtual void UpdateInitialKinematicsParaPosVel(VectorN &q, VectorN &qd) { }
	virtual void UpdateInitialKinematicsParaPosVelAcc(VectorN &q, VectorN &qd, VectorN &qdd) { }
	//20210414
	virtual void SetRunSpeed(double V) { }
	
public:
	void SetName(CString name);
	virtual ~InertiaElem();
protected:
	int SN;
	CString m_name;
	BOOL FLEX;
	BOOL NG;
	double m;
	Vector3x principle_I;
	Matrix principle_mat;
	Vector3x g;
	std::vector<Marker> wsmarker;
};


