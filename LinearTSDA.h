#pragma once
#include "forceelem.h"
class LinearTSDA :
	public ForceElem
{
public:
	
	LinearTSDA(void);
	LinearTSDA(int n,CString name,InertiaElem* f,int fmarker,InertiaElem* t,int tmarker,double stiff,double damper,double length);
	LinearTSDA(const LinearTSDA& sr);
	LinearTSDA& operator=(const LinearTSDA& sr);
	
	//ÖØÐ´Ðéº¯Êý
	LinearTSDA* clone(void) const { return new LinearTSDA(*this);}
	std::vector<VectorN> AssForceVec(Vector3x &fr,Vector3x &frd,EulerAngle &ftheta,EulerAngle &fthetad,Vector3x &tr,Vector3x &trd,EulerAngle &ttheta,EulerAngle &tthetad,double splineindex);
public:

	~LinearTSDA(void);
	double getStiff(void) const {return k;};
	double getDamp(void) const {return c;};
	double getL0(void) const {return l0;};
private:
	double k;
	double c;
	double l0;
};

