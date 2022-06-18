#pragma once
#include "ForceElem.h"
class RotationalTSDA :
	public ForceElem
{
public:
	RotationalTSDA();
	RotationalTSDA(int n, CString name, InertiaElem* f, int fmarker, InertiaElem* t, int tmarker, double stiff, double damper, double length);
	RotationalTSDA(const RotationalTSDA& sr);
	RotationalTSDA& operator=(const RotationalTSDA& sr);

	RotationalTSDA* clone(void) const { return new RotationalTSDA(*this); }
	std::vector<VectorN> AssForceVec(Vector3x &fr, Vector3x &frd, EulerAngle &ftheta, EulerAngle &fthetad, Vector3x &tr, Vector3x &trd, EulerAngle &ttheta, EulerAngle &tthetad, double splineindex);
public:
	virtual ~RotationalTSDA();
	double getStiff(void) const { return k; };
	double getDamp(void) const { return c; };
	double getL0(void) const { return l0; };
private:
	double k;
	double c;
	double l0;
};

