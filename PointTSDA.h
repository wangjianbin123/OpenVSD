#pragma once
#include "ForceElem.h"
class PointTSDA :
	public ForceElem
{
public:
	PointTSDA();
	PointTSDA(int n, CString name, InertiaElem* f, int fmarker, InertiaElem* t, int tmarker, Vector3x &stiff, Vector3x &damper);
	PointTSDA(const PointTSDA& sr);
	PointTSDA& operator=(const PointTSDA& sr);

	//
	PointTSDA* clone(void) const { return new PointTSDA(*this); }
	std::vector<VectorN> AssForceVec(Vector3x &fr, Vector3x &frd, EulerAngle &ftheta, EulerAngle &fthetad, Vector3x &tr, Vector3x &trd, EulerAngle &ttheta, EulerAngle &tthetad, double splineindex);
public:
	virtual ~PointTSDA();
	Vector3x getStiff(void) const { return k; };
	Vector3x getDamp(void) const { return c; };

private:
	Vector3x k;
	Vector3x c;
};

