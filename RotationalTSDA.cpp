#include "pch.h"
#include "RotationalTSDA.h"


RotationalTSDA::RotationalTSDA()
{

}
RotationalTSDA::RotationalTSDA(int n, CString name, InertiaElem* f, int fmarker, InertiaElem* t, int tmarker, double stiff, double damper, double length)
{
	sn = n;
	m_name = name;
	from = f;
	marker_from = fmarker;
	to = t;
	marker_to = tmarker;
	k = stiff;
	c = damper;
	l0 = length;
}
RotationalTSDA::RotationalTSDA(const RotationalTSDA& sr)
{
	sn = sr.getSN();
	m_name = sr.getName();
	from = sr.getfrom();
	marker_from = sr.getfrommarker();
	to = sr.getto();
	marker_to = sr.gettomarker();
	k = sr.getStiff();
	c = sr.getDamp();
	l0 = sr.getL0();
}

RotationalTSDA& RotationalTSDA::operator =(const RotationalTSDA &sr)
{
	if (this == &sr)
	{
		return *this;
	}
	else
	{
		sn = sr.getSN();
		m_name = sr.getName();
		from = sr.getfrom();
		marker_from = sr.getfrommarker();
		to = sr.getto();
		marker_to = sr.gettomarker();
		k = sr.getStiff();
		c = sr.getDamp();
		l0 = sr.getL0();
		return *this;
	}
}

std::vector<VectorN> RotationalTSDA::AssForceVec(Vector3x &fr, Vector3x &frd, EulerAngle &ftheta, EulerAngle &fthetad, Vector3x &tr, Vector3x &trd, EulerAngle &ttheta, EulerAngle &tthetad, double splineindex)
{
	////computation of delta relatively distance
	//Vector3x delta_pos = (from->getMarker_Pos(marker_from, fr, ftheta) - to->getMarker_Pos(marker_to, tr, ttheta));
	//double delta_l = delta_pos.getNorm() - l0;
	////computation of delta relatively velocity
	//Vector3x delta_velo = (from->getMarker_Vel(marker_from, frd, ftheta, fthetad) - to->getMarker_Vel(marker_to, trd, ttheta, tthetad));
	//Vector3x direction(delta_pos);
	//direction.toUnit();
	//double delta_v = delta_velo.dotProduct(direction);
	////computation of force
	//double force = k*delta_l + c*delta_v;
	////return force vectors
	
	////forces
	//Vector3x fi = (-1.0)*force*direction;
	//Vector3x fj = force*direction;
	/////torques
	//Vector3x uip = (ftheta.getRotMat())*(from->getMarkerLocalVec(marker_from));
	//Vector3x ujp = (ttheta.getRotMat())*(to->getMarkerLocalVec(marker_to));
	//Matrix GiT = ~(ftheta.getGMat());
	//Matrix GjT = ~(ttheta.getGMat());
	//Vector3x ti = GiT*(uip.crossProduct(fi));
	//Vector3x tj = GjT*(ujp.crossProduct(fj));
	//VectorN tmpi(6), tmpj(6);
	//for (int i = 0; i<3; i++)
	//{
	//	tmpi[i] = fi[i];
	//	tmpi[i + 3] = ti[i];
	//	tmpj[i] = fj[i];
	//	tmpj[i + 3] = tj[i];
	//}
	//force_array.push_back(tmpi);
	//force_array.push_back(tmpj);

	std::vector<VectorN> force_array;
	return force_array;
}
RotationalTSDA::~RotationalTSDA()
{
}
