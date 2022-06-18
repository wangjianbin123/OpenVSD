#include "pch.h"
#include "PointTSDA.h"

PointTSDA::PointTSDA()
{
}

PointTSDA::PointTSDA(int n, CString name, InertiaElem* f, int fmarker, InertiaElem* t, int tmarker, Vector3x &stiff, Vector3x &damper)
{
	sn = n;
	m_name = name;
	from = f;
	marker_from = fmarker;
	to = t;
	marker_to = tmarker;
	k = stiff;
	c = damper;
}

PointTSDA::PointTSDA(const PointTSDA& sr)
{
	sn = sr.getSN();
	m_name = sr.getName();
	from = sr.getfrom();
	marker_from = sr.getfrommarker();
	to = sr.getto();
	marker_to = sr.gettomarker();
	k = sr.getStiff();
	c = sr.getDamp();
}
PointTSDA& PointTSDA::operator =(const PointTSDA &sr)
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
		return *this;
	}
}

//
std::vector<VectorN> PointTSDA::AssForceVec(Vector3x &fr, Vector3x &frd, EulerAngle &ftheta, EulerAngle &fthetad, Vector3x &tr, Vector3x &trd, EulerAngle &ttheta, EulerAngle &tthetad, double splineindex)
{
	// relative displacements and velocity in global coordinates
	Vector3x delta_pos = (from->getMarker_Pos(marker_from, fr, ftheta) - to->getMarker_Pos(marker_to, tr, ttheta));
	Vector3x delta_velo = (from->getMarker_Vel(marker_from, frd, ftheta, fthetad) - to->getMarker_Vel(marker_to, trd, ttheta, tthetad));

	// from inertiaelem local coordinates

	Matrix from_inertiaelem_inverserot = !(ftheta.getRotMat());
	Vector3x local_delta_pos = from_inertiaelem_inverserot*delta_pos;
	Vector3x local_delta_velo = from_inertiaelem_inverserot*delta_velo;

	//computation of force
	Vector3x force;
	force[0] = k[0] * delta_pos[0] + c[0] * delta_velo[0];
	force[1] = k[1] * delta_pos[1] + c[1] * delta_velo[1];
	force[2] = k[2] * delta_pos[2] + c[2] * delta_velo[2];

	//20210421 elastic and damp forces computed in local coordinate
	//force[0] = k[0] * local_delta_pos[0] + c[0] * local_delta_velo[0];
	//force[1] = k[1] * local_delta_pos[1] + c[1] * local_delta_velo[1];
	//force[2] = k[2] * local_delta_pos[2] + c[2] * local_delta_velo[2];
	//
	std::vector<VectorN> force_array;
	//forces
	Vector3x fi = (-1.0)*force;
	Vector3x fj = force;
	///torques
	Vector3x uip = (ftheta.getRotMat())*(from->getMarkerLocalVec(marker_from));
	Vector3x ujp = (ttheta.getRotMat())*(to->getMarkerLocalVec(marker_to));
	Matrix GiT = ~(ftheta.getGMat());
	Matrix GjT = ~(ttheta.getGMat());
	Vector3x ti = GiT*(uip.crossProduct(fi));
	Vector3x tj = GjT*(ujp.crossProduct(fj));
	VectorN tmpi(6), tmpj(6);
	for (int i = 0; i<3; i++)
	{
		tmpi[i] = fi[i];
		tmpi[i + 3] = ti[i];
		tmpj[i] = fj[i];
		tmpj[i + 3] = tj[i];
	}
	force_array.push_back(tmpi);
	force_array.push_back(tmpj);
	return force_array;
}
PointTSDA::~PointTSDA()
{
}
