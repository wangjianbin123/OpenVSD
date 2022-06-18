#include "pch.h"
#include "TSDA.h"

TSDA::TSDA(void)
{
}

TSDA::TSDA(int n, CString name, InertiaElem *f, int fmarker, InertiaElem *t, int tmarker, CubicSpline& stiff, CubicSpline& damper, double length,CubicSpline& ac)
{
	sn=n;
	m_name=name;
	from=f;
	marker_from=fmarker;
	to=t;
	marker_to=tmarker;
	k=stiff;
	c=damper;
	l0=length;
	actuator=ac;
}

TSDA::TSDA(const TSDA& sr)
{
	sn=sr.getSN();
	m_name=sr.getName();
	from=sr.getfrom();
	marker_from=sr.getfrommarker();
	to=sr.getto();
	marker_to=sr.gettomarker();
	k=sr.getStiff();
	c=sr.getDamp();
	l0=sr.getL0();
	actuator=sr.getActuator();
}
TSDA& TSDA::operator =(const TSDA &sr)
{
	if(this==&sr)
	{
		return *this;
	}
	else
	{
		sn=sr.getSN();
		m_name=sr.getName();
		from=sr.getfrom();
		marker_from=sr.getfrommarker();
		to=sr.getto();
		marker_to=sr.gettomarker();
		k=sr.getStiff();
		c=sr.getDamp();
		l0=sr.getL0();
		actuator=sr.getActuator();
		return *this;
	}
}
std::vector<VectorN> TSDA::AssForceVec(Vector3x &fr,Vector3x &frd,EulerAngle &ftheta,EulerAngle &fthetad,Vector3x &tr,Vector3x &trd,EulerAngle &ttheta,EulerAngle &tthetad,double splineindex)//20130710 modified
{
	//computation of delta relatively distance
	Vector3x delta_pos=(from->getMarker_Pos(marker_from,fr,ftheta)-to->getMarker_Pos(marker_to,tr,ttheta));
	double l=delta_pos.getNorm();
	double delta_l=l-l0;
	Vector3x delta_velo=(from->getMarker_Vel(marker_from,frd,ftheta,fthetad)-to->getMarker_Vel(marker_to,trd,ttheta,tthetad));
	Vector3x direction(delta_pos);
	direction.toUnit();
	double delta_v=delta_velo.dotProduct(direction);
	//computation of force
	double force=k.ValueAtC(delta_l)*delta_l+c.ValueAtC(delta_v)*delta_v+actuator.ValueAtC(splineindex);
	
	//return force vectors
	std::vector<VectorN> force_array;
	//force in global coordinate
	Vector3x fi=(-1.0)*force*direction;
	Vector3x fj=force*direction;
	//uip ujp in global coordinate
	Vector3x uip=(ftheta.getRotMat())*(from->getMarkerLocalVec(marker_from));
	Vector3x ujp=(ttheta.getRotMat())*(to->getMarkerLocalVec(marker_to));

	Matrix GiT=~(ftheta.getGMat());
	Matrix GjT=~(ttheta.getGMat());
	///////////////////////////////////////////////////////
	Vector3x ti = GiT*(uip.crossProduct(fi));
	Vector3x tj = GiT*(ujp.crossProduct(fj));
	VectorN tmpi(6),tmpj(6);
	for (int i=0;i<3;i++)
	{
		tmpi[i]=fi[i];
		tmpi[i+3]=ti[i];
		tmpj[i]=fj[i];
		tmpj[i+3]=tj[i];
	}
	force_array.push_back(tmpi);
	force_array.push_back(tmpj);
	return force_array;
}
TSDA::~TSDA(void)
{
}
