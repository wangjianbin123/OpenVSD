#include "pch.h"
#include "RWheelSet.h"

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


#ifndef _V_LIM_
#define V_LIM 1e-3
#endif

#include <vector>

RWheelSet::RWheelSet(void) :InertiaElem()
{
	R0.Empty();
	Rd0.Empty();
	Rdd0.Empty();
	THETA0.Empty();
	THETAd0.Empty();
	THETAdd0.Empty();
	R.Empty();
	Rd.Empty();
	Rdd.Empty();
	THETA.Empty();
	THETAd.Empty();
	THETAdd.Empty();
	l_radius=0.0;
	r_radius=0.0;
	ld=0;
	FLAG=0;
	//20210511
	ContGeoFlag = 0;
	LeftWheelGeometry.Empty();
	RightWheelGeometry.Empty();
	LeftConPara0.Empty();
	RightConPara0.Empty();
	currentLeftContPara.Empty();
	currentRightContPara.Empty();
	LeftWheelContact.Empty();
	RightWheelContact.Empty();
	currentForceVector.Empty();
	ConPntFLAG = 0;
	leftcurrentcontactforce.clear();
	rightcurrentcontactforce.clear();
	//
	RollingRadiusDiff.Empty();
	ConsEquContactPnts.Empty();
	//20210511
	leftWheelRailContactPntCSVec.clear();
	rightWheelRailContactPntCSVec.clear();
	//
	LinearSearchContactPnts.Empty();
	LinearSearchRadiusDiff.Empty();
	leftLinearSearchContactPnts.clear();
	rightLinearSearchContactPnts.clear();
}

RWheelSet::RWheelSet(int number, CString name, double mass, Vector3x &I, Vector3x &r0, Vector3x &rd0, Vector3x &rdd0, EulerAngle &theta0, EulerAngle &thetad0,
	EulerAngle &thetadd0, double lradius, double rradius, double l, Matrix &lp, Matrix &rp, int flag, int wrcontgeoflag, Track &tk, int sindex1, int sindex2, int sindex3, WRcontForce &wf) :InertiaElem(number, name, 0, 1, mass, I), R0(r0), Rd0(rd0), Rdd0(rdd0),
	THETA0(theta0), THETAd0(thetad0), THETAdd0(thetadd0), l_radius(lradius), r_radius(rradius), ld(l), FLAG(flag), ContGeoFlag(wrcontgeoflag), TK(tk), WheelForce(wf)
{
	R = R0;
	Rd = Rd0;
	Rdd = Rdd0;
	THETA = THETA0;
	THETAd = THETAd0;
	THETAdd = THETAdd0;
	currentForceVector.SetVectorN(6);
	leftcurrentcontactforce.clear();
	rightcurrentcontactforce.clear();
	
	this->InitWheelGeometry(lp, rp, sindex1, sindex2, sindex3);
	this->InitMarker();
	this->InitContGeo();
	//20210511
	//this->InitWheelRailContactGeometry(-0.0071, 0.0071, 0.0001, TK);
}
RWheelSet::RWheelSet(int number, CString name, double mass, Vector3x &I, Vector3x &r0, Vector3x &rd0, Vector3x &rdd0, EulerAngle &theta0, EulerAngle &thetad0,
	EulerAngle &thetadd0, double lradius, double rradius, double l, Matrix &lp, Matrix &rp, int flag, int wrcontgeoflag, Track &tk, int sindex1, int sindex2, int sindex3, int csm1, int csm2, int csm3, WRcontForce &wf) :InertiaElem(number, name, 0, 1, mass, I), R0(r0), Rd0(rd0), Rdd0(rdd0),
	THETA0(theta0), THETAd0(thetad0), THETAdd0(thetadd0), l_radius(lradius), r_radius(rradius), ld(l), FLAG(flag), ContGeoFlag(wrcontgeoflag), TK(tk), WheelForce(wf)
{
	R = R0;
	Rd = Rd0;
	Rdd = Rdd0;
	THETA = THETA0;
	THETAd = THETAd0;
	THETAdd = THETAdd0;
	currentForceVector.SetVectorN(6);
	leftcurrentcontactforce.clear();
	rightcurrentcontactforce.clear();

	this->InitWheelGeometry(lp, rp, sindex1, sindex2, sindex3, csm1, csm2, csm3);
	this->InitMarker();
	this->InitContGeo();
	//20210511
	//this->InitWheelRailContactGeometry(-0.0071, 0.0071, 0.0001, TK);
}
RWheelSet::RWheelSet(const RWheelSet &ws)
{
	SN=ws.getSN();
	m_name=ws.getName();
	FLEX=ws.getFlexible();
	NG=ws.getGround();
	m=ws.getMass();
	principle_I=ws.getPrincipleI();
	principle_mat=ws.getPrincipleMat();
	g=ws.getGravity();
	wsmarker=ws.getMarkerVector();
	R0=ws.getR0();
	Rd0=ws.getRD0(); 
	Rdd0=ws.getRDD0();
	THETA0=ws.getTHETA0(); 
	THETAd0=ws.getTHETAD0(); 
	THETAdd0=ws.getTHETADD0();
	R=ws.getR(); 
	THETA=ws.getTHETA();
	Rd=ws.getRD(); 
	THETAd=ws.getTHETAD(); 
	Rdd=ws.getRDD(); 
	THETAdd=ws.getTHETADD();
	l_radius=ws.getLeftRadius();
	r_radius=ws.getRightRadius();
	ld=ws.getLd();
	TK=ws.getTrack();
	FLAG=ws.getCreepModel();
	//20210511
	ContGeoFlag = ws.getContGeoFlag();
	LeftConPara0=ws.getLeftConPara0();
	RightConPara0=ws.getRightConPara0();
	currentLeftContPara=ws.getcurrentLeftContPara();
	currentRightContPara=ws.getcurrentRightContPara();
	LeftWheelGeometry=ws.getLeftWheelGeometry();
	RightWheelGeometry=ws.getRightWheelGeometry();
	LeftWheelContact=ws.getLeftWheelContact();
	RightWheelContact=ws.getRightWheelContact();
	WheelForce=ws.getWheelForce();
	force=ws.getTPF();
	obsvec=ws.getObserverHdr();
	currentForceVector = ws.getCurrentForceVector();
	leftcurrentcontactforce = ws.getleftcurrentcontactforce();
	rightcurrentcontactforce = ws.getrightcurrentcontactforce();
	ConPntFLAG = ws.getConPntFLAG();
	RollingRadiusDiff = ws.getRollingRadiusDiff();
	ConsEquContactPnts = ws.getConsEquContactPnts();
	LinearSearchContactPnts = ws.getLinearSearchContactPnts();
	LinearSearchRadiusDiff = ws.getLinearSearchRadiusDiff();
	leftLinearSearchContactPnts = ws.getleftLinearSearchContactPnts();
	rightLinearSearchContactPnts = ws.getrightLinearSearchContactPnts();
}
RWheelSet& RWheelSet::operator=(const RWheelSet &ws)
{
	if (this==&ws)
	{
		return *this;
	}
	else
	{
		SN=ws.getSN();
		m_name=ws.getName();
		FLEX=ws.getFlexible();
		NG=ws.getGround();
		m=ws.getMass();
		principle_I=ws.getPrincipleI();
		principle_mat=ws.getPrincipleMat();
		g=ws.getGravity();
		wsmarker=ws.getMarkerVector();
		R0=ws.getR0(); 
		Rd0=ws.getRD0(); 
		Rdd0=ws.getRDD0();
		THETA0=ws.getTHETA0(); 
		THETAd0=ws.getTHETAD0(); 
		THETAdd0=ws.getTHETADD0();
		R=ws.getR(); 
		THETA=ws.getTHETA(); 
		Rd=ws.getRD(); 
		THETAd=ws.getTHETAD(); 
		Rdd=ws.getRDD(); 
		THETAdd=ws.getTHETADD();
		l_radius=ws.getLeftRadius();
		r_radius=ws.getRightRadius();
		ld=ws.getLd();
		TK=ws.getTrack();
		FLAG=ws.getCreepModel();
		ContGeoFlag = ws.getContGeoFlag();
		LeftConPara0=ws.getLeftConPara0();
		RightConPara0=ws.getRightConPara0();
		currentLeftContPara=ws.getcurrentLeftContPara();
		currentRightContPara=ws.getcurrentRightContPara();
		LeftWheelGeometry=ws.getLeftWheelGeometry();
		RightWheelGeometry=ws.getRightWheelGeometry();
		LeftWheelContact=ws.getLeftWheelContact();
		RightWheelContact=ws.getRightWheelContact();
		WheelForce=ws.getWheelForce();
		force=ws.getTPF();
		obsvec=ws.getObserverHdr();
		currentForceVector = ws.getCurrentForceVector();
		leftcurrentcontactforce = ws.getleftcurrentcontactforce();
		rightcurrentcontactforce = ws.getrightcurrentcontactforce();
		ConPntFLAG = ws.getConPntFLAG();
		RollingRadiusDiff = ws.getRollingRadiusDiff();
		ConsEquContactPnts = ws.getConsEquContactPnts();
		LinearSearchContactPnts = ws.getLinearSearchContactPnts();
		LinearSearchRadiusDiff = ws.getLinearSearchRadiusDiff();
		leftLinearSearchContactPnts = ws.getleftLinearSearchContactPnts();
		rightLinearSearchContactPnts = ws.getrightLinearSearchContactPnts();
		return *this;
	}
}
//构造函数调用的初始化函数

void RWheelSet::InitMaterial(double ew, double er, double miuw, double miur,double fr)
{
	WheelForce.setMaterial(ew, er, miuw, miur, fr);
}

void RWheelSet::InitWheelGeometry(Matrix &l,Matrix &r,int smothindex1,int smothindex2,int smothindex3)
{
	//wheel profiles
	LeftWheelGeometry.setParameter(l_radius, l, smothindex1, smothindex2, smothindex3);
	RightWheelGeometry.setParameter(r_radius, r, smothindex1, smothindex2, smothindex3);
	//wheel rail contact geometry
	LeftWheelContact.setParameter(LeftWheelGeometry, 1);
	RightWheelContact.setParameter(RightWheelGeometry, 0);
}

void RWheelSet::InitWheelGeometry(Matrix &l, Matrix &r, int smothindex1, int smothindex2, int smothindex3, int csm1, int csm2, int csm3)
{
	//wheel profiles
	LeftWheelGeometry.setParameter(l_radius, l, smothindex1, smothindex2, smothindex3, csm1, csm2, csm3);
	RightWheelGeometry.setParameter(r_radius, r, smothindex1, smothindex2, smothindex3, csm1, csm2, csm3);
	//wheel rail contact geometry
	LeftWheelContact.setParameter(LeftWheelGeometry, 1);
	RightWheelContact.setParameter(RightWheelGeometry, 0);
}

void RWheelSet::InitMarker(void)
{
	//Left wheelset geometry coordinates
	Vector3x r1(0.0,ld/2.0,0.0);
	EulerAngle ea1(0.0,0.0,0.0);
	Marker LWG(1,r1,ea1);
	wsmarker.push_back(LWG); // Marker 1
	//Right wheelset geometry coordinates
	Vector3x r2(0.0,(-1.0)*ld/2.0,0.0);
	EulerAngle ea2(0.0,0.0,0.0);
	Marker RWG(2,r2,ea2); //Marker 2
	wsmarker.push_back(RWG);
}
void RWheelSet::InitContGeo(void)
{
	VectorN lInit0(4);
	lInit0[0]=R0.getRy()*-1.0;
	lInit0[1] = 0.0;
	lInit0[2]=R0.getRx();
	lInit0[3] = R0.getRy()*-1.0;;
	VectorN rInit0(4);
	rInit0[0] =R0.getRy()*-1.0;
	rInit0[1] = 0.0;
	rInit0[2]= R0.getRx();
	rInit0[3] =R0.getRy()*-1.0;
	//
	LeftConPara0=LeftWheelContact.AssConPara(TK,lInit0,this->getMarker_Pos(1,this->getR0(),this->getTHETA0()),this->getTHETA0().getRotMat());
	RightConPara0=RightWheelContact.AssConPara(TK,rInit0,this->getMarker_Pos(2,this->getR0(),this->getTHETA0()),this->getTHETA0().getRotMat());
	//
	currentLeftContPara=LeftConPara0;
	currentRightContPara=RightConPara0;
}
Vector3x RWheelSet::getMarker_Pos(int num,Vector3x &r,EulerAngle &theta)
{
	// return result in Global Coordinate
	return (r + (theta.getRotMat())*(wsmarker[num].getOrigin())); 
}
Vector3x RWheelSet::getPoint_Pos(Vector3x &loc,Vector3x &r,EulerAngle &theta)
{
	return (r+theta.getRotMat()*loc);
}
Vector3x RWheelSet::getMarker_Vel(int num,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad)
{
	// return result in Global Coordinate
	return rd - (theta.getRotMat()*((wsmarker[num].getOrigin()).toSkewMatrix())*(theta.getGconjMat()))*thetad.toVector3x();
}
Vector3x RWheelSet::getPoint_Vel(Vector3x &loc,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad)
{
	return rd - (theta.getRotMat()*(loc.toSkewMatrix())*(theta.getGconjMat()))*thetad.toVector3x();
}
Vector3x RWheelSet::AssGravityForce(void)
{
	return m*g;
}
EulerAngle RWheelSet::getEulerAngle(void)
{
	return THETA;
}
Vector3x RWheelSet::getMarkerLocalVec(int num)
{
	return wsmarker[num].getOrigin();
}
Matrix RWheelSet::AssMassMat(EulerAngle &theta)
{
	Matrix Mrr(3,3);
	Matrix GconjMat = theta.getGconjMat();
	Mrr[0][0]=m; 
	Mrr[1][1]=m; 
	Mrr[2][2]=m;
	Matrix Mtt = (~GconjMat)*principle_mat*GconjMat;
	Matrix M(6,6);
	M.putSubMat(0,0,Mrr); 
	M.putSubMat(3,3,Mtt);
	return M;
}
VectorN RWheelSet::AssQuadForceVec(EulerAngle &theta,EulerAngle &thetad)
{
	Matrix GconjMat = theta.getGconjMat();
	VectorN omega_n = GconjMat*thetad.toVectorN();
	VectorN iomega = principle_mat*omega_n;
	VectorN force = (-1.0)*(~GconjMat)*((omega_n.toSkewMatrix()*iomega) + (principle_mat*this->getGConjMatDer(theta, thetad))*thetad.toVectorN());
	return force;
}

VectorN RWheelSet::AssExternalForce(EulerAngle &theta,double splineindex)
{
	Vector3x GraForce=this->AssGravityForce();
	Vector3x tmp;
	if (!force.empty())
	{
		for (std::vector<TPFElem>::iterator it = force.begin(); it<force.end(); it++)
		{
			if ((*it).getflag() == 0)
			{
				tmp += theta.getRotMat()*(*it).getForceVec(splineindex);
			}
		}
	}
	tmp+=GraForce;
	return tmp.toVectorN();
}

VectorN RWheelSet::AssExternalTorque(EulerAngle &theta,double splineindex)
{
	VectorN tmp(3);
	Matrix rotmat = theta.getRotMat();
	Matrix G = theta.getGMat();
	if (!force.empty())
	{
		for (std::vector<TPFElem>::iterator it = force.begin(); it<force.end(); it++)
		{
			if ((*it).getflag() == 1)
			{
				tmp += (~G)*(rotmat*(*it).getForceVec(splineindex).toVectorN());
			}
			else
			{
				Vector3x f = (*it).getForceVec(splineindex);
				Vector3x l = (*it).getat().getOrigin();
				Vector3x localM = l.crossProduct(f);
				VectorN tmp2 = (~G)*(rotmat*localM.toVectorN());
				tmp += tmp2;
			}
		}
	}
	return tmp;
}

VectorN RWheelSet::AssembleForceVector(Vector3x &r,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad,double splineindex)
{
	VectorN force=this->AssExternalForce(theta,splineindex);
	VectorN torque=this->AssExternalTorque(theta,splineindex);
	VectorN quarforce=this->AssQuadForceVec(theta,thetad);
	//
	Matrix rotmat = theta.getRotMat();
	Matrix Gconjmat = theta.getGconjMat();
	Matrix G = theta.getGMat();

	Vector3x lwprofilevec = this->getMarker_Pos(1, r, theta);
	Vector3x rwprofilevec = this->getMarker_Pos(2, r, theta);
	
	// get the current contact points
	//VectorN leftconpara = LeftWheelContact.AssConPara(TK, this->getcurrentLeftContPara(), lwprofilevec, rotmat);
	//VectorN rightconpara = RightWheelContact.AssConPara(TK, this->getcurrentRightContPara(), rwprofilevec, rotmat);

	VectorN leftconpara = this->getcurrentLeftContPara();
	VectorN rightconpara = this->getcurrentRightContPara();
	//
	Vector3x leftT1 = TK.LeftRail_T1(leftconpara[2], leftconpara[3]);
	Vector3x leftT2 = TK.LeftRail_T2(leftconpara[2], leftconpara[3]);
	Vector3x leftN = TK.LeftRail_N(leftconpara[2], leftconpara[3]);
	Vector3x rightT1 = TK.RightRail_T1(rightconpara[2], rightconpara[3]);
	Vector3x rightT2 = TK.RightRail_T2(rightconpara[2], rightconpara[3]);
	Vector3x rightN = TK.RightRail_N(rightconpara[2], rightconpara[3]);

	double leftPene = LeftWheelContact.Penetration(TK, leftconpara, lwprofilevec, rotmat);
	double rightPene = RightWheelContact.Penetration(TK, rightconpara, rwprofilevec, rotmat);
	
	Vector3x leftmarker(0.0, ld / 2.0, 0.0);
	Vector3x lconpnt =  LeftWheelGeometry.getLocalPos(leftconpara[0], leftconpara[1]) + leftmarker;
	Vector3x lconpnt_global = this->getPoint_Pos(lconpnt, r, theta);
	Vector3x rightmarker(0.0, (-1.0)*ld / 2.0, 0.0);
	Vector3x rconpnt = RightWheelGeometry.getLocalPos(rightconpara[0], rightconpara[1]) + rightmarker;
	Vector3x rconpnt_global = this->getPoint_Pos(rconpnt, r, theta);
	Vector3x lconpnt_v = this->getPoint_Vel(lconpnt, rd, theta, thetad);
	double leftpenevelo = lconpnt_v*leftN;//penetration speed
	Vector3x rconpnt_v = this->getPoint_Vel(rconpnt, rd, theta, thetad);
	double rightpenevelo = rconpnt_v*rightN;//penetration speed

	VectorN lK(4);
	lK[0] = LeftWheelGeometry.getCurvatureP1(leftconpara[0]);
	lK[1] = LeftWheelGeometry.getCurvatureP2(leftconpara[0]);
	lK[2] = 0.0;
	lK[3] = TK.LeftRail_curvature(leftconpara[3]);
	VectorN rK(4);
	rK[0] = RightWheelGeometry.getCurvatureP1(rightconpara[0]);
	rK[1] = RightWheelGeometry.getCurvatureP2(rightconpara[0]);
	rK[2] = 0.0;
	rK[3] = TK.RightRail_curvature(rightconpara[3]);

	Vector3x omega = rotmat*(Gconjmat*thetad.toVector3x());
	double lV = rd*leftT1;
	VectorN leftcreepage(3);
	if (leftPene<0.0&&abs(lV)>=V_LIM)
	{
		leftcreepage[0] = (lconpnt_v*leftT1) / lV;
		leftcreepage[1] = (lconpnt_v*leftT2) / lV;
		leftcreepage[2] = (omega*leftN) / lV;
	}
	VectorN lForce(3);
	if (FLAG == 0)
	{
		lForce = WheelForce.Kalker_Creepforce(lK, leftcreepage, theta[0], leftPene);
	}
	if (FLAG == 1)
	{
		lForce = WheelForce.SHE_Creepforce(lK, leftcreepage, theta[0], leftPene);
	}
	if (FLAG == 2)
	{
		lForce = WheelForce.Polach_Creepforce(lK, leftcreepage, theta[0], leftPene);
	}
	//20210406
	if (FLAG == 3)
	{
		lForce = WheelForce.None_Creepforce(lK, leftcreepage, theta[0], leftPene);
	}
	VectorN rightcreepage(3);
	double rV = rd*rightT1;
	if (rV != 0.0&&abs(rV)>=V_LIM)
	{
		rightcreepage[0] = (rconpnt_v*rightT1) / rV;
		rightcreepage[1] = (rconpnt_v*rightT2) / rV;
		rightcreepage[2] = (omega*rightN) / rV;
	}
	VectorN rForce(3);
	if (FLAG == 0)
	{
		rForce = WheelForce.Kalker_Creepforce(rK, rightcreepage, theta[0], rightPene);
	}
	if (FLAG == 1)
	{
		rForce = WheelForce.SHE_Creepforce(rK, rightcreepage, theta[0], rightPene);
	}
	if (FLAG == 2)
	{
		rForce = WheelForce.Polach_Creepforce(rK, rightcreepage, theta[0], rightPene);
	}
	//20210406
	if (FLAG == 3)
	{
		rForce = WheelForce.None_Creepforce(rK, rightcreepage, theta[0], rightPene);
	}
	//normal forces
	double lnormalforce = WheelForce.LeftNormalForce(leftconpara, leftPene, theta[0]);
	double rnormalforce=WheelForce.RightNormalForce(rightconpara,rightPene,theta[0]);
	double lnormaldampforce=WheelForce.LeftNormalDampForce(leftpenevelo,leftPene);
	double rnormaldampforce=WheelForce.RightNormalDampForce(rightpenevelo,rightPene);

	Vector3x lcontactforce = lForce[0] * leftT1 + lForce[1] * leftT2 + (lnormalforce + lnormaldampforce)*leftN;
	Vector3x rcontactforce = rForce[0] * rightT1 + rForce[1] * rightT2 + (rnormalforce + rnormaldampforce)*rightN;

	Vector3x lcontacttorque = (~G)*((rotmat*lconpnt).crossProduct(lcontactforce));
	Vector3x rcontacttorque = (~G)*((rotmat*rconpnt).crossProduct(rcontactforce));

	Vector3x lspint = leftN*(lForce[2]);
	Vector3x lspintorque = (~G)*lspint;
	Vector3x rspint = rightN*(rForce[2]);
	Vector3x rspintorque = (~G)*rspint;
	lcontacttorque += lspintorque;
	rcontacttorque += rspintorque;

	std::vector<VectorN> res;
	// creepage3 contact_force3 contact_ torque3 penetraton1 
	res.push_back(leftcreepage);
	res.push_back(lForce);
	res.push_back(lcontactforce.toVectorN());
	VectorN leftpenetration(1);
	leftpenetration[0] = leftPene;
	res.push_back(leftpenetration);
	this->UpdateLeftCurrentContactForce(res);
	res.clear();
	res.push_back(rightcreepage);
	res.push_back(rForce);
	res.push_back(rcontactforce.toVectorN());
	VectorN rightpenetration(1);
	rightpenetration[0] = rightPene;
	res.push_back(rightpenetration);

	this->UpdateRightCurrentContactForce(res);
	res.clear();
	//
	VectorN tmp(6);
	for (int i=0;i<3;i++)
	{
		tmp[i] = force[i] + lcontactforce[i] + rcontactforce[i];
	}
	for (int j=3;j<6;j++)
	{
		tmp[j] = torque[j - 3] + quarforce[j - 3] + lcontacttorque[j - 3] + rcontacttorque[j - 3];
	}
	return tmp;
}
void RWheelSet::UpdateLeftWRContPara(VectorN &lcontpara)
{
	currentLeftContPara=lcontpara;
}
void RWheelSet::UpdateRightWRContPara(VectorN &rcontpara)
{
	currentRightContPara=rcontpara;
}
void RWheelSet::UpdateInitialKinematicsParaPos(VectorN &q)
{
	R0[0] = q[0];
	R0[1] = q[1];
	R0[2] = q[2];
	THETA0[0] = q[3];
	THETA0[1] = q[4];
	THETA0[2] = q[5];
}

void RWheelSet::UpdateInitialKinematicsParaPosVel(VectorN &q, VectorN &qd)
{
	//
	R0[0] = q[0];
	R0[1] = q[1];
	R0[2] = q[2];
	THETA0[0] = q[3];
	THETA0[1] = q[4];
	THETA0[2] = q[5];
	//
	Rd0[0] = qd[0];
	Rd0[1] = qd[1];
	Rd0[2] = qd[2];
	THETAd0[0] = qd[3];
	THETAd0[1] = qd[4];
	THETAd0[2] = qd[5];
}
void RWheelSet::UpdateInitialKinematicsParaPosVelAcc(VectorN &q, VectorN &qd, VectorN &qdd)
{
	//
	R0[0] = q[0];
	R0[1] = q[1];
	R0[2] = q[2];
	THETA0[0] = q[3];
	THETA0[1] = q[4];
	THETA0[2] = q[5];
	//
	Rd0[0] = qd[0];
	Rd0[1] = qd[1];
	Rd0[2] = qd[2];
	THETAd0[0] = qd[3];
	THETAd0[1] = qd[4];
	THETAd0[2] = qd[5];
	//
	Rdd0[0] = qdd[0];
	Rdd0[1] = qdd[1];
	Rdd0[2] = qdd[2];
	THETAdd0[0] = qdd[3];
	THETAdd0[1] = qdd[4];
	THETAdd0[2] = qdd[5];
}
void RWheelSet::SetRunSpeed(double V)
{
	Rd0[0] = V;
	THETAd0[2] = 2 * V / ((this->getLeftWheelGeometry().get_radius()) +( this->getRightWheelGeometry().get_radius()));
}
void RWheelSet::addForceVec(TPFElem &tpf)
{
	force.push_back(tpf);
}
// re-verified at 20141016
Matrix RWheelSet::getGConjMatDer(EulerAngle &theta,EulerAngle &thetad)
{
	Matrix mat(3, 3);
	mat[0][0] = sin(theta[1])*sin(theta[2])*thetad[1] - cos(theta[1])*cos(theta[2])*thetad[2];
	mat[0][1] = (-1.0)*sin(theta[2])*thetad[2];
	mat[0][2] = 0.0;
	mat[1][0] = cos(theta[1])*thetad[1];
	mat[1][1] = 0.0;
	mat[1][2] = 0.0;
	mat[2][0] = (-1.0)*sin(theta[1])*cos(theta[2])*thetad[1] - cos(theta[1])*sin(theta[2])*thetad[2];
	mat[2][1] = cos(theta[2])*thetad[2];
	mat[2][2] = 0.0;
	return mat;
}
void RWheelSet::CreateObserverHdr(Observer& obs)
{
	ObserverHdr* pObs=new ObserverHdr(obs);
	obsvec.push_back(*pObs);
}
void RWheelSet::TriggeIntPntResPushBackrProcess(void)
{
	for (std::vector<ObserverHdr>::iterator ite=obsvec.begin();ite<obsvec.end();ite++)
	{
		(*ite)->IntPntResPushBackprocess(*this);
	}
}
void RWheelSet::TriggerSave(void)
{
	for (std::vector<ObserverHdr>::iterator ite=obsvec.begin();ite<obsvec.end();ite++)
	{
		(*ite)->save();
	}
}
void RWheelSet::UpdateKimematicsPara(VectorN &q,VectorN &qd)
{
	// For incremental stepsize computations
	Vector3x old_R = R;
	Vector3x old_Rd = Rd;
	EulerAngle old_THETA = THETA;
	EulerAngle old_THETAd = THETAd;
	//R T
	R[0] = q[0];
	R[1] = q[1];
	R[2] = q[2];
	THETA[0] = q[3];
	THETA[1] = q[4];
	THETA[2] = q[5];
	//Rd Td
	Rd[0] = qd[0];
	Rd[1] = qd[1];
	Rd[2] = qd[2];
	THETAd[0] = qd[3];
	THETAd[1] = qd[4];
	THETAd[2] = qd[5];

	//
	double incremental_forward_x = R.getNorm() - old_R.getNorm();
	double incremental_theta = 2 * (R.getNorm() - old_R.getNorm()) / (this->getLeftRadius() + this->getRightRadius());
	
	//Contact Points
	VectorN left_start_iteration = this->getcurrentLeftContPara(); 
	//left_start_iteration[1] += incremental_theta;
	//left_start_iteration[3] += incremental_forward_x;
	VectorN right_start_iteration = this->getcurrentRightContPara();
	//right_start_iteration[1] += incremental_theta;
	//right_start_iteration[3] += incremental_forward_x;
	//
	VectorN leftconpara = LeftWheelContact.AssConPara(TK, left_start_iteration, this->getMarker_Pos(1, R, THETA), THETA.getRotMat());
	VectorN rightconpara = RightWheelContact.AssConPara(TK, right_start_iteration, this->getMarker_Pos(2, R, THETA), THETA.getRotMat());
	//
	this->UpdateLeftWRContPara(leftconpara);
	this->UpdateRightWRContPara(rightconpara);
}

void RWheelSet::UpdateEquivalentPara(VectorN &q, VectorN &qd)
{
	//R T
	R[0] = q[0];
	R[1] = q[1];
	R[2] = q[2];
	THETA[0] = q[3];
	THETA[1] = q[4];
	THETA[2] = q[5];
	//Rd Td
	Rd[0] = qd[0];
	Rd[1] = qd[1];
	Rd[2] = qd[2];
	THETAd[0] = qd[3];
	THETAd[1] = qd[4];
	THETAd[2] = qd[5];
}
void RWheelSet::UpdateKinematicsParaAcc(VectorN &q, VectorN &qd, VectorN &qdd)
{
	// R T
	R[0] = q[0];
	R[1] = q[1];
	R[2] = q[2];
	THETA[0] = q[3];
	THETA[1] = q[4];
	THETA[2] = q[5];
	//Rd Td
	Rd[0] = qd[0];
	Rd[1] = qd[1];
	Rd[2] = qd[2];
	THETAd[0] = qd[3];
	THETAd[1] = qd[4];
	THETAd[2] = qd[5];
	// Rdd Tdd
	Rdd[0] = qd[0];
	Rdd[1] = qd[1];
	Rdd[2] = qd[2];
	THETAdd[0] = qd[3];
	THETAdd[1] = qd[4];
	THETAdd[2] = qd[5];
	// Contact Points
	VectorN leftconpara = LeftWheelContact.AssConPara(TK, this->getcurrentLeftContPara(), this->getMarker_Pos(1, R, THETA), THETA.getRotMat());
	VectorN rightconpara = RightWheelContact.AssConPara(TK, this->getcurrentRightContPara(), this->getMarker_Pos(2, R, THETA), THETA.getRotMat());
	this->UpdateLeftWRContPara(leftconpara);
	this->UpdateRightWRContPara(rightconpara);
}
VectorN RWheelSet::getCurrentForcePara(void)
{
	return currentForceVector;
}
void RWheelSet::UpdateForcePara(VectorN &vecn)
{
	currentForceVector = vecn;
}
void RWheelSet::InitLinearSearchContactGeometry(double startpnt, double endpnt, double stepsize, Track &tk)
{
	//20210104
	this->InitConsolWnd();
	double linearsearchstep = 0.00005;
	WheelGeometry leftwheelgeo = this->getLeftWheelGeometry();
	WheelGeometry rightwheelgeo = this->getRightWheelGeometry();
	Matrix leftwheelgeometry = leftwheelgeo.get_Profile().getPoints();
	Matrix rightwheelgeometry = rightwheelgeo.get_Profile().getPoints();
	Matrix leftrailgeometry = tk.getLProfile();
	Matrix rightrailgeometry = tk.getRProfile();
	int lwnumpnt = leftwheelgeometry.Row();
	int rwnumpnt = rightwheelgeometry.Row();
	int lrnumpnt = leftrailgeometry.Row();
	int rrnumpnt = rightrailgeometry.Row();
	double lwstartpnt = leftwheelgeometry[0][0];
	double lwendpnt = leftwheelgeometry[lwnumpnt - 1][0];
	double rwstartpnt = rightwheelgeometry[0][0];
	double rwendpnt = rightwheelgeometry[rwnumpnt - 1][0];
	double lrstartpnt = leftrailgeometry[0][0];
	double lrendpnt = leftrailgeometry[lrnumpnt - 1][0];
	double rrstartpnt = rightrailgeometry[0][0];
	double rrendpnt = rightrailgeometry[rrnumpnt - 1][0];
	//
	std::vector<double> ymov;
	std::vector<double> radiusdiff;
	for (double y = startpnt; y <= endpnt; y += stepsize)
	{
		printf("%12.10f\n",y);
		double leftlimit = 1e6;
		double rightlimit = 1e6;
		Vector3x origin = R0;
		origin[0] = 0.0;
		origin[1] = y;
		origin[2] = 1.0;
		EulerAngle pos0 = THETA0;
		VectorN leftcontactpnt(4);
		VectorN rightcontactpnt(4);
		Matrix rotmat = pos0.getRotMat();
		for (double i = lrstartpnt; i <= lrendpnt; i += linearsearchstep)
		{
			Vector3x lprofpos(0, this->getLd() / 2.0, 0.0);
			//double lvdistance = (origin + rotmat*(lprofpos + leftwheelgeo.getLocalPos(i-y, 0.0)) - tk.LeftPositon(0.0, i))[2];
			double lvdistance =1.0+(leftwheelgeo.getLocalPos(i - y, 0.0))[2] - tk.LeftPositon(0.0, i)[2];
			if (lvdistance <= leftlimit)
			{
				leftlimit = lvdistance;
				leftcontactpnt[0] = i-y;
				leftcontactpnt[3] = i;
			}
		}
		for (double j = rrstartpnt; j <= rrendpnt; j += linearsearchstep)
		{
			Vector3x rprofpos(0, (-1.0)*this->getLd() / 2.0, 0.0);
			//double rvdistance = (origin + rotmat*(rprofpos + rightwheelgeo.getLocalPos(j-y, 0.0)) - tk.RightPosition(0.0, j))[2];
			double rvdistance = 1.0+(rightwheelgeo.getLocalPos(j - y, 0.0))[2] - tk.RightPosition(0.0, j)[2];
			if (rvdistance <= rightlimit)
			{
				rightlimit = rvdistance;
				rightcontactpnt[0] = j - y;
				rightcontactpnt[3] = j;
			}
		}
		double diff = leftwheelgeo.getRadius(leftcontactpnt[0]) - rightwheelgeo.getRadius(rightcontactpnt[0]);
		ymov.push_back(y);
		radiusdiff.push_back(diff);
		leftLinearSearchContactPnts.push_back(leftcontactpnt);
		rightLinearSearchContactPnts.push_back(rightcontactpnt);
	}
	int k = static_cast<int>(ymov.size());
	LinearSearchRadiusDiff.SetMatrix(k, 2);
	for (int ii = 0; ii < k; ii++)
	{
		LinearSearchRadiusDiff[ii][0] = ymov[ii];
		LinearSearchRadiusDiff[ii][1] = radiusdiff[ii]*(-1);
	}
	LinearSearchContactPnts.SetMatrix(k, 9);
	for (int jj = 0; jj < k; jj++)
	{
		LinearSearchContactPnts[jj][0] = LinearSearchRadiusDiff[jj][0];
		LinearSearchContactPnts[jj][1] = leftLinearSearchContactPnts[jj][0];
		LinearSearchContactPnts[jj][2] = leftLinearSearchContactPnts[jj][1];
		LinearSearchContactPnts[jj][3] = leftLinearSearchContactPnts[jj][2];
		LinearSearchContactPnts[jj][4] = leftLinearSearchContactPnts[jj][3];
		LinearSearchContactPnts[jj][5] = rightLinearSearchContactPnts[jj][0];
		LinearSearchContactPnts[jj][6] = rightLinearSearchContactPnts[jj][1];
		LinearSearchContactPnts[jj][7] = rightLinearSearchContactPnts[jj][2];
		LinearSearchContactPnts[jj][8] = rightLinearSearchContactPnts[jj][3];
	}
}

void RWheelSet::InitWheelRailContactGeometry(double startpnt, double endpnt, double stepsize, Track &tk)
{
	this->InitConsolWnd();
	EulerAngle pos0 = THETA0;
	Vector3x origin = R0;
	double diff = 0.0;
	VectorN lcontpara(4);
	VectorN rcontpara(4);
	std::vector<double> posresx;
	std::vector<double> posresy;
	std::vector<double> negresx;
	std::vector<double> negresy;
	std::vector<VectorN> posleftwrcongeo;
	std::vector<VectorN> posrightwrcongeo;
	std::vector<VectorN> negleftwrcongeo;
	std::vector<VectorN> negrightwrcongeo;

	for (double i = 0.0; i <= endpnt; i += stepsize)
	{
		origin[1] = i;
		//
		VectorN leftcontpnt = this->LeftWheelContact.AssConPara(tk, lcontpara, this->getMarker_Pos(1, origin, pos0), pos0.getRotMat());
		VectorN rightcontpnt = this->RightWheelContact.AssConPara(tk, rcontpara, this->getMarker_Pos(2, origin, pos0), pos0.getRotMat());
		diff = this->RightWheelGeometry.getRadius(rightcontpnt[0]) - this->LeftWheelGeometry.getRadius(leftcontpnt[0]);
		//
		lcontpara = leftcontpnt;
		rcontpara = rightcontpnt;
		//
		posresx.push_back(i);
		posresy.push_back(diff);
		posleftwrcongeo.push_back(lcontpara);
		posrightwrcongeo.push_back(rcontpara);
	}

	lcontpara.SetVectorN(4);
	rcontpara.SetVectorN(4);

	for (double i = 0.0; i >= startpnt; i -= stepsize)
	{
		origin[1] = i;
		//
		VectorN leftcontpnt = this->LeftWheelContact.AssConPara(tk, lcontpara, this->getMarker_Pos(1, origin, pos0), pos0.getRotMat());
		VectorN rightcontpnt = this->RightWheelContact.AssConPara(tk, rcontpara, this->getMarker_Pos(2, origin, pos0), pos0.getRotMat());
		diff = this->RightWheelGeometry.getRadius(rightcontpnt[0]) - this->LeftWheelGeometry.getRadius(leftcontpnt[0]);

		lcontpara = leftcontpnt;
		rcontpara = rightcontpnt;
		//
		negresx.push_back(i);
		negresy.push_back(diff);
		negleftwrcongeo.push_back(lcontpara);
		negrightwrcongeo.push_back(rcontpara);
	}

	int possize = static_cast<int>(posresx.size());
	int negsize = static_cast<int>(negresx.size());
	Matrix resmat(possize+negsize-1, 2);
	Matrix mx(possize + negsize-1, 9);

	for (int k = negsize-1; k>0; k--)
	{
		int index = negsize - 1 - k;
		resmat[index][0] = negresx[k];
		resmat[index][1] = negresy[k];
		mx[index][0] = negresx[k];
		mx[index][1] = negleftwrcongeo[k][0];
		mx[index][2] = negleftwrcongeo[k][1];
		mx[index][3] = negleftwrcongeo[k][2];
		mx[index][4] = negleftwrcongeo[k][3];
		mx[index][5] = negrightwrcongeo[k][0];
		mx[index][6] = negrightwrcongeo[k][1];
		mx[index][7] = negrightwrcongeo[k][2];
		mx[index][8] = negrightwrcongeo[k][3];
	}

	for (int k = 0; k<possize; k++)
	{
		int index = negsize - 1 +k;
		resmat[index][0] = posresx[k];
		resmat[index][1] = posresy[k];
		mx[index][0] = posresx[k];
		mx[index][1] = posleftwrcongeo[k][0];
		mx[index][2] = posleftwrcongeo[k][1];
		mx[index][3] = posleftwrcongeo[k][2];
		mx[index][4] = posleftwrcongeo[k][3];
		mx[index][5] = posrightwrcongeo[k][0];
		mx[index][6] = posrightwrcongeo[k][1];
		mx[index][7] = posrightwrcongeo[k][2];
		mx[index][8] = posrightwrcongeo[k][3];
	}
	this->RollingRadiusDiff = resmat;
	this->ConsEquContactPnts = mx;
	//
	CubicSpline leftwheelshift(ConsEquContactPnts.getDoubleCols(0,1));
	CubicSpline leftrailshift(ConsEquContactPnts.getDoubleCols(0, 4));
	leftWheelRailContactPntCSVec.push_back(leftwheelshift);
	leftWheelRailContactPntCSVec.push_back(leftrailshift);
	//
	CubicSpline rightwheelshift(ConsEquContactPnts.getDoubleCols(0, 5));
	CubicSpline rightrailshift(ConsEquContactPnts.getDoubleCols(0, 8));
	rightWheelRailContactPntCSVec.push_back(rightwheelshift);
	rightWheelRailContactPntCSVec.push_back(rightrailshift);

	printf("wheel rail contact points lookup table initialized ...\n");
	FreeConsole();
}

std::vector<VectorN> RWheelSet::AssContPntsByLookupTable(Track &tk, Vector3x &ws_r, EulerAngle &ws_theta)
{
	std::vector<VectorN> wscontpnts;
	wscontpnts.clear();
	VectorN lcontpara(4);
	VectorN rcontpara(4);
	//
	return wscontpnts;
}
Matrix RWheelSet::EquivalentConicity(int flag,double ecstartpnt,double ecendpnt,double ECsize)
{
	Matrix wsrot=THETA0.getRotMat();
	Vector3x origin=R0;
	VectorN lcontpara(4);
	VectorN rcontpara(4);
	Matrix rollradiusdiff;
	if (flag == 0)// linear search algorithm
	{
		rollradiusdiff = this->LinearSearchRadiusDiff;
	}
	else // constraint equivalent algorithm
	{
		rollradiusdiff = this->RollingRadiusDiff;
	}
	CubicSpline RRDcurve(rollradiusdiff);
	//---------gauss integration
	std::vector<double> garmax;
	std::vector<double> garmay;
	double conven_delta=0.00000001; 
	double weight[5]={0.236927,0.478629,0.568889,0.478629,0.236927};
	double intpnt[5]={-0.90610,-0.538469,0.0,0.538469,0.90610};
	double inte_interval=2*3.14159267;
	for (double jj=ecstartpnt;jj<=ecendpnt;jj+=ECsize)
	{
		garmax.push_back(jj*1000);
		double inte1=0.0;
		double inte2=0.0;
		double low,high,tmp,coff;
		int sub=2;
		for (int k=0;k<sub;k++)
		{
			low=0.0+k*inte_interval/sub;
			high=0.0+(k+1)*inte_interval/sub;
			tmp=0.0;
			for (int j=0;j<5;j++)
			{
				coff=((high-low)/2)*intpnt[j]+((high+low)/2);
				tmp+=weight[j]*((high-low)/2)*RRDcurve.ValueAt(jj*sin(coff))*sin(coff);
			}
			inte2+=tmp;
		}
		while (abs(inte2-inte1)>conven_delta)
		{
			inte1=inte2;
			sub*=2;
			inte2=0.0;
			for (int k=0;k<sub;k++)
			{
				low=0.0+k*inte_interval/sub;
				high=0.0+(k+1)*inte_interval/sub;
				tmp=0.0;
				for (int j=0;j<5;j++)
				{
					coff=((high-low)/2)*intpnt[j]+((high+low)/2);
					tmp+=weight[j]*((high-low)/2)*RRDcurve.ValueAt(jj*sin(coff))*sin(coff);
				}
				inte2+=tmp;
			}
		}
		garmay.push_back(inte2/(-2.0*3.14159267*jj));
	}
	int size=(int)garmax.size();
	Matrix res(size,2);
	for (int ii=0;ii<size;ii++)
	{
		res[ii][0]=garmax[ii];
		res[ii][1]=garmay[ii];
	}
	return res;
}
VectorN RWheelSet::getCurrentKinematicPara(void)
{
	//10
	VectorN lw1 = leftcurrentcontactforce[0];//creepage 3
	VectorN lw2 = leftcurrentcontactforce[1];//creepforce 3
	VectorN lw3 = leftcurrentcontactforce[2];//contact force 3
	VectorN lw4 = leftcurrentcontactforce[3];//penetraton 1
	//10
	VectorN rw1 = rightcurrentcontactforce[0];//creepage 3
	VectorN rw2 = rightcurrentcontactforce[1];//creepforce 3
	VectorN rw3 = rightcurrentcontactforce[2];//contact force 3
	VectorN rw4 = rightcurrentcontactforce[3];//penetraton 1

	//9
	Vector3x current_R = this->getR();//3
	Vector3x current_Rd = this->getRD();//3
	Vector3x current_Rdd = this->getRDD();//3

	//9
	EulerAngle current_THETA = this->getTHETA();
	EulerAngle current_THETAd = this->getTHETAD();
	EulerAngle current_THETAdd = this->getTHETADD();

	//3
	Vector3x omega = ((this->getTHETA()).getRotMat())*((this->getTHETA()).getGconjMat()*this->getTHETAD().toVector3x());

	VectorN kine(41);
	kine[0] = current_R[0]; //x
	kine[1] = current_R[1]; //y
	kine[2] = current_R[2]; //z
	kine[3] = current_THETA[0]; //yaw
	kine[4] = current_THETA[1]; //roll
	kine[5] = current_THETA[2]; //pitch by y axis

	kine[6] = current_Rd[0]; //xd
	kine[7] = current_Rd[1]; //yd
	kine[8] = current_Rd[2]; //zd
	kine[9] = current_THETAd[0]; //
	kine[10] = current_THETAd[1]; //
	kine[11] = current_THETAd[2]; //

	kine[12] = current_Rdd[0]; //xdd
	kine[13] = current_Rdd[1]; //ydd
	kine[14] = current_Rdd[2]; //zdd
	kine[15] = current_THETAdd[0]; //xdd
	kine[16] = current_THETAdd[1]; //ydd
	kine[17] = current_THETAdd[2]; //zdd

	kine[18] = omega[0];  // angular_v x
	kine[19] = omega[1]; // angular_v y
	kine[20] = omega[2]; // angular_v z

	kine[21] = lw1[0]; // left_creepage x
	kine[22] = lw1[1]; // left_creepage y
	kine[23] = lw1[2]; // left_creepage spin
	kine[24] = lw2[0]; // left_creepforce x
	kine[25] = lw2[1]; // left_creepforce y
	kine[26] = lw2[2]; // left_creepforce spin
	kine[27] = lw3[0]; // left_contactforce 
	kine[28] = lw3[1]; // left_contactforce
	kine[29] = lw3[2]; // left_contactforce
	kine[30] = rw1[0]; // right_creepage x
	kine[31] = rw1[1]; // right_creepage y
	kine[32] = rw1[2]; // right_creepage spin
	kine[33] = rw2[0]; // right_creepforce x
	kine[34] = rw2[1]; // right_creepforce y
	kine[35] = rw2[2]; // right_creepforce spin
	kine[36] = rw3[0]; // right_contactforce
	kine[37] = rw3[1]; // right_contactforce
	kine[38] = rw3[2]; // right_contactforce

	kine[39] = lw4[0]; // left_penetration
	kine[40] = rw4[0]; // right_penetration
	
	return kine;
}
//void RWheelSet::InitConsolWnd(void)
//{
//	int nCrt = 0;
//	FILE* fp;
//	AllocConsole();
//	nCrt = _open_osfhandle((intptr_t)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
//	fp = _fdopen(nCrt, "w");
//	*stdout = *fp;
//	setvbuf(stdout, NULL, _IONBF, 0);
//}

void RWheelSet::InitConsolWnd(void)
{
	FILE* file = nullptr;
	AllocConsole();
	freopen_s(&file, "CONOUT$", "w", stdout);
}
RWheelSet::~RWheelSet(void)
{
	R0.Empty();
	Rd0.Empty();
	Rdd0.Empty();
	THETA0.Empty();
	THETAd0.Empty();
	THETAdd0.Empty();
	R.Empty();
	Rd.Empty();
	Rdd.Empty();
	THETA.Empty();
	THETAd.Empty();
	THETAdd.Empty();
	l_radius = 0;
	r_radius = 0.0;
	ld = 0;
	FLAG = 0;
	LeftWheelGeometry.Empty();
	RightWheelGeometry.Empty();
	LeftConPara0.Empty();
	RightConPara0.Empty();
	currentLeftContPara.Empty();
	currentRightContPara.Empty();
	LeftWheelContact.Empty();
	RightWheelContact.Empty();
	currentForceVector.Empty();
	ConPntFLAG = 0;
	leftcurrentcontactforce.clear();
	rightcurrentcontactforce.clear();
	RollingRadiusDiff.Empty();
	ConsEquContactPnts.Empty();
	LinearSearchContactPnts.Empty();
	LinearSearchRadiusDiff.Empty();
	leftLinearSearchContactPnts.clear();
	rightLinearSearchContactPnts.clear();
}