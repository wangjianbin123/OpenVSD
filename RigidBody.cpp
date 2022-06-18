#include "pch.h"
#include "RigidBody.h"

#include <vector>

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


RigidBody::RigidBody(void):InertiaElem()
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
	force.clear();
	obsvec.clear();
	currentForceVector.Empty();
}
RigidBody::RigidBody(int number,CString name,BOOL FLEX,BOOL NotGround,double mass,Vector3x &I, Vector3x &r0, Vector3x &rd0, Vector3x &rdd0, EulerAngle &theta0, 
					 EulerAngle &thetad0, EulerAngle &thetadd0):InertiaElem(number,name,FLEX,NotGround,mass,I)
{
	R0 = r0;
	Rd0 = rd0;
	Rdd0 = rdd0;
	THETA0 = theta0;
	THETAd0 = thetad0;
	THETAdd0 = thetadd0;
	//
	R=R0;
	Rd=Rd0;
	Rdd=Rdd0;
	THETA=THETA0;
	THETAd=THETAd0;
	THETAdd=THETAdd0;
	force.clear();
	obsvec.clear();
	currentForceVector.SetVectorN(6);
}
RigidBody::RigidBody(const RigidBody &sr)
{
	SN=sr.getSN();
	m_name=sr.getName();
	FLEX=sr.getFlexible();
	NG=sr.getGround();
	m=sr.getMass();
	principle_I=sr.getPrincipleI();
	principle_mat=sr.getPrincipleMat();
	g=sr.getGravity();
	wsmarker=sr.getMarkerVector();
	R0=sr.getR0(); 
	Rd0=sr.getRD0(); 
	Rdd0=sr.getRDD0();
	THETA0=sr.getTHETA0(); 
	THETAd0=sr.getTHETAD0(); 
	THETAdd0=sr.getTHETADD0();
	R=sr.getR();
	THETA=sr.getTHETA(); 
	Rd=sr.getRD();
	THETAd=sr.getTHETAD();
	Rdd=sr.getRDD(); 
	THETAdd=sr.getTHETADD();
	force=sr.getTPF();
	obsvec=sr.getObserverHdr();
	currentForceVector = sr.getCurrentForceVector();
}
RigidBody& RigidBody::operator=(const RigidBody &sr)
{
	if (this==&sr)
	{
		return *this;
	}
	else
	{
		SN=sr.getSN();
		m_name=sr.getName();
		FLEX=sr.getFlexible();
		NG=sr.getGround();
		m=sr.getMass();
		principle_I=sr.getPrincipleI();
		principle_mat=sr.getPrincipleMat();
		g=sr.getGravity();
		wsmarker=sr.getMarkerVector();
		R0=sr.getR0(); 
		Rd0=sr.getRD0(); 
		Rdd0=sr.getRDD0();
		THETA0=sr.getTHETA0(); 
		THETAd0=sr.getTHETAD0(); 
		THETAdd0=sr.getTHETADD0();
		R=sr.getR(); 
		THETA=sr.getTHETA(); 
		Rd=sr.getRD(); 
		THETAd=sr.getTHETAD();
		Rdd=sr.getRDD(); 
		THETAdd=sr.getTHETADD();
		force=sr.getTPF();
		obsvec=sr.getObserverHdr();
		currentForceVector = sr.getCurrentForceVector();
		return *this;
	}
}
Vector3x RigidBody::getMarker_Pos(int num,Vector3x &r,EulerAngle &theta)
{
	if (NG)// freedom rigid body
	{
		return r+(theta.getRotMat())*(wsmarker[num].getOrigin());
	}
	else// fixed rigid body represent ground
	{
		return R0+(this->getTHETA0().getRotMat())*(wsmarker[num].getOrigin());
	}
}
Vector3x RigidBody::getMarker_Vel(int num,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad)
{
	if(NG)
	{
		return rd - (theta.getRotMat()*((wsmarker[num].getOrigin()).toSkewMatrix())*(theta.getGconjMat()))*thetad.toVector3x();
	}
	else
	{
		Vector3x vec(0.0,0.0,0.0);
		return vec;
	}
}
void RigidBody::addForceVec(TPFElem &tpf)
{
	force.push_back(tpf);
}
EulerAngle RigidBody::getEulerAngle(void)
{
	return THETA;
}
Vector3x RigidBody::getMarkerLocalVec(int num)
{
	return wsmarker[num].getOrigin();
}
Matrix RigidBody::AssMassMat(EulerAngle &theta)
{
	Matrix Mrr(3,3);
	Matrix GconjMat = theta.getGconjMat();
	Mrr[0][0]=m; 
	Mrr[1][1]=m;
	Mrr[2][2]=m;
	Matrix Mtt=(~GconjMat)*principle_mat*(GconjMat);
	Matrix M(6,6);
	M.putSubMat(0,0,Mrr);
	M.putSubMat(3,3,Mtt);
	return M;
}
VectorN RigidBody::AssQuadForceVec(EulerAngle &theta,EulerAngle &thetad)
{
	Matrix GconjMat = theta.getGconjMat();
	VectorN omega_n=GconjMat*thetad.toVectorN();
	VectorN iomega=principle_mat*omega_n;
	VectorN tmp3=(omega_n.toSkewMatrix())*iomega;
	VectorN tmp4=(principle_mat*this->getGConjMatDer(theta,thetad))*thetad.toVectorN();
	VectorN force=(-1.0)*(~GconjMat)*(tmp3+tmp4);
	return force;
}
Vector3x RigidBody::AssGravityForce(void)
{
	Vector3x tmp=m*g;
	return tmp;
}
VectorN RigidBody::AssExternalForce(EulerAngle &theta,double splineindex)
{
	Vector3x GraForce=this->AssGravityForce();
	Vector3x tmp; tmp+=GraForce;
	if(!force.empty())
	{
		for (std::vector<TPFElem>::iterator it=force.begin();it<force.end();it++)
		{
			if ((*it).getflag()==0)
			{
				tmp+=theta.getRotMat()*(*it).getForceVec(splineindex);
			}
		}
	}
	return tmp.toVectorN();
}
VectorN RigidBody::AssExternalTorque(EulerAngle &theta,double splineindex)
{
	VectorN tmp(3);
	Matrix rotmat = theta.getRotMat();
	Matrix G = theta.getGMat();
	if (!force.empty())
	{
		for (std::vector<TPFElem>::iterator it=force.begin();it<force.end();it++)
		{
			if ((*it).getflag()==1)
			{
				tmp+=(~G)*(rotmat*(*it).getForceVec(splineindex).toVectorN());
			}
			else
			{
				Vector3x f=(*it).getForceVec(splineindex);
				Vector3x l=(*it).getat().getOrigin();
				Vector3x localM=l.crossProduct(f);
				VectorN tmp2=(~G)*(rotmat*localM.toVectorN());
				tmp+=tmp2;
			}
		}
	}
	return tmp;
}
VectorN RigidBody::AssembleForceVector(Vector3x &r,Vector3x &rd,EulerAngle &theta,EulerAngle &thetad,double splineindex)
{
	VectorN tmp(6);
	VectorN force=this->AssExternalForce(theta,splineindex);
	VectorN torque=this->AssExternalTorque(theta,splineindex);
	VectorN quarforce=this->AssQuadForceVec(theta,thetad);
	for(int i=0;i<3;i++)
	{
		tmp[i]=force[i];
	}
	for(int j=3;j<6;j++)
	{
		tmp[j]=torque[j-3]+quarforce[j-3];
	}
	return tmp;
}
void RigidBody::CreateObserverHdr(Observer& obs)
{
	ObserverHdr* pObs=new ObserverHdr(obs);
	obsvec.push_back(*pObs);
}
void RigidBody::TriggeIntPntResPushBackrProcess(void)
{
	for (std::vector<ObserverHdr>::iterator ite=obsvec.begin();ite<obsvec.end();ite++)
	{
		(*ite)->IntPntResPushBackprocess(*this);
	}
}
void RigidBody::TriggerSave(void)
{
	for (std::vector<ObserverHdr>::iterator ite=obsvec.begin();ite<obsvec.end();ite++)
	{
		(*ite)->save();
	}
}
void RigidBody::UpdateKimematicsPara(VectorN &q,VectorN &qd)
{
	R[0] = q[0];
	R[1] = q[1];
	R[2] = q[2];
	THETA[0] = q[3];
	THETA[1] = q[4];
	THETA[2] = q[5];
	Rd[0] = qd[0];
	Rd[1] = qd[1];
	Rd[2] = qd[2];
	THETAd[0] = qd[3];
	THETAd[1] = qd[4];
	THETAd[2] = qd[5];
}

void RigidBody::UpdateEquivalentPara(VectorN &q, VectorN &qd)
{
	R[0] = q[0];
	R[1] = q[1];
	R[2] = q[2];
	THETA[0] = q[3];
	THETA[1] = q[4];
	THETA[2] = q[5];
	Rd[0] = qd[0];
	Rd[1] = qd[1];
	Rd[2] = qd[2];
	THETAd[0] = qd[3];
	THETAd[1] = qd[4];
	THETAd[2] = qd[5];
}
void RigidBody::UpdateKinematicsParaAcc(VectorN &q, VectorN &qd, VectorN &qdd)
{
	R[0] = q[0];
	R[1] = q[1];
	R[2] = q[2];
	THETA[0] = q[3];
	THETA[1] = q[4];
	THETA[2] = q[5];
	Rd[0] = qd[0];
	Rd[1] = qd[1];
	Rd[2] = qd[2];
	THETAd[0] = qd[3];
	THETAd[1] = qd[4];
	THETAd[2] = qd[5];
	Rdd[0] = qd[0];
	Rdd[1] = qd[1];
	Rdd[2] = qd[2];
	THETAdd[0] = qd[3];
	THETAdd[1] = qd[4];
	THETAdd[2] = qd[5];
}
//20210414 NG statement
void RigidBody::UpdateInitialKinematicsParaPos(VectorN &q)
{
	if (NG)
	{
		R0[0] = q[0];
		R0[1] = q[1];
		R0[2] = q[2];
		THETA0[0] = q[3];
		THETA0[1] = q[4];
		THETA0[2] = q[5];
	}
	else
	{
		R0[0] = 0;
		R0[1] = 0;
		R0[2] = 0;
		THETA0[0] = 0;
		THETA0[1] = 0;
		THETA0[2] = 0;
	}
}
void RigidBody::UpdateInitialKinematicsParaPosVel(VectorN &q, VectorN &qd)
{
	if (NG)
	{
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
	else
	{
		R0[0] = 0;
		R0[1] = 0;
		R0[2] = 0;
		THETA0[0] = 0;
		THETA0[1] = 0;
		THETA0[2] = 0;
		//
		Rd0[0] = 0;
		Rd0[1] = 0;
		Rd0[2] = 0;
		THETAd0[0] = 0;
		THETAd0[1] = 0;
		THETAd0[2] = 0;
	}

}
void RigidBody::UpdateInitialKinematicsParaPosVelAcc(VectorN &q, VectorN &qd, VectorN &qdd)
{
	if (NG)
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
	else
	{
		//
		R0[0] = 0;
		R0[1] = 0;
		R0[2] = 0;
		THETA0[0] = 0;
		THETA0[1] = 0;
		THETA0[2] = 0;
		//
		Rd0[0] = 0;
		Rd0[1] = 0;
		Rd0[2] = 0;
		THETAd0[0] = 0;
		THETAd0[1] = 0;
		THETAd0[2] = 0;
		//
		Rdd0[0] = 0;
		Rdd0[1] = 0;
		Rdd0[2] = 0;
		THETAdd0[0] = 0;
		THETAdd0[1] = 0;
		THETAdd0[2] = 0;
	}
}
void RigidBody::SetRunSpeed(double V)
{
	if (NG)
	{
		Rd0[0] = V;
	}
	else //GND
	{
		Rd0[0] = 0.0;
	}
}
VectorN RigidBody::getCurrentKinematicPara(void)
{
	//For rigidbody, kinematics states output
	VectorN vecnr=R.toVectorN();
	VectorN vecntheta = THETA.toVectorN();
	VectorN vecnrd = Rd.toVectorN();
	VectorN vecnthetad = THETAd.toVectorN();
	VectorN vecnrdd = Rdd.toVectorN();
	VectorN vecnthetadd = THETAdd.toVectorN();
	// r theta rd thetad rdd thetadd 20210331
	VectorN tmp0 = vecnr.Concat(vecntheta);
	VectorN tmp1 = tmp0.Concat(vecnrd);
	VectorN tmp2 = tmp1.Concat(vecnthetad);
	VectorN tmp3 = tmp2.Concat(vecnrdd);
	VectorN tmp4 = tmp3.Concat(vecnthetadd);
	return tmp4;
}
VectorN RigidBody::getCurrentForcePara(void)
{
	return currentForceVector;
}
void RigidBody::UpdateForcePara(VectorN &vecn)
{
	currentForceVector = vecn;
}
RigidBody::~RigidBody(void)
{

}
Matrix RigidBody::getGConjMatDer(EulerAngle &theta,EulerAngle &thetad)
{
	Matrix mat(3,3);
	mat[0][0]=sin(theta[1])*sin(theta[2])*thetad[1]-cos(theta[1])*cos(theta[2])*thetad[2];
	mat[0][1] = (-1.0)*sin(theta[2])*thetad[2];
	mat[0][2] = 0.0;
	mat[1][0] = cos(theta[1])*thetad[1];
	mat[1][1]=0;
	mat[1][2] = 0.0;
	mat[2][0]=(-1.0)*sin(theta[1])*cos(theta[2])*thetad[1]-cos(theta[1])*sin(theta[2])*thetad[2];
	mat[2][1] = cos(theta[2])*thetad[2];
	mat[2][2]=0;
	return mat;
}
